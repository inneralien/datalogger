use defmt::*;
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::mode::Blocking;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::Channel as MessageChannel;
use embassy_sync::once_lock::OnceLock;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};

use crate::bme280::raw_temp::RawTemp;
use crate::bme280::TEMP_CALIB;

// BME280
// The LSB of the address byte is the R/W bit added by the I2C protocol
#[allow(clippy::unusual_byte_groupings)]
const DEVICE_ADDR: u8 = 0b0111_011; // 0x76 or 0x77

const BME280_ADDRESS: u8 = 0x77;
const BME280_CHIPID: u8 = 0x60;
const BME280_REGISTER_CHIPID: u8 = 0xD0;
// """overscan values for temperature, pressure, and humidity"""
const OVERSCAN_X1: u8 = 0x01;
const OVERSCAN_X16: u8 = 0x05;
// """mode values"""
//const BME280_MODES = (0x00, 0x01, 0x03);
// """iir_filter values"""
const IIR_FILTER_DISABLE: u8 = 0;
// """
// standby timeconstant values
// TC_X[_Y] where X=milliseconds and Y=tenths of a millisecond
// """
const STANDBY_TC_125: u8 = 0x02; // 125ms
                                 // """mode values"""
                                 // const MODE_SLEEP: u8 = 0x00;
                                 // const MODE_FORCE: u8 = 0x01;
                                 // const MODE_NORMAL: u8 = 0x03;
                                 // """Other Registers"""
const BME280_REGISTER_SOFTRESET: u8 = 0xE0;
const _BME280_REGISTER_CTRL_HUM: u8 = 0xF2;
const BME280_REGISTER_STATUS: u8 = 0xF3;
const BME280_REGISTER_CTRL_MEAS: u8 = 0xF4;
const BME280_REGISTER_CONFIG: u8 = 0xF5;
const BME280_REGISTER_TEMPDATA: u8 = 0xFA;
const _BME280_REGISTER_HUMIDDATA: u8 = 0xFD;

const BME280_REGISTER_PRESS: u8 = 0xF7;

const BME280_RESET_SLEEP_TIME: Duration = embassy_time::Duration::from_millis(2);

// OnceCell for temp_calib data
//static TEMP_CALIB: OnceLock<[i16; 3]> = OnceLock::new();

pub struct BME280Message {
    _temperature: f32,
}

pub enum ControlSignal {
    Start,
    Stop,
    Cancel,
}

enum Mode {
    Sleep = 0b00,
    Force = 0b01,
    Normal = 0b11,
}

#[derive(Debug, Default)]
struct Measurements {
    temperature: u32,
    pressure: u32,
    humidity: u16,
}

pub struct Bme280I2c<'a> {
    i2c: I2c<'a, Blocking>,
    measurements: Measurements,
    temp_calib: Option<[i16; 3]>,
    overscan_temperature: u8,
    overscan_pressure: u8,
}

impl<'a> Bme280I2c<'a> {
    pub fn new(i2c: I2c<'a, Blocking>) -> Self {
        Self {
            i2c,
            measurements: Measurements::default(),
            temp_calib: None,
            overscan_temperature: OVERSCAN_X1,
            overscan_pressure: OVERSCAN_X16,
        }
    }

    pub async fn init(&mut self) -> Result<&mut Self, Error> {
        let chipid = self.chipid()?;
        if chipid[0] != BME280_CHIPID {
            defmt::error!("BME280 chipid mismatch: {}", chipid[0]);
        }
        self.soft_reset().await?;
        self.read_calibration_coeffs().await?;
        Ok(self)
    }

    async fn soft_reset(&mut self) -> Result<&mut Self, Error> {
        self.i2c
            .blocking_write(BME280_ADDRESS, &[BME280_REGISTER_SOFTRESET, 0xB6])?;
        Timer::after(BME280_RESET_SLEEP_TIME).await;
        Ok(self)
    }

    fn chipid(&mut self) -> Result<[u8; 1], Error> {
        let mut data = [0u8; 1];
        self.i2c
            .blocking_write_read(BME280_ADDRESS, &[BME280_REGISTER_CHIPID], &mut data)?;
        Ok(data)
    }

    fn set_mode(&mut self, mode: Mode) -> Result<&mut Self, Error> {
        let mut ctrl_meas = self.overscan_temperature << 5;
        ctrl_meas += self.overscan_pressure << 2;
        ctrl_meas += mode as u8;
        self.i2c
            .blocking_write(BME280_ADDRESS, &[BME280_REGISTER_CTRL_MEAS, ctrl_meas])?;
        Ok(self)
    }

    fn read_config(&mut self) -> Result<[u8; 1], Error> {
        let mut data = [0u8; 1];
        self.i2c
            .blocking_write_read(BME280_ADDRESS, &[BME280_REGISTER_CONFIG], &mut data)?;
        Ok(data)
    }

    fn _read_all_sensors(&mut self) -> Result<&mut Self, Error> {
        let mut data = [0u8; 8];
        self.i2c
            .blocking_write_read(BME280_ADDRESS, &[BME280_REGISTER_PRESS], &mut data)?;
        self.measurements.pressure =
            ((data[0] as u32) << 12) | ((data[1] as u32) << 4) | ((data[2] as u32) >> 4);
        self.measurements.temperature =
            ((data[3] as u32) << 12) | ((data[4] as u32) << 4) | ((data[5] as u32) >> 4);
        self.measurements.humidity = ((data[6] as u16) << 8) | (data[7] as u16);
        Ok(self)
    }

    async fn read_calibration_coeffs(&mut self) -> Result<&mut Self, Error> {
        let mut data = [0u8; 24];
        self.i2c
            .blocking_write_read(BME280_ADDRESS, &[0x88], &mut data)?;
        let values = unpack_coefficient_bytes(&data);
        debug!("Calibration Coefficients: {:#?}", values);
        if let Err(e) = TEMP_CALIB.init(values) {
            error!("Calibration Coefficients have already been set: {:?}", e);
        };
        Ok(self)
    }

    async fn get_status(&mut self) -> Result<u8, Error> {
        let mut data = [0u8; 1];
        self.i2c
            .blocking_write_read(BME280_ADDRESS, &[BME280_REGISTER_STATUS], &mut data)?;
        Ok(data[0])
    }

    /// Read out just the temperature from the sensor
    ///
    /// TODO: For some reason the very first read after reset is  always the
    /// default value:
    /// RAW Bytes        | C i32    | C f32 | F f32
    /// [0x80, 0x0, 0x0] | 2030_i32 | 20.3C | 68.53999F
    //    pub async fn read_temperature(&mut self) -> Result<&mut Self, Error> {
    pub async fn read_temperature(&mut self) -> Result<f32, Error> {
        // Set mode to force
        self.set_mode(Mode::Force)?;
        // Wait for conversion to complete
        while self.get_status().await? & 0x08 != 0 {
            // The datasheet says to wait 2ms
            Timer::after(Duration::from_millis(2)).await;
        }
        trace!("Conversion complete");

        let mut raw_temp = RawTemp([0u8; 3]);
        self.i2c.blocking_write_read(
            BME280_ADDRESS,
            &[BME280_REGISTER_TEMPDATA],
            &mut raw_temp.0,
        )?;

        let degrees_milli_c = raw_temp.as_i32().await;
        // info!(
        // "Temp raw {} {}_i32 {}C  {}F",
        // &raw_temp,
        // degrees_milli_c,
        // (degrees_milli_c as f32) * 0.01,
        // (degrees_milli_c as f32) * 0.01 * 1.8 + 32.0
        // );

        Ok((degrees_milli_c as f32) * 0.01)
    }
}

fn unpack_coefficient_bytes(bytes: &[u8]) -> [i16; 3] {
    [
        u16::from_le_bytes([bytes[0], bytes[1]]) as i16,
        i16::from_le_bytes([bytes[2], bytes[3]]),
        i16::from_le_bytes([bytes[4], bytes[5]]),
    ]
}
pub struct BME280I2c {
    _data_channel: MessageChannel<ThreadModeRawMutex, BME280Message, 2>,
    _control_signal: Signal<CriticalSectionRawMutex, ControlSignal>,
}

impl BME280I2c {
    pub fn new(
        _data_channel: MessageChannel<ThreadModeRawMutex, BME280Message, 2>,
        _control_signal: Signal<CriticalSectionRawMutex, ControlSignal>,
    ) -> Self {
        BME280I2c {
            _data_channel,
            _control_signal,
        }
    }

    pub async fn run() {
        loop {
            //            let message = BME280I2c.read().await;
            //            self.data_channel.send(message).await;
        }
    }
}
