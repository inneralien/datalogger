//! Driver for the BME280 temperature, humidity, and pressure sensor from Adafruit.

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::mode::Blocking;
use embassy_stm32::time::Hertz;
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, Timer};
use micromath::F32Ext;
use {defmt_rtt as _, panic_probe as _};

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
static TEMP_CALIB: OnceLock<[i16; 3]> = OnceLock::new();

#[derive(Debug, Default)]
struct RawTemp([u8; 3]);

impl RawTemp {
    /// Convert the raw temperature data to a signed 32-bit integer in 100ths of a Degress Celsius
    ///
    /// For example, a value of 2000 would be 20.00 degrees Celsius.
    /// To convert to degrees Celsius simply, divide by 100.0
    /// To convert to Fahrenheit, `F = C * 1.8 + 32`
    async fn as_i32(&self) -> i32 {
        // _read24() equivalent from Python lib
        // https://github.com/adafruit/Adafruit_CircuitPython_BME280/blob/0baf6d0373b94974793b4c34dec7f7e4279ed9ab/adafruit_bme280/basic.py#L305
        let folded = self.0.iter().fold(0_i32, |acc, e| acc * 256 + (*e as i32));
        // Divide by 16 by shifting
        let adc_t = folded >> 4;
        let mut degrees_c = 0;
        // Get the static calibration data
        if !TEMP_CALIB.is_set() {
            error!(
                "Calling `as_i32` before calibration coefficients have been read will never work."
            );
        } else {
            let temp_calib = TEMP_CALIB.get().await;
            // These equations are taken from the BME280 datasheet
            let var1 =
                (((adc_t >> 3) - ((temp_calib[0] as i32) << 1)) * (temp_calib[1] as i32)) >> 11;
            let var2 = ((((adc_t >> 4) - (temp_calib[0] as i32))
                * ((adc_t >> 4) - (temp_calib[0] as i32)))
                >> 12)
                * (temp_calib[2] as i32)
                >> 14;
            let t_fine = var1 + var2;
            degrees_c = (t_fine * 5 + 128) >> 8;
        }
        degrees_c
    }
}

impl Format for RawTemp {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{:#x}", self.0);
    }
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

struct Bme280I2c<'a> {
    i2c: I2c<'a, Blocking>,
    measurements: Measurements,
    temp_calib: Option<[i16; 3]>,
    overscan_temperature: u8,
    overscan_pressure: u8,
}

impl<'a> Bme280I2c<'a> {
    fn new(i2c: I2c<'a, Blocking>) -> Self {
        Self {
            i2c,
            measurements: Measurements::default(),
            temp_calib: None,
            overscan_temperature: OVERSCAN_X1,
            overscan_pressure: OVERSCAN_X16,
        }
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
    async fn read_temperature(&mut self) -> Result<&mut Self, Error> {
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
        info!(
            "Temp raw {} {}_i32 {}C  {}F",
            &raw_temp,
            degrees_milli_c,
            (degrees_milli_c as f32) * 0.01,
            (degrees_milli_c as f32) * 0.01 * 1.8 + 32.0
        );

        Ok(self)
    }
}

fn unpack_coefficient_bytes(bytes: &[u8]) -> [i16; 3] {
    [
        u16::from_le_bytes([bytes[0], bytes[1]]) as i16,
        i16::from_le_bytes([bytes[2], bytes[3]]),
        i16::from_le_bytes([bytes[4], bytes[5]]),
    ]
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello BME280!");
    let p = embassy_stm32::init(Default::default());

    let button = Input::new(p.PA0, Pull::Down);
    //    let _led1 = Output::new(p.PA5, Level::High, Speed::Low);

    let i2c = I2c::new_blocking(p.I2C3, p.PA8, p.PC9, Hertz(100_000), Default::default());
    let mut bme = Bme280I2c::new(i2c);

    // A buffer of size 3 will read back the first 3 bytes starting at offset
    // &[SECONDS]
    //    let mut data = [0u8; 1];
    let interval: Duration = embassy_time::Duration::from_millis(500);

    // Read out the Device ID
    //    match read_chipid(&mut i2c) {
    match bme.chipid() {
        Ok(data) => {
            info!("\nDevice ID: {:08b} : {:x}", data[0], data[0])
        }
        Err(Error::Timeout) => error!("Operation timed out"),
        Err(e) => error!("I2c Error: {:?}", e),
    }

    // Soft Reset
    match bme.soft_reset().await {
        Ok(_) => {
            info!("\nSoft Reset")
        }
        Err(Error::Timeout) => error!("Operation timed out"),
        Err(e) => error!("I2c Error: {:?}", e),
    }

    // Read Calibration Coefficients
    for _ in 0..2 {
        match bme.read_calibration_coeffs().await {
            Ok(_) => {
                info!("\nCalibration Coefficients")
            }
            Err(Error::Timeout) => error!("Operation timed out"),
            Err(e) => error!("I2c Error: {:?}", e),
        }
    }

    // Read out Config
    match bme.read_config() {
        Ok(data) => {
            info!("\nConfig: {:08b} : {:x}", data[0], data[0])
        }
        Err(Error::Timeout) => error!("Operation timed out"),
        Err(e) => error!("I2c Error: {:?}", e),
    }

    // let mut data = [0u8; 1];
    loop {
        if button.is_high() {
            // Read the contents of the Status register repeatedly
            //            match i2c.blocking_write_read(DEVICE_ADDR, &[BME280_REGISTER_STATUS], &mut data) {
            // match i2c.blocking_write_read(BME280_ADDRESS, &[BME280_REGISTER_CHIPID], &mut data) {
            //     Ok(()) => {
            //         info!("\nID: {:08b} : {:x}", data[0], data[0])
            //     }
            //     Err(Error::Timeout) => error!("Operation timed out"),
            //     Err(e) => error!("I2c Error: {:?}", e),
            // }
            match bme.read_temperature().await {
                Ok(data) => {
                    // info!(
                    //     "\nPressure: {}\nTemperature: {}\nHumidity: {}",
                    //     data.pressure, data.temperature, data.humidity
                    // )
                }
                Err(Error::Timeout) => error!("Operation timed out"),
                Err(e) => error!("I2c Error: {:?}", e),
            };
            Timer::after(interval).await;
        }
    }
}

// fn read_chipid(i2c: &mut I2c<Blocking>) -> Result<[u8; 1], Error> {
// let mut data = [0u8; 1];
// i2c.blocking_write_read(BME280_ADDRESS, &[BME280_REGISTER_CHIPID], &mut data)?;
// Ok(data)
// }
//
// fn read_config(i2c: &mut I2c<Blocking>) -> Result<[u8; 1], Error> {
// let mut data = [0u8; 1];
// i2c.blocking_write_read(BME280_ADDRESS, &[BME280_REGISTER_CONFIG], &mut data)?;
// Ok(data)
// }
//
// fn read_all_sensors(i2c: &mut I2c<Blocking>) -> Result<[u8; 8], Error> {
// let mut data = [0u8; 8];
// i2c.blocking_write_read(BME280_ADDRESS, &[BME280_REGISTER_PRESS], &mut data)?;
// Ok(data)
// }

// fn read_all_sensors_ds(i2c: &mut I2c<Blocking>) -> Result<BME280, Error> {
//     let mut data = [0u8; 8];
//     i2c.blocking_write_read(BME280_ADDRESS, &[BME280_REGISTER_PRESS], &mut data)?;
//     let pressure = ((data[0] as u32) << 12) | ((data[1] as u32) << 4) | ((data[2] as u32) >> 4);
//     let temperature = ((data[3] as u32) << 12) | ((data[4] as u32) << 4) | ((data[5] as u32) >> 4);
//     let humidity = ((data[6] as u16) << 8) | (data[7] as u16);
//     Ok(BME280 {
//         pressure,
//         temperature,
//         humidity,
//     })
// }
