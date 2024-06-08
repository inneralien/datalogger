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
    async fn as_i32(&self) -> i32 {
        // _read24() equivalent from Python lib
        // https://github.com/adafruit/Adafruit_CircuitPython_BME280/blob/0baf6d0373b94974793b4c34dec7f7e4279ed9ab/adafruit_bme280/basic.py#L305
        // let folded = self.0.iter().fold(0.0, |acc, e| acc * 256.0 + (*e as f32));
        // debug!("vals: {}", self.0);
        let folded = self.0.iter().fold(0_i32, |acc, e| acc * 256 + (*e as i32));
        // debug!("folded: {}", folded);
        // /16
        let adc_t = folded >> 4;
        // let adc_t =
        // u32::from(self.0[0]) << 12 | u32::from(self.0[1]) << 4 | u32::from(self.0[2]) >> 4;
        // debug!("adc_t: {}", adc_t);

        let temp_calib = TEMP_CALIB.get().await;
        // debug!("temp_calib[0]: {}", temp_calib[0]);
        // debug!("temp_calib[1]: {}", temp_calib[1]);
        // debug!("temp_calib[2]: {}", temp_calib[2]);

        let adc_t_div_14 = adc_t >> 14;
        let adc_t_div_17 = adc_t >> 17;
        // 2^14
        // debug!(
        //     "adc_t >> 3: {}   adc_t / 16384: {}",
        //     adc_t >> 14,
        //     (adc_t as i32) / 16384
        // );
        // debug!("temp_calib[0] << 1: {}", (temp_calib[0] as i32) << 1);
        // debug!("temp_calib[1]: {}", temp_calib[1] as i32);

        // debug!("adc_t / 16384: {}", (adc_t as i32) / 16384);
        let ref_var1 =
            ((adc_t as i32) / 16384 - (temp_calib[0] as i32) / 1024) * (temp_calib[1] as i32);
        let var1 = (((adc_t >> 3) - ((temp_calib[0] as i32) << 1)) * (temp_calib[1] as i32)) >> 11;
        // debug!("var1: {}", var1 as f32);
        // debug!("ref_var1: {}", ref_var1);
        let var2 = ((((adc_t >> 4) - (temp_calib[0] as i32))
            * ((adc_t >> 4) - (temp_calib[0] as i32)))
            >> 12)
            * (temp_calib[2] as i32)
            >> 14;
        // 2^17
        // let var2 = (((adc_t as i32) / 131072) - temp_calib[0] as i32 / 8192)
        // * ((adc_t as i32) / 131072 - temp_calib[0] as i32 / 8192)
        // * temp_calib[2] as i32;
        // debug!("var2: {}", var2);

        let t_fine = var1 + var2;
        // debug!("t_fine: {}", t_fine);
        //        (t_fine as i32) / 5120.0
        let degrees_c = (t_fine * 5 + 128) >> 8;
        // debug!("degrees_c: {}", degrees_c);
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
        // debug!("Calibration Coefficients: {:x}", data);
        let values = unpack_bytes(&data);
        debug!("Calibration Coefficients: {:#?}", values);
        if let Err(e) = TEMP_CALIB.init(values) {
            error!(
                ">>> ERROR INITIALIZING CALIBRATION COEFFICIENTS: {:?} <<<",
                e
            );
        };
        //        self.temp_calib = Some(values);
        Ok(self)
    }

    async fn get_status(&mut self) -> Result<u8, Error> {
        let mut data = [0u8; 1];
        self.i2c
            .blocking_write_read(BME280_ADDRESS, &[BME280_REGISTER_STATUS], &mut data)?;
        Ok(data[0])
    }

    async fn read_temperature(&mut self) -> Result<&mut Self, Error> {
        // def _read_temperature(self) -> None:
        // # perform one measurement
        // if self.mode != MODE_NORMAL:
        //     self.mode = MODE_FORCE
        //     # Wait for conversion to complete
        //     while self._get_status() & 0x08:
        //         sleep(0.002)
        // raw_temperature = (
        //     self._read24(_BME280_REGISTER_TEMPDATA) / 16
        // )  # lowest 4 bits get dropped
        //
        // var1 = (
        //     raw_temperature / 16384.0 - self._temp_calib[0] / 1024.0
        // ) * self._temp_calib[1]
        //
        // var2 = (
        //     (raw_temperature / 131072.0 - self._temp_calib[0] / 8192.0)
        //     * (raw_temperature / 131072.0 - self._temp_calib[0] / 8192.0)
        // ) * self._temp_calib[2]
        //
        // self._t_fine = int(var1 + var2)

        // Set mode to force
        self.set_mode(Mode::Force)?;
        // Wait for conversion to complete
        // debug!("Waiting for conversion to complete");
        while self.get_status().await? & 0x08 != 0 {
            // debug!("in while loop");
            Timer::after(Duration::from_millis(2)).await;
        }
        debug!("Conversion complete");

        let mut raw_temp = RawTemp([0u8; 3]);
        self.i2c.blocking_write_read(
            BME280_ADDRESS,
            &[BME280_REGISTER_TEMPDATA],
            &mut raw_temp.0,
        )?;
        let value = raw_temp.as_i32().await;
        info!(
            "Temp {} raw   {}C  {}F",
            &raw_temp,
            (value as f32) * 0.01,
            (value as f32) * 0.01 * 1.8 + 32.0
        );
        // let a = raw_temp.iter().fold(0.0, |acc, e| acc * 256.0 + e);

        Ok(self)
    }
}

fn unpack_bytes(bytes: &[u8]) -> [i16; 3] {
    // let mut values = Vec::new();
    //
    // values.push(u16::from_le_bytes([bytes[0], bytes[1]]) as i16)?;
    // values.push(i16::from_le_bytes([bytes[2], bytes[3]]))?;
    // values.push(i16::from_le_bytes([bytes[4], bytes[5]]))?;

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
    match bme.read_calibration_coeffs().await {
        Ok(_) => {
            info!("\nCalibration Coefficients")
        }
        Err(Error::Timeout) => error!("Operation timed out"),
        Err(e) => error!("I2c Error: {:?}", e),
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

// Unit tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_from_raw_temp() {
        let mut raw_temp = RawTemp([0u8; 3]);
        raw_temp.0[0] = 0x86;
        raw_temp.0[1] = 0x19;
        raw_temp.0[2] = 0x00;

        assert_eq!(raw_temp.as_i32(), 28.0);
    }
}
