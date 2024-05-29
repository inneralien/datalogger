//! Driver for the BME280 temperature, humidity, and pressure sensor from Adafruit.

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
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
const MODE_SLEEP: u8 = 0x00;
const MODE_FORCE: u8 = 0x01;
const MODE_NORMAL: u8 = 0x03;
// """Other Registers"""
const BME280_REGISTER_SOFTRESET: u8 = 0xE0;
const BME280_REGISTER_CTRL_HUM: u8 = 0xF2;
const BME280_REGISTER_STATUS: u8 = 0xF3;
const BME280_REGISTER_CTRL_MEAS: u8 = 0xF4;
const BME280_REGISTER_CONFIG: u8 = 0xF5;
const BME280_REGISTER_TEMPDATA: u8 = 0xFA;
const BME280_REGISTER_HUMIDDATA: u8 = 0xFD;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello BME280!");
    let p = embassy_stm32::init(Default::default());

    let button = Input::new(p.PA0, Pull::Down);
    //    let _led1 = Output::new(p.PA5, Level::High, Speed::Low);

    let mut i2c = I2c::new_blocking(p.I2C3, p.PA8, p.PC9, Hertz(100_000), Default::default());

    // A buffer of size 3 will read back the first 3 bytes starting at offset
    // &[SECONDS]
    let mut data = [0u8; 1];
    let interval: Duration = embassy_time::Duration::from_millis(500);

    loop {
        if button.is_high() {
            // Read the contents of the Status register repeatedly
            //            match i2c.blocking_write_read(DEVICE_ADDR, &[BME280_REGISTER_STATUS], &mut data) {
            match i2c.blocking_write_read(BME280_ADDRESS, &[BME280_REGISTER_CHIPID], &mut data) {
                Ok(()) => {
                    info!("\nID: {:08b} : {:x}", data[0], data[0])
                }
                Err(Error::Timeout) => error!("Operation timed out"),
                Err(e) => error!("I2c Error: {:?}", e),
            }
            Timer::after(interval).await;
        }
    }
}
