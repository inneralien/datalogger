#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{Error, I2c};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

// DS1307 Read/Write addresses
// The LSB of the address byte is the R/W bit added by the I2C protocol
#[allow(clippy::unusual_byte_groupings)]
const DEVICE_ADDR: u8 = 0b1101_000;

//const ADDRESS: u8 = 0x5F;
const SECONDS: u8 = 0x00;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello world!");
    let p = embassy_stm32::init(Default::default());

    let button = Input::new(p.PC13, Pull::Down);
    let _led1 = Output::new(p.PA5, Level::High, Speed::Low);

    let mut i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB9, Hertz(100_000), Default::default());

    // A buffer of size 3 will read back the first 3 bytes starting at offset
    // &[SECONDS]
    let mut data = [0u8; 3];
    let interval: Duration = embassy_time::Duration::from_millis(500);

    loop {
        if button.is_low() {
            // Read the contents of the SECONDS address repeatedly
            match i2c.blocking_write_read(DEVICE_ADDR, &[SECONDS], &mut data) {
                Ok(()) => {
                    info!(
                        "\nseconds: {:08b}\nminutes: {:08b}\nhours:   {:08b}",
                        data[0], data[1], data[2]
                    )
                }
                Err(Error::Timeout) => error!("Operation timed out"),
                Err(e) => error!("I2c Error: {:?}", e),
            }
            Timer::after(interval).await;
        }
    }

    //    match i2c.blocking_write_read(ADDRESS, &[SECONDS], &mut data) {
    //        Ok(()) => info!("Whoami: {}", data[0]),
    //        Err(Error::Timeout) => error!("Operation timed out"),
    //        Err(e) => error!("I2c Error: {:?}", e),
    //    }
}
