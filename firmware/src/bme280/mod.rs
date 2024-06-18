pub mod bme280;
pub(crate) mod raw_temp;

use embassy_sync::once_lock::OnceLock;
static TEMP_CALIB: OnceLock<[i16; 3]> = OnceLock::new();

// Re-export the public API things
// This avoids having to do firmware::bme280::bme280::BME280I2c, etc
//use bme280::RawTemp;
pub use bme280::{BME280I2c, BME280Message, ControlSignal};
