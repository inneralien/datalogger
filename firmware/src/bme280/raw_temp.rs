use defmt::*;

use super::TEMP_CALIB;

#[derive(Debug, Default)]
pub(crate) struct RawTemp(pub [u8; 3]);

impl RawTemp {
    /// Convert the raw temperature data to a signed 32-bit integer in 100ths of a Degress Celsius
    ///
    /// For example, a value of 2000 would be 20.00 degrees Celsius.
    /// To convert to degrees Celsius simply, divide by 100.0
    /// To convert to Fahrenheit, `F = C * 1.8 + 32`
    pub async fn as_i32(&self) -> i32 {
        // _read24() equivalent from Python lib
        // https://github.com/adafruit/Adafruit_CircuitPython_BME280/blob/0baf6d0373b94974793b4c34dec7f7e4279ed9ab/adafruit_bme280/basic.py#L305
        let folded = self.0.iter().fold(0_i32, |acc, e| acc * 256 + (*e as i32));
        // Divide by 16 by shifting
        let adc_t = folded >> 4;
        let mut degrees_c = 0;
        // Get the static calibration data
        if !TEMP_CALIB.is_set() {
            error!("called `as_i32` before calibration coefficients have been read");
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
