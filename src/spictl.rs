use std::io;
use spidev::{Spidev, SpidevOptions, SpidevTransfer, SPI_MODE_1, SPI_MODE_0};
use cupi::{CuPi, delay_ms, DigitalWrite};
use cupi::PinOutput;

const DAC_RST_N: usize = 25;
const HV_GAIN: f64 = ((HV_FULL_SCALE * 1.2) / 12.0); // 20% fudge factor due to low loading
const VREF: f64 = 0.6;
use HV_FULL_SCALE; // defined in main.rs
use HV_MIN_SERVO_V;

#[derive(Debug)]
pub enum HvSetErr {
//    None,
}

fn create_adc() -> io::Result<Spidev> {
    let mut spi = try!(Spidev::open("/dev/spidev0.0"));
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(4_000_000) 
        .lsb_first(false)
        .mode(SPI_MODE_0)
        .build();
    spi.configure(&options).unwrap();
    
    Ok(spi)
}

pub struct AdcRead {
    initialized: bool,
    spi: Spidev,
    hv_gain: f64,
}

impl AdcRead {
    pub fn new() -> Result <AdcRead, HvSetErr> {
        let adc = AdcRead {
            initialized: true,
            spi: create_adc().unwrap(),
            hv_gain: 0.59120,  // this should become a parameter later on
        };
        
        Ok(adc)
    }

    pub fn cleanup(&mut self) {
        if self.initialized {
            self.initialized = false;
        }
    }

    // returns the ADC code
    pub fn read_code(&self) -> u16 {
        let tx_buf = [0; 2];
        let mut rx_buf = [0; 2];
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            self.spi.transfer(&mut transfer).unwrap();
        }
        
//        println!("{:?}", &rx_buf);
        let retval: u16 = ((rx_buf[0] & 0x1f) as u16) << 7 | (((rx_buf[1] & 0xfe) as u16) >> 1);

        return retval
    }

    // returns the estimated HV input
    pub fn read_hv(&self) -> f64 {
        let read_voltage = (self.read_code() as f64) * self.hv_gain;

        return read_voltage
    }
}

impl Drop for AdcRead {
    fn drop(&mut self) {
        self.cleanup();
    }
}


fn create_spi() -> io::Result<Spidev> {
    let mut spi = try!(Spidev::open("/dev/spidev0.1"));
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(5_000_000) // actually 50_000_000 is theoretical peak speed, but slow down to reduce noise
        .lsb_first(false)
        .mode(SPI_MODE_1)
        .build();
    spi.configure(&options).unwrap();
    
    Ok(spi)
}

pub struct HvSet {
    initialized: bool,
    spi: Spidev,
    pin_dac_rst_n: PinOutput,
    hvcode: u16,
}

impl HvSet {
    pub fn new(cupi: &CuPi) -> Result<HvSet,HvSetErr> {
        let mut hvset = HvSet {
            initialized: true,
            spi: create_spi().unwrap(),
            pin_dac_rst_n: cupi.pin(DAC_RST_N).unwrap().output(),
            hvcode: 9, // set for 0.7V default
        };
        
        hvset.pin_dac_rst_n.low().unwrap(); // bring DAC into reset
        delay_ms(1);
        hvset.pin_dac_rst_n.high().unwrap(); // bring DAC out of reset

        hvset.spi = create_spi().unwrap();

        delay_ms(2);
        let mut tx_buf = [0x18, 0x02];  // unlock RDAC wiper
        let mut rx_buf = [0; 2];
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            hvset.spi.transfer(&mut transfer).unwrap();
        }

        tx_buf = [0x20, 0x00];  // power on command
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            hvset.spi.transfer(&mut transfer).unwrap();
        }
    
        tx_buf = [(0x04 | (hvset.hvcode >> 8) & 0xFF) as u8 , (hvset.hvcode & 0xFF) as u8]; // write command
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            hvset.spi.transfer(&mut transfer).unwrap();
        }

        // this is ironically documented as a footnote
        tx_buf = [0x80, 0x01]; // prep for SDO Hi-Z
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            hvset.spi.transfer(&mut transfer).unwrap();
        }
        tx_buf = [0x00, 0x00]; // nop command
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            hvset.spi.transfer(&mut transfer).unwrap();
        }
        
        delay_ms(2);
        
        Ok(hvset)
    }

    pub fn cleanup(&mut self) {
        if self.initialized {
            self.initialized = false;
        }
    }

    // sets DAC to a given code. Causes top resistor to be (code * (100k / 1024)) ohms
    pub fn set_code(&mut self, code: u16) -> u16 {
        self.hvcode = code;
        if self.hvcode > 1023 {
            self.hvcode = 1023;
        }

        delay_ms(1);
        let mut tx_buf = [0x18, 0x02];  // unlock RDAC wiper
        let mut rx_buf = [0; 2];
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            self.spi.transfer(&mut transfer).unwrap();
        }

        tx_buf = [0x20, 0x00];  // power on command
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            self.spi.transfer(&mut transfer).unwrap();
        }
        delay_ms(1);
        
        let mut tx_buf = [(0x04 | (self.hvcode >> 8) & 0xFF) as u8 , (self.hvcode & 0xFF) as u8]; // write command
        let mut rx_buf = [0; 2];
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            self.spi.transfer(&mut transfer).unwrap();
        }
        delay_ms(1);
        
        tx_buf = [0x08, 0x00]; // read command
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            self.spi.transfer(&mut transfer).unwrap();
        }
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            self.spi.transfer(&mut transfer).unwrap();
        }
        println!("{:?}", &rx_buf);
        let retval: u16 = ((rx_buf[0] & 0x3) as u16) << 8 | (rx_buf[1] as u16);

        // this is ironically documented as a footnote
        tx_buf = [0x80, 0x01]; // prep for SDO Hi-Z
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            self.spi.transfer(&mut transfer).unwrap();
        }
        tx_buf = [0x00, 0x00]; // nop command
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            self.spi.transfer(&mut transfer).unwrap();
        }
        
        return retval
    }

    pub fn set_hv_target(&mut self, voltage: u16) -> u16 {
        // Vout = gain * vref * (DAC-R / 5100 + 1)
        // DAC-R = code * (100_000 / 1024)
        // Vout = gain * vref * ((code * (100_000 / 1024)) / 5100 + 1)
        // Vout / (gain * vref) - 1 = (code * (100_000 / 1024)) / 5100
        // (Vout / (gain * vref) - 1) * 5100 = (code * (100_000 / 1024))
        // ((Vout / (gain * vref) - 1) * 5100) * (1024 / 100_000) = code
        let mut v: u16 = voltage;
        if voltage < HV_MIN_SERVO_V {
            println!("Warning: voltage target less than minimum, setting to {}V", HV_MIN_SERVO_V);
            v = HV_MIN_SERVO_V;
        }
        if voltage > HV_FULL_SCALE as u16 {
            println!("Warning: voltage target greater than full scale, setting to {}", HV_FULL_SCALE);
            v = HV_FULL_SCALE as u16;
        }
        let code: u16 = ((((v as f64) / (HV_GAIN * VREF) - 1.0) * 5100.0) * (1024.0 / 100_000.0)) as u16;
        self.set_code( code );

        return code;
    }

}

impl Drop for HvSet {
    fn drop(&mut self) {
        self.cleanup();
    }
}

#[test]
fn cycleResistance(Spidev: &spi) {
    let mut tx_buf = [0; 2];
    let mut rx_buf = [0; 2];
    
    let mut i: u16 = 0;
    let mut dir = 0;
    for _ in 0..2 {
        tx_buf = [(0x04 | (i >> 8) & 0xFF) as u8 , (i & 0xFF) as u8]; // write command
        if dir == 0 {
            i = i + 1;
            if i == 1024 {
                dir = 1;
            }
        } else {
            i = i - 1;
            if i == 0 {
                dir = 0;
            }
        }
    
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            spi.transfer(&mut transfer).unwrap();
        }
        tx_buf = [0x08, 0x00]; // read command
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            spi.transfer(&mut transfer).unwrap();
        }
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            spi.transfer(&mut transfer).unwrap();
        }
        println!("{:?}", &rx_buf);
        delay_ms(4);

    //    println!("{:?}", spi.transfer(&mut transfer));
    }
}
