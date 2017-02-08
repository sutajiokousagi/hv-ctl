use std::io;
use spidev::{Spidev, SpidevOptions, SpidevTransfer, SPI_MODE_1};
use cupi::{CuPi, delay_ms, DigitalWrite};
use cupi::PinOutput;

const DAC_RST_N: usize = 25;
const HV_FULL_SCALE: f64 = 200.0;  // is 1000.0 for production
const HV_GAIN: f64 = (HV_FULL_SCALE / 12.0);
const VREF: f64 = 0.6;

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

#[derive(Debug)]
pub enum HvSetErr {
//    None,
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
        
        let mut tx_buf = [(0x04 | (self.hvcode >> 8) & 0xFF) as u8 , (self.hvcode & 0xFF) as u8]; // write command
        let mut rx_buf = [0; 2];
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            self.spi.transfer(&mut transfer).unwrap();
        }
        
        tx_buf = [0x08, 0x00]; // read command
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            self.spi.transfer(&mut transfer).unwrap();
        }
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            self.spi.transfer(&mut transfer).unwrap();
        }
        // println!("{:?}", &rx_buf);
        let retval: u16 = ((rx_buf[0] & 0x3) as u16) << 8 | (rx_buf[1] as u16);

        return retval
    }

    pub fn set_hv_target(&mut self, voltage: u16) -> u16 {
        // Vout = gain * vref * (DAC-R / 5100 + 1)
        // DAC-R = code * (100_000 / 1024)
        // Vout = gain * vref * ((code * (100_000 / 1024)) / 5100 + 1)
        // Vout / (gain * vref) - 1 = (code * (100_000 / 1024)) / 5100
        // (Vout / (gain * vref) - 1) * 5100 = (code * (100_000 / 1024))
        // ((Vout / (gain * vref) - 1) * 5100) * (1024 / 100_000) = code
        let code: u16 = ((((voltage as f64) / (HV_GAIN * VREF) - 1.0) * 5100.0) * (1024.0 / 100_000.0)) as u16;
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
