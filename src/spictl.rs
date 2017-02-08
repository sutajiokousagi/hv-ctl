use std::io;
use spidev::{Spidev, SpidevOptions, SpidevTransfer, SPI_MODE_1};
use cupi::{CuPi, delay_ms, DigitalWrite};

const DAC_RST_N: usize = 25;

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

pub fn test_spi(cupi: &CuPi) {
    
    let mut dac_rst_n = cupi.pin(DAC_RST_N).unwrap().output();
    dac_rst_n.low().unwrap(); // bring DAC into reset
    delay_ms(1);
    dac_rst_n.high().unwrap(); // bring DAC out of reset
    
    let spi = create_spi().unwrap();

    delay_ms(2);
    let mut tx_buf = [0x18, 0x02];  // unlock RDAC wiper
    let mut rx_buf = [0; 2];
    {
        let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
        spi.transfer(&mut transfer).unwrap();
    }

    tx_buf = [0x20, 0x00];  // power on command
    {
        let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
        spi.transfer(&mut transfer).unwrap();
    }
    
    let mut i: u16 = 0;
    let mut dir = 0;
    loop {
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
    }

//    println!("{:?}", spi.transfer(&mut transfer));
}
