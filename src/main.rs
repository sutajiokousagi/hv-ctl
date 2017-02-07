extern crate cupi;
extern crate cupi_shift;

mod slewrate;
mod system;
use slewrate::*;

#[macro_use]
extern crate quick_error;

use cupi::{CuPi, delay_ms, DigitalWrite};
use cupi_shift::Shifter;

const BRD_LOCKOUT_N: usize = 29;
//const BRD_MOSI: usize = ;
//const BRD_MISO: usize = ;
//const BRD_CLK: usize = ;
//const BRD_ADC_CS_N: usize = ;
//const BRD_DAC_CS_N: usize = ;

const BRD_SRCLK: usize = 22;
const BRD_RCLK: usize = 21;
const BRD_SRDOUT: usize  = 24;
const BRD_SRCLR: usize = 23;  // clear when high

fn main() {
    let cupi = CuPi::new().unwrap();

    let mut pin_lockout_n = cupi.pin(BRD_LOCKOUT_N).unwrap().output();
    pin_lockout_n.low().unwrap(); // force lockout to be active

    let mut pin_srclr = cupi.pin(BRD_SRCLR).unwrap().output();
    pin_srclr.low().unwrap(); // don't be clearing the shift registers

    slewrate().unwrap();
    
    let (data_pin, latch_pin, clock_pin) = (BRD_SRDOUT, BRD_RCLK, BRD_SRCLK);

    let mut shifter = Shifter::new(data_pin, latch_pin, clock_pin);

    let colsel = shifter.add(8);
    let rowsel = shifter.add(8);
    let hvctl = shifter.add(8);
    
    shifter.set(rowsel, 0, true);
    shifter.set(colsel, 0, true);
    
    loop {
        println!("control on");
        shifter.set(hvctl, 0b11111111, true);
        delay_ms(500);

        println!("lockout toggle");
        pin_lockout_n.high().unwrap(); // undo lockout
        delay_ms(500);
        pin_lockout_n.low().unwrap(); // redo lockout
        delay_ms(500);
        
        println!("walk control");
        let mut shift_val = 1;
        for _ in 0..8 {
            shifter.set(hvctl, shift_val, true);
            shift_val <<= 1;
            delay_ms(500);
        }
        println!("control off");
        shifter.set(hvctl, 0, true);
        delay_ms(500);

        println!("lockout toggle");
        pin_lockout_n.high().unwrap(); // undo lockout
        delay_ms(500);
        pin_lockout_n.low().unwrap(); // redo lockout
        delay_ms(500);

        println!("row walk");
        shift_val = 0b10000000;
        for _ in 0..2 {
            shifter.set(rowsel, shift_val, true);
            shift_val >>= 1;
            delay_ms(500);
        }
        shifter.set(rowsel, 0, true);

        println!("col walk");
        shift_val = 0b10000000;
        for _ in 0..2 {
            shifter.set(colsel, shift_val, true);
            shift_val >>= 1;
            delay_ms(500);
        }
        shifter.set(colsel, 0, true);
        
    }
}
