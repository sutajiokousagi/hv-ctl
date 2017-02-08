extern crate cupi;
extern crate cupi_shift;
extern crate spidev;

mod system;

mod slewrate;
use slewrate::*;

mod hvctl;
use hvctl::*;

mod spictl;
use spictl::*;

#[macro_use]
extern crate quick_error;

#[macro_use] extern crate enum_primitive;

use cupi::CuPi;
use cupi::delay_ms;

//const BRD_MOSI: usize = ;
//const BRD_MISO: usize = ;
//const BRD_CLK: usize = ;
//const BRD_ADC_CS_N: usize = ;
//const BRD_DAC_CS_N: usize = ;

#[test]
fn testlights() {
    let cupi = CuPi::new().unwrap();

    let mut hvcfg = HvConfig::new(cupi).unwrap();
    
    slewrate().unwrap();

    // four iterations; replace with "loop" if you want it to go forever
    for _ in 0..3 {
        println!("control on");
        hvcfg.update_ctl(0xFF, HvLockout::HvGenOff);
        delay_ms(500);

        println!("lockout toggle");
        hvcfg.update_ctl(0xFF, HvLockout::HvGenOn);
        delay_ms(500);
        hvcfg.update_ctl(0xFF, HvLockout::HvGenOff);
        delay_ms(500);
        
        println!("walk control");
        let mut shift_val = 1;
        for _ in 0..8 {
            hvcfg.update_ctl(shift_val, HvLockout::HvGenOff);
            shift_val <<= 1;
            delay_ms(500);
        }
        println!("control off");
        hvcfg.update_ctl(0, HvLockout::HvGenOff);
        delay_ms(500);

        println!("lockout toggle, voff");
        hvcfg.update_ctl(0x00, HvLockout::HvGenOn);
        delay_ms(500);
        hvcfg.update_ctl(0x00, HvLockout::HvGenOff);
        delay_ms(500);

        println!("row walk");
        hvcfg.update_rowsel(RowSel::RowSel1);
        delay_ms(500);
        hvcfg.update_rowsel(RowSel::RowSel2);
        delay_ms(500);
        hvcfg.update_rowsel(RowSel::RowNone);
        delay_ms(500);

        println!("col walk");
        hvcfg.update_colsel(ColSel::ColSel1);
        delay_ms(500);
        hvcfg.update_colsel(ColSel::ColSel2);
        delay_ms(500);
        hvcfg.update_colsel(ColSel::ColNone);
        delay_ms(500);

        println!("specific control");
        hvcfg.update_ctl(HvCtl::HvEngage as u8 |
                         HvCtl::HvgenEna as u8 |
                         HvCtl::SelLocap as u8 |
                         HvCtl::Sel1000Ohm as u8, HvLockout::HvGenOn);
        delay_ms(1000);
        
        println!("control off");
        hvcfg.update_ctl(0, HvLockout::HvGenOff);
    }
}

fn main() {
    let cupi = CuPi::new().unwrap();

    let mut hvcfg = HvConfig::new(&cupi).unwrap();
    slewrate().unwrap();

    // set control state to off/safe
    println!("Set control state to safe");
    hvcfg.update_ctl(0, HvLockout::HvGenOff);
    hvcfg.update_rowsel(RowSel::RowNone);
    hvcfg.update_colsel(ColSel::ColNone);

    delay_ms(100);

    let mut hvset = HvSet::new(&cupi).unwrap();
    println!("Code set to {}", hvset.set_code(16));
    
}
