extern crate cupi;
extern crate cupi_shift;

mod system;

mod slewrate;
use slewrate::*;

mod hvctl;
use hvctl::*;

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

fn main() {
    let cupi = CuPi::new().unwrap();

    let mut hvcfg = HvConfig::new(cupi).unwrap();
    
    slewrate().unwrap();

    // light validation loop
    loop {
        println!("control on");
        hvcfg.update_ctl(0xFF, HvEngage::HvGenOff);
        delay_ms(500);

        println!("lockout toggle");
        hvcfg.update_ctl(0xFF, HvEngage::HvGenOn);
        delay_ms(500);
        hvcfg.update_ctl(0xFF, HvEngage::HvGenOff);
        delay_ms(500);
        
        println!("walk control");
        let mut shift_val = 1;
        for _ in 0..8 {
            hvcfg.update_ctl(shift_val, HvEngage::HvGenOff);
            shift_val <<= 1;
            delay_ms(500);
        }
        println!("control off");
        hvcfg.update_ctl(0, HvEngage::HvGenOff);
        delay_ms(500);

        println!("lockout toggle, voff");
        hvcfg.update_ctl(0x00, HvEngage::HvGenOn);
        delay_ms(500);
        hvcfg.update_ctl(0x00, HvEngage::HvGenOff);
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
        
    }
}
