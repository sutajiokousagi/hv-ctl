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

use std::process;

#[macro_use] extern crate text_io;

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
    
    let mut target: u16 = 50;
    let mut code: u16 = hvset.set_hv_target(target);
    let mut resistance: f64 = (code as f64) * (100_000.0 / 1024.0);
    let mut lv: f64 = 0.6 * ((resistance / 5100.0) + 1.0);
    let mut hv: f64 = lv * (200.0 / 12.0); // 200.0 for initial testing, 1000,0 for production based on HV supply
    println!("Target {}V. Code set to {}, resistance {}ohms, lv {}V, hv {}V", target, code, resistance, lv, hv );

    let mut adc = AdcRead::new().unwrap();
    
    let mut engage = false;
    loop {
        target = read!("{}");
        if target == 0 {
            hvcfg.update_ctl(0, HvLockout::HvGenOff);
            engage = false;
        } else if target == 1 {
            engage = true;
            hvcfg.update_ctl(HvCtl::HvEngage as u8 |
                             HvCtl::HvgenEna as u8 |
                             HvCtl::SelLocap as u8 |
                             HvCtl::Sel1000Ohm as u8, HvLockout::HvGenOn);
        } else if target == 2 {
            println!("ADC read: {}", adc.read());
        } else if target == 5 {
            process::exit(0);
        } else {
            if engage {
                hvcfg.update_ctl(HvCtl::HvEngage as u8 |
                                 HvCtl::HvgenEna as u8 |
                                 HvCtl::SelLocap as u8 |
                                 HvCtl::Sel1000Ohm as u8, HvLockout::HvGenOn);
            } else {
                hvcfg.update_ctl(HvCtl::HvgenEna as u8 |
                                 HvCtl::SelLocap as u8 |
                                 HvCtl::Sel1000Ohm as u8, HvLockout::HvGenOn);
            }
            code = hvset.set_hv_target(target);
            resistance = (code as f64) * (100_000.0 / 1024.0);
            lv = 0.6 * ((resistance / 5100.0) + 1.0);
            hv = lv * (200.0 / 12.0); // 200.0 for initial testing, 1000,0 for production based on HV supply
            println!("Target {}V. Code set to {}, resistance {}ohms, lv {}V, hv {}V", target, code, resistance, lv, hv );
        }
    }
    
    //    panic!("hard exitting.");
}
