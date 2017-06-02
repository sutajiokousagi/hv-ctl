extern crate cupi;
extern crate cupi_shift;
extern crate spidev;
extern crate websocket;

// temporary crates
extern crate rand;

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

use std::str::from_utf8;
use websocket::OwnedMessage;
use websocket::Message;
use websocket::sync::Server;
use std::sync::mpsc::channel;

use std::time::Instant;
use std::time::Duration;
use std::{thread,time};

const HV_FULL_SCALE: f64 = 1000.0;  // set to max V of supply installed: is 1000.0 for production
//const HV_GAIN: f64 = ((HV_FULL_SCALE * 1.2) / 12.0); // 20% fudge factor due to low loading

// use HV_GAIN instead of HV_FULL_SCALE / 12.0V, because the supply offsets are substantial
const HV_GAIN: f64 = 119.0; // empirically derived from measurements, gain factor of DAC->HV output

const HV_MIN_V: u16 = 120; // minimum voltage, after converter loading effects
const HV_MIN_SERVO_V: u16 = 100; // minimum servo voltage -- commanded voltage
const HV_MAX_SERVO_V: f64 = 1200.0; // maximum servo voltage -- commanded voltage
const HV_CONVERGENCE: f64 = 0.2;  // convergence rate for HV offset
const HV_CONVERGENCE_HI_C: f64 = 0.05;  // convergence rate for HV offset
const HV_TOLERANCE: f64 = 5.0; // stop converging when we're within 5V
const HV_PANIC: f64 = 1150.0; // panic voltage -- shut down the system, we're near breakdown (1.2kV)
const HV_FIXED_RES: f64 = 6800.0;

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

fn discharge_and_resume(hvcfg: &mut HvConfig, hv_ctl_state: u8, engage: bool) {
    let caplo_sel: bool = (hv_ctl_state & HvCtl::SelLocap as u8) != 0;
    let caphi_sel: bool = (hv_ctl_state & HvCtl::SelHicap as u8) != 0;

    // don't disturb the lockout status, just disengage the relay
    let mut hvon: HvLockout = HvLockout::HvGenOff;
    if engage {
        hvon = HvLockout::HvGenOn;
    }

    let mut hv_ctl = hv_ctl_state & !(HvCtl::HvEngage as u8); // disengage the HV relay
    
    // disengage relay and give it 10ms to break before engaging the discharge resistor
    hvcfg.update_ctl(hv_ctl, hvon);
    thread::sleep(time::Duration::from_millis(10));

    // discharge the currently selected
    hvcfg.update_ctl(hv_ctl | HvCtl::Sel300Ohm as u8, hvon);
    thread::sleep(time::Duration::from_millis(200));
    hvcfg.update_ctl(hv_ctl, hvon);
    thread::sleep(time::Duration::from_millis(10));

    // deselect all caps (break before make to prevent charge sharing)
    let mut hv_ctl_nocaps = hv_ctl & !(HvCtl::SelLocap as u8) & !(HvCtl::SelHicap as u8);
    hvcfg.update_ctl(hv_ctl_nocaps, hvon);
    thread::sleep(time::Duration::from_millis(1));

    // invert the capacitor selections
    if !caplo_sel {
        hv_ctl_nocaps |= HvCtl::SelLocap as u8;
    }

    if !caphi_sel {
        hv_ctl_nocaps |= HvCtl::SelHicap as u8;
    }
    
    // discharge the unselected caps
    hvcfg.update_ctl(hv_ctl_nocaps | HvCtl::Sel300Ohm as u8, hvon);
    thread::sleep(time::Duration::from_millis(200));
    
    // deselect discharge resistor and wait 10ms for deselect to happen
    hvcfg.update_ctl(hv_ctl_nocaps, hvon);
    thread::sleep(time::Duration::from_millis(10));
    
    // restore original state
    hvcfg.update_ctl(hv_ctl_state,  hvon);
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
    
    let mut targetv: f64 = HV_MIN_V as f64;
    let mut code: u16 = hvset.set_hv_target(targetv as u16);
    let mut resistance: f64 = (code as f64) * (100_000.0 / 1024.0);
    let mut lv: f64 = 0.6 * ((resistance / HV_FIXED_RES) + 1.0);
//    let mut hv: f64 = lv * (HV_FULL_SCALE / 12.0); 
    let mut hv: f64 = lv * HV_GAIN; 
    println!("Target {}V. Code set to {}, resistance {}ohms, lv {}V, hv {}V", targetv, code, resistance, lv, hv );

    let mut adj_target: f64 = targetv;
    let mut converged: bool = false;

    let adc = AdcRead::new().unwrap();
    
    let mut engage = false;

    let server = Server::bind("0.0.0.0:8080").unwrap();

    // state variables for row, col, control
    let mut hv_ctl_state: u8 = HvCtl::HvgenEna as u8;  // pure control state
    
    let mut hv_res_state: u8 = 0;  // intended resistance state -- only engage when HV is disengaged
    let mut hv_row_sel_state: RowSel = RowSel::RowNone;  // row/col sel state -- only engage when HV is disengaged
    let mut hv_col_sel_state: ColSel = ColSel::ColNone;
    
    for request in server.filter_map(Result::ok) {
	if !request.protocols().contains(&"rust-websocket".to_string()) {
	    request.reject().unwrap();
            println!("Connection request does not specify the correct protocol, rejecting.");
	    continue;
	}
        
	let mut client = request.use_protocol("rust-websocket").accept().unwrap();
        
	let ip = client.peer_addr().unwrap();

        println!("Connection from {}", ip);
    
	let (mut receiver, mut sender) = client.split().unwrap();

        let (tx, rx) = channel();
	let tx_1 = tx.clone();
    
        for message in receiver.incoming_messages() {
	    let message = match message {
		Ok(m) => m,
		Err(e) => {
		    println!("Receive Loop: {:?}", e);
		    let _ = tx_1.send(OwnedMessage::Close(None));
                    //		    return;
                    continue;
		}
	    };
        
	    match message {
	        OwnedMessage::Close(_) => {
                    hvset.set_hv_target(50); // set voltage target to default of 50
                    // reset control state
                    hv_ctl_state = HvCtl::HvgenEna as u8; // set to original value
                    hv_res_state = 0;
                    hvcfg.update_ctl(hv_ctl_state, HvLockout::HvGenOff);
                    engage = false;
                    // reset row/col to none
                    hv_row_sel_state = RowSel::RowNone;
                    hv_col_sel_state = ColSel::ColNone;
                    hvcfg.update_colsel(hv_col_sel_state);
                    hvcfg.update_rowsel(hv_row_sel_state);
                    
		    let _ = tx_1.send(OwnedMessage::Close(None));
		    println!("Client {} disconnected", ip);
                    break;
	        },
	        OwnedMessage::Ping(data) => {
		    match tx_1.send(OwnedMessage::Pong(data)) {
			// Send a pong in response
			Ok(()) => (),
			Err(e) => {
			    println!("Receive Loop: {:?}", e);
                            //			    return;
                            continue;
			}
		    }
	        },
                OwnedMessage::Text(data) => {
                    //                    let text = from_utf8(&*message.payload).unwrap();
                    let text = data;
                    if text == "t" {
                        // don't allow trigger until voltage hits required value
                        if !converged {
                            println!("Aborting trigger, HV hasn't converged yet!");
                            continue;
                        }
                        let mut actual_v: f64 = adc.read_hv();
                        let mut delta_v: f64 = (targetv as f64) - actual_v;
                        let now = Instant::now();
                        // times out after 4 seconds
                        while (delta_v.abs() > HV_TOLERANCE) && (now.elapsed().as_secs() < 4) {
                            actual_v = adc.read_hv();
                            delta_v = (targetv as f64) - actual_v;
                        }
                        if now.elapsed().as_secs() >= 4 {
                            println!("WARNING: HV convergence aborted, but still performing zap.");
                        }
                        
                        // connect row/col before engaging HV
                        hvcfg.update_colsel(hv_col_sel_state);
                        hvcfg.update_rowsel(hv_row_sel_state);

                        // trigger the HV system
                        if engage  {
                            hvcfg.update_ctl( hv_ctl_state & !(HvCtl::HvEngage as u8), HvLockout::HvGenOn );
                            // wait 10ms for relay to open before engaging the resistors
                            thread::sleep(time::Duration::from_millis(10));
                            hvcfg.update_ctl( (hv_ctl_state | hv_res_state) & !(HvCtl::HvEngage as u8), HvLockout::HvGenOn );
                        } else {
                            // this is used for "dry runs", eg HV is locked out but we still go through
                            // the motions of triggering the cell
                            hvcfg.update_ctl( hv_ctl_state & !(HvCtl::HvEngage as u8), HvLockout::HvGenOff );
                            // wait 10ms for relay to open before engaging the resistors
                            thread::sleep(time::Duration::from_millis(10));
                            hvcfg.update_ctl( (hv_ctl_state | hv_res_state) & !(HvCtl::HvEngage as u8), HvLockout::HvGenOff );
                        }

                        // start the ADC sampling here
                        // send the trace data
                        let now = Instant::now();
                        let mut s: String = "".to_string();
                        for _ in 0 .. 3500 {
                            // should give elapsed millis
                            let sample_time: f64 = (now.elapsed().as_secs() as f64 * 1_000_000_000.0 +
                                                    (now.elapsed().subsec_nanos() as f64)) / 1_000_000.0;
                            s.push_str( &sample_time.to_string().as_str() );
                            s.push_str( "," );
                            
                            let sample_value: f64 = adc.read_hv();
                            s.push_str( &sample_value.to_string().as_str() );
                            s.push_str( "#" );
                        }
                        let msg: Message = Message::text( s );
                        sender.send_message(&msg).unwrap();
                        let msg: Message = Message::text( "u" );
                        sender.send_message(&msg).unwrap();
                        
                        // disengage the row/col
                        hvcfg.update_rowsel(RowSel::RowNone);
                        hvcfg.update_colsel(ColSel::ColNone);

                        // disengage the resistors
                        if engage  {
                            hvcfg.update_ctl( hv_ctl_state & !(HvCtl::HvEngage as u8), HvLockout::HvGenOn );
                            // wait 10ms for resistor deselection before potentially re-engaging relays
                            thread::sleep(time::Duration::from_millis(10));
                            hvcfg.update_ctl( hv_ctl_state, HvLockout::HvGenOn );
                        } else {
                            hvcfg.update_ctl( hv_ctl_state & !(HvCtl::HvEngage as u8), HvLockout::HvGenOff );
                            // wait 10ms for resistor deselection before potentially re-engaging relays
                            thread::sleep(time::Duration::from_millis(10));
                            hvcfg.update_ctl( hv_ctl_state, HvLockout::HvGenOff );
                        }
                    } else {
                        if text == "HVON" {
                            engage = true;
                            converged = false; // reconverge when turning on
                        } else if text == "h" {
                            engage = false;
                            
                        } else if text == "C10" {
                            discharge_and_resume(&mut hvcfg, hv_ctl_state, engage);
                            hv_ctl_state |= HvCtl::SelLocap as u8;
                            converged = false;
                        } else if text == "c10" {
                            discharge_and_resume(&mut hvcfg, hv_ctl_state, engage);
                            hv_ctl_state &= !(HvCtl::SelLocap as u8);
                            converged = false;
                        } else if text == "C25" {
                            discharge_and_resume(&mut hvcfg, hv_ctl_state, engage);
                            hv_ctl_state |= HvCtl::SelHicap as u8;
                            converged = false;
                        } else if text == "c25" {
                            discharge_and_resume(&mut hvcfg, hv_ctl_state, engage);
                            hv_ctl_state &= !(HvCtl::SelHicap as u8);
                            converged = false;
                            
                        } else if text == "R1000" {
                            hv_res_state |= HvCtl::Sel1000Ohm as u8;
                        } else if text == "r1000" {
                            hv_res_state &= !(HvCtl::Sel1000Ohm as u8);
                        } else if text == "R750" {
                            hv_res_state |= HvCtl::Sel750Ohm as u8;
                        } else if text == "r750" {
                            hv_res_state &= !(HvCtl::Sel750Ohm as u8);
                        } else if text == "R620" {
                            hv_res_state |= HvCtl::Sel620Ohm as u8;
                        } else if text == "r620" {
                            hv_res_state &= !(HvCtl::Sel620Ohm as u8);
                        } else if text == "R300" {
                            hv_res_state |= HvCtl::Sel300Ohm as u8;
                        } else if text == "r300" {
                            hv_res_state &= !(HvCtl::Sel300Ohm as u8);

                            // row/col are "radio buttons"
                        } else if text == "col1" {
                            hv_col_sel_state = ColSel::ColSel1;
                        } else if text == "col2" {
                            hv_col_sel_state = ColSel::ColSel2;
                        } else if text == "colx" {
                            hv_col_sel_state = ColSel::ColNone;
                            
                        } else if text == "row1" {
                            hv_row_sel_state = RowSel::RowSel1;
                        } else if text == "row2" {
                            hv_row_sel_state = RowSel::RowSel2;
                        } else if text == "rowx" {
                            hv_row_sel_state = RowSel::RowNone;

                        } else if text == "hvup" {
                            let actual_v: f64 = adc.read_hv();
	                    let msg: Message = Message::text( "hvup,".to_string() + &actual_v.to_string() );
                            sender.send_message(&msg).unwrap();

                            if actual_v > HV_PANIC {
                                println!("Panic! HV exceeds max limit, locking out HV and setting target to min");
                                // note this causes a state mismatch with the UI...
                                engage = false;
                                hvcfg.update_ctl(hv_ctl_state, HvLockout::HvGenOff);
                                targetv = HV_MIN_V as f64;
                                hvset.set_hv_target(targetv as u16);
                            }

                            // only run feedback loop when the system is engaged
                            if ((hv_ctl_state & (HvCtl::HvEngage as u8)) != 0) && engage && !converged {
                                let delta: f64 = (targetv as f64) - actual_v;
                                if (delta.abs() > HV_TOLERANCE) && (adj_target >= (HV_MIN_SERVO_V as f64))
                                    && (adj_target < (HV_MAX_SERVO_V)) {
                                        let adjustment: f64;
                                        if (hv_ctl_state & HvCtl::SelHicap as u8) == 0 {
                                            adjustment = delta * HV_CONVERGENCE;
                                        } else {
                                            adjustment = delta * HV_CONVERGENCE_HI_C;
                                        }
                                        adj_target = adj_target + adjustment;
                                        code = hvset.set_hv_target(adj_target as u16);
                                        resistance = (code as f64) * (100_000.0 / 1024.0);
                                        lv = 0.6 * ((resistance / HV_FIXED_RES) + 1.0);
//                                        hv = lv * (HV_FULL_SCALE / 12.0);
                                        hv = lv * HV_GAIN;
                                        println!("Adjust to {}V. DAC code {}, resistance {}ohms, lv {}V, hv {}V", adj_target, code, resistance, lv, hv );
                                    }
                                if delta.abs() <= HV_TOLERANCE {
                                    converged = true;
                                }
                            }
                        } else if text.starts_with("shv") {
                            let (_, arg) = text.split_at(3);
                            targetv = match arg.parse::<f64>() {
                                Ok(v) => v,
                                Err(e) => {
                                    println!("HV set argument not an integer! {:?}", e);
                                    continue;
                                }
                            };
                            adj_target = targetv;
                            converged = false;
                            code = hvset.set_hv_target(targetv as u16);
                            resistance = (code as f64) * (100_000.0 / 1024.0);
                            lv = 0.6 * ((resistance / HV_FIXED_RES) + 1.0);
//                            hv = lv * (HV_FULL_SCALE / 12.0);
                            hv = lv * HV_GAIN;
                            println!("Target {}V requested. DAC code {}, resistance {}ohms, lv {}V, hv {}V", targetv, code, resistance, lv, hv );
                        } else if text.starts_with("setRC") {
                            let v: Vec<&str> = text.splitn(3, ',').collect();
                            match v[1].parse().unwrap() { // match row state
                                0 => hv_row_sel_state = RowSel::RowNone,
                                1 => hv_row_sel_state = RowSel::RowSel1,
                                2 => hv_row_sel_state = RowSel::RowSel2,
                                _ => hv_row_sel_state = RowSel::RowNone,
                            }
                            match v[2].parse().unwrap() {
                                0 => hv_col_sel_state = ColSel::ColNone,
                                1 => hv_col_sel_state = ColSel::ColSel1,
                                2 => hv_col_sel_state = ColSel::ColSel2,
                                _ => hv_col_sel_state = ColSel::ColNone,
                            }
                            // use this only for testing, normally row/col set just at trigger time
                            // hvcfg.update_colsel(hv_col_sel_state);  
                            // hvcfg.update_rowsel(hv_row_sel_state);
                        }
                        
                        if ((hv_ctl_state & HvCtl::SelHicap as u8) != 0) ||
                            ((hv_ctl_state & HvCtl::SelLocap as u8) != 0) {
                                // if either cap is selected, engage the relay for charging
                                hv_ctl_state |= HvCtl::HvEngage as u8;
                            } else {
                                // if no caps selected disengage the HV -- we should never
                                // be simply doing a "steady state" HV output in the electroporation
                                // model
                                hv_ctl_state &= !(HvCtl::HvEngage as u8);
                            }

                        if engage  {
                            hvcfg.update_ctl(hv_ctl_state, HvLockout::HvGenOn);
                        } else {
                            hvcfg.update_ctl(hv_ctl_state, HvLockout::HvGenOff);
                        }
                    }
                }
	        _ => sender.send_message(&message).unwrap(),
	    }
        }
    }
    
    panic!("hard exitting.");
}
