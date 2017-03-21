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
use websocket::{Server, Message, Sender, Receiver};
use websocket::message::Type;
use websocket::header::WebSocketProtocol;

use std::time::Instant;

const HV_FULL_SCALE: f64 = 200.0;  // set to max V of supply installed: is 1000.0 for production
const HV_MIN_V: u16 = 60; // minimum voltage, after converter loading effects
const HV_MIN_SERVO_V: u16 = 30; // minimum servo voltage -- commanded voltage
const HV_CONVERGENCE: f64 = 0.5;  // convergence rate for HV offset
const HV_CONVERGENCE_HI_C: f64 = 0.15;  // convergence rate for HV offset
const HV_TOLERANCE: f64 = 4.0; // stop converging when we're within 4V
const HV_PANIC: f64 = 1150.0; // panic voltage -- shut down the system, we're near breakdown (1.2kV)

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
    
    let mut targetv: f64 = HV_MIN_V as f64;
    let mut code: u16 = hvset.set_hv_target(targetv as u16);
    let mut resistance: f64 = (code as f64) * (100_000.0 / 1024.0);
    let mut lv: f64 = 0.6 * ((resistance / 5100.0) + 1.0);
    let mut hv: f64 = lv * (HV_FULL_SCALE / 12.0); 
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
    
    for connection in server {
        let request = connection.unwrap().read_request().unwrap(); // Get the request
        let headers = request.headers.clone(); // Keep the headers so we can check them
            
        request.validate().unwrap(); // Validate the request
        
        let mut response = request.accept(); // Form a response
        
        if let Some(&WebSocketProtocol(ref protocols)) = headers.get() {
	    if protocols.contains(&("rust-websocket".to_string())) {
	        // We have a protocol we want to use
	        response.headers.set(WebSocketProtocol(vec!["rust-websocket".to_string()]));
	    }
        }
    
        let mut client = response.send().unwrap(); // Send the response
        
        let ip = client.get_mut_sender()
	    .get_mut()
	    .peer_addr()
	    .unwrap();
        
        println!("Connection from {}", ip);
    
        let (mut sender, mut receiver) = client.split();
    
        for message in receiver.incoming_messages() {
	    let message: Message = message.unwrap();
        
	    match message.opcode {
	        Type::Close => {
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
                    
		    let message = Message::close();
		    sender.send_message(&message).unwrap();
		    println!("Client {} disconnected", ip);
                    break;
	        },
	        Type::Ping => {
		    let message = Message::pong(message.payload);
		    sender.send_message(&message).unwrap();
	        },
                Type::Text => {
                    let text = from_utf8(&*message.payload).unwrap();
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
                            hvcfg.update_ctl( (hv_ctl_state | hv_res_state) & !(HvCtl::HvEngage as u8), HvLockout::HvGenOn );
                        } else {
                            // this is used for "dry runs", eg HV is locked out but we still go through
                            // the motions of triggering the cell
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
                            hvcfg.update_ctl( hv_ctl_state, HvLockout::HvGenOn );
                        } else {
                            hvcfg.update_ctl( hv_ctl_state, HvLockout::HvGenOff );
                        }
                    } else {
                        if text == "HVON" {
                            engage = true;
                            converged = false; // reconverge when turning on
                        } else if text == "h" {
                            engage = false;
                            
                        } else if text == "C10" {
                            hv_ctl_state |= HvCtl::SelLocap as u8;
                            converged = false;
                        } else if text == "c10" {
                            hv_ctl_state &= !(HvCtl::SelLocap as u8);
                            converged = false;
                        } else if text == "C25" {
                            hv_ctl_state |= HvCtl::SelHicap as u8;
                            converged = false;
                        } else if text == "c25" {
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
                                if (delta.abs() > HV_TOLERANCE) && (adj_target >= (HV_MIN_SERVO_V as f64)) {
                                    let adjustment: f64;
                                    if (hv_ctl_state & HvCtl::SelHicap as u8) == 0 {
                                        adjustment = delta * HV_CONVERGENCE;
                                    } else {
                                        adjustment = delta * HV_CONVERGENCE_HI_C;
                                    }
                                    adj_target = adj_target + adjustment;
                                    code = hvset.set_hv_target(adj_target as u16);
                                    resistance = (code as f64) * (100_000.0 / 1024.0);
                                    lv = 0.6 * ((resistance / 5100.0) + 1.0);
                                    hv = lv * (HV_FULL_SCALE / 12.0);
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
                            lv = 0.6 * ((resistance / 5100.0) + 1.0);
                            hv = lv * (HV_FULL_SCALE / 12.0);
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
