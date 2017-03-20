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
    
    let target: u16 = 50;
    let mut code: u16 = hvset.set_hv_target(target);
    let mut resistance: f64 = (code as f64) * (100_000.0 / 1024.0);
    let mut lv: f64 = 0.6 * ((resistance / 5100.0) + 1.0);
    let mut hv: f64 = lv * (HV_FULL_SCALE / 12.0); 
    println!("Target {}V. Code set to {}, resistance {}ohms, lv {}V, hv {}V", target, code, resistance, lv, hv );

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
                        for _ in 0 .. 20000 {
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
                        } else if text == "h" {
                            engage = false;
                            
                        } else if text == "C10" {
                            hv_ctl_state |= HvCtl::SelLocap as u8;
                        } else if text == "c10" {
                            hv_ctl_state &= !(HvCtl::SelLocap as u8);
                        } else if text == "C25" {
                            hv_ctl_state |= HvCtl::SelHicap as u8;
                        } else if text == "c25" {
                            hv_ctl_state &= !(HvCtl::SelHicap as u8);
                            
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
                        } else if text.starts_with("shv") {
                            let (_, arg) = text.split_at(3);
                            let targetv = match arg.parse::<u16>() {
                                Ok(v) => v,
                                Err(e) => {
                                    println!("HV set argument not an integer! {:?}", e);
                                    continue;
                                }
                            };
                            code = hvset.set_hv_target(targetv);
                            resistance = (code as f64) * (100_000.0 / 1024.0);
                            lv = 0.6 * ((resistance / 5100.0) + 1.0);
                            hv = lv * (HV_FULL_SCALE / 12.0);
                            println!("Target {}V requested. DAC code {}, resistance {}ohms, lv {}V, hv {}V", target, code, resistance, lv, hv );
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
