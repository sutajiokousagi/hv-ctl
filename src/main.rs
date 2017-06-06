extern crate cupi;
extern crate cupi_shift;
extern crate spidev;
extern crate websocket;
extern crate clap;

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
use cupi::board;

use std::str::from_utf8;
use websocket::OwnedMessage;
use websocket::Message;
use websocket::sync::Server;
use std::sync::mpsc::channel;

use std::time::Instant;
use std::time::Duration;
use std::{thread,time};

use clap::{Arg, App, SubCommand};

use std::process;

use std::process::{Command, Stdio};
use std::io::{Read, Write};

const HV_FULL_SCALE: f64 = 1150.0;  // set to max V of supply installed: is 1000.0 for production
//const HV_GAIN: f64 = ((HV_FULL_SCALE * 1.2) / 12.0); // 20% fudge factor due to low loading

// use HV_GAIN instead of HV_FULL_SCALE / 12.0V, because the supply offsets are substantial
const HV_GAIN: f64 = 100.0; // empirically derived from measurements, gain factor of DAC->HV output

// loop limits and numbers
const HV_MIN_V: u16 = 100; // minimum voltage, after converter loading effects
const HV_MIN_SERVO_V: u16 = 100; // minimum servo voltage -- commanded voltage
const HV_MAX_SERVO_V: f64 = 1150.0; // maximum servo voltage -- commanded voltage
const HV_CONVERGENCE: f64 = 0.2;  // convergence rate for HV offset
const HV_CONVERGENCE_HI_C: f64 = 0.05;  // convergence rate for HV offset
const HV_TOLERANCE: f64 = 5.0; // stop converging when we're within 5V
const HV_PANIC: f64 = 1150.0; // panic voltage -- shut down the system, we're near breakdown (1.2kV)
const HV_FIXED_RES: f64 = 6800.0;

const HV_MIN_USER_V: u32 = 150;  // maximum user-requested voltages
const HV_MAX_USER_V: u32 = 900;

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
    let mut PID_KP: f64 = 4.0;
    let mut PID_KI: f64 = 5.5; 
    let mut PID_KD: f64 = 0.02;
    const PID_Ts: f64 = 0.01;
    const PID_N: f64 = 1.0 / PID_Ts;
    let mut runtime: u32 = 2000; // if you can't zap in 20 seconds, something is wrong??
    let mut max_deltav: u32 = 350;
    let mut convergence_thresh: f64 = 0.01; // fraction of full scale deviation to be considered converged
    let mut convergence_dwell: f64 = 500.0;   // time duration for convergence measurement, in ms

    let board = board();
    println!("{:?}", board);
    
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

    // sudo ./hv-ctl -x 4 -y 3 -v 250 -r 100 -c 25 --outfile=log.txt
    let matches = App::new("hv-ctl")
        .version("0.1")
        .author("bunnie <bunnie@kosagi.com>")
        .about("Control program for the high-throughput electroporation array")
        .arg(Arg::with_name("websocket")
             .short("w")
             .long("websocket")
             .help("Run as websocket server")
        )
        .arg(Arg::with_name("row")
             .short("y")
             .long("row")
             .help("Select row to activate")
             .takes_value(true)
             .required(true)
             .conflicts_with("websocket"))
        .arg(Arg::with_name("col")
             .short("x")
             .long("col")
             .help("Select column to activate")
             .takes_value(true)
             .required(true)
             .conflicts_with("websocket"))
        .arg(Arg::with_name("volts")
             .short("v")
             .long("volts")
             .help("Voltage")
             .takes_value(true)
             .required(true)
             .conflicts_with("websocket"))
        .arg(Arg::with_name("resistor values")
             .short("r")
             .long("res")
             .multiple(true)
             .help("Select resistor values to engage in parallel: 300, 620, 750, 1000 ohms; if none selected, then infinite")
             .takes_value(true)
             .conflicts_with("websocket"))
        .arg(Arg::with_name("capacitor values")
             .short("c")
             .long("cap")
             .multiple(true)
             .help("Select capacitor values to engage in parallel: 10 or 25 uF; if none selected, then none")
             .takes_value(true)
             .required(true)
             .conflicts_with("websocket"))
        .arg(Arg::with_name("KP")
             .long("kp")
             .help("Override default KP loop control parameter")
             .takes_value(true)
             .conflicts_with("websocket"))
        .arg(Arg::with_name("KI")
             .long("ki")
             .help("Override default KI loop control parameter")
             .takes_value(true)
             .conflicts_with("websocket"))
        .arg(Arg::with_name("KD")
             .long("kd")
             .help("Override default KD loop control parameter")
             .takes_value(true)
             .conflicts_with("websocket"))
        .arg(Arg::with_name("runtime")
             .long("runtime")
             .help("Set runtime for testing")
             .takes_value(true)
             .conflicts_with("websocket"))
        .arg(Arg::with_name("maxdelta")
             .long("maxdelta")
             .help("Set maximum servo delta V over output voltage")
             .takes_value(true)
             .conflicts_with("websocket"))
        .arg(Arg::with_name("d")
             .short("d")
             .multiple(true)
             .help("Set debug verbosity level"))
        .get_matches();

    let mut debug_level = 0;
    match matches.occurrences_of("d") {
        0 => debug_level = 0,
        1 => debug_level = 1,
        2 | _ => debug_level = 2,
    }
    
    let do_websockets = matches.is_present("websocket");
    
    if !do_websockets {
        // setup gnuplot output
        let mut child = Command::new("gnuplot")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .expect("gnuplot command failed to start");
        let mut child_stdin = child.stdin.take().expect("No stdin found on child");
        let mut child_stdout = child.stdout.take().expect("No stdout found on child");
        
        writeln!(child_stdin, "set terminal png size 800, 1200; set output 'test.png'; set datafile separator \",\"; plot '-' using 1:2 with lines title 'Control', '' using 1:2 with lines title 'Output'").expect("Unable to write to child");
        
        // state variables for row, col, control
        let mut hv_ctl_state: u8 = HvCtl::HvgenEna as u8;  // pure control state
    
        let mut hv_res_state: u8 = 0;  // intended resistance state -- only engage when HV is disengaged
        let mut hv_row_sel_state: RowSel = RowSel::RowNone;  // row/col sel state -- only engage when HV is disengaged
        let mut hv_col_sel_state: ColSel = ColSel::ColNone;

        if matches.is_present("KP") {
            PID_KP = matches.value_of("KP").unwrap().parse::<f64>().expect("KP should be a number") as f64;
            println!("Expert mode KP override = {}", PID_KP);
        }
        if matches.is_present("KI") {
            PID_KI = matches.value_of("KI").unwrap().parse::<f64>().expect("KI should be a number") as f64;
            println!("Expert mode KI override = {}", PID_KI);
        }
        if matches.is_present("KD") {
            PID_KD = matches.value_of("KD").unwrap().parse::<f64>().expect("KD should be a number") as f64;
            println!("Expert mode KD override = {}", PID_KD);
        }
        if matches.is_present("runtime") {
            runtime = matches.value_of("runtime").unwrap().parse::<u32>().expect("runtime should be an integer") as u32;
            println!("Expert mode runtime override = {}", runtime);
        }
        
        let y_coord: u8 = matches.value_of("row").unwrap().parse::<u32>().expect("Row should be an integer 1-12") as u8;
        let x_coord: u8 = matches.value_of("col").unwrap().parse::<u32>().expect("Column should be an integer 1-4") as u8;
        let voltage: u32 = matches.value_of("volts").unwrap().parse::<u32>().expect("Voltage should be an integer 120-1000");

        if voltage < HV_MIN_USER_V {
            panic!("Voltage should be bigger than {}", HV_MIN_USER_V);
        }
        if voltage > HV_MAX_USER_V {
            panic!("Voltage should be less than {}", HV_MAX_USER_V);
        }

        let mut res_infinite = true;
        if matches.is_present("resistor values") {
            res_infinite = false;
            if let Some(in_v) = matches.values_of("resistor values") {
                for resval in in_v {
                    if debug_level > 0 {
                        println!("resistor value {} chosen", resval);
                    }
                    match resval {
                        "300" => hv_res_state |= HvCtl::Sel300Ohm as u8,
                        "620" => hv_res_state |= HvCtl::Sel620Ohm as u8,
                        "750" => hv_res_state |= HvCtl::Sel750Ohm as u8,
                        "1000" => hv_res_state |= HvCtl::Sel1000Ohm as u8,
                        _ => panic!("Resistor value of {} is not valid: can only be 300, 620, 750, or 1000", resval),
                    }
                }
            }
        }
        
        let mut cap_none = true;
        if matches.is_present("capacitor values") {
            cap_none = false;
            if let Some(in_v) = matches.values_of("capacitor values") {
                for capval in in_v {
                    if debug_level > 0 {
                        println!("capacitor value {} chosen", capval);
                    }
                    match capval {
                        "10" => hv_ctl_state |= HvCtl::SelLocap as u8,
                        "25" => hv_ctl_state |= HvCtl::SelHicap as u8,
                        _ => panic!("Capacitor value of {} is not valid: can only be 10 or 25", capval),
                    }
                }
            }
        }

        if (hv_ctl_state & HvCtl::SelLocap as u8 != 0) && (hv_ctl_state & HvCtl::SelHicap as u8 != 0) {
            max_deltav = 100;
        }
        if (hv_ctl_state & HvCtl::SelLocap as u8 != 0) && (hv_ctl_state & HvCtl::SelHicap as u8 == 0) {
            max_deltav = 350;
        }
        if (hv_ctl_state & HvCtl::SelLocap as u8 == 0) && (hv_ctl_state & HvCtl::SelHicap as u8 != 0) {
            max_deltav = 200;
        }

        if matches.is_present("maxdelta") {
            max_deltav = matches.value_of("maxdelta").unwrap().parse::<u32>().expect("maxdelta should be an integer") as u32;
            println!("Expert mode runtime override = {}", max_deltav);
        } else {
            println!( "Default maxdeltav {}", max_deltav );
        }
        
        match y_coord {
            1 => hv_row_sel_state = RowSel::RowSel1,
            2 => hv_row_sel_state = RowSel::RowSel2,
            3 => hv_row_sel_state = RowSel::RowNone,
            4 => hv_row_sel_state = RowSel::RowNone,
            5 => hv_row_sel_state = RowSel::RowNone,
            6 => hv_row_sel_state = RowSel::RowNone,
            7 => hv_row_sel_state = RowSel::RowNone,
            8 => hv_row_sel_state = RowSel::RowNone,
            10 => hv_row_sel_state = RowSel::RowNone,
            11 => hv_row_sel_state = RowSel::RowNone,
            12 => hv_row_sel_state = RowSel::RowNone,
            _ => panic!("Row value of {} is invalid. It should be an integer 1-12", y_coord),
        }

        match x_coord {
            1 => hv_col_sel_state = ColSel::ColSel1,
            2 => hv_col_sel_state = ColSel::ColSel2,
            3 => hv_col_sel_state = ColSel::ColNone,
            4 => hv_col_sel_state = ColSel::ColNone,
            _ => panic!("Column value of {} is invalid. It should be an integer 1-4", x_coord),
        }

        if voltage < 120 || voltage > 1000 {
            panic!("Voltage value of {} is invalid. It should be an integer 120-1000", voltage);
        }
        if debug_level > 0 {
            println!("Using row value: {}", y_coord);
            println!("Using column value: {}", x_coord);
            println!("Using voltage value: {}", voltage);
        }

        // discharge all caps before beginning
        hvcfg.update_ctl(HvCtl::Sel300Ohm as u8 | HvCtl::SelHicap as u8 | HvCtl::SelLocap as u8, HvLockout::HvGenOff);
        thread::sleep(time::Duration::from_millis(100));
        hvcfg.update_ctl(0, HvLockout::HvGenOff);

        // begin capacitor charging, starting at HV_MIN_V per initialization routine
        hv_ctl_state |= HvCtl::HvEngage as u8;
        hvcfg.update_ctl(hv_ctl_state, HvLockout::HvGenOn);

        // loop coefficients
        let a0 = (1.0 + PID_N * PID_Ts);
        let a1 = -(2.0 + PID_N * PID_Ts);
        let a2 = 1.0;
        let b0 = PID_KP * (1.0 + PID_N * PID_Ts) + PID_KI * PID_Ts * (1.0 + PID_N * PID_Ts) + PID_KD * PID_N;
        let b1 = -(PID_KP * (2.0 + PID_N * PID_Ts) + PID_KI * PID_Ts + 2.0 * PID_KD * PID_N);
        let b2 = PID_KP + PID_KD * PID_N;
        let ku1 = a1 / a0;
        let ku2 = a2 / a0;
        let ke0 = b0 / a0;
        let ke1 = b1 / a0;
        let ke2 = b2 / a0;
        
        let mut loop_done = false;
        let mut u0 = 0.0;
        let mut u1 = 0.0;
        let mut u2 = 0.0;
        let mut e0 = 0.0;
        let mut e1 = 0.0;
        let mut e2 = 0.0;
        let mut actual_v: f64 = adc.read_hv();
        let mut loop_iter = 0;
        let mut s: String = "".to_string();
        
        let mut now = Instant::now();
        let mut conv_begin: f64 = (now.elapsed().as_secs() as f64 * 1_000_000_000.0 +
                                        (now.elapsed().subsec_nanos() as f64)) / 1_000_000.0;
        let mut in_conv = false;
        let mut converged = false;
        while !loop_done {
            e2 = e1;
            e1 = e0;
            u2 = u1;
            u1 = u0;
            
            // now start the charging loop
            actual_v = adc.read_hv();
            if actual_v > HV_PANIC {
                println!("Panic! HV exceeds max limit, discharging caps and exiting");
                targetv = HV_MIN_V as f64;
                hvset.set_hv_target(targetv as u16);
                discharge_and_resume(&mut hvcfg, hv_ctl_state, false);
                hvcfg.update_ctl(0, HvLockout::HvGenOff); // leave everything off before exit
                process::exit(0);
            }

            e0 = voltage as f64 - actual_v;
            u0 = -ku1 * u1 - ku2 * u2 + ke0 * e0 + ke1 * e1 + ke2 * e2;

            if u0 > HV_MAX_SERVO_V as f64 {
                u0 = HV_MAX_SERVO_V as f64;
            }
            if u0 > actual_v + max_deltav as f64 {
                u0 = actual_v + max_deltav as f64;
            }
            if u0 < HV_MIN_SERVO_V as f64 {
                u0 = HV_MIN_SERVO_V as f64;
            }
//            println!("Set point: {}", u0 as u16);
            if loop_iter >= 3 { // let the loop run 3 steps to initialize the loop filters to rational values
                hvset.set_hv_target(u0 as u16);
            }
            writeln!(child_stdin, "{}, {}", loop_iter, u0).expect("Unable to write to child");
            s.push_str( &*loop_iter.to_string() );
            s.push_str( ", " );
            s.push_str( &*actual_v.to_string() );
            s.push_str( "\n" );
            loop_iter = loop_iter + 1;
            
            let mut sample_time: f64 = (now.elapsed().as_secs() as f64 * 1_000_000_000.0 +
                                        (now.elapsed().subsec_nanos() as f64)) / 1_000_000.0;
            while sample_time < loop_iter as f64 * 10.0 {
                //thread::sleep(time::Duration::from_millis(10));
                sample_time = (now.elapsed().as_secs() as f64 * 1_000_000_000.0 +
                               (now.elapsed().subsec_nanos() as f64)) / 1_000_000.0;
            }

            if ((voltage as f64) > (actual_v * (1.0 - convergence_thresh))) &&
                ((voltage as f64) < (actual_v * (1.0 + convergence_thresh))) {
                if !in_conv {
                    in_conv = true;
                    conv_begin = sample_time;
                }

                if (sample_time - conv_begin) > convergence_dwell {
                    converged = true;
                }
            } else {
                in_conv = false;
            }
            
            if converged {
                loop_done = true;
            }
            if loop_iter > runtime  { // cutoff convergence after runtime is exceeded
                loop_done = true;
            }
        }
        
        writeln!(child_stdin, "e").expect("Unable to write to child");
        s.push_str( "e\n" );
        writeln!(child_stdin, "{}", s).expect("Unable to write to child"); // overlay the actual voltage over control voltage
        // println!("{}",s);

        // now trigger the EP event
        if converged {
            // connect row/col before engaging HV
            hvcfg.update_colsel(hv_col_sel_state);
            hvcfg.update_rowsel(hv_row_sel_state);
                            
            // trigger the HV system
            hvcfg.update_ctl( hv_ctl_state & !(HvCtl::HvEngage as u8), HvLockout::HvGenOff ); // generator off, caps are charged!
            // wait 10ms for relay to open before engaging the resistors
            thread::sleep(time::Duration::from_millis(10));
            hvcfg.update_ctl( (hv_ctl_state | hv_res_state) & !(HvCtl::HvEngage as u8), HvLockout::HvGenOff );
                            
            // start the ADC sampling here
            // send the trace data
            now = Instant::now();
            let mut s: String = "".to_string();
            for _ in 0 .. 5000 {
                // should give elapsed millis
                let sample_time: f64 = (now.elapsed().as_secs() as f64 * 1_000_000_000.0 +
                                        (now.elapsed().subsec_nanos() as f64)) / 1_000_000.0;
                s.push_str( &sample_time.to_string().as_str() );
                s.push_str( "," );
                
                let sample_value: f64 = adc.read_hv();
                s.push_str( &sample_value.to_string().as_str() );
                s.push_str( "\n" );
            }
            
            // disengage the row/col
            hvcfg.update_rowsel(RowSel::RowNone);
            hvcfg.update_colsel(ColSel::ColNone);
            
            // disengage the resistors and relay
            hvcfg.update_ctl( hv_ctl_state & !(HvCtl::HvEngage as u8), HvLockout::HvGenOff );
            thread::sleep(time::Duration::from_millis(10));

            // setup gnuplot output
            let mut child2 = Command::new("gnuplot")
            .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .spawn()
                .expect("gnuplot command failed to start");
            let mut child2_stdin = child2.stdin.take().expect("No stdin found on child");
            let mut child2_stdout = child2.stdout.take().expect("No stdout found on child");
        
            writeln!(child2_stdin, "set terminal png size 800, 1200; set output 'zap.png'; set datafile separator \",\"; plot '-' using 1:2 with lines title 'EP waveform'").expect("Unable to write to child");
            s.push_str( "e\n" );
            writeln!(child2_stdin, "{}", s).expect("Unable to write to child"); // overlay the actual voltage over control voltage
            
        } else {
            // discharge caps before exiting
            discharge_and_resume(&mut hvcfg, hv_ctl_state, false);
        }
        
        hvcfg.update_ctl(0, HvLockout::HvGenOff); // leave everything off before exit
        
        process::exit(0);
    } else {
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
    } // close of do_websockets
    
    panic!("hard exitting.");
}
