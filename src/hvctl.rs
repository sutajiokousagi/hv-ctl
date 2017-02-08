use std::fmt;
use cupi_shift::Shifter;
use cupi::CuPi;
use cupi::PinOutput;
use cupi::DigitalWrite;

const BRD_SRCLK: usize = 22;
const BRD_RCLK: usize = 21;
const BRD_SRDOUT: usize  = 24;
const BRD_SRCLR: usize = 23;  // clear when high

const BRD_LOCKOUT_N: usize = 29;

enum_from_primitive! {
    #[derive(Debug, PartialEq, Copy, Clone)]
    pub enum HvCtl {
        HvEngage =    0b1000_0000,
        HvgenEna =    0b0100_0000,
        Sel300Ohm =   0b0010_0000,
        Sel620Ohm =   0b0001_0000,
        Sel750Ohm =   0b0000_1000,
        Sel1000Ohm =  0b0000_0100,
        SelHicap =    0b0000_0010,
        SelLocap =    0b0000_0001,
    }
}

impl fmt::Display for HvCtl {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            HvCtl::HvEngage => write!(f, "HvEngage"),
            HvCtl::HvgenEna => write!(f, "HvgenEna"),
            HvCtl::Sel300Ohm => write!(f, "Sel300Ohm"),
            HvCtl::Sel620Ohm => write!(f, "Sel620Ohm"),
            HvCtl::Sel750Ohm => write!(f, "Sel750Ohm"),
            HvCtl::Sel1000Ohm => write!(f, "Sel1000Ohm"),
            HvCtl::SelHicap => write!(f, "SelHicap"),
            HvCtl::SelLocap => write!(f, "SelLocap"),
        }
    }
}

enum_from_primitive! {
    #[derive(Debug, PartialEq, Copy, Clone)]
    pub enum RowSel {
        RowNone = 0,
        RowSel1 = 0b1000_0000,
        RowSel2 = 0b0100_0000,
    }
}

impl fmt::Display for RowSel {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            RowSel::RowNone => write!(f, "No Row"),
            RowSel::RowSel1 => write!(f, "RowSel1"),
            RowSel::RowSel2 => write!(f, "RowSel2"),
        }
    }
}

enum_from_primitive! {
    #[derive(Debug, PartialEq, Copy, Clone)]
    pub enum ColSel {
        ColNone = 0,
        ColSel1 = 0b1000_0000,
        ColSel2 = 0b0100_0000,
    }
}

impl fmt::Display for ColSel {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            ColSel::ColNone => write!(f, "No Column"),
            ColSel::ColSel1 => write!(f, "ColSel1"),
            ColSel::ColSel2 => write!(f, "ColSel2"),
        }
    }
}

enum_from_primitive! {
    #[derive(Debug, PartialEq, Copy, Clone)]
    pub enum HvLockout {
        HvGenOn = 1,
        HvGenOff = 0,
    }
}

impl fmt::Display for HvLockout {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            HvLockout::HvGenOn => write!(f, "HV generator allowed to be on"),
            HvLockout::HvGenOff => write!(f, "HV locked out"),
        }
    }
}

#[derive(Debug)]
pub enum HvErr {
//    None,
}

pub struct HvConfig {
    initialized: bool,
    shifter: Shifter,
    colsel: usize,
    rowsel: usize,
    hvsel: usize,
    pin_lockout_n: PinOutput,
    pin_srclr: PinOutput,
}

impl HvConfig {
    pub fn new(cupi: &CuPi) -> Result<HvConfig,HvErr> {
        let (data_pin, latch_pin, clock_pin) = (BRD_SRDOUT, BRD_RCLK, BRD_SRCLK);
        let mut hvcfg = HvConfig {
            initialized: true,
            shifter: Shifter::new(data_pin, latch_pin, clock_pin),
            colsel: 0,
            rowsel: 0,
            hvsel: 0,
            pin_lockout_n: cupi.pin(BRD_LOCKOUT_N).unwrap().output(),
            pin_srclr: cupi.pin(BRD_SRCLR).unwrap().output(),
        };

        hvcfg.colsel = hvcfg.shifter.add(8);
        hvcfg.rowsel = hvcfg.shifter.add(8);
        hvcfg.hvsel = hvcfg.shifter.add(8);
        
        hvcfg.shifter.set(hvcfg.rowsel, 0, false);
        hvcfg.shifter.set(hvcfg.colsel, 0, false);
        hvcfg.shifter.set(hvcfg.hvsel, 0, true);

        hvcfg.pin_lockout_n.low().unwrap(); // force lockout to be active
        hvcfg.pin_srclr.low().unwrap(); // don't be clearing the shift registers

        Ok(hvcfg)
    }

    pub fn cleanup(&mut self) {
        if self.initialized {
            self.initialized = false;
        }
    }

    pub fn update_ctl(&mut self, val: u8, hvon: HvLockout) {
        self.shifter.set(self.hvsel, val as usize, true);
        if hvon == HvLockout::HvGenOn {
            self.pin_lockout_n.high().unwrap();
        } else {
            self.pin_lockout_n.low().unwrap();
        }
    }

    pub fn update_rowsel(&mut self, val: RowSel) {
        self.shifter.set(self.rowsel, val as usize, true);
    }

    pub fn update_colsel(&mut self, val: ColSel) {
        self.shifter.set(self.colsel, val as usize, true);
    }
}

impl Drop for HvConfig {
    fn drop(&mut self) {
        self.cleanup();
    }
}
