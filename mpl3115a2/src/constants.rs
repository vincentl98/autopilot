pub(crate) const I2C_ADDRESS: u16 = 0x60;
pub(crate) const CHIP_ID: u8 = 0xC4;

// Data Ready Status
pub(crate) const PTOW: u8 = 0x80;
pub(crate) const POW: u8 = 0x40;
//Pressure overwrite
pub(crate) const TOW: u8 = 0x20;
pub(crate) const PTDR: u8 = 0x08;
// Pressure/Altitude OR Temperature data ready
pub(crate) const PDR: u8 = 0x04;
// Pressure/Altitude new data available
pub(crate) const TDR: u8 = 0x02;    // Temperature new Data Available.

// Control Register 1
pub(crate) const ALT: u8 = 0x80;
pub(crate) const RAW: u8 = 0x40;
pub(crate) const OS2: u8 = 0x20;
pub(crate) const OS1: u8 = 0x10;
pub(crate) const OS0: u8 = 0x08;
pub(crate) const RST: u8 = 0x04;
pub(crate) const OST: u8 = 0x02;
pub(crate) const ACTIVE: u8 = 0x01;
//Active mode
pub(crate) const OS_1: u8 = 0x00;
//  6 ms min between samples
pub(crate) const OS_2: u8 = OS0;
//  10 ms
pub(crate) const OS_4: u8 = OS1;
//  18 ms
pub(crate) const OS_8: u8 = OS1 | OS0;
//  34 ms
pub(crate) const OS_16: u8 = OS2;
//  66 ms
pub(crate) const OS_32: u8 = OS2 | OS0;
// 130 ms
pub(crate) const OS_64: u8 = OS2 | OS1;
// 258 ms
pub(crate) const OS_128: u8 = OS2 | OS1 | OS0; // 512 ms

pub(crate) const BAR_MASK: u8 = 0x80;
pub(crate) const ALT_MASK: u8 = 0xEF;
pub(crate) const ACTIVE_MASK: u8 = 0xF1;
pub(crate) const STANDBY_MASK: u8 = 0xFE;

// Control Register 2
pub(crate) const ALARM_SEL: u8 = 0x20;
pub(crate) const LOAD_OUTPUT: u8 = 0x10;
pub(crate) const ST3: u8 = 0x08;
pub(crate) const ST2: u8 = 0x04;
pub(crate) const ST1: u8 = 0x02;
pub(crate) const ST0: u8 = 0x01;
pub(crate) const CLEAR_ST: u8 = 0xF0;

// Control Register 3
pub(crate) const IPOL1: u8 = 0x10;
//Set to 1 for Active HIGH on INT1
pub(crate) const PP_OD1: u8 = 0x08;
//Set to 0 for internal pull up
pub(crate) const IPOL2: u8 = 0x02;
//1 Active high, 0 Active Low Interrupt on Pad2
pub(crate) const PP_OD2: u8 = 0x01;       //Set to 0 for internal pull up

// Control Register 4
pub(crate) const INT_EN_DRDY: u8 = 0x80;
//Set to 1: Data Ready interrupt enabled
pub(crate) const INT_EN_FIFO: u8 = 0x40;
//Set to 1: FIFO interrupt enabled
pub(crate) const INT_EN_PW: u8 = 0x20;
//Set to 1: Pressure window interrupt enabled
pub(crate) const INT_EN_TW: u8 = 0x10;
//Set to 1: Temperature window interrupt enabled
pub(crate) const INT_EN_PTH: u8 = 0x08;
//Set to 1: Pressure Threshold interrupt enabled
pub(crate) const INT_EN_TTH: u8 = 0x04;
//Set to 1: Temperature Threshold interrupt enabled
pub(crate) const INT_EN_PCHG: u8 = 0x02;
//Set to 1: Pressure Change interrupt enabled.
pub(crate) const INT_EN_TCHG: u8 = 0x01;
//Set to 1: Temperature Change interrupt enabled
pub(crate) const INT_EN_CLEAR: u8 = 0x00;

// Control Register 5

pub(crate) const INT_CFG_DRDY: u8 = 0x80;
//1: Interrupt is routed to INT1
pub(crate) const INT_CFG_FIFO: u8 = 0x40;
pub(crate) const INT_CFG_PW: u8 = 0x20;
pub(crate) const INT_CFG_TW: u8 = 0x10;
pub(crate) const INT_CFG_PTH: u8 = 0x08;
pub(crate) const INT_CFG_TTH: u8 = 0x04;
pub(crate) const INT_CFG_PCHG: u8 = 0x02;
pub(crate) const INT_CFG_TCHG: u8 = 0x01;

pub(crate) const INT_CFG_CLEAR: u8 = 0x00;

pub(crate) const INT2: u8 = 0;
pub(crate) const INT1: u8 = 1;

pub(crate) const DISABLED: u8 = 0x00;
pub(crate) const CIRCULAR: u8 = 0x40;
pub(crate) const FULL_STOP: u8 = 0x80;
pub(crate) const F_MODE: u8 = DISABLED;

// PT_DATA_CFG - Sensor data event flag register
pub(crate) const DREM: u8 = 0x04;
// Data Ready Event Mode
pub(crate) const PDEFE: u8 = 0x02;
// Pressure Data Event Flag Enabled
pub(crate) const TDEFE: u8 = 0x01;
// Temperature Data Event Flag Enabled

// INT_SOURCE Register

pub(crate) const SRC_DRDY: u8 = 0x80;
pub(crate) const SRC_FIFO: u8 = 0x40;
pub(crate) const SRC_PW: u8 = 0x20;
pub(crate) const SRC_TW: u8 = 0x10;
pub(crate) const SRC_PTH: u8 = 0x08;
pub(crate) const SRC_TTH: u8 = 0x04;
pub(crate) const SRC_PCHG: u8 = 0x02;
pub(crate) const SRC_TCHG: u8 = 0x01;