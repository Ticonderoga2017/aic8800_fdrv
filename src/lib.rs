#![no_std]

extern crate alloc;

// 模块声明
pub mod consts;
pub mod core;
pub mod crypto;
pub mod net;
pub mod protocol;
pub mod thread;
pub mod wifi;

// 重新导出公共API
pub use consts::*;
pub use core::{BusState, WifiBus, init};
pub use net::WifiNetDevice;
pub use wifi::api::{
    ConnectionStatus, WifiAuthType, WifiClient, WifiConfig, WifiEncryption, WifiError, WifiNetwork,
};
pub use wifi::{connect, disconnect, get_status, scan};
