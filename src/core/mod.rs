//! 核心功能模块
//!
//! 包含 FDRV 初始化和总线抽象

mod bus;
mod init;

pub use bus::{BusState, WifiBus, sdio1_irq_handler, set_global_bus};
pub use init::*;
