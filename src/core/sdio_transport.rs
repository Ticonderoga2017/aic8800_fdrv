//! SDIO 传输层抽象
//!
//! 封装所有 SDIO 操作，屏蔽 Mutex 加锁和 trait 方法分派细节。
//! 上层代码只需调用 `transport.read_byte()` 等方法。

use alloc::sync::Arc;
use axsync::Mutex;
use axdriver_sdio::{SdioHost, SdioCardIrq};

use crate::consts::{BUFFER_SIZE, FLOW_CTRL_CMD_RETRY};
use aic8800_common::{
    SDIOWIFI_FLOW_CTRL_REG, SDIOWIFI_FLOWCTRL_MASK,
};

/// SDIO 传输层
///
/// 对 `Arc<Mutex<dyn SdioHost>>` 的封装，提供简洁的 SDIO 操作接口。
/// 所有方法内部处理加锁和 trait 方法分派。
pub struct SdioTransport {
    sdio: Arc<Mutex<dyn SdioHost>>,
    card_irq: Option<Arc<dyn SdioCardIrq>>
}

impl SdioTransport {
    /// 从任意 SdioHost 实现创建 SdioTransport
    pub fn new<H: SdioHost + 'static>(sdio: H) -> Arc<Self> {
        let card_irq = sdio.card_irq_ctrl();
        Arc::new(Self {
            sdio: Arc::new(Mutex::new(sdio)),
            card_irq,
        })
    }

    // ===== 基础 SDIO 操作 =====

    /// CMD52: 单字节读
    pub fn read_byte(&self, func: u8, addr: u32) -> Result<u8, axdriver_sdio::error::SdioError> {
        self.sdio.lock().read_byte(func, addr)
    }

    /// CMD52: 单字节写
    pub fn write_byte(&self, func: u8, addr: u32, val: u8) -> Result<(), axdriver_sdio::error::SdioError> {
        self.sdio.lock().write_byte(func, addr, val)
    }

    /// CMD53: FIFO 读
    pub fn read_fifo(&self, func: u8, addr: u32, buf: &mut [u8]) -> Result<(), axdriver_sdio::error::SdioError> {
        self.sdio.lock().read_fifo(func, addr, buf)
    }

    /// CMD53: FIFO 写
    pub fn write_fifo(&self, func: u8, addr: u32, buf: &[u8]) -> Result<(), axdriver_sdio::error::SdioError> {
        self.sdio.lock().write_fifo(func, addr, buf)
    }

    // ===== 中断控制 =====

    /// 屏蔽 SDIO 卡中断（CARD_INT）
    ///
    /// 在 SDIO 总线操作（CMD52/CMD53）期间调用，防止 CARD_INT
    /// 电平触发导致 ISR 重入。操作完成后调用 `unmask_card_irq()` 恢复。
    pub(crate) fn mask_card_irq(&self) {
        if let Some(ref ctrl) = self.card_irq {
            ctrl.mask_card_irq();
        }
    }

    /// 恢复 SDIO 卡中断（CARD_INT）
    pub(crate) fn unmask_card_irq(&self) {
        if let Some(ref ctrl) = self.card_irq {
            ctrl.unmask_card_irq();
        }
    }

    /// 使能 SDHCI 中断信号
    pub fn enable_irq(&self) {
        self.sdio.lock().enable_irq();
    }

    /// 禁用 SDHCI 中断信号
    pub fn disable_irq(&self) {
        self.sdio.lock().disable_irq();
    }

    // ===== 流控 =====

    /// 读取流控寄存器原始值
    pub fn read_flow_ctrl(&self) -> Result<u8, axdriver_sdio::error::SdioError> {
        self.sdio.lock().read_byte(1, SDIOWIFI_FLOW_CTRL_REG)
    }

    /// 读取流控值（已应用 MASK）
    pub fn read_flow_ctrl_value(&self) -> Result<u8, axdriver_sdio::error::SdioError> {
        self.read_flow_ctrl().map(|fc| fc & SDIOWIFI_FLOWCTRL_MASK)
    }

    /// 检查流控是否可用（fc_val != 0）
    pub fn check_flow_ctrl_available(&self) -> bool {
        matches!(self.read_flow_ctrl_value(), Ok(v) if v != 0)
    }

    /// 检查是否有足够空间发送指定长度的数据
    pub fn check_flow_ctrl_for_size(&self, send_len: usize) -> bool {
        match self.read_flow_ctrl_value() {
            Ok(v) if v != 0 => (v as usize) * BUFFER_SIZE > send_len,
            _ => false,
        }
    }

    /// 等待流控就绪（yield 模式，不占 CPU）
    ///
    /// 当流控不足时让出 CPU，等待 RX 线程处理数据后流控自然恢复，
    /// 而非 busy-wait 旋转浪费 CPU 时间。
    pub fn wait_flow_ctrl(&self, max_retries: u32, _spin_count: u32) -> bool {
        for _ in 0..max_retries {
            if self.check_flow_ctrl_available() {
                return true;
            }
            axtask::yield_now();
        }
        false
    }

    /// 等待流控就绪（带长度检查，yield 模式）
    pub fn wait_flow_ctrl_for_size(&self, send_len: usize, max_retries: u32, _spin_count: u32) -> bool {
        for _ in 0..max_retries {
            if self.check_flow_ctrl_for_size(send_len) {
                return true;
            }
            axtask::yield_now();
        }
        false
    }
}
