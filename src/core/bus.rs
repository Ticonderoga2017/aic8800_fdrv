use aic8800_sdio::SdioHost;
use alloc::{collections::VecDeque, sync::Arc, vec::Vec};
use axpoll::PollSet;
use axsync::Mutex;
use core::ptr::read_volatile;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, AtomicUsize, Ordering};
use kspin::SpinNoIrq;
use sdhci_cv1800::{CviSdhci, mask_unmask_card_irq_raw, regs::*};

/// 总线状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BusState {
    Down,
    Up,
}

/// TX 帧封装
pub struct TxFrame {
    pub data: Vec<u8>,
    pub priority: u8,
}

/// SDIO 总线共享资源
pub struct WifiBus {
    /// SDHCI 控制器实例（TX/RX 线程共享，通过 Mutex 互斥）
    pub sdio: Arc<Mutex<CviSdhci>>,

    /// SDHCI MMIO 基地址（ISR 裸写用，不经过 Mutex）
    pub sdio_mmio_base: AtomicUsize,

    /// 总线状态
    pub state: SpinNoIrq<BusState>,

    // ---- SDHCI 完成标志（ISR → SDHCI 驱动等待者）----
    pub sdhci_cmd_complete: AtomicBool,
    pub sdhci_xfer_complete: AtomicBool,
    pub sdhci_buf_rd_ready: AtomicBool,
    pub sdhci_error_status: AtomicU16,
    pub sdhci_pollset: PollSet,

    // ---- RX 侧 ----
    /// RX PollSet：SDIO Card Interrupt → ISR 唤醒 wifi-rx 线程
    pub rx_irq_pollset: PollSet,
    pub rx_irq_pending: AtomicBool,

    /// RX 帧队列（ISR 不写此队列；wifi-rx 线程读 FIFO 后按类型分发）
    /// 数据帧队列（wifi-rx → NetDevice）
    pub data_rx_queue: SpinNoIrq<VecDeque<Vec<u8>>>,
    pub data_rx_pollset: PollSet,

    /// EAPOL 帧队列（rx_thread 识别 ethertype=0x888E 后路由到此）  
    pub eapol_queue: SpinNoIrq<VecDeque<Vec<u8>>>,
    /// EAPOL PollSet（唤醒 wait_for_eapol 等待者）  
    pub eapol_pollset: PollSet,

    /// CMD 响应队列（wifi-rx → CmdMgr）
    pub cmd_rsp_queue: SpinNoIrq<VecDeque<Vec<u8>>>,
    pub cmd_rsp_pollset: PollSet,

    /// TX Confirm 队列（wifi-rx → TX 确认处理）
    pub tx_cfm_queue: SpinNoIrq<VecDeque<Vec<u8>>>,
    pub tx_cfm_pollset: PollSet,

    // ---- TX 侧 ----
    /// TX 帧队列（上层 → wifi-tx 线程）
    pub tx_queue: SpinNoIrq<VecDeque<TxFrame>>,
    pub tx_pktcnt: AtomicU32,
    pub tx_wake_pollset: PollSet,

    /// CMD 发送请求（CmdMgr → wifi-tx 线程）
    pub cmd_pending: SpinNoIrq<Option<Vec<u8>>>,
    pub cmd_pending_flag: AtomicBool,

    /// CmdMgr 错误标志：shutdown 时设为 true，  
    /// send_cmd 等待者被唤醒后检查此标志返回 Err  
    pub cmd_rsp_error: AtomicBool,

    pub ind_queue: SpinNoIrq<VecDeque<Vec<u8>>>, // 异步 indication 队列
    pub ind_pollset: PollSet,                    // indication 到达通知

    pub cmd_expected_cfm_id: AtomicU16, // 当前等待的 CFM msg_id

    pub connected_vif_idx: AtomicU8, // 当前连接的 VIF 索引（0-7），0xFF 表示未连接
    pub connected_sta_idx: AtomicU8, // 当前连接的 STA 索引（0-7），0xFF 表示未连接
    pub connected_sta_mac: SpinNoIrq<Option<[u8; 6]>>, // 当前连接的 STA MAC 地址
    pub connected_ap_mac: SpinNoIrq<Option<[u8; 6]>>, // 当前连接的 AP MAC 地址
}

impl WifiBus {
    pub fn new(sdio: CviSdhci) -> Arc<Self> {
        let base = sdio.mmio_base();
        Arc::new(Self {
            sdio: Arc::new(Mutex::new(sdio)),
            sdio_mmio_base: AtomicUsize::new(base),
            state: SpinNoIrq::new(BusState::Down),
            sdhci_cmd_complete: AtomicBool::new(false),
            sdhci_xfer_complete: AtomicBool::new(false),
            sdhci_buf_rd_ready: AtomicBool::new(false),
            sdhci_error_status: AtomicU16::new(0),
            sdhci_pollset: PollSet::new(),
            rx_irq_pollset: PollSet::new(),
            rx_irq_pending: AtomicBool::new(false),
            data_rx_queue: SpinNoIrq::new(VecDeque::new()),
            data_rx_pollset: PollSet::new(),
            eapol_queue: SpinNoIrq::new(VecDeque::new()),
            eapol_pollset: PollSet::new(),
            cmd_rsp_queue: SpinNoIrq::new(VecDeque::new()),
            cmd_rsp_pollset: PollSet::new(),
            tx_cfm_queue: SpinNoIrq::new(VecDeque::new()),
            tx_cfm_pollset: PollSet::new(),
            tx_queue: SpinNoIrq::new(VecDeque::new()),
            tx_pktcnt: AtomicU32::new(0),
            tx_wake_pollset: PollSet::new(),
            cmd_pending: SpinNoIrq::new(None),
            cmd_pending_flag: AtomicBool::new(false),
            cmd_rsp_error: AtomicBool::new(false),
            ind_queue: SpinNoIrq::new(VecDeque::new()),
            ind_pollset: PollSet::new(),
            cmd_expected_cfm_id: AtomicU16::new(0),
            connected_vif_idx: AtomicU8::new(0xFF),
            connected_sta_idx: AtomicU8::new(0xFF),
            connected_sta_mac: SpinNoIrq::new(None),
            connected_ap_mac: SpinNoIrq::new(None),
        })
    }

    /// 关闭总线，停止所有线程
    pub fn shutdown(self: &Arc<Self>) {
        // 1. 设置 BUS_DOWN（线程循环会检测此状态并退出）
        *self.state.lock() = BusState::Down;

        // 2. 禁用 AIC8800 芯片端 SDIO 中断
        {
            let sdio = self.sdio.lock();
            let _ = sdio.write_byte(1, SDIOWIFI_INTR_CONFIG_REG, 0x00);
            sdio.disable_irq();
        }

        // 3. 唤醒 TX 线程，等待其退出
        //    TX 线程检测 BUS_DOWN 后会停止发送并退出
        self.tx_wake_pollset.wake();
        // flush TX 队列（TX 线程退出后不会再访问）
        self.tx_queue.lock().clear();

        // 4. 唤醒 RX 线程，等待其退出
        self.rx_irq_pollset.wake();

        // 5. flush RX 相关队列
        self.data_rx_queue.lock().clear();

        // 6. 唤醒所有 CMD 等待者并标记错误
        self.cmd_rsp_error.store(true, Ordering::Release);
        self.cmd_rsp_pollset.wake();
        self.tx_cfm_pollset.wake();
        self.sdhci_pollset.wake();

        self.eapol_pollset.wake();
        self.eapol_queue.lock().clear();

        self.ind_pollset.wake();
        self.ind_queue.lock().clear();

        clear_global_bus();

        log::info!("[wifi-bus] shutdown: interrupts disabled, threads notified");
    }
}

/// 全局 WifiBus 引用（init 后设置，ISR 读取）
static WIFI_BUS_PTR: AtomicUsize = AtomicUsize::new(0);

pub fn set_global_bus(bus: &Arc<WifiBus>) {
    let ptr = Arc::into_raw(Arc::clone(bus)); // refcount + 1
    let old = WIFI_BUS_PTR.swap(ptr as usize, Ordering::AcqRel);
    if old != 0 {
        // 释放旧的引用
        unsafe {
            Arc::from_raw(old as *const WifiBus);
        }
    }
}

pub fn get_global_bus() -> Option<&'static WifiBus> {
    let ptr = WIFI_BUS_PTR.load(Ordering::Acquire);
    if ptr == 0 {
        None
    } else {
        unsafe { Some(&*(ptr as *const WifiBus)) }
    }
}

/// shutdown 时调用，释放全局引用  
pub fn clear_global_bus() {
    let old = WIFI_BUS_PTR.swap(0, Ordering::AcqRel);
    if old != 0 {
        unsafe {
            Arc::from_raw(old as *const WifiBus);
        }
    }
}

use core::sync::atomic::AtomicU64;
pub(crate) static IRQ_COUNT: AtomicU64 = AtomicU64::new(0);

/// PLIC IRQ #38 处理函数
///
/// 约束：不持锁、不分配堆、不调度。仅操作 Atomic + MMIO 裸写 + waker.wake()
pub fn sdio1_irq_handler() {
    IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
    // 注意：ISR 中禁止调用 log::info!/warn!/error!
    // 这些宏需要获取 console 锁，如果被打断的代码持有该锁会死锁
    // log::info!("[ISR] SDIO1 IRQ#38 triggered (count={})", cnt + 1);

    let Some(bus) = get_global_bus() else { return };
    let base = bus.sdio_mmio_base.load(Ordering::Acquire);
    if base == 0 {
        return;
    }

    // 读 NORM_AND_ERR_INT_STS (offset 0x030)
    let status = unsafe { read_volatile((base + SDHCI_INT_STATUS_NORM as usize) as *const u32) };

    if status == 0 {
        return;
    }

    // CARD_INT (bit 8): AIC8800 有数据要发给主机
    if status & (NORM_INT_CARD_INT as u32) != 0 {
        // 屏蔽 CARD_INT 信号，防止重复触发（电平触发）
        mask_unmask_card_irq_raw(base, true);
        // 唤醒 wifi-rx 线程
        bus.rx_irq_pending.store(true, Ordering::Release);
        bus.rx_irq_pollset.wake();
    }
}
