use aic8800_sdio::SdioHost;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::task::Poll;
use core::{future::poll_fn, sync::atomic::Ordering};

use axtask::future::block_on;
use log;

use crate::core::bus::{BusState, TxFrame, WifiBus};
use sdhci_cv1800::{mask_unmask_card_irq_raw, regs::*};

const TAIL_LEN: usize = 4;
const BUFFER_SIZE: usize = 1536;
const FLOW_CTRL_CMD_RETRY: u32 = 10;
/// 每次 tx_process 最多处理的数据帧数，防止饿死其他任务  
const TX_BATCH_LIMIT: u32 = 64;
const MAX_TX_QUEUE_LEN: usize = 256;

#[derive(Debug)]
pub enum TxError {
    QueueFull,
}

fn align_up(val: usize, align: usize) -> usize {
    (val + align - 1) & !(align - 1)
}

fn has_pending_work(bus: &WifiBus) -> bool {
    bus.cmd_pending_flag.load(Ordering::Acquire) || bus.tx_pktcnt.load(Ordering::Acquire) > 0
}

/// 对 CMD 帧做 TX_ALIGNMENT + TAIL_LEN + BLOCK_SIZE 对齐  
fn pad_cmd_frame(cmd: &mut Vec<u8>) -> usize {
    // Step 1: TX_ALIGNMENT (4 字节) 对齐
    let aligned = align_up(cmd.len(), TX_ALIGNMENT);
    cmd.resize(aligned, 0);

    // Step 2: 如果不是 BLOCK_SIZE 整数倍，追加 TAIL_LEN
    if cmd.len() % SDIOWIFI_FUNC_BLOCKSIZE != 0 {
        cmd.extend_from_slice(&[0u8; TAIL_LEN]);
    }

    // Step 3: 向上取整到 BLOCK_SIZE
    let final_len = align_up(cmd.len(), SDIOWIFI_FUNC_BLOCKSIZE);
    cmd.resize(final_len, 0);
    final_len
}

/// 启动 wifi-tx 线程
pub fn start(bus: Arc<WifiBus>) {
    axtask::spawn_with_name(
        move || {
            log::info!("[wifi-tx] thread started");
            block_on(poll_fn(|cx| {
                // 检查总线状态
                if *bus.state.lock() == BusState::Down {
                    return Poll::Ready(());
                }

                // 处理所有待发帧
                let did_work = tx_process(&bus);

                // 注册 waker
                bus.tx_wake_pollset.register(cx.waker());

                // 双重检查
                if did_work || has_pending_work(&bus) {
                    cx.waker().wake_by_ref();
                }

                bus.cmd_rsp_pollset.wake();

                Poll::Pending
            }))
        },
        "wifi-tx".into(),
    );
}

// ===== CMD 发送处理 =====

/// 处理 CMD 帧发送
fn process_cmd_tx(bus: &WifiBus) -> bool {
    if !bus.cmd_pending_flag.load(Ordering::Acquire) {
        return false;
    }

    let cmd_buf = bus.cmd_pending.lock().take();
    let Some(mut cmd) = cmd_buf else {
        return false;
    };

    bus.cmd_pending_flag.store(false, Ordering::Release);

    let send_len = pad_cmd_frame(&mut cmd);
    let base = bus.sdio_mmio_base.load(Ordering::Acquire);

    if base != 0 {
        mask_unmask_card_irq_raw(base, true);
    }

    let (fc_ok, did_work) = perform_cmd_flow_control_and_send(bus, &cmd, send_len);

    // 恢复中断状态 - 根据原版逻辑的3个分支处理
    if fc_ok && did_work {
        // 先 wake RX 线程（CARD_INT 仍然 masked，ISR 不会触发）
        bus.rx_irq_pollset.wake();
        // 最后才 unmask CARD_INT
        if base != 0 {
            mask_unmask_card_irq_raw(base, false);
        }
    } else if !fc_ok {
        // 流控失败
        log::error!("[wifi-tx] CMD flow_ctrl timeout, dropping CMD");
        bus.cmd_rsp_error.store(true, Ordering::Release);
        bus.cmd_rsp_pollset.wake();
        // unmask CARD_INT 恢复原状
        if base != 0 {
            mask_unmask_card_irq_raw(base, false);
        }
    } else {
        // fc_ok && !did_work (write_fifo 失败)
        if base != 0 {
            mask_unmask_card_irq_raw(base, false);
        }
    }

    did_work
}

/// 执行 CMD 流控检查并发送
///
/// # 返回
/// (fc_ok, did_work) - 流控是否通过、是否成功发送
fn perform_cmd_flow_control_and_send(bus: &WifiBus, cmd: &[u8], send_len: usize) -> (bool, bool) {
    let mut fc_ok = false;
    let mut did_work = false;
    let mut sdio = bus.sdio.lock();

    // 流控检查
    for retry in 0..FLOW_CTRL_CMD_RETRY {
        match sdio.read_byte(1, SDIOWIFI_FLOW_CTRL_REG) {
            Ok(fc) => {
                let fc_val = fc & SDIOWIFI_FLOWCTRL_MASK;
                if fc_val != 0 && (fc_val as usize) * BUFFER_SIZE > send_len {
                    fc_ok = true;
                    break;
                }
            }
            Err(e) => {
                log::error!("[wifi-tx] CMD flow_ctrl read err: {:?}", e);
                break;
            }
        }

        // 让出锁和 CPU 再重试
        drop(sdio);
        for _ in 0..10_000 {
            core::hint::spin_loop();
        }
        sdio = bus.sdio.lock();
    }

    // 发送 CMD
    if fc_ok {
        if let Err(e) = sdio.write_fifo(1, SDIOWIFI_WR_FIFO_ADDR, cmd) {
            log::error!("[wifi-tx] CMD write_fifo failed: {:?}", e);
        } else {
            did_work = true;
        }
    }

    (fc_ok, did_work)
}

// ===== 数据发送处理 =====

/// 处理数据帧批量发送
fn process_data_tx(bus: &WifiBus) -> bool {
    const SDIO_HEADER_LEN: usize = 4;
    const HOSTDESC_SIZE: usize = 28;
    const ETH_HEADER_LEN: usize = 14;

    let vif_idx = bus.connected_vif_idx.load(Ordering::Acquire);
    let sta_idx = bus.connected_sta_idx.load(Ordering::Acquire);

    // 未连接则清空队列
    if vif_idx == 0xFF {
        while let Some(_) = bus.tx_queue.lock().pop_front() {
            bus.tx_pktcnt.fetch_sub(1, Ordering::AcqRel);
        }
        return false;
    }

    let mut did_work = false;
    let mut batch_count: u32 = 0;

    while bus.tx_pktcnt.load(Ordering::Acquire) > 0 {
        // Batch 限制
        if batch_count >= TX_BATCH_LIMIT {
            break;
        }

        // CMD 优先
        if bus.cmd_pending_flag.load(Ordering::Acquire) {
            break;
        }

        // 检查总线状态
        if *bus.state.lock() == BusState::Down {
            break;
        }

        // 检查流控
        if !check_data_flow_control(bus) {
            break;
        }

        // 发送单帧
        if send_single_data_frame(bus, vif_idx, sta_idx) {
            did_work = true;
            batch_count += 1;
        }
    }

    did_work
}

/// 检查数据流控状态
fn check_data_flow_control(bus: &WifiBus) -> bool {
    let sdio = bus.sdio.lock();
    let fc = match sdio.read_byte(1, SDIOWIFI_FLOW_CTRL_REG) {
        Ok(v) => v & SDIOWIFI_FLOWCTRL_MASK,
        Err(_) => return false,
    };
    drop(sdio);

    fc > DATA_FLOW_CTRL_THRESH
}

/// 发送单个数据帧
fn send_single_data_frame(bus: &WifiBus, vif_idx: u8, sta_idx: u8) -> bool {
    const ETH_HEADER_LEN: usize = 14;

    // 从队列取帧
    let frame = bus.tx_queue.lock().pop_front();
    let Some(frame) = frame else {
        return false;
    };
    bus.tx_pktcnt.fetch_sub(1, Ordering::AcqRel);

    let eth_frame = &frame.data;
    if eth_frame.len() < ETH_HEADER_LEN {
        return false; // 不合法的以太网帧
    }

    // 构造 SDIO 数据帧
    let buf = match build_data_frame(eth_frame, vif_idx, sta_idx) {
        Ok(b) => b,
        Err(_) => return false,
    };

    // 发送
    let sdio = bus.sdio.lock();
    if let Err(e) = sdio.write_fifo(1, SDIOWIFI_WR_FIFO_ADDR, &buf) {
        log::error!("[wifi-tx] DATA write_fifo failed: {:?}", e);
        return false;
    }

    true
}

/// 数据帧构造错误
enum DataFrameBuildError {
    InvalidFrameLength,
}

/// 构造 SDIO 数据帧
fn build_data_frame(
    eth_frame: &[u8],
    vif_idx: u8,
    sta_idx: u8,
) -> Result<Vec<u8>, DataFrameBuildError> {
    const SDIO_HEADER_LEN: usize = 4;
    const HOSTDESC_SIZE: usize = 28;
    const ETH_HEADER_LEN: usize = 14;

    if eth_frame.len() < ETH_HEADER_LEN {
        return Err(DataFrameBuildError::InvalidFrameLength);
    }

    // 提取以太网帧字段
    let eth_dest = &eth_frame[0..6];
    let eth_src = &eth_frame[6..12];
    let ethertype = &eth_frame[12..14];
    let payload = &eth_frame[ETH_HEADER_LEN..];
    let payload_len = payload.len();

    // 计算长度和对齐
    let sdio_payload_len = payload_len + HOSTDESC_SIZE;
    let raw_len = sdio_payload_len + SDIO_HEADER_LEN;
    let aligned_len = align_up(raw_len, TX_ALIGNMENT);
    let sdio_hdr_len = aligned_len - SDIO_HEADER_LEN;

    let final_len = if aligned_len % SDIOWIFI_FUNC_BLOCKSIZE != 0 {
        align_up(aligned_len + TAIL_LEN, SDIOWIFI_FUNC_BLOCKSIZE)
    } else {
        aligned_len
    };

    let mut buf = vec![0u8; final_len];

    // 填充 SDIO header
    buf[0] = (sdio_hdr_len & 0xFF) as u8;
    buf[1] = ((sdio_hdr_len >> 8) & 0x0F) as u8;
    buf[2] = SDIO_TYPE_DATA;
    buf[3] = 0x00;

    // 填充 HostDesc
    fill_hostdesc(
        &mut buf,
        payload_len,
        eth_dest,
        eth_src,
        ethertype,
        vif_idx,
        sta_idx,
    );

    // 填充 payload
    let payload_start = SDIO_HEADER_LEN + HOSTDESC_SIZE;
    buf[payload_start..payload_start + payload_len].copy_from_slice(payload);

    Ok(buf)
}

/// 填充 HostDesc
fn fill_hostdesc(
    buf: &mut [u8],
    payload_len: usize,
    eth_dest: &[u8],
    eth_src: &[u8],
    ethertype: &[u8],
    vif_idx: u8,
    sta_idx: u8,
) {
    const SDIO_HEADER_LEN: usize = 4;
    const HOSTDESC_SIZE: usize = 28;

    let hd = &mut buf[SDIO_HEADER_LEN..SDIO_HEADER_LEN + HOSTDESC_SIZE];

    // packet_len [0..2]
    hd[0..2].copy_from_slice(&(payload_len as u16).to_le_bytes());
    // flags_ext [2..4] = 0

    // hostid [4..8]: 设置 bit 31 请求 TX CFM
    hd[4..8].copy_from_slice(&0x8000_0001u32.to_le_bytes());

    // eth_dest_addr [8..14]
    hd[8..14].copy_from_slice(eth_dest);
    // eth_src_addr [14..20]
    hd[14..20].copy_from_slice(eth_src);
    // ethertype [20..22]
    hd[20..22].copy_from_slice(ethertype);
    hd[22] = 0;

    // tid [23] = 0 (BE)
    hd[23] = 0;

    // vif_idx [24]
    hd[24] = vif_idx;
    // sta_idx [25]
    hd[25] = sta_idx;
    // flags [26..28] = 0
}

// ===== 主处理函数 =====

/// TX 处理主逻辑
///
/// 1. CMD 优先：如果有 cmd_pending，先发 CMD
/// 2. Data：检查 flow_ctrl → dequeue → 构造帧 → send
fn tx_process(bus: &WifiBus) -> bool {
    let mut did_work = false;

    // 检查总线状态
    if *bus.state.lock() == BusState::Down {
        return false;
    }

    // Step 1: CMD 优先发送
    if process_cmd_tx(bus) {
        did_work = true;
    }

    // Step 2: DATA 批量发送
    if process_data_tx(bus) {
        did_work = true;
    }

    did_work
}

/// 将以太网帧入队 TX 队列
pub fn enqueue_data_frame(bus: &Arc<WifiBus>, eth_frame: Vec<u8>) -> Result<(), TxError> {
    let mut queue = bus.tx_queue.lock();
    if queue.len() >= MAX_TX_QUEUE_LEN {
        return Err(TxError::QueueFull);
    }

    queue.push_back(TxFrame {
        data: eth_frame,
        priority: 0, // 默认优先级，后续可按 802.1p/DSCP 分类
    });
    drop(queue);

    bus.tx_pktcnt.fetch_add(1, Ordering::AcqRel);
    bus.tx_wake_pollset.wake();
    Ok(())
}
