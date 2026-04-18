//! FDRV 初始化模块
//!
//! 包含驱动初始化相关函数

use crate::consts::*;
use crate::core::bus::{IRQ_COUNT, WifiBus, set_global_bus};
use crate::thread::{rx, tx};
use alloc::vec::Vec;
use core::sync::atomic::Ordering;
use cvi_sdhci::CviSdhci;
use log::{error, info, log, warn};

// ===== polling_send_cmd 辅助函数 =====

/// 计算轮询命令帧的长度和对齐
fn calculate_polling_frame_layout(param_len: usize) -> usize {
    let lmac_len = LMAC_MSG_HEADER_SIZE + param_len;
    let sdio_payload_len = DUMMY_WORD_LEN + lmac_len;
    let sdio_len = sdio_payload_len + SDIO_HEADER_SIZE;
    let raw_len = SDIO_HEADER_SIZE + DUMMY_WORD_LEN + lmac_len;

    let aligned = (raw_len + TX_ALIGNMENT - 1) & !(TX_ALIGNMENT - 1);
    if aligned % SDIOWIFI_FUNC_BLOCKSIZE != 0 {
        let with_tail = aligned + TAIL_LEN;
        ((with_tail / SDIOWIFI_FUNC_BLOCKSIZE) + 1) * SDIOWIFI_FUNC_BLOCKSIZE
    } else {
        aligned
    }
}

/// 构造轮询命令帧
fn build_polling_cmd_frame(msg_id: u16, dest_id: u16, param: &[u8]) -> Vec<u8> {
    let lmac_len = LMAC_MSG_HEADER_SIZE + param.len();
    let sdio_payload_len = DUMMY_WORD_LEN + lmac_len;
    let sdio_len = sdio_payload_len + SDIO_HEADER_SIZE;

    let final_len = calculate_polling_frame_layout(param.len());
    let mut buf: Vec<u8> = vec![0u8; final_len];

    // sdio_header [0..4]
    buf[0] = (sdio_len & U8_MASK) as u8;
    buf[1] = ((sdio_len >> 8) & (LOW_NIBBLE_MASK as u16)) as u8;
    buf[2] = SDIO_TYPE_CFG_CMD_RSP;
    buf[3] = 0x00;

    // lmac_msg header [8..16]
    let off = SDIO_HEADER_SIZE + DUMMY_WORD_LEN; // = 8
    buf[off..off + 2].copy_from_slice(&msg_id.to_le_bytes());
    buf[off + 2..off + 4].copy_from_slice(&dest_id.to_le_bytes());
    buf[off + 4..off + 6].copy_from_slice(&DRV_TASK_ID.to_le_bytes());
    buf[off + 6..off + 8].copy_from_slice(&(param.len() as u16).to_le_bytes());

    if !param.is_empty() {
        buf[off + 8..off + 8 + param.len()].copy_from_slice(param);
    }

    buf
}

/// 轮询模式流控检查
fn check_flow_control_polling(sdio: &CviSdhci) -> Result<(), &'static str> {
    for retry in 0..FLOW_CONTROL_MAX_RETRY {
        match sdio.read_byte(1, SDIOWIFI_FLOW_CTRL_REG) {
            Ok(fc) if fc & FLOW_CONTROL_MASK != 0 => return Ok(()),
            Ok(_) => {}
            Err(_) => return Err("flow_ctrl read error"),
        }
        if retry >= FLOW_CONTROL_MAX_RETRY - 1 {
            return Err("flow_ctrl timeout");
        }
        for _ in 0..FLOW_CONTROL_RETRY_INTERVAL {
            core::hint::spin_loop();
        }
    }
    Ok(())
}

/// 轮询等待响应
fn poll_for_response(
    sdio: &CviSdhci,
    expected_cfm: u16,
    cfm_buf: &mut [u8],
) -> Result<usize, &'static str> {
    for retry in 0..RESPONSE_MAX_RETRY {
        let raw = sdio
            .read_byte(1, SDIOWIFI_BLOCK_CNT_REG)
            .map_err(|_| "read block_cnt error")?;

        if raw & SDIO_OTHER_INTERRUPT != 0 {
            // SDIO_OTHER_INTERRUPT — 重试
            for _ in 0..RESPONSE_POLL_INTERVAL {
                core::hint::spin_loop();
            }
            continue;
        }

        let block_cnt = raw & BLOCK_COUNT_MASK;
        if block_cnt == 0 {
            if retry > RESPONSE_MAX_RETRY - 1 {
                return Err("response timeout");
            }
            for _ in 0..RESPONSE_POLL_INTERVAL {
                core::hint::spin_loop();
            }
            continue;
        }

        // 读取并解析响应
        match read_and_parse_response(sdio, block_cnt, expected_cfm, cfm_buf) {
            Ok(len) => return Ok(len),
            Err(e) => {
                // 解析失败（可能是固件启动通知），继续等待
                log::warn!("[polling] {}", e);
                continue;
            }
        }
    }

    Err("response timeout")
}

/// 读取并解析响应帧
fn read_and_parse_response(
    sdio: &CviSdhci,
    block_cnt: u8,
    expected_cfm: u16,
    cfm_buf: &mut [u8],
) -> Result<usize, &'static str> {
    let read_len = (block_cnt as usize) * SDIOWIFI_FUNC_BLOCKSIZE;
    let mut rx_buf: Vec<u8> = vec![0u8; read_len];

    if sdio
        .read_fifo(1, SDIOWIFI_RD_FIFO_ADDR, &mut rx_buf)
        .is_err()
    {
        // CRC 错误 — 延迟后重试
        for _ in 0..RESPONSE_READ_DELAY {
            core::hint::spin_loop();
        }
        return Err("CRC error, retrying");
    }

    // 解析响应：E2A 方向无 dummy word
    // rx_buf[0..4] = sdio_header
    // rx_buf[4..12] = lmac_msg header (id, dest_id, src_id, param_len)
    if read_len < PROTO_HEADER_SIZE - 4 {
        return Err("response too short");
    }

    let resp_id = u16::from_le_bytes([rx_buf[4], rx_buf[5]]);
    if resp_id != expected_cfm {
        return Err(&format!(
            "unexpected resp_id=0x{:04x}, expected=0x{:04x}",
            resp_id, expected_cfm
        ));
    }

    // 拷贝 param 部分到 cfm_buf
    let param_offset = PROTO_HEADER_SIZE - 4; // sdio_header(4) + lmac_msg_header(8)
    let cfm_len = cfm_buf.len().min(read_len.saturating_sub(param_offset));
    if cfm_len > 0 {
        cfm_buf[..cfm_len].copy_from_slice(&rx_buf[param_offset..param_offset + cfm_len]);
    }

    Ok(cfm_len)
}

/// 轮询模式发送 LMAC 命令并等待 CFM
///
/// 用于 FDRV 初始化阶段（中断未使能），直接操作 SDIO 寄存器。
/// 与 `aic8800_fw::ipc_msg::IpcTransport::send_msg` 类似，但使用
/// TASK_MM 消息格式而非 TASK_DBG。
pub fn polling_send_cmd(
    sdio: &CviSdhci,
    msg_id: u16,
    dest_id: u16,
    param: &[u8],
    wait_cfm: bool,
    cfm_buf: &mut [u8],
) -> Result<usize, &'static str> {
    // ---- 构造帧 ----
    let buf = build_polling_cmd_frame(msg_id, dest_id, param);

    // ---- 流控 ----
    check_flow_control_polling(sdio)?;

    // ---- 写 FIFO ----
    sdio.write_fifo(1, SDIOWIFI_WR_FIFO_ADDR, &buf)
        .map_err(|_| "write_fifo error")?;

    if !wait_cfm {
        return Ok(0);
    }

    // ---- 轮询等待响应 ----
    let expected_cfm = msg_id + 1;
    poll_for_response(sdio, expected_cfm, cfm_buf)
}

/// 排空残留数据
fn drain_stale_data(sdio: &CviSdhci) {
    for i in 0..10 {
        match sdio.read_byte(1, SDIOWIFI_BLOCK_CNT_REG) {
            Ok(raw) => {
                // 跳过 SDIO_OTHER_INTERRUPT (bit 7)
                if raw & SDIO_OTHER_INTERRUPT != 0 {
                    log::debug!("[fdrv] drain: SDIO_OTHER_INTERRUPT, raw=0x{:02x}", raw);
                    continue;
                }
                let block_cnt = raw & SDIOWIFI_FLOWCTRL_MASK;
                if block_cnt == 0 {
                    info!("[fdrv] drain: no stale data (iteration {})", i);
                    break;
                }
                info!(
                    "[fdrv] drain: block_cnt={}, reading and discarding",
                    block_cnt
                );
                let data_len = (block_cnt as usize) * SDIOWIFI_FUNC_BLOCKSIZE;
                let mut buf: Vec<u8> = vec![0u8; data_len];
                match sdio.read_fifo(1, SDIOWIFI_RD_FIFO_ADDR, &mut buf) {
                    Ok(()) => info!("[fdrv] drain: discarded {} bytes", data_len),
                    Err(e) => {
                        warn!(
                            "[fdrv] drain: read_fifo failed: {:?} (CRC error expected)",
                            e
                        );
                        // CRC 错误是预期的（固件刚启动），继续排空
                    }
                }
            }
            Err(e) => {
                warn!("[fdrv] drain: read_byte failed: {:?}", e);
                break;
            }
        }
    }
}

// ===== 初始化辅助函数 =====

/// 等待固件 SDIO 接口稳定
fn wait_for_firmware_stabilization() {
    info!("[fdrv] waiting for firmware SDIO interface to stabilize...");
    for _ in 0..CHIP_STARTUP_DELAY {
        core::hint::spin_loop();
    } // ~800ms
}

/// 排空初始化前的残留数据
fn drain_initial_stale_data(sdio: &CviSdhci) {
    for i in 0..5u32 {
        match sdio.read_byte(1, SDIOWIFI_BLOCK_CNT_REG) {
            Ok(raw) => {
                let cnt = raw & BLOCK_COUNT_MASK;
                if cnt == 0 {
                    info!("[fdrv] drain: no stale data (iteration {})", i);
                    break;
                }
                let len = (cnt as usize) * SDIOWIFI_FUNC_BLOCKSIZE;
                let mut discard: Vec<u8> = vec![0u8; len];
                let _ = sdio.read_fifo(1, SDIOWIFI_RD_FIFO_ADDR, &mut discard);
                info!("[fdrv] drain: discarded {} bytes (block_cnt={})", len, cnt);
            }
            Err(e) => {
                warn!("[fdrv] drain: read_byte error: {:?}", e);
                break;
            }
        }
    }
}

/// 发送 MM_SET_STACK_START_REQ 命令
fn send_stack_start_command(sdio: &CviSdhci) -> Result<(), &'static str> {
    let mut cfm = [0u8; 2]; // mm_set_stack_start_cfm: is_5g_support(1) + vendor_info(1)
    match polling_send_cmd(
        sdio,
        MM_SET_STACK_START_REQ,
        TASK_MM,
        &STACK_START_PARAM,
        true,
        &mut cfm,
    ) {
        Ok(len) => {
            info!(
                "[fdrv] MM_SET_STACK_START_CFM OK, len={}, is_5g={}, vendor=0x{:02x}",
                len,
                if len > 0 { cfm[0] } else { 0 },
                if len > 1 { cfm[1] } else { 0 }
            );
            Ok(())
        }
        Err(e) => {
            error!("[fdrv] MM_SET_STACK_START_REQ failed: {}", e);
            Err("MM_SET_STACK_START_REQ failed")
        }
    }
}

/// 发送 MM_RESET_REQ 命令
fn send_reset_command(sdio: &CviSdhci) -> Result<(), &'static str> {
    let mut reset_cfm = [0u8; 0];
    match polling_send_cmd(sdio, MM_RESET_REQ, TASK_MM, &[], true, &mut reset_cfm) {
        Ok(_) => {
            info!("[fdrv] MM_RESET_CFM OK");
            Ok(())
        }
        Err(e) => {
            error!("[fdrv] MM_RESET_REQ failed: {}", e);
            Err("MM_RESET_REQ failed")
        }
    }
}

/// 发送 LMAC 初始化命令
fn send_lmac_init_commands(sdio: &CviSdhci) -> Result<(), &'static str> {
    // 2a. MM_SET_STACK_START_REQ
    send_stack_start_command(sdio)?;

    // 2b. MM_RESET_REQ
    send_reset_command(sdio)?;

    Ok(())
}

/// 排空 LMAC 初始化后的残留数据
fn drain_post_init_data(sdio: &CviSdhci) {
    for _ in 0..INIT_DELAY {
        core::hint::spin_loop();
    } // ~200ms

    for i in 0..10u32 {
        match sdio.read_byte(1, SDIOWIFI_BLOCK_CNT_REG) {
            Ok(raw) => {
                let cnt = raw & BLOCK_COUNT_MASK;
                if cnt == 0 {
                    break;
                }
                let len = (cnt as usize) * SDIOWIFI_FUNC_BLOCKSIZE;
                let mut discard: Vec<u8> = vec![0u8; len];
                sdio.read_fifo(1, SDIOWIFI_RD_FIFO_ADDR, &mut discard);
                info!("[fdrv] post-init drain: discarded {} bytes", len);
            }
            Err(_) => break,
        }
    }
}

/// 注册 PLIC IRQ#38
fn register_plic_irq() -> Result<(), &'static str> {
    let registered = axplat::irq::register(38, sdio1_irq_handler);
    if !registered {
        return Err("IRQ#38 registration failed");
    }
    info!("[fdrv] PLIC IRQ#38 registered");
    Ok(())
}

/// 使能中断（SDHCI CARD_INT 和 AIC8800 芯片端）
fn enable_interrupts(bus: &WifiBus) -> Result<(), &'static str> {
    // 使能 SDHCI CARD_INT 信号
    {
        let sdio = bus.sdio.lock();
        sdio.enable_irq();
    }

    // 使能 AIC8800 芯片端 SDIO 中断
    {
        let sdio = bus.sdio.lock();
        if sdio.write_byte(1, SDIOWIFI_INTR_CONFIG_REG, 0x07).is_err() {
            return Err("SDIOWIFI_INTR_CONFIG_REG write failed");
        }
    }

    // 验证 IRQ 触发
    for _ in 0..100_000 {
        core::hint::spin_loop();
    }
    let irq_cnt = IRQ_COUNT.load(Ordering::Relaxed);
    info!("[VERIFY-1] IRQ#38 triggered {} times", irq_cnt);

    Ok(())
}

/// 启动 RX/TX 线程
fn start_driver_threads(bus: &alloc::sync::Arc<WifiBus>) {
    *bus.state.lock() = BusState::Up;
    rx::start(alloc::sync::Arc::clone(&bus));
    tx::start(alloc::sync::Arc::clone(&bus));
}

/// FDRV 初始化入口
///
/// 在 firmware_init 成功后调用。执行以下步骤：
/// 1. 等待固件 SDIO 接口稳定
/// 2. 排空残留数据
/// 3. 轮询模式发送 MM_SET_STACK_START_REQ + MM_RESET_REQ（LMAC 初始化）
/// 4. 注册 PLIC IRQ#38
/// 5. 使能 CARD_INT 信号 + AIC8800 芯片端中断
/// 6. 启动 RX/TX 线程
pub fn init(sdio: CviSdhci) -> Result<alloc::sync::Arc<WifiBus>, &'static str> {
    // ---- Step 0: 等待固件 SDIO 接口稳定 ----
    wait_for_firmware_stabilization();

    // ---- Step 1: 排空残留数据 ----
    drain_initial_stale_data(&sdio);

    // ---- Step 2: 轮询模式 LMAC 初始化 ----
    send_lmac_init_commands(&sdio)?;

    // ---- Step 3: 排空 LMAC 初始化产生的残留数据 ----
    drain_post_init_data(&sdio);

    // ---- Step 4: 创建 WifiBus ----
    let bus = WifiBus::new(sdio);
    set_global_bus(&bus);

    // ---- Step 5: 注册 PLIC IRQ#38 ----
    register_plic_irq()?;

    // ---- Step 6: 使能中断 ----
    enable_interrupts(&bus)?;

    // ---- Step 7: 启动线程 ----
    start_driver_threads(&bus);

    info!("[fdrv] AIC8800 FDRV initialized");
    Ok(bus)
}

/// SDIO IRQ 处理器
extern "C" fn sdio1_irq_handler() {
    use crate::core::bus::IRQ_COUNT;
    IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
}
