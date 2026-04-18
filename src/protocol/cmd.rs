use aic8800_sdio::SdioHost;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::sync::atomic::Ordering;
use core::task::Poll;
use core::{future::poll_fn, time::Duration};

use axtask::future::{self, block_on};
use log;

use crate::consts::*;
use crate::{
    core::bus::{BusState, WifiBus},
    protocol::lmac_msg::*,
};
use sdhci_cv1800::{mask_unmask_card_irq_raw, regs::*};

fn align_up(val: usize, align: usize) -> usize {
    (val + align - 1) & !(align - 1)
}

fn current_time_ms() -> u64 {
    axhal::time::monotonic_time_nanos() / 1_000_000
}

/// 构造 SDIO CMD 帧  
/// 格式：[4B sdio_header][4B dummy][8B lmac_msg][NB param]  
/// 对齐：TX_ALIGNMENT → TAIL_LEN → SDIOWIFI_FUNC_BLOCKSIZE  
fn build_cmd_frame(msg_id: u16, dest_task: u16, param: &[u8]) -> Vec<u8> {
    // lmac_msg header (8B) + param
    let lmac_len = 8 + param.len();
    // sdio_header length field = sdio_payload_len + 4 (header itself)
    let sdio_len = DUMMY_WORD_LEN + lmac_len;
    // total raw length = sdio_header(4) + dummy(4) + lmac_msg(8) + param
    let raw_len = 4 + DUMMY_WORD_LEN + lmac_len;

    // Step 1: TX_ALIGNMENT
    let aligned_len = align_up(raw_len, TX_ALIGNMENT);

    // Step 2: TAIL_LEN + BLOCK_SIZE alignment
    let final_len = if aligned_len % SDIOWIFI_FUNC_BLOCKSIZE != 0 {
        align_up(aligned_len + TAIL_LEN, SDIOWIFI_FUNC_BLOCKSIZE)
    } else {
        aligned_len
    };

    let mut buf = vec![0u8; final_len];

    // sdio_header [0..4]
    buf[0] = (sdio_len & 0xFF) as u8;
    buf[1] = ((sdio_len >> 8) & 0x0F) as u8;
    buf[2] = SDIO_TYPE_CFG_CMD_RSP; // 0x11  
    buf[3] = 0x00; // AIC8801: reserved; AIC8800D80: CRC8  
    // dummy word [4..8] = 0x00000000 (already zeroed)

    // lmac_msg header [8..16]
    let msg_offset = 4 + DUMMY_WORD_LEN; // = 8  
    buf[msg_offset..msg_offset + 2].copy_from_slice(&msg_id.to_le_bytes());
    buf[msg_offset + 2..msg_offset + 4].copy_from_slice(&dest_task.to_le_bytes());
    buf[msg_offset + 4..msg_offset + 6].copy_from_slice(&DRV_TASK_ID.to_le_bytes()); // src_id  
    buf[msg_offset + 6..msg_offset + 8].copy_from_slice(&(param.len() as u16).to_le_bytes());

    // param [16..16+param.len()]
    if !param.is_empty() {
        buf[msg_offset + 8..msg_offset + 8 + param.len()].copy_from_slice(param);
    }

    buf
}

/// 发送 LMAC 命令并等待 CFM
///
/// # 参数
/// - `bus`: 共享总线
/// - `msg_id`: 消息 ID (如 MM_RESET_REQ)
/// - `dest_id`: 目标任务 ID (如 TASK_MM)
/// - `param`: 命令参数（结构体的字节表示）
/// - `timeout_ms`: 超时时间
///
/// # 返回
/// - `Ok(Vec<u8>)`: CFM 的 param 部分
/// - `Err(CmdError)`: 超时或错误
pub fn send_cmd(
    bus: &Arc<WifiBus>,
    msg_id: u16,
    dest_id: u16,
    param: &[u8],
    timeout_ms: u64,
) -> Result<Vec<u8>, CmdError> {
    send_cmd_with_cfm_id(bus, msg_id, dest_id, param, msg_id + 1, timeout_ms)
}

// ===== CMD 发送辅助函数 =====

/// 准备 CMD 发送（清空队列、设置标志）
fn prepare_cmd_send(bus: &Arc<WifiBus>, expected_cfm_id: u16) {
    // 清空残留的 CMD 响应队列
    {
        let mut queue = bus.cmd_rsp_queue.lock();
        if !queue.is_empty() {
            log::warn!("[cmd_mgr] discarding {} stale CMD responses", queue.len());
            queue.clear();
        }
    }

    // 清除错误标志并设置期望的 CFM ID
    bus.cmd_rsp_error.store(false, Ordering::Release);
    bus.cmd_expected_cfm_id
        .store(expected_cfm_id, Ordering::Release);
}

/// 通过 TX 线程发送 CMD 帧
fn send_cmd_via_tx_thread(bus: &Arc<WifiBus>, frame: Vec<u8>) {
    let mut cmd_slot = bus.cmd_pending.lock();
    *cmd_slot = Some(frame);
    bus.cmd_pending_flag.store(true, Ordering::Release);
    drop(cmd_slot);
    bus.tx_wake_pollset.wake(); // 唤醒 TX 线程
}

/// 验证 CFM 并提取参数
///
/// # 返回
/// 如果 CFM ID 匹配，返回 Ok(param)，否则返回 Err
fn validate_and_extract_cfm_param(rsp: &[u8], expected_cfm_id: u16) -> Result<Vec<u8>, ()> {
    if rsp.len() < LmacMsg::SIZE {
        return Err(()); // 响应格式无效
    }

    let msg = LmacMsg::from_le_bytes(rsp);
    if msg.id != expected_cfm_id {
        return Err(()); // CFM ID 不匹配
    }

    let param_start = LmacMsg::SIZE;
    let param_end = param_start + msg.param_len as usize;

    if rsp.len() >= param_end {
        Ok(rsp[param_start..param_end].to_vec())
    } else {
        Ok(rsp[param_start..].to_vec())
    }
}

/// 从 cmd_rsp_queue 尝试获取 CFM
///
/// # 返回
/// Some(result) - 找到并验证了响应，Some(Ok(param)) 或 Some(Err) 表示 ID 不匹配
/// None - 队列为空
fn try_get_cfm_from_queue(
    bus: &Arc<WifiBus>,
    expected_cfm_id: u16,
) -> Option<Result<Vec<u8>, CmdError>> {
    let mut queue = bus.cmd_rsp_queue.lock();
    let rsp = queue.pop_front()?;

    // 验证 CFM 并提取参数
    match validate_and_extract_cfm_param(&rsp, expected_cfm_id) {
        Ok(param) => Some(Ok(param)),
        Err(()) => {
            // CFM ID 不匹配，移到 ind_queue
            log::warn!(
                "[cmd_mgr] CFM id mismatch: expected 0x{:04x}, got 0x{:04x}",
                expected_cfm_id,
                LmacMsg::from_le_bytes(&rsp).id
            );
            bus.ind_queue.lock().push_back(rsp);
            bus.ind_pollset.wake();
            Some(Err(CmdError::InvalidResponse))
        }
    }
}

/// 等待 CFM 响应（带超时和错误检查）
fn wait_for_cfm_response(
    bus: &Arc<WifiBus>,
    expected_cfm_id: u16,
    timeout_ns: u64,
    start: u64,
) -> Result<Vec<u8>, CmdError> {
    block_on(poll_fn(|cx| {
        // 检查总线关闭
        if bus.cmd_rsp_error.load(Ordering::Acquire) || *bus.state.lock() == BusState::Down {
            return Poll::Ready(Err(CmdError::BusDown));
        }

        // 检查超时
        let elapsed = axhal::time::monotonic_time_nanos() - start;
        if elapsed > timeout_ns {
            log::error!(
                "[cmd_mgr] TIMEOUT waiting for cfm 0x{:04x} (elapsed={}ms)",
                expected_cfm_id,
                elapsed / 1_000_000
            );
            return Poll::Ready(Err(CmdError::Timeout));
        }

        // 尝试从队列取响应
        if let Some(result) = try_get_cfm_from_queue(bus, expected_cfm_id) {
            return Poll::Ready(result);
        }

        // 注册 waker，等待 RX 线程唤醒
        bus.cmd_rsp_pollset.register(cx.waker());

        // 双重检查
        if let Some(result) = try_get_cfm_from_queue(bus, expected_cfm_id) {
            return Poll::Ready(result);
        }

        // 保持活跃以确保超时检查能触发
        cx.waker().wake_by_ref();
        Poll::Pending
    }))
}

/// 发送 LMAC 命令并等待指定 CFM ID 的响应
///
/// 与 `send_cmd` 的区别：可以指定期望的 CFM ID。
/// 用于扫描等命令（`SCANU_START_REQ` 等待 `SCANU_START_CFM_ADDTIONAL`）。
///
/// 不匹配的消息（indication）会被路由到 `bus.ind_queue`。
pub fn send_cmd_with_cfm_id(
    bus: &Arc<WifiBus>,
    msg_id: u16,
    dest_id: u16,
    param: &[u8],
    expected_cfm_id: u16,
    timeout_ms: u64,
) -> Result<Vec<u8>, CmdError> {
    // 检查总线状态
    if *bus.state.lock() == BusState::Down {
        return Err(CmdError::BusDown);
    }

    let timeout = if timeout_ms == 0 {
        CMD_TX_TIMEOUT_DEFAULT_MS
    } else {
        timeout_ms
    };

    // 构造 SDIO 帧
    let frame = build_cmd_frame(msg_id, dest_id, param);

    // 准备发送
    prepare_cmd_send(bus, expected_cfm_id);

    // 通过 TX 线程发送
    send_cmd_via_tx_thread(bus, frame);

    // 等待 CFM 响应
    let start = axhal::time::monotonic_time_nanos();
    let timeout_ns = timeout * 1_000_000;
    let result = wait_for_cfm_response(bus, expected_cfm_id, timeout_ns, start);

    // 清除期望的 CFM ID（无论成功、超时还是错误）
    bus.cmd_expected_cfm_id.store(0, Ordering::Release);

    if let Err(e) = &result {
        log::error!("[cmd_mgr] send_cmd 0x{:04x} error: {:?}", msg_id, e);
    }

    result
}

/// 发送命令不等待 CFM（用于 IND 类通知或不需要回复的消息）
pub fn send_cmd_no_cfm(
    bus: &Arc<WifiBus>,
    msg_id: u16,
    dest_id: u16,
    param: &[u8],
) -> Result<(), CmdError> {
    if *bus.state.lock() == BusState::Down {
        return Err(CmdError::BusDown);
    }

    log::info!(
        "[cmd_mgr] TX (no_cfm) msg_id=0x{:04x}, dest=0x{:04x}",
        msg_id,
        dest_id
    );

    let frame = build_cmd_frame(msg_id, dest_id, param);
    {
        let mut cmd_slot = bus.cmd_pending.lock();
        *cmd_slot = Some(frame);
        bus.cmd_pending_flag.store(true, Ordering::Release);
    }
    bus.tx_wake_pollset.wake();

    Ok(())
}

// ================================================================
// 1. TX Power Index (AIC8801 默认值)
// ================================================================

/// 发送 MM_SET_TXPWR_IDX_LVL_REQ
pub fn send_txpwr_idx_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<(), CmdError> {
    // txpwr_idx_conf_t: 10 bytes
    // [enable, dsss, ofdmlowrate_2g4, ofdm64qam_2g4, ofdm256qam_2g4,
    //  ofdm1024qam_2g4, ofdmlowrate_5g, ofdm64qam_5g, ofdm256qam_5g, ofdm1024qam_5g]
    let param: [u8; 10] = [
        1,  // enable
        9,  // dsss
        8,  // ofdmlowrate_2g4
        8,  // ofdm64qam_2g4
        8,  // ofdm256qam_2g4
        8,  // ofdm1024qam_2g4
        11, // ofdmlowrate_5g
        10, // ofdm64qam_5g
        9,  // ofdm256qam_5g
        9,  // ofdm1024qam_5g
    ];
    send_cmd(bus, MM_SET_TXPWR_IDX_LVL_REQ, TASK_MM, &param, timeout_ms)?;
    Ok(())
}

// ================================================================
// 2. TX Power Offset
// ================================================================

/// 发送 MM_SET_TXPWR_OFST_REQ
pub fn send_txpwr_ofst_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<(), CmdError> {
    // txpwr_ofst_conf_t: 8 bytes
    // [enable, chan_1_4, chan_5_9, chan_10_13, chan_36_64, chan_100_120, chan_122_140, chan_142_165]
    let param: [u8; 8] = [
        1, // enable
        0, // chan_1_4
        0, // chan_5_9
        0, // chan_10_13
        0, // chan_36_64
        0, // chan_100_120
        0, // chan_122_140
        0, // chan_142_165
    ];
    send_cmd(bus, MM_SET_TXPWR_OFST_REQ, TASK_MM, &param, timeout_ms)?;
    Ok(())
}

// ================================================================
// 3. RF Calibration
// ================================================================

/// 发送 MM_SET_RF_CALIB_REQ (AIC8801 版本，非 v2)
pub fn send_rf_calib_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<Vec<u8>, CmdError> {
    // mm_set_rf_calib_req 结构体 (AIC8801):
    // [0..4]   cal_cfg_24g   (u32 LE) = 0xbf
    // [4..8]   cal_cfg_5g    (u32 LE) = 0x3f
    // [8..12]  param_alpha   (u32 LE) = 0x0c34c008
    // [12..16] bt_calib_en   (u32 LE) = 0
    // [16..20] bt_calib_param(u32 LE) = 0x264203
    // [20]     xtal_cap      (u8)     = 0
    // [21]     xtal_cap_fine (u8)     = 0
    // 总计 22 字节

    let mut param = [0u8; 22];
    param[0..4].copy_from_slice(&0x000000bfu32.to_le_bytes()); // cal_cfg_24g
    param[4..8].copy_from_slice(&0x0000003fu32.to_le_bytes()); // cal_cfg_5g
    param[8..12].copy_from_slice(&0x0c34c008u32.to_le_bytes()); // param_alpha
    param[12..16].copy_from_slice(&0u32.to_le_bytes()); // bt_calib_en
    param[16..20].copy_from_slice(&0x00264203u32.to_le_bytes()); // bt_calib_param
    param[20] = 0; // xtal_cap
    param[21] = 0; // xtal_cap_fine

    let rsp = send_cmd(bus, MM_SET_RF_CALIB_REQ, TASK_MM, &param, timeout_ms)?;
    // CFM: mm_set_rf_calib_cfm = 4 x u32 (rxgain_24g_addr, rxgain_5g_addr, txgain_24g_addr, txgain_5g_addr)
    // if rsp.len() >= 16 {
    //     let rxgain_24g = u32::from_le_bytes([rsp[0], rsp[1], rsp[2], rsp[3]]);
    //     let txgain_24g = u32::from_le_bytes([rsp[8], rsp[9], rsp[10], rsp[11]]);
    //     log::info!("[lmac] RF calib OK: rxgain_24g=0x{:08x}, txgain_24g=0x{:08x}", rxgain_24g, txgain_24g);
    // }
    Ok(rsp)
}

// ================================================================
// 4. ME_CONFIG_REQ — 最小 HT 配置
// ================================================================

/// 发送 ME_CONFIG_REQ（最小配置：HT only, 20MHz, 1SS）
/// 对应 Linux: rwnx_send_me_config_req (line 2566-2688)
///
/// me_config_req 结构体布局:
///   struct mac_htcapability ht_cap;    // 32 bytes
///   struct mac_vhtcapability vht_cap;  // 12 bytes
///   struct mac_hecapability he_cap;    // 52 bytes (估算)
///   u16 tx_lft;
///   u8  phy_bw_max;
///   bool ht_supp;
///   bool vht_supp;
///   bool he_supp;
///   bool he_ul_on;
///   bool ps_on;
///   bool ant_div_on;
///   bool dpsm;
///
/// 最小移植策略：全部填零，只设置关键字段
pub fn send_me_config_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<Vec<u8>, CmdError> {
    // mac_htcapability:  26 bytes (2+1+16+2+4+1)
    // mac_vhtcapability: 12 bytes (4+2+2+2+2)
    // mac_hecapability:  54 bytes (6+11+12+25)
    // tail fields:       10 bytes (2+1+1+1+1+1+1+1+1)
    // 总计: 102 bytes

    const HT_CAP_SIZE: usize = 26;
    const VHT_CAP_SIZE: usize = 12;
    const HE_CAP_SIZE: usize = 54;
    const ME_CONFIG_SIZE: usize = HT_CAP_SIZE + VHT_CAP_SIZE + HE_CAP_SIZE + 10; // 102  

    let mut param = [0u8; ME_CONFIG_SIZE];

    // ht_cap (offset 0)
    param[0..2].copy_from_slice(&0x0001u16.to_le_bytes()); // ht_capa_info = LDPC  
    param[2] = 3 | (7 << 2); // a_mpdu_param  
    param[3] = 0xFF; // mcs_rate[0]  

    // vht_cap (offset 26) — 全零
    // he_cap  (offset 38) — 全零

    let tail = HT_CAP_SIZE + VHT_CAP_SIZE + HE_CAP_SIZE; // 92  
    // tx_lft (u16) = 0
    // phy_bw_max (u8) = 0
    param[tail + 2] = 0; // PHY_CHNL_BW_20  
    // ht_supp = 1
    param[tail + 3] = 1;
    // vht_supp..dpsm = 0

    log::info!("[lmac] sending ME_CONFIG_REQ (HT only, 20MHz, 1SS)");
    send_cmd(bus, ME_CONFIG_REQ, TASK_ME, &param, timeout_ms)
}

/// 发送 ME_CHAN_CONFIG_REQ（2.4GHz 信道 1-14）
///
/// me_chan_config_req 结构体:
///   mac_chan_def chan2G4[14];  // 14 * 4 bytes = 56 bytes
///   mac_chan_def chan5G[28];   // 28 * 4 bytes = 112 bytes
///   u8 chan2G4_cnt;
///   u8 chan5G_cnt;
///
/// mac_chan_def:
///   u16 freq;      // MHz
///   u8  band;      // 0 = 2.4GHz
///   u8  flags;     // 0 = enabled
///   s8  tx_power;  // dBm (Linux 用 30 dBm)
///   → 实际 5 bytes，但可能有 padding → 按 Linux 对齐
///
/// 注意：mac_chan_def 在 Linux 中是 5 字节 + padding
/// 实际布局需要与固件匹配
pub fn send_me_chan_config_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<Vec<u8>, CmdError> {
    // mac_chan_def 大小: freq(2) + band(1) + flags(1) + tx_power(1) = 5 bytes
    // 假设无 padding = 6 bytes per entry
    const MAX_2G4: usize = 14;
    const MAX_5G: usize = 28;

    // 总大小 = 14*5 + 28*5 + 1 + 1 = 70 + 140 + 2 = 212
    let total_size = MAX_2G4 * MAC_CHAN_DEF_SIZE + MAX_5G * MAC_CHAN_DEF_SIZE + 2;
    let mut param = alloc::vec![0u8; total_size];

    // 填充 chan2G4[0..14]
    let chan_cnt = CHAN_2G4_FREQS.len().min(MAX_2G4);
    for i in 0..chan_cnt {
        let off = i * MAC_CHAN_DEF_SIZE;
        param[off..off + 2].copy_from_slice(&CHAN_2G4_FREQS[i].to_le_bytes()); // freq
        param[off + 2] = 0; // band = NL80211_BAND_2GHZ
        param[off + 3] = 0; // flags = 0 (enabled)
        param[off + 4] = 30; // tx_power = 30 dBm (与 Linux 一致)
    }

    // chan5G[0..28] 全零（不使用 5GHz）

    // chan2G4_cnt 和 chan5G_cnt 在尾部
    let cnt_offset = MAX_2G4 * MAC_CHAN_DEF_SIZE + MAX_5G * MAC_CHAN_DEF_SIZE;
    param[cnt_offset] = chan_cnt as u8; // chan2G4_cnt
    param[cnt_offset + 1] = 0; // chan5G_cnt

    log::info!(
        "[lmac] sending ME_CHAN_CONFIG_REQ ({} 2.4GHz channels)",
        chan_cnt
    );
    send_cmd(bus, ME_CHAN_CONFIG_REQ, TASK_ME, &param, timeout_ms)
}

// ================================================================
// 6. MM_START_REQ — 启动 MAC
// ================================================================

/// 发送 MM_START_REQ
/// 对应 Linux: rwnx_send_start (line 473-492)
///
/// mm_start_req 结构体:
///   phy_cfg_tag phy_cfg;     // 16 * u32 = 64 bytes (全零即可)
///   u32 uapsd_timeout;       // 300 (Linux 默认)
///   u16 lp_clk_accuracy;     // 20 ppm (Linux 默认)
pub fn send_mm_start_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<Vec<u8>, CmdError> {
    // phy_cfg: 16 * 4 = 64 bytes (全零 — AIC8801 不需要 PHY 配置)
    // uapsd_timeout: 4 bytes
    // lp_clk_accuracy: 2 bytes
    // 总计 70 bytes

    let mut param = [0u8; 70];
    // phy_cfg[0..64] = 全零
    // uapsd_timeout = 300 (Linux 默认值)
    param[64..68].copy_from_slice(&300u32.to_le_bytes());
    // lp_clk_accuracy = 20 ppm
    param[68..70].copy_from_slice(&20u16.to_le_bytes());

    log::info!("[lmac] sending MM_START_REQ");
    send_cmd(bus, MM_START_REQ, TASK_MM, &param, timeout_ms)
}

/// 发送 MM_ADD_IF_REQ（创建 STA 接口）
/// 对应 Linux: rwnx_send_add_if (line 508-562)
///
/// mm_add_if_req 结构体:
///   u8 type;           // MM_STA = 0
///   mac_addr addr;     // 6 bytes (struct mac_addr { u16 array[3]; })
///   bool p2p;          // false
///
/// 返回 CFM 中的 vif_index (mm_add_if_cfm.inst_nbr)
pub fn send_mm_add_if_req(
    bus: &Arc<WifiBus>,
    mac_addr: &[u8; 6],
    timeout_ms: u64,
) -> Result<u8, CmdError> {
    let mut param = [0u8; 10]; // sizeof(mm_add_if_req) with alignment = 10  
    param[0] = MM_STA; // type at offset 0  
    // param[1] = 0 (padding for mac_addr alignment)
    param[2..8].copy_from_slice(mac_addr); // addr at offset 2  
    param[8] = 0; // p2p at offset 8  
    // param[9] = 0 (trailing padding)

    let rsp = send_cmd(bus, MM_ADD_IF_REQ, TASK_MM, &param, timeout_ms)?;

    // mm_add_if_cfm:
    //   u8 status;     // 0 = success
    //   u8 inst_nbr;   // VIF index
    if rsp.len() >= 2 {
        let status = rsp[0];
        let vif_idx = rsp[1];
        if status != 0 {
            log::error!("[lmac] MM_ADD_IF_CFM status={} (error)", status);
            return Err(CmdError::FirmwareError);
        }
        log::info!("[lmac] MM_ADD_IF_CFM OK: vif_index={}", vif_idx);
        Ok(vif_idx)
    } else {
        log::error!("[lmac] MM_ADD_IF_CFM too short: {} bytes", rsp.len());
        Err(CmdError::InvalidResponse)
    }
}

// ================================================================
// 扫描命令
// ================================================================

/// 发送 SCANU_START_REQ（WiFi 扫描）  
///  
/// 扫描流程：  
///   1. 发送 SCANU_START_REQ  
///   2. 固件返回 N 个 SCANU_RESULT_IND（每个 AP 一个）→ 路由到 ind_queue  
///   3. 固件返回 SCANU_START_CFM_ADDTIONAL（扫描完成）→ 作为 CFM 返回  
///  
/// 参数：  
///   - `vif_idx`: VIF 索引（从 MM_ADD_IF_REQ 获得）  
///   - `ssid`: 可选的目标 SSID（None = 被动扫描/广播扫描）  
///   - `timeout_ms`: 超时时间（建议 15000-20000ms）  
///  
/// 返回 SCANU_START_CFM 的 param（3 字节: vif_idx, status, result_cnt）
pub fn send_scanu_start_req(
    bus: &Arc<WifiBus>,
    vif_idx: u8,
    ssid: Option<&[u8]>,
    timeout_ms: u64,
) -> Result<Vec<u8>, CmdError> {
    // 构造 scanu_start_req param
    // 布局:
    //   chan[SCAN_CHANNEL_MAX] * MAC_CHAN_DEF_SIZE  = 42 * 5 = 210
    //   ssid[SCAN_SSID_MAX] * MAC_SSID_SIZE        = 3 * 33 = 99
    //   bssid                                       = 6
    //   add_ies (u32)                               = 4
    //   add_ie_len (u16)                            = 2
    //   vif_idx (u8)                                = 1
    //   chan_cnt (u8)                                = 1
    //   ssid_cnt (u8)                               = 1
    //   no_cck (bool/u8)                            = 1
    //   duration (u32)                              = 4
    //   总计                                        = 329
    let mut param = vec![0u8; SCANU_START_REQ_SIZE];

    // ---- 填充 chan[0..14]（2.4GHz 信道 1-14）----
    let chan_cnt = CHAN_2G4_FREQS.len().min(SCAN_CHANNEL_MAX);
    for i in 0..chan_cnt {
        let off = i * MAC_CHAN_DEF_SIZE;
        param[off..off + 2].copy_from_slice(&CHAN_2G4_FREQS[i].to_le_bytes()); // freq  
        param[off + 2] = 0; // band = NL80211_BAND_2GHZ  
        param[off + 3] = 0; // flags = 0  
        param[off + 4] = 30; // tx_power = 30 dBm  
    }

    // ---- 填充 ssid[0]（如果指定）----
    let ssid_offset = SCAN_CHANNEL_MAX * MAC_CHAN_DEF_SIZE;
    let ssid_cnt = if let Some(s) = ssid {
        let len = s.len().min(MAC_SSID_LEN);
        param[ssid_offset] = len as u8; //ssid[0].length
        param[ssid_offset + 1..ssid_offset + 1 + len].copy_from_slice(&s[..len]); // ssid[0].array  
        1u8
    } else {
        0u8
    };

    // ---- 填充 bssid（广播地址 FF:FF:FF:FF:FF:FF）----
    let bssid_offset = ssid_offset + SCAN_SSID_MAX * MAC_SSID_SIZE + 1; // 252 + 99 + 1 = 352  
    param[bssid_offset..bssid_offset + 6].copy_from_slice(&[0xFF; 6]);

    // ---- 填充尾部字段 ----
    let tail_offset = bssid_offset + MAC_ADDR_SIZE + 2; // 352 + 6 + 2
    // add_ies (u32) = 0
    // add_ie_len (u16) = 0
    param[tail_offset + 6] = vif_idx; // vif_idx  
    param[tail_offset + 7] = chan_cnt as u8; // chan_cnt  
    param[tail_offset + 8] = ssid_cnt; // ssid_cnt  
    param[tail_offset + 9] = 0;
    // duration (u32) = 0（使用固件默认值）

    log::info!(
        "[cmd_mgr] sending SCANU_START_REQ: vif_idx={}, chan_cnt={}, ssid_cnt={}, param_size={}",
        vif_idx,
        chan_cnt,
        ssid_cnt,
        param.len()
    );

    // 等待 SCANU_START_CFM_ADDTIONAL (0x1009)，而非 SCANU_START_CFM (0x1001)
    send_cmd_with_cfm_id(
        bus,
        SCANU_START_REQ, // 0x1000
        TASK_SCANU,
        &param,
        SCANU_START_CFM_ADDTIONAL, // 0x1009
        timeout_ms,
    )
}

// ===== 扫描结果收集辅助函数 =====

/// 检查是否为扫描相关的消息 ID
fn is_scan_related_message(msg_id: u16) -> bool {
    msg_id == SCANU_RESULT_IND || msg_id == SCANU_START_CFM || msg_id == SCANU_START_CFM_ADDTIONAL
}

/// 在 ind_queue 中查找并移除扫描相关消息
///
/// # 返回
/// Option<(msg_data, msg_id)> - 找到的消息数据和 ID，None 表示未找到
fn find_scan_message_in_queue(bus: &Arc<WifiBus>) -> Option<(Vec<u8>, u16)> {
    let mut queue = bus.ind_queue.lock();
    for i in 0..queue.len() {
        if queue[i].len() < LmacMsg::SIZE {
            continue;
        }

        let msg = LmacMsg::from_le_bytes(&queue[i]);
        if is_scan_related_message(msg.id) {
            let msg_data = queue.remove(i).unwrap();
            return Some((msg_data, msg.id));
        }
    }
    None
}

/// 处理扫描消息
///
/// # 返回
/// (should_continue, scan_result) - 是否继续收集、可选的扫描结果
fn process_scan_message(msg_data: &[u8], msg_id: u16) -> (bool, Option<ScanResult>) {
    let msg = LmacMsg::from_le_bytes(msg_data);

    if msg.id == SCANU_START_CFM {
        log::info!("[collect] SCANU_START_CFM received, scan truly complete");
        return (false, None); // 停止收集
    }

    if msg.id == SCANU_START_CFM_ADDTIONAL {
        log::info!("[collect] SCANU_START_CFM_ADDTIONAL in ind_queue, skipping");
        return (true, None); // 继续收集
    }

    // SCANU_RESULT_IND
    let param = &msg_data[LmacMsg::SIZE..];
    if let Some(result) = parse_scanu_result_ind(param) {
        (true, Some(result))
    } else {
        (true, None)
    }
}

/// 合并两个扫描结果（保留 RSSI 更强的）
fn merge_scan_result_by_rssi(existing: &mut ScanResult, new: &ScanResult) {
    let new_freq = new.center_freq;
    let new_rssi = new.rssi;

    if new_rssi > existing.rssi {
        // 新结果的 RSSI 更强，替换现有结果
        let old_freq = existing.center_freq;
        let old_rsn = if existing.rsn_ie.is_empty() {
            existing.rsn_ie.clone()
        } else {
            Vec::new()
        };

        *existing = *new;

        // 保留旧频率如果新频率为0
        if existing.center_freq == 0 && old_freq != 0 {
            existing.center_freq = old_freq;
        }

        // 保留旧 RSN IE 如果新结果没有
        if existing.rsn_ie.is_empty() && !old_rsn.is_empty() {
            existing.rsn_ie = old_rsn;
        }
    } else {
        // 现有结果的 RSSI 更强或相等，只更新频率
        if existing.center_freq == 0 && new_freq != 0 {
            existing.center_freq = new_freq;
        }

        // 如果现有条目没有 RSN IE 但新条目有，用新的
        if existing.rsn_ie.is_empty() && !new.rsn_ie.is_empty() {
            existing.rsn_ie = new.rsn_ie.clone();
        }
    }
}

/// 对扫描结果进行去重（按 BSSID，保留 RSSI 最强的）
fn dedup_scan_results(results: Vec<ScanResult>) -> Vec<ScanResult> {
    let before_dedup = results.len();
    let mut deduped: Vec<ScanResult> = Vec::new();

    for r in results {
        if let Some(existing) = deduped.iter_mut().find(|e| e.bssid == r.bssid) {
            merge_scan_result_by_rssi(existing, &r);
        } else {
            deduped.push(r);
        }
    }

    if before_dedup != deduped.len() {
        log::info!(
            "[collect] deduplicated: {} -> {} unique APs",
            before_dedup,
            deduped.len()
        );
    }

    deduped
}

/// 从 ind_queue 中收集所有 SCANU_RESULT_IND 并解析为 ScanResult
///
/// 在 `send_scanu_start_req` 返回后调用。
/// 扫描期间固件发送的 SCANU_RESULT_IND 已被路由到 ind_queue。
pub fn collect_scan_results(bus: &Arc<WifiBus>, timeout_ms: u64) -> Vec<ScanResult> {
    let mut results = Vec::new();
    let deadline = current_time_ms() + timeout_ms;

    loop {
        let now = current_time_ms();
        if now >= deadline {
            log::warn!("[collect] scan collection timed out after {}ms", timeout_ms);
            break;
        }

        // 从 ind_queue 中取出扫描相关消息
        match find_scan_message_in_queue(bus) {
            Some((msg_data, msg_id)) => {
                let (should_continue, scan_result) = process_scan_message(&msg_data, msg_id);

                if !should_continue {
                    break;
                }

                if let Some(result) = scan_result {
                    results.push(result);
                }
            }
            None => {
                // 队列中没有扫描相关消息，短暂等待
                axtask::yield_now();
            }
        }
    }

    // 按 BSSID 去重，保留 RSSI 最强的
    let deduped = dedup_scan_results(results);

    log::info!("[collect] total {} APs found", deduped.len());
    deduped
}

fn channel_to_freq(channel: u8) -> u16 {
    match channel {
        1..=13 => 2407 + (channel as u16) * 5,
        14 => 2484,
        36..=177 => 5000 + (channel as u16) * 5,
        _ => 0,
    }
}

/// 解析 SCANU_RESULT_IND 的 param 部分  
///  
/// param 布局（scanu_result_ind）:  
///   [0..2]  length (u16)     — 802.11 帧长度  
///   [2..4]  framectrl (u16)  — Frame Control  
///   [4..6]  center_freq (u16)  
///   [6]     band (u8)  
///   [7]     sta_idx (u8)  
///   [8]     inst_nbr (u8)  
///   [9]     rssi (i8)  
///   [10..]  payload — 802.11 管理帧（Beacon/ProbeResp）
fn parse_scanu_result_ind(param: &[u8]) -> Option<ScanResult> {
    if param.len() < 12 {
        log::warn!("[parse] SCANU_RESULT_IND too short: {} bytes", param.len());
        return None;
    }

    let length = u16::from_le_bytes([param[0], param[1]]);
    let framectrl = u16::from_le_bytes([param[2], param[3]]);
    let mut center_freq = u16::from_le_bytes([param[4], param[5]]);
    let band = param[6];
    let _sta_idx = param[7];
    let _inst_nbr = param[8];
    let rssi = param[9] as i8;
    let payload = &param[12..];

    // payload 是 802.11 管理帧（Beacon/ProbeResp）
    // 802.11 管理帧头部：
    //   [0..2]   Frame Control (已在 framectrl 中)
    //   [2..4]   Duration
    //   [4..10]  DA (destination address)
    //   [10..16] SA (source address) = BSSID for beacon
    //   [16..22] BSSID
    //   [22..24] Sequence Control
    //   [24..32] Timestamp (8 bytes)
    //   [32..34] Beacon Interval
    //   [34..36] Capability Info
    //   [36..]   Information Elements

    if payload.len() < 36 {
        log::warn!(
            "[parse] payload too short for 802.11 header: {}",
            payload.len()
        );
        return None;
    }

    let mut bssid = [0u8; 6];
    bssid.copy_from_slice(&payload[16..22]);

    let beacon_interval = u16::from_le_bytes([payload[32], payload[33]]);
    let capability = u16::from_le_bytes([payload[34], payload[35]]);

    // 解析 SSID IE (Element ID = 0)
    let ie_data = &payload[36..];
    let mut ssid = [0u8; MAC_SSID_LEN];
    let mut ssid_len: u8 = 0;
    let mut ds_channel: u8 = 0; // DS Parameter Set 中的 channel  
    let mut rsn_ie = Vec::new();

    let mut ie_offset = 0;
    while ie_offset + 2 <= ie_data.len() {
        let ie_id = ie_data[ie_offset];
        let ie_len = ie_data[ie_offset + 1] as usize;

        if ie_offset + 2 + ie_len > ie_data.len() {
            break;
        }

        match ie_id {
            0 => {
                ssid_len = ie_len.min(MAC_SSID_LEN) as u8;
                ssid[..ssid_len as usize]
                    .copy_from_slice(&ie_data[ie_offset + 2..ie_offset + 2 + ssid_len as usize]);
            }
            3 => {
                if ie_len >= 1 {
                    ds_channel = ie_data[ie_offset + 2];
                }
            }
            0x30 => {
                rsn_ie = ie_data[ie_offset..ie_offset + 2 + ie_len].to_vec();
            }
            _ => {}
        }
        ie_offset += 2 + ie_len;
    }

    if center_freq == 0 && ds_channel != 0 {
        center_freq = channel_to_freq(ds_channel);
        log::info!(
            "[parse] center_freq was 0, fallback from DS channel {} -> {} MHz",
            ds_channel,
            center_freq
        );
    }

    log::info!(
        "[parse] AP: ssid=\"{}\", bssid={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}, freq={}, rssi={}",
        core::str::from_utf8(&ssid[..ssid_len as usize]).unwrap_or("<invalid>"),
        bssid[0],
        bssid[1],
        bssid[2],
        bssid[3],
        bssid[4],
        bssid[5],
        center_freq,
        rssi
    );

    Some(ScanResult {
        ssid,
        ssid_len,
        bssid,
        center_freq,
        rssi,
        capability,
        beacon_interval,
        raw_payload: payload.to_vec(),
        rsn_ie,
    })
}

// ===== Indication 等待辅助函数 =====

/// 检查超时并记录队列状态
fn check_timeout_and_log_queue(bus: &Arc<WifiBus>, target_msg_id: u16, deadline: u64) -> bool {
    if axhal::time::monotonic_time_nanos() >= deadline {
        let queue = bus.ind_queue.lock();
        log::error!(
            "[wait_ind] TIMEOUT waiting for msg_id=0x{:04x}, ind_queue has {} messages:",
            target_msg_id,
            queue.len()
        );
        for (i, msg_data) in queue.iter().enumerate() {
            if msg_data.len() >= LmacMsg::SIZE {
                let msg = LmacMsg::from_le_bytes(msg_data);
                log::error!(
                    "[wait_ind]   [{}] msg_id=0x{:04x}, param_len={}",
                    i,
                    msg.id,
                    msg.param_len
                );
            } else {
                log::error!(
                    "[wait_ind]   [{}] raw_len={} (too short)",
                    i,
                    msg_data.len()
                );
            }
        }
        return true; // 超时
    }
    false
}

/// 提取消息参数部分
fn extract_message_param(raw: &[u8]) -> Vec<u8> {
    let param_start = LmacMsg::SIZE;
    if raw.len() > param_start {
        raw[param_start..].to_vec()
    } else {
        Vec::new()
    }
}

/// 获取消息参数部分的引用
fn extract_message_param_ref(raw: &[u8]) -> &[u8] {
    let param_start = LmacMsg::SIZE;
    if raw.len() > param_start {
        &raw[param_start..]
    } else {
        &[]
    }
}

/// 处理 abort 消息并记录日志
fn handle_abort_message(msg_id: u16, param: &[u8]) {
    match msg_id {
        SM_DISCONNECT_IND => {
            let reason = if param.len() >= 4 {
                u16::from_le_bytes([param[2], param[3]])
            } else {
                0xFFFF
            };
            log::error!(
                "[wait_ind] SM_DISCONNECT_IND received! reason_code={}, param={:02x?}",
                reason,
                &param[..param.len().min(16)]
            );
        }
        SM_EXTERNAL_AUTH_REQUIRED_IND => {
            log::error!(
                "[wait_ind] SM_EXTERNAL_AUTH_REQUIRED_IND received! AP requires SAE/WPA3 external auth, param={:02x?}",
                &param[..param.len().min(48)]
            );
        }
        _ => {
            log::error!(
                "[wait_ind] abort msg_id=0x{:04x} received, param={:02x?}",
                msg_id,
                &param[..param.len().min(16)]
            );
        }
    }
}

/// 在队列中查找并移除目标或 abort 消息
fn try_find_message_in_queue(
    bus: &Arc<WifiBus>,
    target_msg_id: u16,
    abort_ids: &[u16],
) -> Option<Result<Vec<u8>, CmdError>> {
    let mut queue = bus.ind_queue.lock();
    for i in 0..queue.len() {
        if queue[i].len() < LmacMsg::SIZE {
            continue;
        }

        let msg = LmacMsg::from_le_bytes(&queue[i]);

        // 检查是否为目标消息
        if msg.id == target_msg_id {
            let raw = queue.remove(i).unwrap();
            let param = extract_message_param(&raw);
            return Some(Ok(param));
        }

        // 检查是否为 abort 消息
        if abort_ids.contains(&msg.id) {
            let raw = queue.remove(i).unwrap();
            let param = extract_message_param_ref(&raw);
            handle_abort_message(msg.id, param);
            return Some(Err(CmdError::FirmwareError));
        }
    }

    None
}

/// 从 ind_queue 中等待指定 msg_id 的 indication
/// 超时返回 Err(CmdError::Timeout)
pub fn wait_for_indication(
    bus: &Arc<WifiBus>,
    target_msg_id: u16,
    abort_msg_ids: &[u16],
    timeout_ms: u64,
) -> Result<Vec<u8>, CmdError> {
    let deadline = axhal::time::monotonic_time_nanos() + timeout_ms as u64 * 1_000_000;

    let result = block_on(poll_fn(|cx| {
        // 超时检查
        if check_timeout_and_log_queue(bus, target_msg_id, deadline) {
            return Poll::Ready(Err(CmdError::Timeout));
        }

        // 在 ind_queue 中查找目标 msg_id 或 abort msg_id
        if let Some(result) = try_find_message_in_queue(bus, target_msg_id, abort_msg_ids) {
            return Poll::Ready(result);
        }

        // 注册 waker，等待 ind_pollset 通知
        bus.ind_pollset.register(cx.waker());

        // 注册后再检查一次（防止 race）
        if let Some(result) = try_find_message_in_queue(bus, target_msg_id, abort_msg_ids) {
            return Poll::Ready(result);
        }

        // 保持活跃（与 rx_thread 相同策略）
        cx.waker().wake_by_ref();
        Poll::Pending
    }));

    result
}

/// 发送 SM_CONNECT_REQ（连接到 AP）
/// sm_connect_req 结构体布局（含 C padding）:  
///   [0..33]   mac_ssid ssid        (1+32)  
///   [33]      padding              (1 byte, align mac_addr to u16)  
///   [34..40]  mac_addr bssid       (6)  
///   [40..45]  mac_chan_def chan     (5)  
///   [45..48]  padding              (3 bytes, align u32 flags)  
///   [48..52]  u32 flags  
///   [52..54]  u16 ctrl_port_ethertype  
///   [54..56]  u16 ie_len  
///   [56..58]  u16 listen_interval  
///   [58]      bool dont_wait_bcmc  
///   [59]      u8 auth_type  
///   [60]      u8 uapsd_queues  
///   [61]      u8 vif_idx  
///   [62..64]  padding              (2 bytes, align u32 ie_buf)  
///   [64..320] u32 ie_buf[64]       (256)  
///   总计: 320 bytes  
///  
/// 返回 SM_CONNECT_CFM 的 param（1 字节: status）  
pub fn send_sm_connect_req(
    bus: &Arc<WifiBus>,
    vif_idx: u8,
    ssid: &[u8],
    bssid: &[u8; 6],
    channel_freq: u16,
    flags: u32,
    auth_type: u8,
    ie: &[u8], // RSN IE 等附加 IE
    timeout_ms: u64,
) -> Result<Vec<u8>, CmdError> {
    const SM_CONNECT_REQ_SIZE: usize = 320;

    let mut param = vec![0u8; SM_CONNECT_REQ_SIZE];

    // ssid [0..33]
    let ssid_len = ssid.len().min(MAC_SSID_LEN);
    param[0] = ssid_len as u8;
    param[1..1 + ssid_len].copy_from_slice(&ssid[..ssid_len]);

    // bssid [34..40] (1 byte padding after ssid)
    param[34..40].copy_from_slice(bssid);

    // chan [40..45]
    if channel_freq != 0 && channel_freq != 0xFFFF {
        param[40..42].copy_from_slice(&channel_freq.to_le_bytes()); // freq  
        param[42] = 0; // band = 2.4GHz  
        param[43] = 0; // flags  
        param[44] = 30; // tx_power  
    } else {
        // 不指定信道：freq = 0xFFFF
        param[40..42].copy_from_slice(&0xFFFFu16.to_le_bytes());
    }

    // flags [48..52]
    param[48..52].copy_from_slice(&flags.to_le_bytes());

    // ctrl_port_ethertype [52..54] = ETH_P_PAE (0x888E) in network byte order
    param[52..54].copy_from_slice(&ETH_P_PAE.to_be_bytes());

    // ie_len [54..56]
    let ie_len = ie.len().min(256);
    param[54..56].copy_from_slice(&(ie_len as u16).to_le_bytes());

    // listen_interval [56..58] = 1
    param[56..58].copy_from_slice(&1u16.to_le_bytes());

    // dont_wait_bcmc [58] = 0 (wait for BC/MC)
    param[58] = 0;

    // auth_type [59]
    param[59] = auth_type;

    // uapsd_queues [60] = 0
    param[60] = 0;

    // vif_idx [61]
    param[61] = vif_idx;

    // ie_buf [64..64+ie_len]
    if ie_len > 0 {
        param[64..64 + ie_len].copy_from_slice(&ie[..ie_len]);
    }

    // 诊断：打印关键字段的原始字节，确认布局正确
    log::info!(
        "[cmd_mgr] SM_CONNECT_REQ raw: offset48..56={:02x?}, offset56..64={:02x?}",
        &param[48..56], // flags + ethertype + ie_len
        &param[56..64], // listen_interval + dont_wait_bcmc + auth + uapsd + vif + padding
    );

    log::info!(
        "[cmd_mgr] sending SM_CONNECT_REQ: vif={}, ssid_len={}, auth={}, flags=0x{:08x}, ie_len={}",
        vif_idx,
        ssid_len,
        auth_type,
        flags,
        ie_len
    );

    // Linux 驱动等待 SM_CONNECT_CFM (msg_id + 1)
    send_cmd(bus, SM_CONNECT_REQ, TASK_SM, &param, timeout_ms)
}

/// 发送 SM_DISCONNECT_REQ  
/// 对应 Linux: rwnx_send_sm_disconnect_req (rwnx_msg_tx.c:3239-3258)  
///  
/// sm_disconnect_req:  
///   u16 reason_code;  [0..2]  
///   u8  vif_idx;      [2]  
///   总计: 3 bytes  
pub fn send_sm_disconnect_req(
    bus: &Arc<WifiBus>,
    vif_idx: u8,
    reason_code: u16,
    timeout_ms: u64,
) -> Result<Vec<u8>, CmdError> {
    let mut param = [0u8; 3];
    param[0..2].copy_from_slice(&reason_code.to_le_bytes());
    param[2] = vif_idx;

    log::info!(
        "[cmd_mgr] sending SM_DISCONNECT_REQ: vif={}, reason={}",
        vif_idx,
        reason_code
    );

    send_cmd(bus, SM_DISCONNECT_REQ, TASK_SM, &param, timeout_ms)
}

/// 发送 MM_KEY_ADD_REQ（安装加密密钥）  
/// 对应 Linux: rwnx_send_key_add (rwnx_msg_tx.c:641-679)  
///  
/// mm_key_add_req 结构体:  
///   u8  key_idx;       [0]  
///   u8  sta_idx;       [1]  
///   mac_sec_key key;   [2..35]  (u8 length + u32 array[8] → 1+3padding+32=36? 或 1+32=33?)  
///   u8  cipher_suite;  
///   u8  inst_nbr;  
///   u8  spp;  
///   bool pairwise;  
///  
/// mac_sec_key: { u8 length; u32 array[8]; }  
///   C layout: length at [0], padding [1..4], array at [4..36] → 总 36 bytes  
///   或者 length at [0], array at [4..36] (u32 alignment) → 总 36 bytes  
///  
/// mm_key_add_req 总大小:  
///   key_idx(1) + sta_idx(1) + padding(2) + mac_sec_key(36) + cipher(1) + inst_nbr(1) + spp(1) + pairwise(1)  
///   = 44 bytes  
pub fn send_key_add_req(
    bus: &Arc<WifiBus>,
    vif_idx: u8,
    sta_idx: u8,
    pairwise: bool,
    key: &[u8],
    key_idx: u8,
    cipher_suite: u8,
    timeout_ms: u64,
) -> Result<u8, CmdError> {
    const MM_KEY_ADD_REQ_SIZE: usize = 44;

    let mut param = [0u8; MM_KEY_ADD_REQ_SIZE];

    // key_idx [0]
    param[0] = key_idx;
    // sta_idx [1]
    param[1] = sta_idx;
    // padding [2..4]

    // mac_sec_key [4..40]:
    //   length [4]
    //   padding [5..8]
    //   array [8..40] (u32 array[8], 32 bytes)
    let key_len = key.len().min(MAC_SEC_KEY_LEN);
    param[4] = key_len as u8;
    // 密钥数据写入 array 字段（offset 8）
    // 注意：Linux 驱动用 memcpy(&key.array[0], key, key_len)
    // array 是 u32[]，但 memcpy 按字节拷贝，所以直接拷贝即可
    param[8..8 + key_len].copy_from_slice(&key[..key_len]);

    // cipher_suite [40]
    param[40] = cipher_suite;
    // inst_nbr [41]
    param[41] = vif_idx;
    // spp [42]
    param[42] = 0;
    // pairwise [43]
    param[43] = if pairwise { 1 } else { 0 };

    log::info!(
        "[cmd_mgr] sending MM_KEY_ADD_REQ: sta={}, key_idx={}, cipher={}, pairwise={}, key_len={}",
        sta_idx,
        key_idx,
        cipher_suite,
        pairwise,
        key_len
    );

    let rsp = send_cmd(bus, MM_KEY_ADD_REQ, TASK_MM, &param, timeout_ms)?;

    // mm_key_add_cfm: status(u8) + hw_key_idx(u8) + aligned[2]
    if rsp.len() >= 2 {
        let status = rsp[0];
        let hw_key_idx = rsp[1];
        if status != 0 {
            log::error!("[cmd_mgr] MM_KEY_ADD_CFM status={} (error)", status);
            return Err(CmdError::FirmwareError);
        }
        log::info!("[cmd_mgr] MM_KEY_ADD_CFM OK: hw_key_idx={}", hw_key_idx);
        Ok(hw_key_idx)
    } else {
        log::error!("[cmd_mgr] MM_KEY_ADD_CFM too short: {} bytes", rsp.len());
        Err(CmdError::InvalidResponse)
    }
}

/// 发送 MM_KEY_DEL_REQ  
/// 对应 Linux: rwnx_send_key_del (rwnx_msg_tx.c:682-698)  
///  
/// mm_key_del_req: { u8 hw_key_idx; } → 1 byte  
pub fn send_key_del_req(
    bus: &Arc<WifiBus>,
    hw_key_idx: u8,
    timeout_ms: u64,
) -> Result<Vec<u8>, CmdError> {
    let param = [hw_key_idx];

    log::info!(
        "[cmd_mgr] sending MM_KEY_DEL_REQ: hw_key_idx={}",
        hw_key_idx
    );

    send_cmd(bus, MM_KEY_DEL_REQ, TASK_MM, &param, timeout_ms)
}

/// 发送 ME_SET_CONTROL_PORT_REQ  
/// 对应 Linux: rwnx_send_me_set_control_port_req (rwnx_msg_tx.c:2779-2797)  
///  
/// me_set_control_port_req:  
///   u8   sta_idx;            [0]  
///   bool control_port_open;  [1]  
///   总计: 2 bytes  
pub fn send_set_control_port_req(
    bus: &Arc<WifiBus>,
    sta_idx: u8,
    open: bool,
    timeout_ms: u64,
) -> Result<Vec<u8>, CmdError> {
    let param = [sta_idx, if open { 1 } else { 0 }];

    log::info!(
        "[cmd_mgr] sending ME_SET_CONTROL_PORT_REQ: sta_idx={}, open={}",
        sta_idx,
        open
    );

    send_cmd(bus, ME_SET_CONTROL_PORT_REQ, TASK_ME, &param, timeout_ms)
}

/// 发送 MM_SET_FILTER_REQ，配置固件 RX 过滤器  
///  
/// filter 是一个 u32 位掩码，控制固件转发哪些帧给主机。  
/// Linux 驱动中 STA 模式的典型值：  
///   NXMAC_ACCEPT_UNICAST_BIT    (1 << 0) = 0x01  
///   NXMAC_ACCEPT_MULTICAST_BIT  (1 << 1) = 0x02  
///   NXMAC_ACCEPT_BROADCAST_BIT  (1 << 2) = 0x04  
///   NXMAC_ACCEPT_PROBE_REQ_BIT  (1 << 3) = 0x08 (AP 模式用)  
///  
/// STA 模式推荐值：0x07 (unicast + multicast + broadcast)  
pub fn send_mm_set_filter_req(
    bus: &Arc<WifiBus>,
    filter: u32,
    timeout_ms: u64,
) -> Result<(), CmdError> {
    // mm_set_filter_req 结构体：
    //   [0..4] u32 filter (LE)
    let mut param = [0u8; 4];
    param[0..4].copy_from_slice(&filter.to_le_bytes());

    log::info!(
        "[cmd_mgr] sending MM_SET_FILTER_REQ: filter=0x{:08x}",
        filter
    );
    send_cmd(bus, MM_SET_FILTER_REQ, TASK_MM, &param, timeout_ms)?;
    log::info!("[cmd_mgr] MM_SET_FILTER_CFM OK");
    Ok(())
}

/// 发送 MM_SET_IDLE_REQ  
/// 对应 Linux: rwnx_send_set_idle (rwnx_msg_tx.c)  
///  
/// mm_set_idle_req:  
///   u8 hw_idle;  [0]   0 = active, 1 = idle  
///   总计: 1 byte  
pub fn send_mm_set_idle_req(
    bus: &Arc<WifiBus>,
    idle: bool,
    timeout_ms: u64,
) -> Result<Vec<u8>, CmdError> {
    let param = [if idle { 1u8 } else { 0u8 }];

    log::info!("[cmd_mgr] sending MM_SET_IDLE_REQ: idle={}", idle);

    send_cmd(bus, MM_SET_IDLE_REQ, TASK_MM, &param, timeout_ms)
}

/// MM_GET_MAC_ADDR_REQ / MM_GET_MAC_ADDR_CFM  
/// 参考 Linux: rwnx_send_get_macaddr_req (rwnx_msg_tx.c:1334-1355)  
///  
/// mm_get_mac_addr_req: u32 get = 1;  (4 bytes)  
/// mm_get_mac_addr_cfm: u8 mac_addr[6];  (6 bytes)  
pub fn send_get_mac_addr_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<[u8; 6], CmdError> {
    let param = 1u32.to_le_bytes(); // get = 1 

    log::info!("[cmd_mgr] sending MM_GET_MAC_ADDR_REQ");

    let rsp = send_cmd(bus, MM_GET_MAC_ADDR_REQ, TASK_MM, &param, timeout_ms)?;

    if rsp.len() >= 6 {
        let mut mac = [0u8; 6];
        mac.copy_from_slice(&rsp[..6]);
        log::info!(
            "[cmd_mgr] MAC: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            mac[0],
            mac[1],
            mac[2],
            mac[3],
            mac[4],
            mac[5]
        );
        Ok(mac)
    } else {
        log::error!(
            "[cmd_mgr] MM_GET_MAC_ADDR_CFM too short: {} bytes",
            rsp.len()
        );
        Err(CmdError::InvalidResponse)
    }
}

/// 等待 EAPOL 帧（从 eapol_queue 中取出）    
///    
/// 返回完整的 EAPOL 帧（从 802.1X Version 字段开始，不含 Ethernet 头）    
pub fn wait_for_eapol(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<Vec<u8>, CmdError> {
    let timeout = Some(Duration::from_millis(timeout_ms));

    let fut = poll_fn(|cx| {
        // 尝试从 eapol_queue 取出
        {
            let mut queue = bus.eapol_queue.lock();
            if let Some(eapol) = queue.pop_front() {
                // 清除剩余的过期帧（AP 重传的旧 M1 等）
                queue.clear();
                log::info!("[cmd_mgr] EAPOL frame received: {} bytes", eapol.len());
                return Poll::Ready(eapol);
            }
        }

        // 注册 waker，等待 RX 线程通过 eapol_pollset.wake() 唤醒
        bus.eapol_pollset.register(cx.waker());

        // 双重检查：防止 register 之前 RX 线程已 push + wake
        {
            let mut queue = bus.eapol_queue.lock();
            if let Some(eapol) = queue.pop_front() {
                queue.clear();
                return Poll::Ready(eapol);
            }
        }

        // 不调用 cx.waker().wake_by_ref()！
        // 依赖 eapol_pollset.wake()（来自 RX 线程）唤醒本任务，
        // 超时由 future::timeout 处理。
        Poll::Pending
    });

    match block_on(future::timeout(timeout, fut)) {
        Ok(eapol) => Ok(eapol),
        Err(_timeout) => {
            log::error!("[cmd_mgr] wait_for_eapol timeout ({}ms)", timeout_ms);
            Err(CmdError::Timeout)
        }
    }
}

// ===== EAPOL 帧发送辅助函数 =====

/// EAPOL 帧布局信息
struct EapolFrameLayout {
    sdio_hdr_len: usize,
    final_len: usize,
}

/// 计算 EAPOL 帧的布局和对齐
fn calculate_eapol_frame_layout(eapol_len: usize) -> EapolFrameLayout {
    const SDIO_HEADER_LEN: usize = 4;
    const HOSTDESC_SIZE: usize = 28;
    const TX_ALIGNMENT: usize = 4;

    // SDIO payload = hostdesc + payload
    let sdio_payload_len = HOSTDESC_SIZE + eapol_len;
    let raw_len = SDIO_HEADER_LEN + sdio_payload_len;

    // 对齐到 TX_ALIGNMENT
    let aligned = (raw_len + TX_ALIGNMENT - 1) & !(TX_ALIGNMENT - 1);

    // SDIO header 长度字段
    let sdio_hdr_len = aligned - SDIO_HEADER_LEN;

    // 对齐到 SDIOWIFI_FUNC_BLOCKSIZE
    let final_len = if aligned % SDIOWIFI_FUNC_BLOCKSIZE != 0 {
        let with_tail = aligned + 4; // TAIL_LEN
        ((with_tail / SDIOWIFI_FUNC_BLOCKSIZE) + 1) * SDIOWIFI_FUNC_BLOCKSIZE
    } else {
        aligned
    };

    EapolFrameLayout {
        sdio_hdr_len,
        final_len,
    }
}

/// 填充 SDIO header（EAPOL 帧）
fn fill_sdio_header_for_eapol(buf: &mut [u8], layout: &EapolFrameLayout) {
    buf[0] = (layout.sdio_hdr_len & 0xFF) as u8;
    buf[1] = ((layout.sdio_hdr_len >> 8) & 0x0F) as u8;
    buf[2] = 0x01; // SDIO_TYPE_DATA = 0x00, 但 Linux 用 0x01 for data
    buf[3] = 0x00;
}

/// 填充 HostDesc（EAPOL 帧）
fn fill_hostdesc_for_eapol(
    hd: &mut [u8],
    payload_len: usize,
    dst_mac: &[u8; 6],
    src_mac: &[u8; 6],
    vif_idx: u8,
    sta_idx: u8,
) {
    // packet_len = ethernet frame length
    hd[0..2].copy_from_slice(&(payload_len as u16).to_le_bytes());
    // flags_ext = 0 (already zeroed)

    // hostid: 设置 bit 31 表示需要 TX 确认
    hd[4..8].copy_from_slice(&0x8000_0000u32.to_le_bytes());

    // eth_dest_addr [8..14]
    hd[8..14].copy_from_slice(dst_mac);
    // eth_src_addr [14..20]
    hd[14..20].copy_from_slice(src_mac);
    // ethertype [20..22] = 0x888E (网络字节序)
    hd[20..22].copy_from_slice(&0x888Eu16.to_be_bytes());
    hd[22] = 3;
    hd[23] = 7;
    // vif_idx
    hd[24] = vif_idx;
    // staid
    hd[25] = sta_idx;
    // flags [26..28] = 0 (already zeroed)
}

/// 构造 EAPOL 帧缓冲区
fn build_eapol_frame_buffer(
    dst_mac: &[u8; 6],
    src_mac: &[u8; 6],
    eapol: &[u8],
    vif_idx: u8,
    sta_idx: u8,
) -> Vec<u8> {
    const SDIO_HEADER_LEN: usize = 4;
    const HOSTDESC_SIZE: usize = 28;

    let payload_len = eapol.len();
    let layout = calculate_eapol_frame_layout(payload_len);

    let mut buf = vec![0u8; layout.final_len];

    // 填充 SDIO header
    fill_sdio_header_for_eapol(&mut buf, &layout);

    // 填充 hostdesc
    let hd = &mut buf[SDIO_HEADER_LEN..SDIO_HEADER_LEN + HOSTDESC_SIZE];
    fill_hostdesc_for_eapol(hd, payload_len, dst_mac, src_mac, vif_idx, sta_idx);

    // 填充 EAPOL payload
    let eth_start = SDIO_HEADER_LEN + HOSTDESC_SIZE;
    buf[eth_start..eth_start + eapol.len()].copy_from_slice(eapol);

    buf
}

/// 执行 EAPOL 流控检查
fn perform_eapol_flow_control(bus: &WifiBus) -> Result<(), CmdError> {
    const MAX_RETRIES: usize = 50;

    for _ in 0..MAX_RETRIES {
        let sdio = bus.sdio.lock();
        match sdio.read_byte(1, SDIOWIFI_FLOW_CTRL_REG) {
            Ok(fc) if fc & 0x7F != 0 => {
                return Ok(());
            }
            _ => {}
        }
        drop(sdio);
        for _ in 0..10_000 {
            core::hint::spin_loop();
        }
    }

    log::error!("[cmd_mgr] EAPOL TX flow_ctrl timeout");
    Err(CmdError::Timeout)
}

/// 发送 EAPOL 帧到 SDIO
fn send_eapol_frame_to_sdio(bus: &Arc<WifiBus>, buf: &[u8]) -> Result<(), CmdError> {
    let base = bus.sdio_mmio_base.load(Ordering::Acquire);

    // Mask 卡中断
    if base != 0 {
        mask_unmask_card_irq_raw(base, true);
    }

    // 流控检查
    if let Err(e) = perform_eapol_flow_control(bus) {
        if base != 0 {
            mask_unmask_card_irq_raw(base, false);
        }
        return Err(e);
    }

    // 发送数据
    let sdio = bus.sdio.lock();
    if let Err(e) = sdio.write_fifo(1, SDIOWIFI_WR_FIFO_ADDR, buf) {
        log::error!("[cmd_mgr] EAPOL TX write_fifo failed: {:?}", e);
        drop(sdio);
        if base != 0 {
            mask_unmask_card_irq_raw(base, false);
        }
        return Err(CmdError::SdioError);
    }
    drop(sdio);

    // 唤醒 RX 线程
    bus.rx_irq_pollset.wake();

    // 恢复卡中断
    if base != 0 {
        mask_unmask_card_irq_raw(base, false);
    }

    Ok(())
}

/// 发送 EAPOL DATA 帧
///
/// `dst_mac`: AP 的 MAC 地址
/// `src_mac`: STA 的 MAC 地址
/// `eapol`: 完整的 EAPOL 帧（从 802.1X Version 字段开始）
pub fn send_eapol_data_frame(
    bus: &Arc<WifiBus>,
    dst_mac: &[u8; 6],
    src_mac: &[u8; 6],
    eapol: &[u8],
    vif_idx: u8,
    sta_idx: u8,
) -> Result<(), CmdError> {
    // 构造 EAPOL 帧缓冲区
    let buf = build_eapol_frame_buffer(dst_mac, src_mac, eapol, vif_idx, sta_idx);

    // 发送到 SDIO
    send_eapol_frame_to_sdio(bus, &buf)
}
