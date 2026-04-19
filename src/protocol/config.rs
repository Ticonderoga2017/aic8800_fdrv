//! 芯片配置命令（TX power、RF、ME、MM 等）

use alloc::sync::Arc;
use alloc::vec::Vec;
use alloc::vec;

use crate::core::bus::WifiBus;
use crate::protocol::lmac_msg::*;
use crate::protocol::cmd::send_cmd;

/// 发送 MM_SET_TXPWR_IDX_LVL_REQ
pub fn send_txpwr_idx_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<(), CmdError> {
    let mut param = [0u8; 6];
    param[0] = 0; // 2.4GHz
    param[1] = 0; // 5GHz
    param[2] = 0; // reserved
    send_cmd(bus, MM_SET_TXPWR_IDX_LVL_REQ, TASK_MM, &param, timeout_ms)?;
    Ok(())
}

/// 发送 MM_SET_TXPWR_OFST_REQ
pub fn send_txpwr_ofst_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<(), CmdError> {
    let param = [0u8; 6];
    send_cmd(bus, MM_SET_TXPWR_OFST_REQ, TASK_MM, &param, timeout_ms)?;
    Ok(())
}

/// 发送 MM_SET_RF_CALIB_REQ
///
/// 对应 Linux rwnx_msg_tx.c:1289-1328 (AIC8801 v1 路径):
///   cal_cfg_24g   (u32) 2.4GHz 校准配置位掩码
///   cal_cfg_5g    (u32) 5GHz 校准配置位掩码
///   param_alpha   (u32) 校准 alpha 参数
///   bt_calib_en   (u32) BT 校准使能
///   bt_calib_param(u32) BT 校准参数
///   xtal_cap      (u8)  晶振 cap（来自用户配置，默认 0）
///   xtal_cap_fine (u8)  晶振 cap fine
pub fn send_rf_calib_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<Vec<u8>, CmdError> {
    let mut param = [0u8; MM_SET_RF_CALIB_REQ_SIZE];

    // cal_cfg_24g (offset 0)
    param[0..4].copy_from_slice(&AIC8801_RF_CAL_CFG_24G.to_le_bytes());
    // cal_cfg_5g (offset 4)
    param[4..8].copy_from_slice(&AIC8801_RF_CAL_CFG_5G.to_le_bytes());
    // param_alpha (offset 8)
    param[8..12].copy_from_slice(&AIC8801_RF_PARAM_ALPHA.to_le_bytes());
    // bt_calib_en (offset 12) = 0
    // bt_calib_param (offset 16)
    param[16..20].copy_from_slice(&AIC8801_RF_BT_CALIB_PARAM.to_le_bytes());
    // xtal_cap (offset 20) = 0
    // xtal_cap_fine (offset 21) = 0

    send_cmd(bus, MM_SET_RF_CALIB_REQ, TASK_MM, &param, timeout_ms)
}

/// 发送 ME_CONFIG_REQ（最小配置：HT only, 20MHz, 1SS）
///
/// me_config_req 结构体布局 (对应 Linux lmac_msg.h:1843-1868):
///   mac_htcapability  ht_cap;     // 26 bytes
///   mac_vhtcapability vht_cap;    // 12 bytes
///   mac_hecapability  he_cap;     // 54 bytes
///   u16 tx_lft;                   // 2 bytes
///   u8  phy_bw_max;               // 1 byte
///   bool ht_supp;                 // 1 byte
///   bool vht_supp .. bool dpsm;   // 6 bytes
///   Total: 102 bytes
pub fn send_me_config_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<Vec<u8>, CmdError> {
    let mut param = vec![0u8; ME_CONFIG_REQ_SIZE];

    // ht_cap (offset 0)
    param[0..2].copy_from_slice(&HT_CAPA_INFO_LDPC.to_le_bytes());
    param[2] = HT_AMPDU_FACTOR_MAX | (HT_AMPDU_DENSITY_MAX << 2);
    param[3] = HT_MCS_RATE_1SS;

    // vht_cap (offset MAC_HT_CAPABILITY_SIZE=26) — 全零（不使用 VHT）
    // he_cap (offset MAC_HT_CAPABILITY_SIZE + MAC_VHT_CAPABILITY_SIZE=38) — 全零（不使用 HE）

    let tail = MAC_HT_CAPABILITY_SIZE + MAC_VHT_CAPABILITY_SIZE + MAC_HE_CAPABILITY_SIZE; // 92
    // tx_lft = 0
    param[tail + ME_CONFIG_TAIL_PHY_BW_OFF] = PHY_CHNL_BW_20;
    param[tail + ME_CONFIG_TAIL_HT_SUPP_OFF] = 1; // ht_supp = true
    // vht_supp..dpsm = 0

    send_cmd(bus, ME_CONFIG_REQ, TASK_ME, &param, timeout_ms)
}

/// 发送 ME_CHAN_CONFIG_REQ（2.4GHz 信道 1-14）
///
/// me_chan_config_req: chan2G4[14] + chan5G[28] + chan2G4_cnt(u8) + chan5G_cnt(u8)
pub fn send_me_chan_config_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<Vec<u8>, CmdError> {
    let total_size = ME_CHAN_MAX_2G4 * MAC_CHAN_DEF_SIZE + ME_CHAN_MAX_5G * MAC_CHAN_DEF_SIZE + 2;
    let mut param = vec![0u8; total_size];

    let chan_cnt = CHAN_2G4_FREQS.len().min(ME_CHAN_MAX_2G4);
    for i in 0..chan_cnt {
        let off = i * MAC_CHAN_DEF_SIZE;
        param[off..off + 2].copy_from_slice(&CHAN_2G4_FREQS[i].to_le_bytes());
        param[off + 2] = 0; // band = 2.4GHz
        param[off + 3] = 0; // flags
        param[off + 4] = ME_CHAN_TX_POWER_DEFAULT as u8;
    }

    let cnt_offset = ME_CHAN_MAX_2G4 * MAC_CHAN_DEF_SIZE + ME_CHAN_MAX_5G * MAC_CHAN_DEF_SIZE;
    param[cnt_offset] = chan_cnt as u8;

    send_cmd(bus, ME_CHAN_CONFIG_REQ, TASK_ME, &param, timeout_ms)
}

/// 发送 MM_START_REQ
///
/// mm_start_req: phy_cfg_tag(64B) + uapsd_timeout(u32) + lp_clk_accuracy(u16)
/// 对应 Linux: rwnx_send_start (line 473-492)
pub fn send_mm_start_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<Vec<u8>, CmdError> {
    let mut param = [0u8; MM_START_REQ_SIZE];
    // phy_cfg[0..64] = 全零（AIC8801 不需要 PHY 配置）
    param[MM_START_PHY_CFG_SIZE..MM_START_PHY_CFG_SIZE + 4]
        .copy_from_slice(&MM_START_UAPSD_TIMEOUT_MS.to_le_bytes());
    param[MM_START_PHY_CFG_SIZE + 4..MM_START_PHY_CFG_SIZE + 6]
        .copy_from_slice(&MM_START_LP_CLK_ACCURACY_PPM.to_le_bytes());

    send_cmd(bus, MM_START_REQ, TASK_MM, &param, timeout_ms)
}

/// 发送 MM_SET_FILTER_REQ
pub fn send_mm_set_filter_req(
    bus: &Arc<WifiBus>,
    filter: u32,
    timeout_ms: u64,
) -> Result<(), CmdError> {
    let mut param = [0u8; 4];
    param[0..4].copy_from_slice(&filter.to_le_bytes());

    send_cmd(bus, MM_SET_FILTER_REQ, TASK_MM, &param, timeout_ms)?;
    Ok(())
}

/// 发送 MM_SET_IDLE_REQ
pub fn send_mm_set_idle_req(
    bus: &Arc<WifiBus>,
    idle: bool,
    timeout_ms: u64,
) -> Result<Vec<u8>, CmdError> {
    let param = [if idle { 1u8 } else { 0u8 }];
    send_cmd(bus, MM_SET_IDLE_REQ, TASK_MM, &param, timeout_ms)
}

/// 发送 ME_SET_CONTROL_PORT_REQ
pub fn send_set_control_port_req(
    bus: &Arc<WifiBus>,
    sta_idx: u8,
    open: bool,
    timeout_ms: u64,
) -> Result<Vec<u8>, CmdError> {
    let param = [sta_idx, if open { 1 } else { 0 }];
    send_cmd(bus, ME_SET_CONTROL_PORT_REQ, TASK_ME, &param, timeout_ms)
}

/// 发送 MM_GET_MAC_ADDR_REQ
pub fn send_get_mac_addr_req(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<[u8; 6], CmdError> {
    let param = MM_GET_MAC_ADDR_REQ_GET.to_le_bytes();

    let rsp = send_cmd(bus, MM_GET_MAC_ADDR_REQ, TASK_MM, &param, timeout_ms)?;

    if rsp.len() >= 6 {
        let mut mac = [0u8; 6];
        mac.copy_from_slice(&rsp[..6]);
        Ok(mac)
    } else {
        log::error!("[cmd_mgr] MM_GET_MAC_ADDR_CFM too short: {} bytes", rsp.len());
        Err(CmdError::InvalidResponse)
    }
}
