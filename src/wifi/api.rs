//! 高级 WiFi API
//!
//! 为外部程序提供简单易用的 WiFi 连接接口
//!
//! # 使用示例
//!
//! ```no_run
//! use aic8800_fdrv::wifi_api::{WifiConfig, WifiClient};
//! use alloc::sync::Arc;
//!
//! // 1. 创建 WiFi 客户端
//! let bus = Arc::new(/* 已初始化的 WifiBus */);
//! let client = WifiClient::new(bus);
//!
//! // 2. 扫描网络
//! let networks = client.scan(None, 10000)?;
//!
//! // 3. 连接到 WPA2 加密网络
//! let config = WifiConfig::wpa2_psk("MyNetwork", "password123");
//! client.connect(&config, 15000)?;
//!
//! // 4. 等待连接完成
//! client.wait_for_connection(10000)?;
//! ```

use crate::core::bus::WifiBus;
use crate::crypto::wpa2::*;
use crate::protocol::cmd::*;
use crate::protocol::lmac_msg::*;
use crate::wifi::manager::{self, build_wpa2_rsn_ie_from_ap, disconnect};
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

/// WiFi 连接配置
#[derive(Clone, Debug)]
pub struct WifiConfig {
    /// SSID (网络名称)
    pub ssid: Vec<u8>,
    /// 密码 (对于 WPA2-PSK)
    pub password: Option<Vec<u8>>,
    /// BSSID (可选，用于指定连接到特定 AP)
    pub bssid: Option<[u8; 6]>,
    /// 认证类型
    pub auth_type: WifiAuthType,
}

/// WiFi 认证类型
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WifiAuthType {
    /// 开放网络 (无密码)
    Open,
    /// WPA2-PSK (预共享密钥)
    Wpa2Psk,
    /// WPA3-PSK (未来支持)
    Wpa3Psk,
}

impl WifiConfig {
    /// 创建开放网络配置
    pub fn open(ssid: &str) -> Self {
        Self {
            ssid: ssid.as_bytes().to_vec(),
            password: None,
            bssid: None,
            auth_type: WifiAuthType::Open,
        }
    }

    /// 创建 WPA2-PSK 网络配置
    pub fn wpa2_psk(ssid: &str, password: &str) -> Self {
        Self {
            ssid: ssid.as_bytes().to_vec(),
            password: Some(password.as_bytes().to_vec()),
            bssid: None,
            auth_type: WifiAuthType::Wpa2Psk,
        }
    }

    /// 设置 BSSID (可选)
    pub fn with_bssid(mut self, bssid: [u8; 6]) -> Self {
        self.bssid = Some(bssid);
        self
    }
}

/// WiFi 连接错误
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WifiError {
    /// 驱动未初始化
    NotInitialized,
    /// 扫描失败
    ScanFailed,
    /// 未找到指定网络
    NetworkNotFound,
    /// 认证失败
    AuthenticationFailed,
    /// 连接超时
    ConnectionTimeout,
    /// 密码错误
    InvalidPassword,
    /// 网络不可用
    NetworkUnavailable,
    /// 已连接
    AlreadyConnected,
    /// 未连接
    NotConnected,
    /// 操作失败
    OperationFailed(String),
}

impl fmt::Display for WifiError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            WifiError::NotInitialized => write!(f, "Driver not initialized"),
            WifiError::ScanFailed => write!(f, "Scan failed"),
            WifiError::NetworkNotFound => write!(f, "Network not found"),
            WifiError::AuthenticationFailed => write!(f, "Authentication failed"),
            WifiError::ConnectionTimeout => write!(f, "Connection timeout"),
            WifiError::InvalidPassword => write!(f, "Invalid password"),
            WifiError::NetworkUnavailable => write!(f, "Network unavailable"),
            WifiError::AlreadyConnected => write!(f, "Already connected"),
            WifiError::NotConnected => write!(f, "Not connected"),
            WifiError::OperationFailed(msg) => write!(f, "Operation failed: {}", msg),
        }
    }
}

impl core::error::Error for WifiError {}

/// 扫描结果
#[derive(Clone, Debug)]
pub struct WifiNetwork {
    /// SSID
    pub ssid: Vec<u8>,
    /// SSID 长度
    pub ssid_len: u8,
    /// BSSID
    pub bssid: [u8; 6],
    /// 信号强度 (dBm)
    pub rssi: i8,
    /// 信道频率 (MHz)
    pub channel_freq: u16,
    /// 加密类型
    pub encryption: WifiEncryption,
    /// 是否为 WPA2/WPA3 网络
    pub has_rsn: bool,
    /// RSN IE (用于 WPA2 连接)
    pub rsn_ie: Vec<u8>,
}

/// WiFi 加密类型
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WifiEncryption {
    /// 无加密
    None,
    /// WEP
    Wep,
    /// WPA
    Wpa,
    /// WPA2
    Wpa2,
    /// WPA3
    Wpa3,
    /// 未知加密
    Unknown,
}

/// 连接状态
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConnectionStatus {
    /// 未连接
    Disconnected,
    /// 正在连接
    Connecting,
    /// 已连接
    Connected,
    /// 连接失败
    Failed,
}

/// WiFi 客户端
pub struct WifiClient {
    bus: Arc<WifiBus>,
    vif_idx: u8,
}

impl WifiClient {
    /// 创建新的 WiFi 客户端
    pub fn new(bus: Arc<WifiBus>) -> Self {
        Self {
            bus,
            vif_idx: 0, // 默认使用 VIF 0
        }
    }

    /// 设置 VIF 索引
    pub fn with_vif_idx(mut self, vif_idx: u8) -> Self {
        self.vif_idx = vif_idx;
        self
    }

    /// 获取连接状态
    pub fn get_status(&self) -> ConnectionStatus {
        let vif_idx = self
            .bus
            .connected_vif_idx
            .load(core::sync::atomic::Ordering::Acquire);
        if vif_idx == 0xFF {
            ConnectionStatus::Disconnected
        } else {
            ConnectionStatus::Connected
        }
    }

    /// 扫描 WiFi 网络
    ///
    /// # 参数
    /// - `ssid`: 可选的目标 SSID，None 表示扫描所有网络
    /// - `timeout_ms`: 扫描超时时间 (毫秒)
    ///
    /// # 返回
    /// 扫到的网络列表
    pub fn scan(
        &self,
        ssid: Option<&[u8]>,
        timeout_ms: u64,
    ) -> Result<Vec<WifiNetwork>, WifiError> {
        log::info!("[WifiClient] Starting scan...");

        // 执行扫描
        let results = scan(&self.bus, self.vif_idx, ssid, timeout_ms).map_err(|e| {
            log::error!("[WifiClient] Scan failed: {:?}", e);
            WifiError::ScanFailed
        })?;

        // 转换为 WifiNetwork 格式
        let networks = results
            .into_iter()
            .map(|r| WifiNetwork {
                ssid: r.ssid.to_vec(),
                ssid_len: r.ssid_len,
                bssid: r.bssid,
                rssi: r.rssi,
                channel_freq: r.center_freq,
                encryption: if r.rsn_ie.is_empty() {
                    WifiEncryption::None
                } else {
                    WifiEncryption::Wpa2
                },
                has_rsn: !r.rsn_ie.is_empty(),
                rsn_ie: r.rsn_ie,
            })
            .collect();

        log::info!(
            "[WifiClient] Scan complete: {} networks found",
            networks.len()
        );
        Ok(networks)
    }

    /// 查找指定 SSID 的网络
    ///
    /// # 参数
    /// - `ssid`: 目标 SSID
    /// - `timeout_ms`: 扫描超时时间 (毫秒)
    pub fn find_network(&self, ssid: &[u8], timeout_ms: u64) -> Result<WifiNetwork, WifiError> {
        let networks = self.scan(Some(ssid), timeout_ms)?;

        networks
            .into_iter()
            .find(|n| n.ssid[..n.ssid_len as usize] == *ssid)
            .ok_or(WifiError::NetworkNotFound)
    }

    /// 连接到 WiFi 网络
    ///
    /// # 参数
    /// - `config`: WiFi 连接配置
    /// - `timeout_ms`: 连接超时时间 (毫秒)
    pub fn connect(&self, config: &WifiConfig, timeout_ms: u64) -> Result<(), WifiError> {
        log::info!(
            "[WifiClient] Connecting to SSID: {:?}, auth: {:?}",
            core::str::from_utf8(&config.ssid).unwrap_or("<invalid>"),
            config.auth_type
        );

        // 检查是否已连接
        if self.get_status() == ConnectionStatus::Connected {
            return Err(WifiError::AlreadyConnected);
        }

        // 扫描目标网络
        let network = self.find_network(&config.ssid, 10000)?;

        // 构建 RSN IE (对于 WPA2 网络)
        let wpa2_ie = if config.auth_type == WifiAuthType::Wpa2Psk {
            if !network.has_rsn {
                log::warn!(
                    "[WifiClient] Target network doesn't have RSN IE, using default WPA2 IE"
                );
            }
            build_wpa2_rsn_ie_from_ap(&network.rsn_ie)
        } else {
            Vec::new()
        };

        // 构建 flags
        let flags = if config.auth_type == WifiAuthType::Wpa2Psk {
            WPA_WPA2_IN_USE | CONTROL_PORT_HOST | CONTROL_PORT_NO_ENC
        } else {
            0
        };

        // 执行连接
        let connect_result = manager::connect(
            &self.bus,
            self.vif_idx,
            &config.ssid,
            &network.bssid,
            network.channel_freq,
            &wpa2_ie,
            timeout_ms,
        )
        .map_err(|e| match e {
            CmdError::Timeout => WifiError::ConnectionTimeout,
            CmdError::FirmwareError => WifiError::AuthenticationFailed,
            _ => WifiError::OperationFailed(format!("Connection failed: {:?}", e)),
        })?;

        log::info!(
            "[WifiClient] Connection initiated: ap_idx={}, sta_idx={}",
            connect_result.ap_idx,
            connect_result.ap_idx
        );

        // 如果是 WPA2-PSK，需要执行四次握手
        if config.auth_type == WifiAuthType::Wpa2Psk {
            if let Some(ref password) = config.password {
                self.perform_wpa2_handshake(
                    connect_result.ap_idx,
                    &network.bssid,
                    password,
                    timeout_ms,
                )?;
            }
        }

        Ok(())
    }

    /// 执行 WPA2 四次握手
    fn perform_wpa2_handshake(
        &self,
        sta_idx: u8,
        bssid: &[u8; 6],
        password: &[u8],
        timeout_ms: u64,
    ) -> Result<(), WifiError> {
        log::info!("[WifiClient] Starting WPA2 handshake...");

        // 获取自己的 MAC 地址
        let own_mac = get_mac_address(&self.bus, 5000)
            .map_err(|e| WifiError::OperationFailed(format!("Failed to get MAC: {:?}", e)))?;

        // 创建 WPA2 状态机
        let mut wpa2 =
            Wpa2Supplicant::new(own_mac, *bssid, password.to_vec(), sta_idx, self.vif_idx);

        // 执行握手
        wpa2.run_handshake(&self.bus, timeout_ms)
            .map_err(|e| match e {
                WpaError::Timeout => WifiError::ConnectionTimeout,
                WpaError::MicError => WifiError::InvalidPassword,
                WpaError::FirmwareError => WifiError::AuthenticationFailed,
                _ => WifiError::OperationFailed(format!("Handshake failed: {:?}", e)),
            })?;

        log::info!("[WifiClient] WPA2 handshake completed successfully");
        Ok(())
    }

    /// 等待连接完成
    ///
    /// # 参数
    /// - `timeout_ms`: 等待超时时间 (毫秒)
    pub fn wait_for_connection(&self, timeout_ms: u64) -> Result<(), WifiError> {
        let start = axhal::time::monotonic_time_nanos();
        let timeout_ns = timeout_ms * 1_000_000;

        loop {
            if self.get_status() == ConnectionStatus::Connected {
                return Ok(());
            }

            let elapsed = axhal::time::monotonic_time_nanos() - start;
            if elapsed > timeout_ns {
                return Err(WifiError::ConnectionTimeout);
            }

            axtask::yield_now();
        }
    }

    /// 断开连接
    pub fn disconnect(&self) -> Result<(), WifiError> {
        log::info!("[WifiClient] Disconnecting...");

        if self.get_status() == ConnectionStatus::Disconnected {
            return Err(WifiError::NotConnected);
        }

        disconnect(&self.bus, self.vif_idx, 3) // reason code 3 = DEAUTH_LEAVING
            .map_err(|e| WifiError::OperationFailed(format!("Disconnect failed: {:?}", e)))?;

        log::info!("[WifiClient] Disconnected");
        Ok(())
    }

    /// 获取当前连接的 SSID
    pub fn get_current_ssid(&self) -> Option<Vec<u8>> {
        if self.get_status() != ConnectionStatus::Connected {
            return None;
        }

        // 从总线状态获取当前连接信息
        let vif_idx = self
            .bus
            .connected_vif_idx
            .load(core::sync::atomic::Ordering::Acquire);
        if vif_idx != 0xFF {
            // 这里需要从总线获取实际连接的 SSID
            // 暂时返回 None，需要在总线状态中添加 SSID 字段
            None
        } else {
            None
        }
    }

    /// 获取信号强度
    pub fn get_rssi(&self) -> Option<i8> {
        // 这里需要从总线状态获取当前信号强度
        // 暂时返回 None，需要在总线状态中添加 RSSI 字段
        None
    }
}

/// 获取 MAC 地址
fn get_mac_address(bus: &Arc<WifiBus>, timeout_ms: u64) -> Result<[u8; 6], CmdError> {
    send_get_mac_addr_req(bus, timeout_ms)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wifi_config_open() {
        let config = WifiConfig::open("TestNetwork");
        assert_eq!(config.ssid, b"TestNetwork".to_vec());
        assert_eq!(config.auth_type, WifiAuthType::Open);
        assert!(config.password.is_none());
    }

    #[test]
    fn test_wifi_config_wpa2() {
        let config = WifiConfig::wpa2_psk("TestNetwork", "password123");
        assert_eq!(config.ssid, b"TestNetwork".to_vec());
        assert_eq!(config.auth_type, WifiAuthType::Wpa2Psk);
        assert_eq!(config.password, Some(b"password123".to_vec()));
    }

    #[test]
    fn test_wifi_config_with_bssid() {
        let bssid = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
        let config = WifiConfig::open("TestNetwork").with_bssid(bssid);
        assert_eq!(config.bssid, Some(bssid));
    }

    #[test]
    fn test_wifi_error_display() {
        let err = WifiError::NetworkNotFound;
        assert_eq!(format!("{}", err), "Network not found");
    }
}
