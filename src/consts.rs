//! aic8800_fdrv 常量定义模块
//!
//! 集中管理所有协议、超时、配置相关的常量

// ============================================================
// SDIO 协议常量
// ============================================================

/// SDIO 功能块大小 (字节)
pub const SDIOWIFI_FUNC_BLOCKSIZE: usize = 512;

/// Dummy word 长度 (字节)
pub const DUMMY_WORD_LEN: usize = 4;

/// 尾部长度 (字节)
pub const TAIL_LEN: usize = 4;

/// 发送对齐 (字节)
pub const TX_ALIGNMENT: usize = 4;

/// SDIO 类型: 配置命令响应
pub const SDIO_TYPE_CFG_CMD_RSP: u8 = 0x11;

/// SDIO 类型: 数据
pub const SDIO_TYPE_DATA: u8 = 0x00;

/// SDIO 类型: 配置
pub const SDIO_TYPE_CFG: u8 = 0x10;

/// SDIO 类型: 配置数据确认
pub const SDIO_TYPE_CFG_DATA_CFM: u8 = 0x12;

// ============================================================
// 流控常量
// ============================================================

/// 流控允许掩码 (低 7 位)
pub const FLOW_CONTROL_MASK: u8 = 0x7F;

/// 流控重试最大次数
pub const FLOW_CONTROL_MAX_RETRY: u32 = 100;

/// 流控重试间隔 (spin_loop 次数)
pub const FLOW_CONTROL_RETRY_INTERVAL: u32 = 5_000;

// ============================================================
// 响应等待常量
// ============================================================

/// SDIO_OTHER_INTERRUPT 标志位
pub const SDIO_OTHER_INTERRUPT: u8 = 0x80;

/// 块计数掩码 (低 7 位)
pub const BLOCK_COUNT_MASK: u8 = 0x7F;

/// 响应超时最大重试次数
pub const RESPONSE_MAX_RETRY: u32 = 10_000;

/// 响应轮询间隔 (spin_loop 次数)
pub const RESPONSE_POLL_INTERVAL: u32 = 1_000;

/// 响应数据读取延迟 (spin_loop 次数)
pub const RESPONSE_READ_DELAY: u32 = 100_000;

// ============================================================
// 芯片初始化常量
// ============================================================

/// 芯片启动等待时间 (spin_loop 次数, 约 800ms)
pub const CHIP_STARTUP_DELAY: u32 = 20_000_000;

/// 中断稳定等待时间 (spin_loop 次数)
pub const IRQ_STABLE_DELAY: u32 = 100_000;

/// 初始化延迟 (spin_loop 次数, 约 200ms)
pub const INIT_DELAY: u32 = 5_000_000;

/// 中断配置寄存器值 (使能所有中断)
pub const INTR_CONFIG_VALUE: u8 = 0x07;

/// 堆栈起始参数
pub const STACK_START_PARAM: [u8; 4] = [0x01, 0x00, 0x00, 0x00];

// ============================================================
// 协议头部常量
// ============================================================

/// SDIO 头部大小 (字节)
pub const SDIO_HEADER_SIZE: usize = 4;

/// LMAC 消息头部大小 (字节)
pub const LMAC_MSG_HEADER_SIZE: usize = 8;

/// 完整协议头部大小 (SDIO + Dummy + LMAC)
pub const PROTO_HEADER_SIZE: usize = SDIO_HEADER_SIZE + DUMMY_WORD_LEN + LMAC_MSG_HEADER_SIZE; // = 16

// ============================================================
// 超时常量 (毫秒)
// ============================================================

/// 默认命令超时
pub const DEFAULT_CMD_TIMEOUT_MS: u64 = 2000;

/// 扫描超时
pub const SCAN_TIMEOUT_MS: u64 = 15000;

/// 连接超时
pub const CONNECT_TIMEOUT_MS: u64 = 10000;

/// 断连超时
pub const DISCONNECT_TIMEOUT_MS: u64 = 5000;

// ============================================================
// WiFi 配置常量
// ============================================================

/// 默认 VIF 索引
pub const DEFAULT_VIF_IDX: u8 = 0;

/// 最大 SSID 长度
pub const MAX_SSID_LEN: usize = 32;

/// 最大密码长度
pub const MAX_PASSPHRASE_LEN: usize = 64;

/// 默认信道
pub const DEFAULT_CHANNEL: u8 = 6;

// ============================================================
// 缓冲区大小常量
// ============================================================

/// 命令响应缓冲区大小
pub const CMD_RESPONSE_BUF_SIZE: usize = 512;

/// 扫描结果最大数量
pub const MAX_SCAN_RESULTS: usize = 64;

/// TX 队列大小
pub const TX_QUEUE_SIZE: usize = 128;

/// RX 队列大小
pub const RX_QUEUE_SIZE: usize = 256;

// ============================================================
// 数据掩码常量
// ============================================================

/// 字节低 8 位掩码
pub const U8_MASK: u8 = 0xFF;

/// 字节低 4 位掩码
pub const LOW_NIBBLE_MASK: u8 = 0x0F;

/// u16 低 8 位掩码
pub const U16_LOW_MASK: u16 = 0xFF;

/// u16 高 4 位掩码
pub const U16_HIGH_NIBBLE_MASK: u16 = 0x0F;

/// u16 低 10 位掩码 (用于 msg_id 提取)
pub const MSG_INDEX_MASK: u16 = (1 << 10) - 1; // = 0x3FF

/// LMAC 消息 ID 左移位数
pub const LMAC_MSG_ID_SHIFT: u16 = 10;

// ============================================================
// 协议头部大小常量
// ============================================================

/// SDIO 头部大小 (字节)
pub const SDIO_HEADER_SIZE: usize = 4;

/// LMAC 消息头部大小 (字节)
pub const LMAC_MSG_HEADER_SIZE: usize = 8;

/// 完整协议头部大小 (SDIO + Dummy + LMAC)
pub const PROTO_HEADER_SIZE: usize = SDIO_HEADER_SIZE + DUMMY_WORD_LEN + LMAC_MSG_HEADER_SIZE; // = 16

// ============================================================
// HostDesc 常量
// ============================================================

/// HostDesc 大小 (字节)
pub const HOSTDESC_SIZE: usize = 28;

/// HostDesc 中 hostid 需要设置 TX 确认标志
pub const HOSTDESC_TX_CFM_FLAG: u32 = 0x8000_0000;

// ============================================================
// 802.11 帧常量
// ============================================================

/// 802.11 帧头部大小 (字节)
pub const IEEE80211_HDR_SIZE: usize = 24;

/// 802.11 管理 Beacon 帧最小大小 (字节)
pub const IEEE80211_BEACON_MIN_SIZE: usize = 36;

/// 802.3 Ethernet 头部大小 (字节)
pub const ETH_HDR_SIZE: usize = 14;

// ============================================================
// 认证和加密常量
// ============================================================

/// 认证类型：开放系统
pub const WLAN_AUTH_OPEN: u8 = 0;

/// 认证类型：共享密钥
pub const WLAN_AUTH_SHARED_KEY: u8 = 1;

/// 认证类型：快速 BSS 转换
pub const WLAN_AUTH_FT: u8 = 2;

/// 认证类型：SAE (WPA3)
pub const WLAN_AUTH_SAE: u8 = 3;

/// 加密套件：WEP40
pub const MAC_CIPHER_WEP40: u8 = 0;

/// 加密套件：TKIP
pub const MAC_CIPHER_TKIP: u8 = 1;

/// 加密套件：CCMP
pub const MAC_CIPHER_CCMP: u8 = 2;

/// 加密套件：WEP104
pub const MAC_CIPHER_WEP104: u8 = 3;

// ============================================================
// 固件启动类型常量
// ============================================================

/// 固件启动：自动模式
pub const HOST_START_APP_AUTO: u32 = 1;

/// 固件启动：自定义模式
pub const HOST_START_APP_CUSTOM: u32 = 2;

/// 固件启动：函数调用
pub const HOST_START_APP_FNCALL: u32 = 4;

/// 固件启动：虚拟模式 (AIC8800DC)
pub const HOST_START_APP_DUMMY: u32 = 5;

// ============================================================
// 芯片版本寄存器解析常量
// ============================================================

/// 芯片版本寄存器值的高 16 位位移量
pub const CHIP_REV_HIGH_SHIFT: u32 = 16;

/// 芯片版本号掩码 (低 6 位)
pub const CHIP_REV_MASK: u32 = 0x3F;

/// 芯片 ID 高性能标志掩码 (高 2 位)
pub const CHIP_ID_H_MASK: u32 = 0xC0;

/// 芯片 ID 高性能标志值
pub const CHIP_ID_H_VALUE: u32 = 0xC0;

// ============================================================
// VIF 类型常量
// ============================================================

/// VIF 类型：STA 站点
pub const MM_STA: u8 = 0;

/// VIF 类型：IBSS (Ad-Hoc)
pub const MM_IBSS: u8 = 1;

/// VIF 类型：AP 接入点
pub const MM_AP: u8 = 2;

// ============================================================
// PHY 带宽常量
// ============================================================

/// PHY 信道带宽：20MHz
pub const PHY_CHNL_BW_20: u8 = 0;

/// PHY 信道带宽：40MHz
pub const PHY_CHNL_BW_40: u8 = 1;

/// PHY 信道带宽：80MHz
pub const PHY_CHNL_BW_80: u8 = 2;

// ============================================================
// 连接标志常量
// ============================================================

/// 连接标志：控制端口由主机管理
pub const CONTROL_PORT_HOST: u32 = 1 << 0;

/// 连接标志：控制端口帧不加密
pub const CONTROL_PORT_NO_ENC: u32 = 1 << 1;

/// 连接标志：禁用 HT (WEP/TKIP 时需要)
pub const DISABLE_HT: u32 = 1 << 2;

/// 连接标志：使用 WPA/WPA2 认证
pub const WPA_WPA2_IN_USE: u32 = 1 << 3;

/// 连接标志：使用 MFP (802.11w)
pub const MFP_IN_USE: u32 = 1 << 4;

/// 连接标志：重关联 (roaming)
pub const REASSOCIATION: u32 = 1 << 5;

// ============================================================
// 802.11 信息元素 ID 常量
// ============================================================

/// IE ID：SSID
pub const WLAN_EID_SSID: u8 = 0;

/// IE ID：支持的速率
pub const WLAN_EID_SUPP_RATES: u8 = 1;

/// IE ID：DS 参数集 (信道)
pub const WLAN_EID_DS_PARAMS: u8 = 3;

/// IE ID：RSN
pub const WLAN_EID_RSN: u8 = 48;

// ============================================================
// 802.1X/EAPOL 常量
// ============================================================

/// 802.1X Authentication EtherType
pub const ETH_P_PAE: u16 = 0x888E;

/// EAPOL 帧版本
pub const EAPOL_VERSION: u8 = 0x02;

// ============================================================
// EAPOL 帧类型
// ============================================================

/// EAPOL 帧类型：EAPOL-Packet
pub const EAPOL_PACKET: u8 = 0x00;

/// EAPOL 帧类型：EAPOL-Start
pub const EAPOL_START: u8 = 0x01;

/// EAPOL 帧类型：EAPOL-Logoff
pub const EAPOL_LOGOFF: u8 = 0x02;

/// EAPOL 帧类型：EAPOL-Key
pub const EAPOL_KEY: u8 = 0x03;

/// EAPOL 帧类型：EAPOL-Encapsulated-ASF-Alert
pub const EAPOL_ASF_ALERT: u8 = 0x04;

// ============================================================
// 时序和延迟常量 (spin_loop 次数)
// ============================================================

/// 10ms 延迟 (假设 200MHz CPU)
pub const DELAY_10MS: u32 = 500_000;

/// 5ms 延迟
pub const DELAY_5MS: u32 = 250_000;

/// 1ms 延迟
pub const DELAY_1MS: u32 = 50_000;

/// 100us 延迟
pub const DELAY_100US: u32 = 5_000;

/// 50us 延迟
pub const DELAY_50US: u32 = 2_500;

// ============================================================
// TX Power 常量
// ============================================================

/// 默认 TX Power Index (AIC8801)
pub const DEFAULT_TXPWR_DSSS: u8 = 9;

/// 默认 TX Power Index (2.4GHz OFDM 低速率)
pub const DEFAULT_TXPWR_OFDM_LOW_2G4: u8 = 8;

/// 默认 TX Power Index (2.4GHz OFDM 64QAM)
pub const DEFAULT_TXPWR_OFDM64_2G4: u8 = 8;

/// 默认 TX Power Index (2.4GHz OFDM 256QAM)
pub const DEFAULT_TXPWR_OFDM256_2G4: u8 = 8;

/// 默认 TX Power Index (2.4GHz OFDM 1024QAM)
pub const DEFAULT_TXPWR_OFDM1024_2G4: u8 = 8;

/// 默认 TX Power Index (5GHz OFDM 低速率)
pub const DEFAULT_TXPWR_OFDM_LOW_5G: u8 = 11;

/// 默认 TX Power Index (5GHz OFDM 64QAM)
pub const DEFAULT_TXPWR_OFDM64_5G: u8 = 10;

/// 默认 TX Power Index (5GHz OFDM 256QAM)
pub const DEFAULT_TXPWR_OFDM256_5G: u8 = 9;

/// 默认 TX Power Index (5GHz OFDM 1024QAM)
pub const DEFAULT_TXPWR_OFDM1024_5G: u8 = 9;

/// 默认 TX Power Offset (全部为 0)
pub const DEFAULT_TXPWR_OFST: [u8; 8] = [1, 0, 0, 0, 0, 0, 0, 0];

// ============================================================
// RF 校准常量
// ============================================================

/// RF 校准配置：2.4GHz
pub const RF_CALIB_CFG_24G: u32 = 0x0000_00BF;

/// RF 校准配置：5GHz
pub const RF_CALIB_CFG_5G: u32 = 0x0000_003F;

/// RF 校准参数 alpha
pub const RF_CALIB_PARAM_ALPHA: u32 = 0x0C34_C008;

/// RF 校准 BT 参数
pub const RF_CALIB_BT_PARAM: u32 = 0x0026_4203;

// ============================================================
// MM 启动配置常量
// ============================================================

/// 默认 UAPSD 超时值 (ms)
pub const DEFAULT_UAPSD_TIMEOUT: u32 = 300;

/// 默认低功耗时钟精度 (ppm)
pub const DEFAULT_LP_CLK_ACCURACY: u16 = 20;

// ============================================================
// 固件上传常量
// ============================================================

/// 固件上传块大小 (字节)
pub const FW_UPLOAD_CHUNK_SIZE: usize = 1024;

/// 固件上传进度打印间隔 (字节)
pub const FW_UPLOAD_PROGRESS_INTERVAL: usize = 65536;

// ============================================================
// 时钟配置常量
// ============================================================

/// 初始化时钟频率 (Hz)
pub const INIT_CLOCK_FREQ: u32 = 400_000;

/// 高速时钟频率 (Hz)
pub const HIGH_SPEED_CLOCK_FREQ: u32 = 50_000_000;

/// 默认时钟频率 (Hz)
pub const DEFAULT_CLOCK_FREQ: u32 = 25_000_000;

/// 固件启动时钟频率 (Hz)
pub const FIRMWARE_START_CLOCK_FREQ: u32 = 400_000;

// ============================================================
// 结构体大小常量
// ============================================================

/// MM_KEY_ADD_REQ 结构体大小 (字节)
pub const MM_KEY_ADD_REQ_SIZE: usize = 44;

/// MM_KEY_DEL_REQ 结构体大小 (字节)
pub const MM_KEY_DEL_REQ_SIZE: usize = 1;

/// ME_SET_CONTROL_PORT_REQ 结构体大小 (字节)
pub const ME_SET_CONTROL_PORT_REQ_SIZE: usize = 2;

/// SM_CONNECT_REQ 结构体大小 (字节)
pub const SM_CONNECT_REQ_SIZE: usize = 320;

/// SM_DISCONNECT_REQ 结构体大小 (字节)
pub const SM_DISCONNECT_REQ_SIZE: usize = 3;

/// SM_ASSOC_IE_LEN (字节)
pub const SM_ASSOC_IE_LEN: usize = 800;

/// SM_DISCONNECT_IND 结构体大小 (字节)
pub const SM_DISCONNECT_IND_SIZE: usize = 5;

/// MM_ADD_IF_REQ 结构体大小 (字节)
pub const MM_ADD_IF_REQ_SIZE: usize = 10;

/// MM_START_REQ 结构体大小 (字节)
pub const MM_START_REQ_SIZE: usize = 70;

/// ME_CONFIG_REQ 结构体大小 (字节)
pub const ME_CONFIG_REQ_SIZE: usize = 102;

/// ME_CHAN_CONFIG_REQ 2.4GHz 最大信道数
pub const ME_CHAN_MAX_2G4: usize = 14;

/// ME_CHAN_CONFIG_REQ 5GHz 最大信道数
pub const ME_CHAN_MAX_5G: usize = 28;

// ============================================================
// SM_CONNECT_REQ 偏移量常量
// ============================================================

/// SSID 字段偏移量
pub const SM_CONNECT_REQ_SSID_OFFSET: usize = 0;

/// BSSID 字段偏移量
pub const SM_CONNECT_REQ_BSSID_OFFSET: usize = 34;

/// Channel 字段偏移量
pub const SM_CONNECT_REQ_CHAN_OFFSET: usize = 40;

/// Flags 字段偏移量
pub const SM_CONNECT_REQ_FLAGS_OFFSET: usize = 48;

/// EtherType 字段偏移量
pub const SM_CONNECT_REQ_ETHTYPE_OFFSET: usize = 52;

/// IE Length 字段偏移量
pub const SM_CONNECT_REQ_IE_LEN_OFFSET: usize = 54;

/// Listen Interval 字段偏移量
pub const SM_CONNECT_REQ_LISTEN_INTERVAL_OFFSET: usize = 56;

/// Don't wait BCMC 字段偏移量
pub const SM_CONNECT_REQ_DONT_WAIT_BCMC_OFFSET: usize = 58;

/// Auth Type 字段偏移量
pub const SM_CONNECT_REQ_AUTH_TYPE_OFFSET: usize = 59;

/// uAPSD 队列字段偏移量
pub const SM_CONNECT_REQ_UAPSD_QUEUES_OFFSET: usize = 60;

/// VIF Index 字段偏移量
pub const SM_CONNECT_REQ_VIF_IDX_OFFSET: usize = 61;

/// IE Buffer 字段偏移量
pub const SM_CONNECT_REQ_IE_BUF_OFFSET: usize = 64;

// ============================================================
// MM_KEY_ADD_REQ 偏移量常量
// ============================================================

/// Key Index 字段偏移量
pub const MM_KEY_ADD_REQ_KEY_IDX_OFFSET: usize = 0;

/// STA Index 字段偏移量
pub const MM_KEY_ADD_REQ_STA_IDX_OFFSET: usize = 1;

/// MAC Sec Key 字段偏移量
pub const MM_KEY_ADD_REQ_KEY_OFFSET: usize = 4;

/// MAC Sec Key Length 字段偏移量
pub const MM_KEY_ADD_REQ_KEY_LEN_OFFSET: usize = 4;

/// MAC Sec Key Array 字段偏移量
pub const MM_KEY_ADD_REQ_KEY_ARRAY_OFFSET: usize = 8;

/// Cipher Suite 字段偏移量
pub const MM_KEY_ADD_REQ_CIPHER_OFFSET: usize = 40;

/// VIF Index 字段偏移量
pub const MM_KEY_ADD_REQ_VIF_IDX_OFFSET: usize = 41;

/// SPP 字段偏移量
pub const MM_KEY_ADD_REQ_SPP_OFFSET: usize = 42;

/// Pairwise 字段偏移量
pub const MM_KEY_ADD_REQ_PAIRWISE_OFFSET: usize = 43;

// ============================================================
// MAC 地址和 SSID 常量
// ============================================================

/// MAC 地址大小 (字节)
pub const MAC_ADDR_SIZE: usize = 6;

/// SSID 最大长度 (字节)
pub const MAC_SSID_LEN: usize = 32;

/// MAC SSID 结构体大小 (字节)
pub const MAC_SSID_SIZE: usize = 33;

/// MAC Chan Def 结构体大小 (字节)
pub const MAC_CHAN_DEF_SIZE: usize = 6;

/// MAC Sec Key 最大长度 (字节)
pub const MAC_SEC_KEY_LEN: usize = 32;

// ============================================================
// 扫描常量
// ============================================================

/// 最大扫描 SSID 数量
pub const SCAN_SSID_MAX: usize = 3;

/// 最大扫描信道数量
pub const SCAN_CHANNEL_MAX: usize = 42;

/// SCANU_START_REQ 结构体大小 (字节)
pub const SCANU_START_REQ_SIZE: usize = 376;

/// SCANU_START_CFM 结构体大小 (字节)
pub const SCANU_START_CFM_SIZE: usize = 3;

/// SCANU_RESULT_IND 头部大小 (字节)
pub const SCANU_RESULT_IND_HDR_SIZE: usize = 12;

// ============================================================
// 过滤器常量
// ============================================================

/// 接受单播帧
pub const NXMAC_ACCEPT_UNICAST_BIT: u32 = 1 << 0;

/// 接收多播帧
pub const NXMAC_ACCEPT_MULTICAST_BIT: u32 = 1 << 1;

/// 接收广播帧
pub const NXMAC_ACCEPT_BROADCAST_BIT: u32 = 1 << 2;

/// 接收 Probe Request 帧 (AP 模式)
pub const NXMAC_ACCEPT_PROBE_REQ_BIT: u32 = 1 << 3;

/// STA 模式默认过滤器
pub const STA_MODE_FILTER_DEFAULT: u32 =
    NXMAC_ACCEPT_UNICAST_BIT | NXMAC_ACCEPT_MULTICAST_BIT | NXMAC_ACCEPT_BROADCAST_BIT;
