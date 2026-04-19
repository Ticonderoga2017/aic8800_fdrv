# aic8800_fdrv

[![Rust](https://img.shields.io/badge/rust-1.70%2B-orange.svg)](https://www.rust-lang.org/)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)

AIC8800 WiFi 芯片运行时功能驱动（Functional Driver），提供完整的 WiFi STA 模式协议栈，包括 SDIO 传输、LMAC 消息协议、扫描、WPA2-PSK 认证、TX/RX 数据路径和网络设备注册。

## 概述

本 crate 是 AIC8800 WiFi 驱动栈的**核心运行时组件**，位于固件加载层 (`aic8800_fw`) 之上，负责：

- **SDIO 传输管理**：封装 CMD52/CMD53 数据收发，含流控和帧组装/解析
- **LMAC 协议栈**：与芯片固件的完整消息交互（配置、扫描、连接、数据收发）
- **WiFi 客户端**：STA 模式的扫描、WPA2-PSK 连接、断连
- **TX/RX 线程**：异步数据收发，独立线程处理上下行流量
- **网络设备桥接**：将 WiFi 驱动包装为 ArceOS `NetDriverOps` 网络设备

### 主要特性

- **`#![no_std]`** — 裸机环境运行，依赖 `alloc`
- **WPA2-PSK 认证**：完整的 4-way handshake 实现 (PBKDF2 + HMAC-SHA1 + AES)
- **802.11 帧处理**：Beacon 解析、Probe Request/Response、Authentication、Association
- **802.3/EAPOL**：Ethernet 帧封装、EAPOL-Key 帧构造与解析
- **多芯片支持**：AIC8801 / AIC8800DC / AIC8800DW / AIC8800D80 / AIC8800D80X2

## 架构设计

### 模块结构

```
aic8800_fdrv/
├── src/
│   ├── lib.rs              # crate 入口 + API 重新导出
│   ├── consts.rs           # 驱动内部常量（缓冲区、超时、功率等）
│   ├── core/
│   │   ├── bus.rs          # WifiBus — SDIO 传输 + 线程管理
│   │   ├── init.rs         # 驱动初始化入口
│   │   └── sdio_transport.rs # SDIO 帧 read/write 封装
│   ├── crypto/
│   │   └── wpa2.rs         # WPA2-PSK 密钥派生 + 4-way handshake
│   ├── net/
│   │   └── device.rs       # AicWifiNetDev — NetDriverOps 实现
│   ├── protocol/
│   │   ├── cmd.rs          # LMAC 命令构造和发送
│   │   ├── config.rs       # LMAC 配置消息处理
│   │   ├── connection.rs   # WiFi 连接/断连管理
│   │   ├── key.rs          # 密钥安装
│   │   ├── lmac_msg.rs     # LMAC 消息 ID/结构体定义
│   │   └── scan.rs         # WiFi 扫描逻辑
│   ├── thread/
│   │   ├── rx.rs           # RX 线程 — SDIO 数据接收
│   │   └── tx.rs           # TX 线程 — SDIO 数据发送
│   └── wifi/
│       ├── api.rs          # WifiClient — 用户层 API
│       └── manager.rs      # WiFi 状态管理
├── Cargo.toml
└── README.md
```

### 驱动层次

```
┌─────────────────────────────────────────┐
│   应用层 / aic8800_wireless             │
├─────────────────────────────────────────┤
│   ★ aic8800_fdrv (本 crate) ★           │
│   ├── WifiClient   (扫描/连接/断连)      │
│   ├── WifiBus      (SDIO 传输 + 线程)    │
│   ├── WPA2         (4-way handshake)     │
│   ├── TX/RX        (数据路径)            │
│   └── NetDev       (网络设备桥接)         │
├─────────────────────────────────────────┤
│   aic8800_common (寄存器/常量)           │
├─────────────────────────────────────────┤
│   aic8800_fw (固件加载，本 crate 不依赖) │
├─────────────────────────────────────────┤
│   axdriver_sdio (SDIO 抽象层)           │
├─────────────────────────────────────────┤
│   sdhci-cv1800 (SDHCI 控制器)           │
└─────────────────────────────────────────┘
```

## 使用方法

### 基本用法（通过 `WifiClient`）

```rust
use aic8800_fdrv::{init, WifiClient, WifiConfig, WifiBus};
use alloc::sync::Arc;

// 1. 初始化驱动（SDIO 传输 + IRQ + TX/RX 线程）
let bus: Arc<WifiBus> = init(sdio_host)?;

// 2. 创建 WiFi 客户端
let mut client = WifiClient::new(Arc::clone(&bus));

// 3. LMAC 配置（设置信道、功率等）
let mac = client.lmac_configure(6000)?;
log::info!("MAC address: {:02x?}", mac);

// 4. 扫描 WiFi 网络
let networks = client.scan(15000)?;
for net in &networks {
    log::info!("SSID: {}, RSSI: {}", net.ssid, net.rssi);
}

// 5. 连接 WiFi
let config = WifiConfig::wpa2_psk("MyWiFi", "password123");
client.connect(&config, 15000)?;

// 6. 注册网络设备到 axnet
client.store_net_device();
```

### 完整初始化流程

```rust
use aic8800_fdrv::init;
use aic8800_common::ChipVariant;
use axdriver_sdio::SdioHost;

fn setup_wifi<H: SdioHost + SdioIrqControl + 'static>(
    mut sdio: H,
    chip: ChipVariant,
) -> Result<(), WifiError> {
    // 1. 固件加载（由 aic8800_fw 完成，需在调用 init 之前）
    aic8800_fw::firmware_init(&mut sdio, chip)?;

    // 2. 驱动初始化
    let bus = init(sdio)?;

    // 3. 注册 CARD_INT 中断回调
    sdhci_cv1800::irq::register_card_irq_callback(sdio1_irq_handler);

    // 4. LMAC 配置 + 连接
    let mut client = WifiClient::new(bus);
    client.lmac_configure(6000)?;
    client.connect(&WifiConfig::wpa2_psk("SSID", "password"), 15000)?;

    Ok(())
}
```

## 核心 API

### `WifiBus`

SDIO 传输和线程管理的核心结构体。

```rust
pub struct WifiBus { /* ... */ }

impl WifiBus {
    /// 关闭总线（停止线程、清空队列、禁用中断）
    pub fn shutdown(&self);
}
```

### `WifiClient`

用户层 WiFi 操作接口。

```rust
impl WifiClient {
    pub fn new(bus: Arc<WifiBus>) -> Self;

    /// LMAC 配置（信道、TX 功率、RF 校准等）
    pub fn lmac_configure(&mut self, timeout_ms: u64) -> Result<[u8; 6], WifiError>;

    /// 扫描 WiFi 网络
    pub fn scan(&mut self, timeout_ms: u64) -> Result<Vec<WifiNetwork>, WifiError>;

    /// 连接 WiFi
    pub fn connect(&mut self, config: &WifiConfig, timeout_ms: u64) -> Result<(), WifiError>;

    /// 断开连接
    pub fn disconnect(&mut self) -> Result<(), WifiError>;

    /// 注册网络设备到 axnet
    pub fn store_net_device(&self);
}
```

### `WifiConfig`

WiFi 连接配置。

```rust
let open_config = WifiConfig::open("OpenNetwork");
let wpa2_config = WifiConfig::wpa2_psk("SecureNetwork", "password");
```

### `WifiError`

```rust
pub enum WifiError {
    NotInitialized,
    Timeout,
    OperationFailed(String),
    ScanFailed(String),
    ConnectionFailed(String),
}
```

## 依赖项

```toml
[dependencies]
aic8800_common = { git = "https://github.com/Ticonderoga2017/aic8800_common.git" }
axdriver_sdio = { git = "https://github.com/Ticonderoga2017/axdriver_sdio.git" }
axsync = { git = "https://github.com/Ticonderoga2017/sg2002-arceos.git" }
axtask = { git = "https://github.com/Ticonderoga2017/sg2002-arceos.git", features = ["multitask"] }
axhal = { git = "https://github.com/Ticonderoga2017/sg2002-arceos.git", features = ["irq"] }
axnet = { git = "https://github.com/Ticonderoga2017/sg2002-arceos.git" }
axdriver_base = { git = "https://github.com/arceos-org/axdriver_crates.git", tag = "dev-v01" }
axdriver_net = { git = "https://github.com/arceos-org/axdriver_crates.git", tag = "dev-v01" }
hmac = { version = "0.12", default-features = false }
sha1 = { version = "0.10", default-features = false }
digest = { version = "0.10", default-features = false }
aes = { version = "0.8", default-features = false }
pbkdf2 = { version = "0.12", default-features = false }
axpoll = "0.1"
kspin = "0.1"
log = "0.4"
```

## 相关项目

- [aic8800_common](https://github.com/Ticonderoga2017/aic8800_common) — 共享常量和类型
- [aic8800_fw](https://github.com/Ticonderoga2017/aic8800_fw) — 固件加载库
- [aic8800_wireless](https://github.com/Ticonderoga2017/aic8800_wireless) — 顶层一键连接接口
- [axdriver_sdio](https://github.com/Ticonderoga2017/axdriver_sdio) — SDIO 抽象层
- [sdhci-cv1800](https://github.com/Ticonderoga2017/sdhci-cv1800) — SDHCI 控制器驱动
- [sg2002-arceos](https://github.com/Ticonderoga2017/sg2002-arceos) — ArceOS 操作系统

## 许可证

本项目采用 [Apache License 2.0](LICENSE) 许可证。

## 作者

Ticonderoga2017
