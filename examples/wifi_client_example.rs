#!/usr/bin/env rust-script
//! AIC8800 WiFi 客户端使用示例
//!
//! 这个示例展示了如何使用 aic8800_fdrv 提供的高级 API 来连接 WiFi 网络

#![no_std]
#![no_main]

extern crate alloc;

use aic8800_fdrv::{ConnectionStatus, WifiBus, WifiClient, WifiConfig, WifiError};
use alloc::string::String;
use alloc::sync::Arc;

/// 示例 1: 基本网络扫描
fn example_scan_networks(client: &WifiClient) {
    println!("=== 示例 1: 扫描 WiFi 网络 ===");

    match client.scan(None, 10000) {
        Ok(networks) => {
            println!("找到 {} 个网络:", networks.len());

            for (i, network) in networks.iter().enumerate() {
                let ssid = core::str::from_utf8(&network.ssid[..network.ssid_len as usize])
                    .unwrap_or("<invalid>");

                println!("{}. {} ({} dBm)", i + 1, ssid, network.rssi);

                let encryption = if network.has_rsn { "WPA2" } else { "开放" };
                println!("   加密: {}", encryption);

                let bssid = network.bssid;
                println!(
                    "   BSSID: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                    bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]
                );

                let channel = network.channel_freq;
                println!("   频率: {} MHz", channel);
            }
        }
        Err(e) => {
            println!("扫描失败: {:?}", e);
        }
    }
}

/// 示例 2: 查找特定网络
fn example_find_network(client: &WifiClient, target_ssid: &str) {
    println!("\n=== 示例 2: 查找特定网络 ===");

    match client.find_network(target_ssid.as_bytes(), 10000) {
        Ok(network) => {
            println!("找到网络 '{}'", target_ssid);
            println!("信号强度: {} dBm", network.rssi);
            println!("加密: {}", if network.has_rsn { "WPA2" } else { "开放" });
        }
        Err(WifiError::NetworkNotFound) => {
            println!("未找到网络 '{}'", target_ssid);
        }
        Err(e) => {
            println!("查找失败: {:?}", e);
        }
    }
}

/// 示例 3: 连接到开放网络
fn example_connect_open(client: &WifiClient, ssid: &str) {
    println!("\n=== 示例 3: 连接到开放网络 ===");

    let config = WifiConfig::open(ssid);

    match client.connect(&config, 15000) {
        Ok(_) => {
            println!("连接请求已发送，等待连接完成...");

            match client.wait_for_connection(10000) {
                Ok(_) => println!("成功连接到 '{}'", ssid),
                Err(e) => println!("等待连接失败: {:?}", e),
            }
        }
        Err(e) => {
            println!("连接失败: {:?}", e);
        }
    }
}

/// 示例 4: 连接到 WPA2-PSK 网络
fn example_connect_wpa2(client: &WifiClient, ssid: &str, password: &str) {
    println!("\n=== 示例 4: 连接到 WPA2 网络 ===");

    let config = WifiConfig::wpa2_psk(ssid, password);

    match client.connect(&config, 15000) {
        Ok(_) => {
            println!("WPA2 连接请求已发送，等待连接完成...");

            match client.wait_for_connection(10000) {
                Ok(_) => println!("成功连接到 WPA2 网络 '{}'", ssid),
                Err(e) => println!("等待连接失败: {:?}", e),
            }
        }
        Err(e) => match e {
            WifiError::InvalidPassword => println!("密码错误"),
            WifiError::AuthenticationFailed => println!("认证失败"),
            WifiError::ConnectionTimeout => println!("连接超时"),
            _ => println!("连接失败: {:?}", e),
        },
    }
}

/// 示例 5: 检查连接状态
fn example_check_status(client: &WifiClient) {
    println!("\n=== 示例 5: 检查连接状态 ===");

    let status = client.get_status();
    match status {
        ConnectionStatus::Disconnected => println!("状态: 未连接"),
        ConnectionStatus::Connecting => println!("状态: 正在连接"),
        ConnectionStatus::Connected => println!("状态: 已连接"),
        ConnectionStatus::Failed => println!("状态: 连接失败"),
    }

    if let Some(ssid) = client.get_current_ssid() {
        let ssid_str = core::str::from_utf8(&ssid).unwrap_or("<invalid>");
        println!("当前连接到: {}", ssid_str);
    }

    if let Some(rssi) = client.get_rssi() {
        println!("信号强度: {} dBm", rssi);
    }
}

/// 示例 6: 断开连接
fn example_disconnect(client: &WifiClient) {
    println!("\n=== 示例 6: 断开连接 ===");

    match client.disconnect() {
        Ok(_) => println!("已断开连接"),
        Err(WifiError::NotConnected) => println!("当前未连接"),
        Err(e) => println!("断开连接失败: {:?}", e),
    }
}

/// 示例 7: 完整的连接流程 (带错误处理)
fn example_full_connection_flow(client: &WifiClient, ssid: &str, password: Option<&str>) {
    println!("\n=== 示例 7: 完整连接流程 ===");

    // 步骤 1: 检查当前状态
    let current_status = client.get_status();
    if current_status == ConnectionStatus::Connected {
        println!("当前已连接，先断开...");
        let _ = client.disconnect();
        axtask::sleep_ms(1000);
    }

    // 步骤 2: 扫描并查找目标网络
    println!("扫描网络...");
    let network = match client.find_network(ssid.as_bytes(), 10000) {
        Ok(net) => net,
        Err(WifiError::NetworkNotFound) => {
            println!("未找到网络 '{}'", ssid);
            return;
        }
        Err(e) => {
            println!("扫描失败: {:?}", e);
            return;
        }
    };

    println!("找到网络 '{}' ({} dBm)", ssid, network.rssi);

    // 步骤 3: 根据网络类型创建配置
    let config = if network.has_rsn {
        if let Some(pwd) = password {
            WifiConfig::wpa2_psk(ssid, pwd)
        } else {
            println!("网络需要密码，但未提供");
            return;
        }
    } else {
        WifiConfig::open(ssid)
    };

    // 步骤 4: 连接
    println!("开始连接...");
    match client.connect(&config, 15000) {
        Ok(_) => {
            println!("连接请求已发送");

            // 步骤 5: 等待连接完成
            match client.wait_for_connection(10000) {
                Ok(_) => {
                    println!("✓ 成功连接到 '{}'", ssid);

                    // 显示连接信息
                    if let Some(rssi) = client.get_rssi() {
                        println!("信号强度: {} dBm", rssi);
                    }
                }
                Err(e) => {
                    println!("✗ 连接超时");
                }
            }
        }
        Err(e) => {
            println!("✗ 连接失败: {:?}", e);
        }
    }
}

/// 示例 8: 自动重连机制
fn example_auto_reconnect(
    client: &WifiClient,
    ssid: &str,
    password: Option<&str>,
    max_retries: u32,
) {
    println!("\n=== 示例 8: 自动重连 ===");

    let config = match password {
        Some(pwd) => WifiConfig::wpa2_psk(ssid, pwd),
        None => WifiConfig::open(ssid),
    };

    for attempt in 1..=max_retries {
        println!("尝试连接 {}/{}", attempt, max_retries);

        match client.connect(&config, 15000) {
            Ok(_) => {
                if client.wait_for_connection(10000).is_ok() {
                    println!("✓ 第 {} 次尝试成功", attempt);
                    return;
                }
            }
            Err(WifiError::ConnectionTimeout) => {
                println!("超时，重试中...");
            }
            Err(WifiError::NetworkNotFound) => {
                println!("未找到网络，等待后重试...");
                axtask::sleep_ms(2000);
                continue;
            }
            Err(e) => {
                println!("连接失败: {:?}，停止重试", e);
                return;
            }
        }

        axtask::sleep_ms(1000);
    }

    println!("✗ {} 次尝试后仍无法连接", max_retries);
}

/// 主函数 - 展示所有示例
#[no_mangle]
pub extern "C" fn wifi_main() {
    println!("AIC8800 WiFi 客户端示例程序");
    println!("============================\n");

    // 假设 WifiBus 已经初始化
    // 在实际使用中，你需要从驱动初始化代码中获取 WifiBus
    let bus = unsafe {
        // 这里应该从全局或驱动上下文获取已初始化的 WifiBus
        // 示例: &mut GLOBAL_WIFI_BUS
        // 这只是演示，实际实现取决于你的系统架构
        core::ptr::null_mut()
    };

    if bus.is_null() {
        println!("错误: WifiBus 未初始化");
        return;
    }

    let client = WifiClient::new(unsafe { Arc::from_raw(bus) });

    // 运行示例 (根据需要注释/取消注释)

    // 示例 1: 扫描网络
    example_scan_networks(&client);

    // 示例 2: 查找特定网络
    example_find_network(&client, "YourNetworkSSID");

    // 示例 3: 连接开放网络
    // example_connect_open(&client, "OpenNetwork");

    // 示例 4: 连接 WPA2 网络
    // example_connect_wpa2(&client, "YourNetworkSSID", "YourPassword");

    // 示例 5: 检查状态
    example_check_status(&client);

    // 示例 6: 断开连接
    // example_disconnect(&client);

    // 示例 7: 完整连接流程
    // example_full_connection_flow(&client, "YourNetworkSSID", Some("YourPassword"));

    // 示例 8: 自动重连
    // example_auto_reconnect(&client, "YourNetworkSSID", Some("YourPassword"), 3);

    println!("\n示例程序结束");
}

// 在实际使用中，你需要:
// 1. 确保 WifiBus 已正确初始化
// 2. 根据你的系统架构调整 bus 获取方式
// 3. 可能需要处理更多的错误情况和边界条件
// 4. 考虑添加更多的日志和调试信息
