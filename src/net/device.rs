//! AIC8800 Wi-Fi network device implementing `NetDriverOps`.  
//!  
//! This module bridges the `WifiBus` TX/RX queues to the ArceOS network  
//! stack (`axnet`) via the `NetDriverOps` trait.  

use alloc::boxed::Box;
use alloc::vec::Vec;
use alloc::{sync::Arc, vec};
use axnet::SocketOps;
use core::{ptr::NonNull, sync::atomic::Ordering};

use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use axdriver_net::{EthernetAddress, NetBufPtr, NetDriverOps};

use crate::{core::bus::WifiBus, thread::tx};

const MAX_TX_QUEUE_LEN: usize = 128;

/// AIC8800 Wi-Fi network device.  
///  
/// Wraps a shared `WifiBus` and implements `NetDriverOps` so that  
/// `axnet`'s `EthernetDevice` can drive Ethernet-level TX/RX.  
pub struct AicWifiNetDev {
    bus: Arc<WifiBus>,
    mac: [u8; 6],
}

// SAFETY: WifiBus internals are protected by atomics and SpinNoIrq locks.
unsafe impl Send for AicWifiNetDev {}
unsafe impl Sync for AicWifiNetDev {}

impl AicWifiNetDev {
    pub fn new(bus: Arc<WifiBus>, mac: [u8; 6]) -> Self {
        Self { bus, mac }
    }

    /// Allocate a buffer backed by a `Box<[u8]>`, returning a raw `NonNull`
    fn alloc_buf(size: usize) -> Option<NonNull<u8>> {
        if size == 0 {
            return None;
        }
        let boxed: Box<[u8]> = vec![0u8; size].into_boxed_slice();
        let ptr = Box::into_raw(boxed) as *mut u8;
        NonNull::new(ptr)
    }

    /// Free a buffer that was allocated by `alloc_buf`.  
    ///  
    /// # Safety  
    /// `ptr` must have been returned by `alloc_buf` with the same `size`.
    fn free_buf(ptr: NonNull<u8>, size: usize) {
        unsafe {
            let raw_slice = core::ptr::slice_from_raw_parts_mut(ptr.as_ptr(), size);
            let _ = Box::from_raw(raw_slice);
        }
    }
}

impl BaseDriverOps for AicWifiNetDev {
    fn device_type(&self) -> DeviceType {
        DeviceType::Net
    }

    fn device_name(&self) -> &str {
        "aic8800-wifi"
    }

    // 返回 SDIO1 CARD_INT 的 PLIC IRQ 号（与 lib.rs 中 axplat::irq::register(38, ...) 一致）
    // axnet 的 EthernetDevice::register_waker 会调用 register_irq_waker(38, waker)
    // 当 IRQ#38 触发时，irq_hook 会唤醒 axnet poll 循环，使主线程立即处理收到的 DATA 帧
    fn irq_num(&self) -> Option<usize> {
        Some(38)
    }
}

impl NetDriverOps for AicWifiNetDev {
    fn mac_address(&self) -> EthernetAddress {
        EthernetAddress(self.mac)
    }

    fn can_transmit(&self) -> bool {
        self.bus.connected_vif_idx.load(Ordering::Acquire) != 0xFF
            && self.bus.tx_pktcnt.load(Ordering::Acquire) < MAX_TX_QUEUE_LEN as u32
    }

    fn can_receive(&self) -> bool {
        !self.bus.data_rx_queue.lock().is_empty()
    }

    fn rx_queue_size(&self) -> usize {
        self.bus.data_rx_queue.lock().len()
    }

    fn tx_queue_size(&self) -> usize {
        self.bus.tx_pktcnt.load(Ordering::Acquire) as usize
    }

    fn alloc_tx_buffer(&mut self, size: usize) -> DevResult<NetBufPtr> {
        let ptr = Self::alloc_buf(size).ok_or(DevError::NoMemory)?;
        // raw_ptr == pkt_ptr; pkt_len == buf_len == size
        Ok(NetBufPtr::new(ptr, ptr, size))
    }

    fn transmit(&mut self, tx_buf: NetBufPtr) -> DevResult {
        // Copy the Ethernet frame out of the NetBufPtr buffer.
        let eth_frame: Vec<u8> = tx_buf.packet().to_vec();
        let buf_ptr = tx_buf.packet().as_ptr() as *mut u8;
        let buf_size = tx_buf.packet_len();

        // Free the TX buffer immediately (we already copied the data).
        // NetBufPtr does NOT implement Drop, so we must free manually.
        if let Some(nn) = NonNull::new(buf_ptr) {
            Self::free_buf(nn, buf_size);
        }
        // Note: tx_buf goes out of scope here — no Drop, so this is fine.

        // Enqueue the Ethernet frame to the Wi-Fi TX queue.
        // The TX thread will construct the HostDesc and send via SDIO.
        tx::enqueue_data_frame(&self.bus, eth_frame).map_err(|_| DevError::Again)?;

        Ok(())
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        // TX buffers are freed immediately in transmit(), nothing to do.
        Ok(())
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        let frame = self
            .bus
            .data_rx_queue
            .lock()
            .pop_front()
            .ok_or(DevError::Again)?;
        let size = frame.len();
        let ptr = Self::alloc_buf(size).ok_or(DevError::NoMemory)?;

        // Copy the Ethernet frame into the newly allocated buffer.
        unsafe {
            core::ptr::copy_nonoverlapping(frame.as_ptr(), ptr.as_ptr(), size);
        }

        Ok(NetBufPtr::new(ptr, ptr, size))
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        let buf_ptr = rx_buf.packet().as_ptr() as *mut u8;
        let buf_size = rx_buf.packet_len();
        // NetBufPtr has no Drop — free manually.
        if let Some(nn) = NonNull::new(buf_ptr) {
            unsafe {
                Self::free_buf(nn, buf_size);
            }
        }
        Ok(())
    }
}

/// High-level helper: create an `AicWifiNetDev` and register it with `axnet`.  
///  
/// Call this from `wifi_main()` after WPA2 handshake succeeds and  
/// connection state has been saved to `WifiBus`.  
///  
/// # Arguments  
/// * `bus`     — shared `WifiBus` (same Arc used by TX/RX threads)  
/// * `mac`     — STA MAC address (6 bytes)  
/// * `ip`      — static IPv4 address string, e.g. `"192.168.1.100"`  
/// * `prefix`  — subnet prefix length, e.g. `24`  
/// * `gateway` — gateway IPv4 address string, e.g. `"192.168.1.1"`  
pub fn register_wifi_device(bus: Arc<WifiBus>, mac: [u8; 6], ip: &str, prefix: u8, gateway: &str) {
    let dev = AicWifiNetDev::new(bus, mac);
    let boxed: Box<dyn NetDriverOps> = Box::new(dev);
    axnet::register_net_device(boxed, ip, prefix, gateway);
    log::info!("[aic8800] Wi-Fi device registered with axnet");
}

/// 简单的网络测试循环：持续 poll smoltcp，使系统能响应 ARP/ICMP。  
/// 从外部 PC 执行 `ping <board_ip>` 即可验证数据通路。  
/// 按需设置 duration_secs，0 表示永久循环。  
pub fn run_network_poll_loop(duration_secs: u64) {
    log::info!(
        "[net-test] Starting network poll loop ({}s)...",
        duration_secs
    );
    log::info!("[net-test] Try `ping {}` from your PC", "192.168.1.200");

    let start = axhal::time::monotonic_time_nanos();
    let timeout_ns = duration_secs * 1_000_000_000;

    loop {
        axnet::poll_interfaces();
        axtask::yield_now();

        if duration_secs > 0 {
            let elapsed = axhal::time::monotonic_time_nanos() - start;
            if elapsed >= timeout_ns {
                log::info!("[net-test] Poll loop finished after {}s", duration_secs);
                break;
            }
        }
    }
}

/// 读取 RISC-V time CSR（避免额外依赖 axhal）  
fn rdtime() -> u64 {
    let time: u64;
    unsafe { core::arch::asm!("rdtime {}", out(reg) time) };
    time
}

/// 将 ticks 转换为纳秒（SG2002 定时器频率 25MHz，NANOS_PER_TICK = 40）  
const NANOS_PER_TICK: u64 = 40;

fn ticks_to_nanos(ticks: u64) -> u64 {
    ticks * NANOS_PER_TICK
}

/// 网络速度测试：TCP TX 吞吐量  
///  
/// 连接到 PC 上的 TCP 服务器，发送指定大小的数据，测量吞吐量。  
/// PC 端需要先运行：`nc -l <port> > /dev/null` 或 `iperf3 -s -p <port>`  
///  
/// # Arguments  
/// * `server_ip` - PC 的 IP 地址，如 "192.168.1.100"  
/// * `server_port` - PC 监听的端口，如 9000  
/// * `total_bytes` - 要发送的总字节数，如 1_000_000 (1MB)  
/// * `chunk_size` - 每次 send 的块大小，如 1024  
pub fn run_speed_test(server_ip: &str, server_port: u16, total_bytes: usize, chunk_size: usize) {
    use axnet::tcp::TcpSocket;
    use axnet::{SendOptions, SocketAddrEx};
    use core::net::{IpAddr, SocketAddr};

    log::info!("[speed-test] ===== Network Speed Test =====");
    log::info!("[speed-test] Target: {}:{}", server_ip, server_port);
    log::info!(
        "[speed-test] Total: {} bytes, Chunk: {} bytes",
        total_bytes,
        chunk_size
    );

    // Phase 1: ARP warmup - poll interfaces to let ARP resolve
    log::info!("[speed-test] Phase 1: ARP warmup...");
    for _ in 0..500 {
        axnet::poll_interfaces();
        // 短暂忙等
        for _ in 0..1000 {
            core::hint::spin_loop();
        }
    }

    // Phase 2: TCP connect
    log::info!(
        "[speed-test] Phase 2: Connecting to {}:{}...",
        server_ip,
        server_port
    );
    let socket = TcpSocket::new();

    let ip: IpAddr = server_ip.parse().expect("Invalid server IP");
    let remote = SocketAddr::new(ip, server_port);

    if let Err(e) = socket.connect(SocketAddrEx::Ip(remote)) {
        log::error!("[speed-test] TCP connect failed: {:?}", e);
        log::info!(
            "[speed-test] Make sure PC is running: nc -l {} > /dev/null",
            server_port
        );
        // Fall through to poll loop
        run_poll_loop();
        return;
    }
    log::info!("[speed-test] Connected!");

    // Phase 3: TX throughput test
    log::info!("[speed-test] Phase 3: Sending {} bytes...", total_bytes);

    let buf = alloc::vec![0xABu8; chunk_size];
    let mut total_sent: usize = 0;
    let mut send_errors: usize = 0;

    let start_ticks = rdtime();

    while total_sent < total_bytes {
        let remaining = total_bytes - total_sent;
        let to_send = if remaining < chunk_size {
            remaining
        } else {
            chunk_size
        };
        let data: &[u8] = &buf[..to_send];

        match socket.send(data, SendOptions::default()) {
            Ok(n) => {
                total_sent += n;
                if total_sent % (100 * 1024) < n {
                    let pct = total_sent * 100 / total_bytes;
                    log::info!(
                        "[speed-test] Progress: {}/{} bytes ({}%)",
                        total_sent,
                        total_bytes,
                        pct
                    );
                }
            }
            Err(e) => {
                send_errors += 1;
                if send_errors > 100 {
                    log::error!("[speed-test] Too many send errors, aborting. Last: {:?}", e);
                    break;
                }
                // Brief pause then retry
                for _ in 0..100 {
                    axnet::poll_interfaces();
                    core::hint::spin_loop();
                }
            }
        }
    }

    let end_ticks = rdtime();
    let elapsed_ns = ticks_to_nanos(end_ticks - start_ticks);
    let elapsed_ms = elapsed_ns / 1_000_000;

    // Calculate throughput
    let bits = (total_sent as u64) * 8;
    let throughput_kbps = if elapsed_ns > 0 {
        bits * 1_000_000_000 / elapsed_ns / 1000
    } else {
        0
    };
    let throughput_mbps = throughput_kbps / 1000;

    log::info!("[speed-test] ===== Results =====");
    log::info!("[speed-test] Sent: {} bytes", total_sent);
    log::info!("[speed-test] Time: {} ms", elapsed_ms);
    log::info!(
        "[speed-test] Throughput: {} Kbps ({}.{} Mbps)",
        throughput_kbps,
        throughput_mbps,
        (throughput_kbps % 1000) / 100
    );
    log::info!("[speed-test] Send errors: {}", send_errors);
    log::info!("[speed-test] ====================");

    // Shutdown socket
    drop(socket);

    // Phase 4: Enter poll loop (respond to ping)
    run_poll_loop();
}

/// 简单的网络轮询循环，响应 ARP 和 ICMP（ping）  
pub fn run_poll_loop() {
    log::info!("[net-test] Entering poll loop. Ping me to verify connectivity!");
    loop {
        axnet::poll_interfaces();
        for _ in 0..10000 {
            core::hint::spin_loop();
        }
    }
}
