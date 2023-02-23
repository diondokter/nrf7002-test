use core::alloc::Layout;
use core::cell::RefCell;
use core::ffi::{c_char, c_int, c_uint, c_ulong, c_void};
use core::mem::{align_of, size_of, transmute, MaybeUninit};
use core::pin::Pin;
use core::ptr::{null_mut, NonNull};
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::RestoreState;
use embassy_nrf::gpio::{AnyPin, Output};
use embassy_nrf::interrupt;
use embassy_nrf::peripherals::{P0_13, P0_14, P0_17, P0_18, UARTETWISPI0};
use embassy_nrf::spim::{Config, Spim};
use embedded_hal_bus::spi::ExclusiveDevice;
use nrf700x_sys::{
    nrf_wifi_data_config_params, nrf_wifi_event_get_wiphy, nrf_wifi_event_remain_on_channel,
    nrf_wifi_interface_info, nrf_wifi_reg, nrf_wifi_sys_iftype, nrf_wifi_umac_cmd_config_twt,
    nrf_wifi_umac_cmd_teardown_twt, nrf_wifi_umac_event_cookie_rsp,
    nrf_wifi_umac_event_get_channel, nrf_wifi_umac_event_get_tx_power, nrf_wifi_umac_event_mlme,
    nrf_wifi_umac_event_new_scan_display_results, nrf_wifi_umac_event_new_scan_results,
    nrf_wifi_umac_event_new_station, nrf_wifi_umac_event_power_save_info,
    nrf_wifi_umac_event_set_interface, nrf_wifi_umac_event_trigger_scan,
    nrf_wifi_umac_event_twt_sleep, rx_buf_pool_params, va_list, wifi_nrf_fmac_callbk_fns,
    wifi_nrf_fmac_if_carr_state, wifi_nrf_osal_dma_dir, wifi_nrf_osal_host_map, wifi_nrf_osal_ops,
    wifi_nrf_osal_priv, wifi_nrf_status, NRF_WIFI_FEATURE_DISABLE,
};

use crate::wifi_bus::WifiBus;

pub struct NrfWifi {
    osal_priv: *mut wifi_nrf_osal_priv,
    fmac_priv: *mut nrf700x_sys::wifi_nrf_fmac_priv,
    fmac_dev_ctx: *mut nrf700x_sys::wifi_nrf_fmac_dev_ctx,
    vif_index: u8,
}

static OPS_INITIALIZED: AtomicBool = AtomicBool::new(false);
static mut OPS: wifi_nrf_osal_ops = wifi_nrf_osal_ops {
    mem_alloc: None,
    mem_zalloc: None,
    mem_free: None,
    mem_cpy: None,
    mem_set: None,
    iomem_mmap: None,
    iomem_unmap: None,
    iomem_read_reg32: None,
    iomem_write_reg32: None,
    iomem_cpy_from: None,
    iomem_cpy_to: None,
    qspi_read_reg32: None,
    qspi_write_reg32: None,
    qspi_cpy_from: None,
    qspi_cpy_to: None,
    spinlock_alloc: None,
    spinlock_free: None,
    spinlock_init: None,
    spinlock_take: None,
    spinlock_rel: None,
    spinlock_irq_take: None,
    spinlock_irq_rel: None,
    log_dbg: None,
    log_info: None,
    log_err: None,
    llist_node_alloc: None,
    llist_node_free: None,
    llist_node_data_get: None,
    llist_node_data_set: None,
    llist_alloc: None,
    llist_free: None,
    llist_init: None,
    llist_add_node_tail: None,
    llist_get_node_head: None,
    llist_get_node_nxt: None,
    llist_del_node: None,
    llist_len: None,
    nbuf_alloc: None,
    nbuf_free: None,
    nbuf_headroom_res: None,
    nbuf_headroom_get: None,
    nbuf_data_size: None,
    nbuf_data_get: None,
    nbuf_data_put: None,
    nbuf_data_push: None,
    nbuf_data_pull: None,
    tasklet_alloc: None,
    tasklet_free: None,
    tasklet_init: None,
    tasklet_schedule: None,
    tasklet_kill: None,
    sleep_ms: None,
    delay_us: None,
    time_get_curr_us: None,
    time_elapsed_us: None,
    bus_pcie_init: None,
    bus_pcie_deinit: None,
    bus_pcie_dev_add: None,
    bus_pcie_dev_rem: None,
    bus_pcie_dev_init: None,
    bus_pcie_dev_deinit: None,
    bus_pcie_dev_intr_reg: None,
    bus_pcie_dev_intr_unreg: None,
    bus_pcie_dev_dma_map: None,
    bus_pcie_dev_dma_unmap: None,
    bus_pcie_dev_host_map_get: None,
    bus_qspi_init: None,
    bus_qspi_deinit: None,
    bus_qspi_dev_add: None,
    bus_qspi_dev_rem: None,
    bus_qspi_dev_init: None,
    bus_qspi_dev_deinit: None,
    bus_qspi_dev_intr_reg: None,
    bus_qspi_dev_intr_unreg: None,
    bus_qspi_dev_host_map_get: None,
    timer_alloc: None,
    timer_free: None,
    timer_init: None,
    timer_schedule: None,
    timer_kill: None,
    bus_qspi_ps_sleep: None,
    bus_qspi_ps_wake: None,
    bus_qspi_ps_status: None,
};

impl NrfWifi {
    pub fn new<P: OsalPlatform + FmacCallbacks>() -> Self {
        if OPS_INITIALIZED
            .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
            .is_err()
        {
            panic!("Already initialized");
        }

        unsafe {
            OPS = wifi_nrf_osal_ops {
                mem_alloc: Some(P::mem_alloc),
                mem_zalloc: Some(P::mem_zalloc),
                mem_free: Some(P::mem_free),
                mem_cpy: Some(P::mem_cpy),
                mem_set: Some(P::mem_set),
                iomem_mmap: Some(P::iomem_mmap),
                iomem_unmap: Some(P::iomem_unmap),
                iomem_read_reg32: Some(P::iomem_read_reg32),
                iomem_write_reg32: Some(P::iomem_write_reg32),
                iomem_cpy_from: Some(P::iomem_cpy_from),
                iomem_cpy_to: Some(P::iomem_cpy_to),
                qspi_read_reg32: Some(P::qspi_read_reg32),
                qspi_write_reg32: Some(P::qspi_write_reg32),
                qspi_cpy_from: Some(P::qspi_cpy_from),
                qspi_cpy_to: Some(P::qspi_cpy_to),
                spinlock_alloc: Some(P::spinlock_alloc),
                spinlock_free: Some(P::spinlock_free),
                spinlock_init: Some(P::spinlock_init),
                spinlock_take: Some(P::spinlock_take),
                spinlock_rel: Some(P::spinlock_rel),
                spinlock_irq_take: Some(P::spinlock_irq_take),
                spinlock_irq_rel: Some(P::spinlock_irq_rel),
                log_dbg: Some(P::log_dbg),
                log_info: Some(P::log_info),
                log_err: Some(P::log_err),
                llist_node_alloc: Some(P::llist_node_alloc),
                llist_node_free: Some(P::llist_node_free),
                llist_node_data_get: Some(P::llist_node_data_get),
                llist_node_data_set: Some(P::llist_node_data_set),
                llist_alloc: Some(P::llist_alloc),
                llist_free: Some(P::llist_free),
                llist_init: Some(P::llist_init),
                llist_add_node_tail: Some(P::llist_add_node_tail),
                llist_get_node_head: Some(P::llist_get_node_head),
                llist_get_node_nxt: Some(P::llist_get_node_nxt),
                llist_del_node: Some(P::llist_del_node),
                llist_len: Some(P::llist_len),
                nbuf_alloc: Some(P::nbuf_alloc),
                nbuf_free: Some(P::nbuf_free),
                nbuf_headroom_res: Some(P::nbuf_headroom_res),
                nbuf_headroom_get: Some(P::nbuf_headroom_get),
                nbuf_data_size: Some(P::nbuf_data_size),
                nbuf_data_get: Some(P::nbuf_data_get),
                nbuf_data_put: Some(P::nbuf_data_put),
                nbuf_data_push: Some(P::nbuf_data_push),
                nbuf_data_pull: Some(P::nbuf_data_pull),
                tasklet_alloc: Some(P::tasklet_alloc),
                tasklet_free: Some(P::tasklet_free),
                tasklet_init: Some(P::tasklet_init),
                tasklet_schedule: Some(P::tasklet_schedule),
                tasklet_kill: Some(P::tasklet_kill),
                sleep_ms: Some(P::sleep_ms),
                delay_us: Some(P::delay_us),
                time_get_curr_us: Some(P::time_get_curr_us),
                time_elapsed_us: Some(P::time_elapsed_us),
                bus_pcie_init: Some(P::bus_pcie_init),
                bus_pcie_deinit: Some(P::bus_pcie_deinit),
                bus_pcie_dev_add: Some(P::bus_pcie_dev_add),
                bus_pcie_dev_rem: Some(P::bus_pcie_dev_rem),
                bus_pcie_dev_init: Some(P::bus_pcie_dev_init),
                bus_pcie_dev_deinit: Some(P::bus_pcie_dev_deinit),
                bus_pcie_dev_intr_reg: Some(P::bus_pcie_dev_intr_reg),
                bus_pcie_dev_intr_unreg: Some(P::bus_pcie_dev_intr_unreg),
                bus_pcie_dev_dma_map: Some(P::bus_pcie_dev_dma_map),
                bus_pcie_dev_dma_unmap: Some(P::bus_pcie_dev_dma_unmap),
                bus_pcie_dev_host_map_get: Some(P::bus_pcie_dev_host_map_get),
                bus_qspi_init: Some(P::bus_qspi_init),
                bus_qspi_deinit: Some(P::bus_qspi_deinit),
                bus_qspi_dev_add: Some(P::bus_qspi_dev_add),
                bus_qspi_dev_rem: Some(P::bus_qspi_dev_rem),
                bus_qspi_dev_init: Some(P::bus_qspi_dev_init),
                bus_qspi_dev_deinit: Some(P::bus_qspi_dev_deinit),
                bus_qspi_dev_intr_reg: Some(P::bus_qspi_dev_intr_reg),
                bus_qspi_dev_intr_unreg: Some(P::bus_qspi_dev_intr_unreg),
                bus_qspi_dev_host_map_get: Some(P::bus_qspi_dev_host_map_get),
                timer_alloc: Some(P::timer_alloc),
                timer_free: Some(P::timer_free),
                timer_init: Some(P::timer_init),
                timer_schedule: Some(P::timer_schedule),
                timer_kill: Some(P::timer_kill),
                bus_qspi_ps_sleep: Some(P::bus_qspi_ps_sleep),
                bus_qspi_ps_wake: Some(P::bus_qspi_ps_wake),
                bus_qspi_ps_status: Some(P::bus_qspi_ps_status),
            }
        }

        defmt::println!("OSAL init");
        unsafe { P::init() };

        let osal_priv = unsafe { nrf700x_sys::wifi_nrf_osal_init() };

        defmt::println!("FMAC init");
        let mut data_config_params = nrf_wifi_data_config_params {
            rate_protection_type: 0,
            aggregation: NRF_WIFI_FEATURE_DISABLE as u8,
            wmm: NRF_WIFI_FEATURE_DISABLE as u8,
            max_num_tx_agg_sessions: 4,
            max_num_rx_agg_sessions: 4,
            max_tx_aggregation: 64,
            reorder_buf_size: 64,
            max_rxampdu_size: nrf700x_sys::max_rx_ampdu_size::MAX_RX_AMPDU_SIZE_32KB as i32,
        };
        let mut rx_buf_pools = rx_buf_pool_params {
            buf_sz: 512,
            num_bufs: 2,
        };
        let mut fmac_callbacks = P::get_callbacks();
        let fmac_priv = unsafe {
            nrf700x_sys::wifi_nrf_fmac_init(
                &mut data_config_params as *mut _,
                &mut rx_buf_pools as *mut _,
                &mut fmac_callbacks as *mut _,
            )
        };

        let fmac_dev_ctx =
            unsafe { nrf700x_sys::wifi_nrf_fmac_dev_add(fmac_priv, osal_priv.cast()) };

        defmt::println!("VIF add");
        let mut vif_info = nrf700x_sys::nrf_wifi_umac_add_vif_info {
            iftype: nrf_wifi_sys_iftype::NRF_WIFI_UMAC_IFTYPE_P2P_CLIENT as i32,
            nrf_wifi_use_4addr: 0,
            mon_flags: 0,
            mac_addr: [0; 6],
            ifacename: [
                b'T' as _, b'e' as _, b's' as _, b't' as _, b'v' as _, b'i' as _, b'f' as _, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
            ],
        };
        let vif_index = unsafe {
            nrf700x_sys::wifi_nrf_fmac_add_vif(
                fmac_dev_ctx.cast(),
                null_mut(),
                &mut vif_info as *mut _,
            )
        };

        Self {
            osal_priv,
            fmac_priv,
            fmac_dev_ctx,
            vif_index,
        }
    }

    pub fn start_scan(&mut self) {
        unsafe {
            let mut scan_info = nrf700x_sys::nrf_wifi_umac_scan_info {
                scan_mode: nrf700x_sys::scan_mode::AUTO_SCAN as i32,
                scan_reason: nrf700x_sys::scan_reason::SCAN_DISPLAY as i32,
                scan_params: nrf700x_sys::nrf_wifi_scan_params {
                    valid_fields: 0,
                    num_scan_ssids: 4,
                    num_scan_channels: 4,
                    scan_flags: 0,
                    scan_ssids: [nrf700x_sys::nrf_wifi_ssid {
                        nrf_wifi_ssid_len: 4,
                        nrf_wifi_ssid: [
                            b't', b'e', b's', b't', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        ],
                    }; 2],
                    ie: nrf700x_sys::nrf_wifi_ie {
                        ie_len: 0,
                        ie: [0; 400],
                    },
                    supp_rates: nrf700x_sys::nrf_wifi_supp_rates {
                        valid_fields: 0,
                        band: nrf700x_sys::nrf_wifi_band::NRF_WIFI_BAND_2GHZ as i32,
                        nrf_wifi_num_rates: 0,
                        rates: [0; 60],
                    },
                    mac_addr: [0; 6],
                    mac_addr_mask: [0; 6],
                    no_cck: 0,
                    oper_ch_duration: 0,
                    scan_duration: [0; 42],
                    probe_cnt: [0; 42],
                    channels: nrf700x_sys::__IncompleteArrayField::new(),
                },
            };
            nrf700x_sys::wifi_nrf_fmac_scan(self.fmac_priv.cast(), 0, &mut scan_info as *mut _);
        }
    }
}

/// This trait exposes Ops which need to be implemented by the underlying OS
/// in order for the WLAN driver to work. The Ops can be directly mapped to OS
/// primitives where a one-to-one mapping is available. In case a mapping is not
/// available an equivalent function will need to be implemented and that
/// function will then need to be mapped to the corresponding Op.
pub trait OsalPlatform {
    unsafe fn init();

    /// Allocate memory of @size bytes and return a pointer to the start
    /// of the memory allocated
    unsafe extern "C" fn mem_alloc(size: usize) -> *mut c_void;
    /// Allocate memory of @size bytes, zero out the memory and return
    /// a pointer to the start of the zeroed out memory.
    unsafe extern "C" fn mem_zalloc(size: usize) -> *mut c_void;
    /// Free up memory which has been allocated using [mem_alloc] or
    /// [mem_zalloc]
    unsafe extern "C" fn mem_free(buf: *mut c_void);
    /// Copy @count number of bytes from @src location in memory to @dest
    /// location in memory.
    unsafe extern "C" fn mem_cpy(
        dest: *mut c_void,
        src: *const c_void,
        count: usize,
    ) -> *mut c_void;
    /// Fill a block of memory of @size bytes starting at @start with a
    /// particular value represented by @val.
    unsafe extern "C" fn mem_set(start: *mut c_void, val: c_int, size: usize) -> *mut c_void;

    /// Map IO memory of @size pointed to by @addr into CPU
    /// space.
    unsafe extern "C" fn iomem_mmap(addr: c_ulong, size: c_ulong) -> *mut c_void;
    /// Unmap IO memory from CPU space that was mapped using
    /// [iomem_mmap].
    unsafe extern "C" fn iomem_unmap(addr: *mut c_void);
    /// Read the value from a 32 bit device register using a memory
    /// mapped address(@addr).
    unsafe extern "C" fn iomem_read_reg32(addr: *const c_void) -> c_uint;
    /// Write a 32 bit value (@val) to a 32 bit device register
    /// using a memory mapped address(@addr).
    unsafe extern "C" fn iomem_write_reg32(addr: *mut c_void, val: c_uint);
    /// Copy a block of data of size @count bytes from memory
    /// mapped device memory(@src) to host memory(@dest).
    unsafe extern "C" fn iomem_cpy_from(dest: *mut c_void, src: *const c_void, count: usize);
    /// Copy a block of data of size @count bytes from host
    /// memory (@src) to memory mapped device memory(@dest).
    unsafe extern "C" fn iomem_cpy_to(dest: *mut c_void, src: *const c_void, count: usize);

    unsafe extern "C" fn qspi_read_reg32(priv_: *mut c_void, addr: c_ulong) -> c_uint;
    unsafe extern "C" fn qspi_write_reg32(priv_: *mut c_void, addr: c_ulong, val: c_uint);
    unsafe extern "C" fn qspi_cpy_from(
        priv_: *mut c_void,
        dest: *mut c_void,
        addr: c_ulong,
        count: usize,
    );
    unsafe extern "C" fn qspi_cpy_to(
        priv_: *mut c_void,
        addr: c_ulong,
        src: *const c_void,
        count: usize,
    );

    /// Allocate a busy lock.
    unsafe extern "C" fn spinlock_alloc() -> *mut c_void;
    /// Free a busy lock (@lock) allocated by [spinlock_alloc]
    unsafe extern "C" fn spinlock_free(lock: *mut c_void);
    /// Initialize a busy lock (@lock) allocated by [spinlock_alloc].
    unsafe extern "C" fn spinlock_init(lock: *mut c_void);
    /// Acquire a busy lock (@lock) allocated by [spinlock_alloc].
    unsafe extern "C" fn spinlock_take(lock: *mut c_void);
    /// Release a busy lock (@lock) acquired by [spinlock_take].
    unsafe extern "C" fn spinlock_rel(lock: *mut c_void);
    /// Save interrupt states (@flags), disable interrupts and
    /// take a lock (@lock).
    unsafe extern "C" fn spinlock_irq_take(lock: *mut c_void, flags: *mut c_ulong);
    /// Restore interrupt states (@flags) and release lock (@lock)
    /// acquired using @spinlock_irq_take.
    unsafe extern "C" fn spinlock_irq_rel(lock: *mut c_void, flags: *mut c_ulong);

    /// Log a debug message.
    unsafe extern "C" fn log_dbg(fmt: *const c_char, args: va_list) -> c_int;
    /// Log an informational message.
    unsafe extern "C" fn log_info(fmt: *const c_char, args: va_list) -> c_int;
    /// Log an error message.
    unsafe extern "C" fn log_err(fmt: *const c_char, args: va_list) -> c_int;

    unsafe extern "C" fn llist_node_alloc() -> *mut c_void;
    unsafe extern "C" fn llist_node_free(node: *mut c_void);
    unsafe extern "C" fn llist_node_data_get(node: *mut c_void) -> *mut c_void;
    unsafe extern "C" fn llist_node_data_set(node: *mut c_void, data: *mut c_void);
    unsafe extern "C" fn llist_alloc() -> *mut c_void;
    unsafe extern "C" fn llist_free(llist: *mut c_void);
    unsafe extern "C" fn llist_init(llist: *mut c_void);
    unsafe extern "C" fn llist_add_node_tail(llist: *mut c_void, llist_node: *mut c_void);
    unsafe extern "C" fn llist_get_node_head(llist: *mut c_void) -> *mut c_void;
    unsafe extern "C" fn llist_get_node_nxt(
        llist: *mut c_void,
        llist_node: *mut c_void,
    ) -> *mut c_void;
    unsafe extern "C" fn llist_del_node(llist: *mut c_void, llist_node: *mut c_void);
    unsafe extern "C" fn llist_len(llist: *mut c_void) -> c_uint;
    unsafe extern "C" fn nbuf_alloc(size: c_uint) -> *mut c_void;
    unsafe extern "C" fn nbuf_free(nbuf: *mut c_void);
    unsafe extern "C" fn nbuf_headroom_res(nbuf: *mut c_void, size: c_uint);
    unsafe extern "C" fn nbuf_headroom_get(nbuf: *mut c_void) -> c_uint;
    unsafe extern "C" fn nbuf_data_size(nbuf: *mut c_void) -> c_uint;
    unsafe extern "C" fn nbuf_data_get(nbuf: *mut c_void) -> *mut c_void;
    unsafe extern "C" fn nbuf_data_put(nbuf: *mut c_void, size: c_uint) -> *mut c_void;
    unsafe extern "C" fn nbuf_data_push(nbuf: *mut c_void, size: c_uint) -> *mut c_void;
    unsafe extern "C" fn nbuf_data_pull(nbuf: *mut c_void, size: c_uint) -> *mut c_void;
    unsafe extern "C" fn tasklet_alloc() -> *mut c_void;
    unsafe extern "C" fn tasklet_free(tasklet: *mut c_void);
    unsafe extern "C" fn tasklet_init(
        tasklet: *mut c_void,
        callback: Option<unsafe extern "C" fn(arg1: c_ulong)>,
        data: c_ulong,
    );
    unsafe extern "C" fn tasklet_schedule(tasklet: *mut c_void);
    unsafe extern "C" fn tasklet_kill(tasklet: *mut c_void);
    unsafe extern "C" fn sleep_ms(msecs: c_int) -> c_int;
    unsafe extern "C" fn delay_us(usecs: c_int) -> c_int;
    unsafe extern "C" fn time_get_curr_us() -> c_ulong;
    unsafe extern "C" fn time_elapsed_us(start_time_us: c_ulong) -> c_uint;
    unsafe extern "C" fn bus_pcie_init(
        dev_name: *const c_char,
        vendor_id: c_uint,
        sub_vendor_id: c_uint,
        device_id: c_uint,
        sub_device_id: c_uint,
    ) -> *mut c_void;
    unsafe extern "C" fn bus_pcie_deinit(os_pcie_priv: *mut c_void);
    unsafe extern "C" fn bus_pcie_dev_add(
        pcie_priv: *mut c_void,
        osal_pcie_dev_ctx: *mut c_void,
    ) -> *mut c_void;
    unsafe extern "C" fn bus_pcie_dev_rem(os_pcie_dev_ctx: *mut c_void);
    unsafe extern "C" fn bus_pcie_dev_init(os_pcie_dev_ctx: *mut c_void) -> wifi_nrf_status;
    unsafe extern "C" fn bus_pcie_dev_deinit(os_pcie_dev_ctx: *mut c_void);
    unsafe extern "C" fn bus_pcie_dev_intr_reg(
        os_pcie_dev_ctx: *mut c_void,
        callbk_data: *mut c_void,
        callback_fn: Option<unsafe extern "C" fn(callbk_data: *mut c_void) -> c_int>,
    ) -> wifi_nrf_status;
    unsafe extern "C" fn bus_pcie_dev_intr_unreg(os_pcie_dev_ctx: *mut c_void);
    unsafe extern "C" fn bus_pcie_dev_dma_map(
        os_pcie_dev_ctx: *mut c_void,
        virt_addr: *mut c_void,
        size: usize,
        dir: wifi_nrf_osal_dma_dir,
    ) -> *mut c_void;
    unsafe extern "C" fn bus_pcie_dev_dma_unmap(
        os_pcie_dev_ctx: *mut c_void,
        dma_addr: *mut c_void,
        size: usize,
        dir: wifi_nrf_osal_dma_dir,
    );
    unsafe extern "C" fn bus_pcie_dev_host_map_get(
        os_pcie_dev_ctx: *mut c_void,
        host_map: *mut wifi_nrf_osal_host_map,
    );
    unsafe extern "C" fn bus_qspi_init() -> *mut c_void;
    unsafe extern "C" fn bus_qspi_deinit(os_qspi_priv: *mut c_void);
    unsafe extern "C" fn bus_qspi_dev_add(
        qspi_priv: *mut c_void,
        osal_qspi_dev_ctx: *mut c_void,
    ) -> *mut c_void;
    unsafe extern "C" fn bus_qspi_dev_rem(os_qspi_dev_ctx: *mut c_void);
    unsafe extern "C" fn bus_qspi_dev_init(os_qspi_dev_ctx: *mut c_void) -> wifi_nrf_status;
    unsafe extern "C" fn bus_qspi_dev_deinit(os_qspi_dev_ctx: *mut c_void);
    unsafe extern "C" fn bus_qspi_dev_intr_reg(
        os_qspi_dev_ctx: *mut c_void,
        callbk_data: *mut c_void,
        callback_fn: Option<unsafe extern "C" fn(callbk_data: *mut c_void) -> c_int>,
    ) -> wifi_nrf_status;
    unsafe extern "C" fn bus_qspi_dev_intr_unreg(os_qspi_dev_ctx: *mut c_void);
    unsafe extern "C" fn bus_qspi_dev_host_map_get(
        os_qspi_dev_ctx: *mut c_void,
        host_map: *mut wifi_nrf_osal_host_map,
    );
    unsafe extern "C" fn timer_alloc() -> *mut c_void;
    unsafe extern "C" fn timer_free(timer: *mut c_void);
    unsafe extern "C" fn timer_init(
        timer: *mut c_void,
        callback: Option<unsafe extern "C" fn(arg1: c_ulong)>,
        data: c_ulong,
    );
    unsafe extern "C" fn timer_schedule(timer: *mut c_void, duration: c_ulong);
    unsafe extern "C" fn timer_kill(timer: *mut c_void);
    unsafe extern "C" fn bus_qspi_ps_sleep(os_qspi_priv: *mut c_void) -> c_int;
    unsafe extern "C" fn bus_qspi_ps_wake(os_qspi_priv: *mut c_void) -> c_int;
    unsafe extern "C" fn bus_qspi_ps_status(os_qspi_priv: *mut c_void) -> c_int;
}

#[no_mangle]
extern "C" fn get_os_ops() -> *mut nrf700x_sys::wifi_nrf_osal_ops {
    unsafe { &mut OPS as _ }
}

pub type Spi = WifiBus<ExclusiveDevice<Spim<'static, UARTETWISPI0>, Output<'static, AnyPin>>>;

pub trait FmacCallbacks {
    unsafe extern "C" fn if_carr_state_chg_callbk_fn(
        os_vif_ctx: *mut c_void,
        cs: wifi_nrf_fmac_if_carr_state,
    ) -> wifi_nrf_status;
    unsafe extern "C" fn rx_frm_callbk_fn(os_vif_ctx: *mut c_void, frm: *mut c_void);
    unsafe extern "C" fn scan_start_callbk_fn(
        os_vif_ctx: *mut c_void,
        scan_start_event: *mut nrf_wifi_umac_event_trigger_scan,
        event_len: c_uint,
    );
    unsafe extern "C" fn scan_done_callbk_fn(
        os_vif_ctx: *mut c_void,
        scan_done_event: *mut nrf_wifi_umac_event_trigger_scan,
        event_len: c_uint,
    );
    unsafe extern "C" fn scan_abort_callbk_fn(
        os_vif_ctx: *mut c_void,
        scan_done_event: *mut nrf_wifi_umac_event_trigger_scan,
        event_len: c_uint,
    );
    unsafe extern "C" fn scan_res_callbk_fn(
        os_vif_ctx: *mut c_void,
        scan_res: *mut nrf_wifi_umac_event_new_scan_results,
        event_len: c_uint,
        more_res: bool,
    );
    unsafe extern "C" fn disp_scan_res_callbk_fn(
        os_vif_ctx: *mut c_void,
        scan_res: *mut nrf_wifi_umac_event_new_scan_display_results,
        event_len: c_uint,
        more_res: bool,
    );
    unsafe extern "C" fn auth_resp_callbk_fn(
        os_vif_ctx: *mut c_void,
        auth_resp_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    );
    unsafe extern "C" fn assoc_resp_callbk_fn(
        os_vif_ctx: *mut c_void,
        assoc_resp_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    );
    unsafe extern "C" fn deauth_callbk_fn(
        os_vif_ctx: *mut c_void,
        deauth_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    );
    unsafe extern "C" fn disassoc_callbk_fn(
        os_vif_ctx: *mut c_void,
        disassoc_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    );
    unsafe extern "C" fn mgmt_rx_callbk_fn(
        os_vif_ctx: *mut c_void,
        mgmt_rx_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    );
    unsafe extern "C" fn unprot_mlme_mgmt_rx_callbk_fn(
        os_vif_ctx: *mut c_void,
        unprot_mlme_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    );
    unsafe extern "C" fn tx_pwr_get_callbk_fn(
        os_vif_ctx: *mut c_void,
        info: *mut nrf_wifi_umac_event_get_tx_power,
        event_len: c_uint,
    );
    unsafe extern "C" fn chnl_get_callbk_fn(
        os_vif_ctx: *mut c_void,
        info: *mut nrf_wifi_umac_event_get_channel,
        event_len: c_uint,
    );
    unsafe extern "C" fn sta_get_callbk_fn(
        os_vif_ctx: *mut c_void,
        info: *mut nrf_wifi_umac_event_new_station,
        event_len: c_uint,
    );
    unsafe extern "C" fn cookie_rsp_callbk_fn(
        os_vif_ctx: *mut c_void,
        cookie_rsp: *mut nrf_wifi_umac_event_cookie_rsp,
        event_len: c_uint,
    );
    unsafe extern "C" fn tx_status_callbk_fn(
        os_vif_ctx: *mut c_void,
        tx_status_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    );
    unsafe extern "C" fn set_if_callbk_fn(
        os_vif_ctx: *mut c_void,
        set_if_event: *mut nrf_wifi_umac_event_set_interface,
        event_len: c_uint,
    );
    unsafe extern "C" fn roc_callbk_fn(
        os_vif_ctx: *mut c_void,
        roc_event: *mut nrf_wifi_event_remain_on_channel,
        event_len: c_uint,
    );
    unsafe extern "C" fn roc_cancel_callbk_fn(
        os_vif_ctx: *mut c_void,
        roc_cancel_event: *mut nrf_wifi_event_remain_on_channel,
        event_len: c_uint,
    );
    unsafe extern "C" fn get_station_callbk_fn(
        os_vif_ctx: *mut c_void,
        info: *mut nrf_wifi_umac_event_new_station,
        event_len: c_uint,
    );
    unsafe extern "C" fn get_interface_callbk_fn(
        os_vif_ctx: *mut c_void,
        info: *mut nrf_wifi_interface_info,
        event_len: c_uint,
    );
    unsafe extern "C" fn mgmt_tx_status(
        if_priv: *mut c_void,
        mlme_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    );
    unsafe extern "C" fn twt_config_callbk_fn(
        if_priv: *mut c_void,
        twt_config_event_info: *mut nrf_wifi_umac_cmd_config_twt,
        event_len: c_uint,
    );
    unsafe extern "C" fn twt_teardown_callbk_fn(
        if_priv: *mut c_void,
        twt_teardown_event_info: *mut nrf_wifi_umac_cmd_teardown_twt,
        event_len: c_uint,
    );
    unsafe extern "C" fn event_get_wiphy(
        if_priv: *mut c_void,
        get_wiphy: *mut nrf_wifi_event_get_wiphy,
        event_len: c_uint,
    );
    unsafe extern "C" fn twt_sleep_callbk_fn(
        if_priv: *mut c_void,
        twt_sleep_event_info: *mut nrf_wifi_umac_event_twt_sleep,
        event_len: c_uint,
    );
    unsafe extern "C" fn event_get_reg(
        if_priv: *mut c_void,
        get_reg: *mut nrf_wifi_reg,
        event_len: c_uint,
    );
    unsafe extern "C" fn event_get_ps_info(
        if_priv: *mut c_void,
        get_ps_config: *mut nrf_wifi_umac_event_power_save_info,
        event_len: c_uint,
    );

    fn get_callbacks() -> wifi_nrf_fmac_callbk_fns {
        wifi_nrf_fmac_callbk_fns {
            if_carr_state_chg_callbk_fn: Some(Self::if_carr_state_chg_callbk_fn),
            rx_frm_callbk_fn: Some(Self::rx_frm_callbk_fn),
            scan_start_callbk_fn: Some(Self::scan_start_callbk_fn),
            scan_done_callbk_fn: Some(Self::scan_done_callbk_fn),
            scan_abort_callbk_fn: Some(Self::scan_abort_callbk_fn),
            scan_res_callbk_fn: Some(Self::scan_res_callbk_fn),
            disp_scan_res_callbk_fn: Some(Self::disp_scan_res_callbk_fn),
            auth_resp_callbk_fn: Some(Self::auth_resp_callbk_fn),
            assoc_resp_callbk_fn: Some(Self::assoc_resp_callbk_fn),
            deauth_callbk_fn: Some(Self::deauth_callbk_fn),
            disassoc_callbk_fn: Some(Self::disassoc_callbk_fn),
            mgmt_rx_callbk_fn: Some(Self::mgmt_rx_callbk_fn),
            unprot_mlme_mgmt_rx_callbk_fn: Some(Self::unprot_mlme_mgmt_rx_callbk_fn),
            tx_pwr_get_callbk_fn: Some(Self::tx_pwr_get_callbk_fn),
            chnl_get_callbk_fn: Some(Self::chnl_get_callbk_fn),
            sta_get_callbk_fn: Some(Self::sta_get_callbk_fn),
            cookie_rsp_callbk_fn: Some(Self::cookie_rsp_callbk_fn),
            tx_status_callbk_fn: Some(Self::tx_status_callbk_fn),
            set_if_callbk_fn: Some(Self::set_if_callbk_fn),
            roc_callbk_fn: Some(Self::roc_callbk_fn),
            roc_cancel_callbk_fn: Some(Self::roc_cancel_callbk_fn),
            get_station_callbk_fn: Some(Self::get_station_callbk_fn),
            get_interface_callbk_fn: Some(Self::get_interface_callbk_fn),
            mgmt_tx_status: Some(Self::mgmt_tx_status),
            twt_config_callbk_fn: Some(Self::twt_config_callbk_fn),
            twt_teardown_callbk_fn: Some(Self::twt_teardown_callbk_fn),
            event_get_wiphy: Some(Self::event_get_wiphy),
            twt_sleep_callbk_fn: Some(Self::twt_sleep_callbk_fn),
            event_get_reg: Some(Self::event_get_reg),
            event_get_ps_info: Some(Self::event_get_ps_info),
        }
    }
}

// Custom impl ---------------------------------------

pub struct CurrentPlatform;

static HEAP: critical_section::Mutex<RefCell<linked_list_allocator::Heap>> =
    critical_section::Mutex::new(RefCell::new(linked_list_allocator::Heap::empty()));

impl OsalPlatform for CurrentPlatform {
    unsafe fn init() {
        static mut HEAP_BUFFER: [MaybeUninit<u8>; 1024] = [MaybeUninit::uninit(); 1024];
        critical_section::with(|cs| unsafe {
            HEAP.borrow_ref_mut(cs).init_from_slice(&mut HEAP_BUFFER)
        })
    }

    unsafe extern "C" fn mem_alloc(size: usize) -> *mut core::ffi::c_void {
        defmt::trace!("mem_alloc: {}", size);

        let alloc_size = size + size_of::<usize>();

        // We allocate a usize extra that we prepend with the size of the allocation
        let ptr = critical_section::with(|cs| {
            HEAP.borrow_ref_mut(cs).allocate_first_fit(
                Layout::from_size_align(alloc_size, align_of::<usize>()).unwrap(),
            )
        })
        .unwrap()
        .cast::<usize>();

        unsafe {
            ptr.as_ptr().write(size);
        }

        ptr.as_ptr().add(1).cast()
    }

    unsafe extern "C" fn mem_zalloc(size: usize) -> *mut core::ffi::c_void {
        defmt::trace!("mem_zalloc: {}", size);

        let alloc = Self::mem_alloc(size);
        Self::mem_set(alloc, 0, size);
        alloc
    }

    unsafe extern "C" fn mem_free(buf: *mut core::ffi::c_void) {
        defmt::trace!("mem_free: {}", buf);

        // The size can be found just in front of what this pointer points to
        let ptr = buf.cast::<usize>();

        let alloc_size = unsafe { ptr.read() };

        critical_section::with(|cs| {
            HEAP.borrow_ref_mut(cs).deallocate(
                NonNull::new(ptr.cast()).unwrap(),
                Layout::from_size_align(alloc_size, align_of::<usize>()).unwrap(),
            )
        });
    }

    unsafe extern "C" fn mem_cpy(
        dest: *mut core::ffi::c_void,
        src: *const core::ffi::c_void,
        count: usize,
    ) -> *mut core::ffi::c_void {
        defmt::trace!("mem_cpy: {}, {}, {}", dest, src, count);

        dest.copy_from(src, count);
        dest
    }

    unsafe extern "C" fn mem_set(
        start: *mut core::ffi::c_void,
        val: core::ffi::c_int,
        size: usize,
    ) -> *mut core::ffi::c_void {
        defmt::trace!("mem_set: {}, {}, {}", start, val, size);

        unsafe { core::ptr::write_bytes(start, val as u8, size) };
        start
    }

    unsafe extern "C" fn iomem_mmap(
        addr: core::ffi::c_ulong,
        size: core::ffi::c_ulong,
    ) -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn iomem_unmap(addr: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn iomem_read_reg32(addr: *const core::ffi::c_void) -> core::ffi::c_uint {
        todo!()
    }

    unsafe extern "C" fn iomem_write_reg32(addr: *mut core::ffi::c_void, val: core::ffi::c_uint) {
        todo!()
    }

    unsafe extern "C" fn iomem_cpy_from(
        dest: *mut core::ffi::c_void,
        src: *const core::ffi::c_void,
        count: usize,
    ) {
        todo!()
    }

    unsafe extern "C" fn iomem_cpy_to(
        dest: *mut core::ffi::c_void,
        src: *const core::ffi::c_void,
        count: usize,
    ) {
        todo!()
    }

    unsafe extern "C" fn qspi_read_reg32(
        priv_: *mut core::ffi::c_void,
        addr: core::ffi::c_ulong,
    ) -> core::ffi::c_uint {
        todo!()
    }

    unsafe extern "C" fn qspi_write_reg32(
        priv_: *mut core::ffi::c_void,
        addr: core::ffi::c_ulong,
        val: core::ffi::c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn qspi_cpy_from(
        priv_: *mut core::ffi::c_void,
        dest: *mut core::ffi::c_void,
        addr: core::ffi::c_ulong,
        count: usize,
    ) {
        todo!()
    }

    unsafe extern "C" fn qspi_cpy_to(
        priv_: *mut core::ffi::c_void,
        addr: core::ffi::c_ulong,
        src: *const core::ffi::c_void,
        count: usize,
    ) {
        todo!()
    }

    unsafe extern "C" fn spinlock_alloc() -> *mut core::ffi::c_void {
        Self::mem_alloc(size_of::<Spinlock>())
    }

    unsafe extern "C" fn spinlock_free(lock: *mut core::ffi::c_void) {
        lock.cast::<Spinlock>().drop_in_place();
        Self::mem_free(lock);
    }

    unsafe extern "C" fn spinlock_init(lock: *mut core::ffi::c_void) {
        lock.cast::<Spinlock>().write(Spinlock::new());
    }

    unsafe extern "C" fn spinlock_take(lock: *mut core::ffi::c_void) {
        (*lock.cast::<Spinlock>()).take();
    }

    unsafe extern "C" fn spinlock_rel(lock: *mut core::ffi::c_void) {
        (*lock.cast::<Spinlock>()).release();
    }

    unsafe extern "C" fn spinlock_irq_take(
        lock: *mut core::ffi::c_void,
        flags: *mut core::ffi::c_ulong,
    ) {
        (*lock.cast::<Spinlock>()).take();
    }

    unsafe extern "C" fn spinlock_irq_rel(
        lock: *mut core::ffi::c_void,
        flags: *mut core::ffi::c_ulong,
    ) {
        (*lock.cast::<Spinlock>()).release();
    }

    unsafe extern "C" fn log_dbg(
        fmt: *const core::ffi::c_char,
        args: nrf700x_sys::va_list,
    ) -> core::ffi::c_int {
        defmt::println!("log_dbg");

        todo!()
    }

    unsafe extern "C" fn log_info(
        fmt: *const core::ffi::c_char,
        args: nrf700x_sys::va_list,
    ) -> core::ffi::c_int {
        defmt::println!("log_info");

        todo!()
    }

    unsafe extern "C" fn log_err(
        fmt: *const core::ffi::c_char,
        args: nrf700x_sys::va_list,
    ) -> core::ffi::c_int {
        defmt::println!("log_err");

        use tinyrlibc as _;

        extern "C" {
            fn snprintf(
                buf: *mut tinyrlibc::CChar,
                len: usize,
                fmt: *const tinyrlibc::CChar,
                ...
            ) -> i32;
        }

        let mut buffer = [0u8; 256];
        snprintf(&mut buffer as *mut _, 256, fmt.cast(), args);

        defmt::error!("{}", core::str::from_utf8(&buffer).unwrap());

        0
    }

    unsafe extern "C" fn llist_node_alloc() -> *mut core::ffi::c_void {
        Self::mem_alloc(size_of::<Node>())
    }

    unsafe extern "C" fn llist_node_free(node: *mut core::ffi::c_void) {
        node.cast::<Node>().drop_in_place();
        Self::mem_free(node);
    }

    unsafe extern "C" fn llist_node_data_get(
        node: *mut core::ffi::c_void,
    ) -> *mut core::ffi::c_void {
        (*node.cast::<Node>()).data
    }

    unsafe extern "C" fn llist_node_data_set(
        node: *mut core::ffi::c_void,
        data: *mut core::ffi::c_void,
    ) {
        (*node.cast::<Node>()).data = data;
    }

    unsafe extern "C" fn llist_alloc() -> *mut core::ffi::c_void {
        Self::mem_alloc(size_of::<LinkedList>())
    }

    unsafe extern "C" fn llist_free(llist: *mut core::ffi::c_void) {
        llist.cast::<LinkedList>().drop_in_place();
        Self::mem_free(llist);
    }

    unsafe extern "C" fn llist_init(llist: *mut core::ffi::c_void) {
        llist.cast::<LinkedList>().write(LinkedList::new());
    }

    unsafe extern "C" fn llist_add_node_tail(
        llist: *mut core::ffi::c_void,
        llist_node: *mut core::ffi::c_void,
    ) {
        let mut current_node = Self::llist_get_node_head(llist);

        while !Self::llist_get_node_nxt(llist, llist_node).is_null() {
            current_node = Self::llist_get_node_nxt(llist, llist_node);
        }

        (*current_node.cast::<Node>()).next = llist_node;
    }

    unsafe extern "C" fn llist_get_node_head(
        llist: *mut core::ffi::c_void,
    ) -> *mut core::ffi::c_void {
        (*llist.cast::<LinkedList>()).head.cast()
    }

    unsafe extern "C" fn llist_get_node_nxt(
        llist: *mut core::ffi::c_void,
        llist_node: *mut core::ffi::c_void,
    ) -> *mut core::ffi::c_void {
        (*llist_node.cast::<Node>()).next
    }

    unsafe extern "C" fn llist_del_node(
        llist: *mut core::ffi::c_void,
        llist_node: *mut core::ffi::c_void,
    ) {
        let mut current_node = Self::llist_get_node_head(llist);

        while Self::llist_get_node_nxt(llist, llist_node) != llist_node {
            current_node = Self::llist_get_node_nxt(llist, llist_node);
        }

        (*current_node.cast::<Node>()).next = (*llist_node.cast::<Node>()).next;
    }

    unsafe extern "C" fn llist_len(llist: *mut core::ffi::c_void) -> core::ffi::c_uint {
        todo!()
    }

    unsafe extern "C" fn nbuf_alloc(size: core::ffi::c_uint) -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn nbuf_free(nbuf: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn nbuf_headroom_res(nbuf: *mut core::ffi::c_void, size: core::ffi::c_uint) {
        todo!()
    }

    unsafe extern "C" fn nbuf_headroom_get(nbuf: *mut core::ffi::c_void) -> core::ffi::c_uint {
        todo!()
    }

    unsafe extern "C" fn nbuf_data_size(nbuf: *mut core::ffi::c_void) -> core::ffi::c_uint {
        todo!()
    }

    unsafe extern "C" fn nbuf_data_get(nbuf: *mut core::ffi::c_void) -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn nbuf_data_put(
        nbuf: *mut core::ffi::c_void,
        size: core::ffi::c_uint,
    ) -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn nbuf_data_push(
        nbuf: *mut core::ffi::c_void,
        size: core::ffi::c_uint,
    ) -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn nbuf_data_pull(
        nbuf: *mut core::ffi::c_void,
        size: core::ffi::c_uint,
    ) -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn tasklet_alloc() -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn tasklet_free(tasklet: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn tasklet_init(
        tasklet: *mut core::ffi::c_void,
        callback: core::option::Option<unsafe extern "C" fn(arg1: core::ffi::c_ulong)>,
        data: core::ffi::c_ulong,
    ) {
        todo!()
    }

    unsafe extern "C" fn tasklet_schedule(tasklet: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn tasklet_kill(tasklet: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn sleep_ms(msecs: core::ffi::c_int) -> core::ffi::c_int {
        todo!()
    }

    unsafe extern "C" fn delay_us(usecs: core::ffi::c_int) -> core::ffi::c_int {
        todo!()
    }

    unsafe extern "C" fn time_get_curr_us() -> core::ffi::c_ulong {
        todo!()
    }

    unsafe extern "C" fn time_elapsed_us(start_time_us: core::ffi::c_ulong) -> core::ffi::c_uint {
        todo!()
    }

    unsafe extern "C" fn bus_pcie_init(
        dev_name: *const core::ffi::c_char,
        vendor_id: core::ffi::c_uint,
        sub_vendor_id: core::ffi::c_uint,
        device_id: core::ffi::c_uint,
        sub_device_id: core::ffi::c_uint,
    ) -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn bus_pcie_deinit(os_pcie_priv: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn bus_pcie_dev_add(
        pcie_priv: *mut core::ffi::c_void,
        osal_pcie_dev_ctx: *mut core::ffi::c_void,
    ) -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn bus_pcie_dev_rem(os_pcie_dev_ctx: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn bus_pcie_dev_init(
        os_pcie_dev_ctx: *mut core::ffi::c_void,
    ) -> nrf700x_sys::wifi_nrf_status {
        todo!()
    }

    unsafe extern "C" fn bus_pcie_dev_deinit(os_pcie_dev_ctx: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn bus_pcie_dev_intr_reg(
        os_pcie_dev_ctx: *mut core::ffi::c_void,
        callbk_data: *mut core::ffi::c_void,
        callback_fn: core::option::Option<
            unsafe extern "C" fn(callbk_data: *mut core::ffi::c_void) -> core::ffi::c_int,
        >,
    ) -> nrf700x_sys::wifi_nrf_status {
        todo!()
    }

    unsafe extern "C" fn bus_pcie_dev_intr_unreg(os_pcie_dev_ctx: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn bus_pcie_dev_dma_map(
        os_pcie_dev_ctx: *mut core::ffi::c_void,
        virt_addr: *mut core::ffi::c_void,
        size: usize,
        dir: nrf700x_sys::wifi_nrf_osal_dma_dir,
    ) -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn bus_pcie_dev_dma_unmap(
        os_pcie_dev_ctx: *mut core::ffi::c_void,
        dma_addr: *mut core::ffi::c_void,
        size: usize,
        dir: nrf700x_sys::wifi_nrf_osal_dma_dir,
    ) {
        todo!()
    }

    unsafe extern "C" fn bus_pcie_dev_host_map_get(
        os_pcie_dev_ctx: *mut core::ffi::c_void,
        host_map: *mut nrf700x_sys::wifi_nrf_osal_host_map,
    ) {
        todo!()
    }

    unsafe extern "C" fn bus_qspi_init() -> *mut core::ffi::c_void {
        defmt::trace!("bus_qspi_init");

        let spi = Self::mem_alloc(size_of::<Spi>()).cast::<Spi>();

        let bus_irq = interrupt::take!(SERIAL0);
        let bus = Spim::new(
            transmute::<_, UARTETWISPI0>(()),
            bus_irq,
            transmute::<_, P0_17>(()),
            transmute::<_, P0_14>(()),
            transmute::<_, P0_13>(()),
            Config::default(),
        );
        let cs = Output::new(
            AnyPin::from(transmute::<_, P0_18>(())),
            embassy_nrf::gpio::Level::High,
            embassy_nrf::gpio::OutputDrive::Standard,
        );

        spi.write(WifiBus::new(ExclusiveDevice::new(bus, cs)));

        spi.cast()
    }

    unsafe extern "C" fn bus_qspi_deinit(os_qspi_priv: *mut core::ffi::c_void) {
        os_qspi_priv.cast::<Spi>().drop_in_place();
        Self::mem_free(os_qspi_priv);
    }

    unsafe extern "C" fn bus_qspi_dev_add(
        qspi_priv: *mut core::ffi::c_void,
        osal_qspi_dev_ctx: *mut core::ffi::c_void,
    ) -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn bus_qspi_dev_rem(os_qspi_dev_ctx: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn bus_qspi_dev_init(
        os_qspi_dev_ctx: *mut core::ffi::c_void,
    ) -> nrf700x_sys::wifi_nrf_status {
        todo!()
    }

    unsafe extern "C" fn bus_qspi_dev_deinit(os_qspi_dev_ctx: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn bus_qspi_dev_intr_reg(
        os_qspi_dev_ctx: *mut core::ffi::c_void,
        callbk_data: *mut core::ffi::c_void,
        callback_fn: core::option::Option<
            unsafe extern "C" fn(callbk_data: *mut core::ffi::c_void) -> core::ffi::c_int,
        >,
    ) -> nrf700x_sys::wifi_nrf_status {
        todo!()
    }

    unsafe extern "C" fn bus_qspi_dev_intr_unreg(os_qspi_dev_ctx: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn bus_qspi_dev_host_map_get(
        os_qspi_dev_ctx: *mut core::ffi::c_void,
        host_map: *mut nrf700x_sys::wifi_nrf_osal_host_map,
    ) {
        todo!()
    }

    unsafe extern "C" fn timer_alloc() -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn timer_free(timer: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn timer_init(
        timer: *mut core::ffi::c_void,
        callback: core::option::Option<unsafe extern "C" fn(arg1: core::ffi::c_ulong)>,
        data: core::ffi::c_ulong,
    ) {
        todo!()
    }

    unsafe extern "C" fn timer_schedule(
        timer: *mut core::ffi::c_void,
        duration: core::ffi::c_ulong,
    ) {
        todo!()
    }

    unsafe extern "C" fn timer_kill(timer: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn bus_qspi_ps_sleep(
        os_qspi_priv: *mut core::ffi::c_void,
    ) -> core::ffi::c_int {
        todo!()
    }

    unsafe extern "C" fn bus_qspi_ps_wake(
        os_qspi_priv: *mut core::ffi::c_void,
    ) -> core::ffi::c_int {
        todo!()
    }

    unsafe extern "C" fn bus_qspi_ps_status(
        os_qspi_priv: *mut core::ffi::c_void,
    ) -> core::ffi::c_int {
        todo!()
    }
}

impl FmacCallbacks for CurrentPlatform {
    unsafe extern "C" fn if_carr_state_chg_callbk_fn(
        os_vif_ctx: *mut c_void,
        cs: wifi_nrf_fmac_if_carr_state,
    ) -> wifi_nrf_status {
        todo!()
    }

    unsafe extern "C" fn rx_frm_callbk_fn(os_vif_ctx: *mut c_void, frm: *mut c_void) {
        todo!()
    }

    unsafe extern "C" fn scan_start_callbk_fn(
        os_vif_ctx: *mut c_void,
        scan_start_event: *mut nrf_wifi_umac_event_trigger_scan,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn scan_done_callbk_fn(
        os_vif_ctx: *mut c_void,
        scan_done_event: *mut nrf_wifi_umac_event_trigger_scan,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn scan_abort_callbk_fn(
        os_vif_ctx: *mut c_void,
        scan_done_event: *mut nrf_wifi_umac_event_trigger_scan,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn scan_res_callbk_fn(
        os_vif_ctx: *mut c_void,
        scan_res: *mut nrf_wifi_umac_event_new_scan_results,
        event_len: c_uint,
        more_res: bool,
    ) {
        todo!()
    }

    unsafe extern "C" fn disp_scan_res_callbk_fn(
        os_vif_ctx: *mut c_void,
        scan_res: *mut nrf_wifi_umac_event_new_scan_display_results,
        event_len: c_uint,
        more_res: bool,
    ) {
        todo!()
    }

    unsafe extern "C" fn auth_resp_callbk_fn(
        os_vif_ctx: *mut c_void,
        auth_resp_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn assoc_resp_callbk_fn(
        os_vif_ctx: *mut c_void,
        assoc_resp_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn deauth_callbk_fn(
        os_vif_ctx: *mut c_void,
        deauth_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn disassoc_callbk_fn(
        os_vif_ctx: *mut c_void,
        disassoc_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn mgmt_rx_callbk_fn(
        os_vif_ctx: *mut c_void,
        mgmt_rx_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn unprot_mlme_mgmt_rx_callbk_fn(
        os_vif_ctx: *mut c_void,
        unprot_mlme_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn tx_pwr_get_callbk_fn(
        os_vif_ctx: *mut c_void,
        info: *mut nrf_wifi_umac_event_get_tx_power,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn chnl_get_callbk_fn(
        os_vif_ctx: *mut c_void,
        info: *mut nrf_wifi_umac_event_get_channel,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn sta_get_callbk_fn(
        os_vif_ctx: *mut c_void,
        info: *mut nrf_wifi_umac_event_new_station,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn cookie_rsp_callbk_fn(
        os_vif_ctx: *mut c_void,
        cookie_rsp: *mut nrf_wifi_umac_event_cookie_rsp,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn tx_status_callbk_fn(
        os_vif_ctx: *mut c_void,
        tx_status_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn set_if_callbk_fn(
        os_vif_ctx: *mut c_void,
        set_if_event: *mut nrf_wifi_umac_event_set_interface,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn roc_callbk_fn(
        os_vif_ctx: *mut c_void,
        roc_event: *mut nrf_wifi_event_remain_on_channel,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn roc_cancel_callbk_fn(
        os_vif_ctx: *mut c_void,
        roc_cancel_event: *mut nrf_wifi_event_remain_on_channel,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn get_station_callbk_fn(
        os_vif_ctx: *mut c_void,
        info: *mut nrf_wifi_umac_event_new_station,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn get_interface_callbk_fn(
        os_vif_ctx: *mut c_void,
        info: *mut nrf_wifi_interface_info,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn mgmt_tx_status(
        if_priv: *mut c_void,
        mlme_event: *mut nrf_wifi_umac_event_mlme,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn twt_config_callbk_fn(
        if_priv: *mut c_void,
        twt_config_event_info: *mut nrf_wifi_umac_cmd_config_twt,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn twt_teardown_callbk_fn(
        if_priv: *mut c_void,
        twt_teardown_event_info: *mut nrf_wifi_umac_cmd_teardown_twt,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn event_get_wiphy(
        if_priv: *mut c_void,
        get_wiphy: *mut nrf_wifi_event_get_wiphy,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn twt_sleep_callbk_fn(
        if_priv: *mut c_void,
        twt_sleep_event_info: *mut nrf_wifi_umac_event_twt_sleep,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn event_get_reg(
        if_priv: *mut c_void,
        get_reg: *mut nrf_wifi_reg,
        event_len: c_uint,
    ) {
        todo!()
    }

    unsafe extern "C" fn event_get_ps_info(
        if_priv: *mut c_void,
        get_ps_config: *mut nrf_wifi_umac_event_power_save_info,
        event_len: c_uint,
    ) {
        todo!()
    }
}

pub struct LinkedList {
    head: *mut Node,
}

impl LinkedList {
    pub fn new() -> Self {
        Self { head: null_mut() }
    }
}

pub struct Node {
    data: *mut core::ffi::c_void,
    next: *mut core::ffi::c_void,
}

pub struct Spinlock {
    flags: RestoreState,
}

impl Spinlock {
    pub fn new() -> Self {
        Self { flags: RestoreState::invalid() }
    }

    pub fn take(&mut self) {
        self.flags = unsafe { critical_section::acquire() };
    }

    pub fn release(&mut self) {
        unsafe { critical_section::release(self.flags) };
    }
}
