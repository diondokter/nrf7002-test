use core::ffi::{c_char, c_int, c_uint, c_ulong, c_void};
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};

use nrf700x_sys::{
    va_list, wifi_nrf_osal_dma_dir, wifi_nrf_osal_host_map, wifi_nrf_osal_ops, wifi_nrf_osal_priv,
    wifi_nrf_status,
};

pub struct NrfWifi {
    osal: *mut wifi_nrf_osal_priv,
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
    pub fn new<P: OsalPlatform>() -> Self {
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

        unsafe { P::init() };

        let osal = unsafe { nrf700x_sys::wifi_nrf_osal_init() };

        Self {
            osal,
        }
    }

    pub fn start_scan(self: Pin<&mut Self>) {
        todo!()
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
