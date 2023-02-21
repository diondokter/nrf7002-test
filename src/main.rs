#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_variables)]

use core::{
    alloc::Layout,
    cell::RefCell,
    mem::{align_of, size_of, MaybeUninit},
    ptr::NonNull, pin::pin,
};

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::{Duration, Timer};
use nrf_wifi::OsalPlatform;
use panic_probe as _;

use crate::nrf_wifi::NrfWifi;

mod nrf_wifi;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    run(spawner).await;
}

async fn run(_spawner: Spawner) -> ! {
    let mut config = embassy_nrf::config::Config::default();
    config.debug = embassy_nrf::config::Debug::Allowed;
    let p = embassy_nrf::init(config);

    defmt::println!("Started");

    let mut wifi = NrfWifi::new::<CurrentPlatform>();
    pin!(wifi);

    let mut led = Output::new(p.P1_06, Level::Low, OutputDrive::Standard);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(300)).await;
        led.set_low();
        Timer::after(Duration::from_millis(300)).await;
    }
}

struct CurrentPlatform;

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
        let alloc = Self::mem_alloc(size);
        Self::mem_set(alloc, 0, size);
        alloc
    }

    unsafe extern "C" fn mem_free(buf: *mut core::ffi::c_void) {
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
        todo!()
    }

    unsafe extern "C" fn mem_set(
        start: *mut core::ffi::c_void,
        val: core::ffi::c_int,
        size: usize,
    ) -> *mut core::ffi::c_void {
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
        todo!()
    }

    unsafe extern "C" fn spinlock_free(lock: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn spinlock_init(lock: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn spinlock_take(lock: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn spinlock_rel(lock: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn spinlock_irq_take(
        lock: *mut core::ffi::c_void,
        flags: *mut core::ffi::c_ulong,
    ) {
        todo!()
    }

    unsafe extern "C" fn spinlock_irq_rel(
        lock: *mut core::ffi::c_void,
        flags: *mut core::ffi::c_ulong,
    ) {
        todo!()
    }

    unsafe extern "C" fn log_dbg(
        fmt: *const core::ffi::c_char,
        args: nrf700x_sys::va_list,
    ) -> core::ffi::c_int {
        todo!()
    }

    unsafe extern "C" fn log_info(
        fmt: *const core::ffi::c_char,
        args: nrf700x_sys::va_list,
    ) -> core::ffi::c_int {
        todo!()
    }

    unsafe extern "C" fn log_err(
        fmt: *const core::ffi::c_char,
        args: nrf700x_sys::va_list,
    ) -> core::ffi::c_int {
        todo!()
    }

    unsafe extern "C" fn llist_node_alloc() -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn llist_node_free(node: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn llist_node_data_get(
        node: *mut core::ffi::c_void,
    ) -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn llist_node_data_set(
        node: *mut core::ffi::c_void,
        data: *mut core::ffi::c_void,
    ) {
        todo!()
    }

    unsafe extern "C" fn llist_alloc() -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn llist_free(llist: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn llist_init(llist: *mut core::ffi::c_void) {
        todo!()
    }

    unsafe extern "C" fn llist_add_node_tail(
        llist: *mut core::ffi::c_void,
        llist_node: *mut core::ffi::c_void,
    ) {
        todo!()
    }

    unsafe extern "C" fn llist_get_node_head(
        llist: *mut core::ffi::c_void,
    ) -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn llist_get_node_nxt(
        llist: *mut core::ffi::c_void,
        llist_node: *mut core::ffi::c_void,
    ) -> *mut core::ffi::c_void {
        todo!()
    }

    unsafe extern "C" fn llist_del_node(
        llist: *mut core::ffi::c_void,
        llist_node: *mut core::ffi::c_void,
    ) {
        todo!()
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
        todo!()
    }

    unsafe extern "C" fn bus_qspi_deinit(os_qspi_priv: *mut core::ffi::c_void) {
        todo!()
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
