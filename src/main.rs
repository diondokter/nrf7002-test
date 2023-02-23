#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_variables)]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::{Duration, Timer};
use panic_probe as _;

use crate::nrf_wifi::CurrentPlatform;

mod nrf_wifi;
mod wifi_bus;


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    run(spawner).await;
}

async fn run(_spawner: Spawner) -> ! {
    let mut config = embassy_nrf::config::Config::default();
    config.debug = embassy_nrf::config::Debug::Allowed;
    let p = embassy_nrf::init(config);

    defmt::println!("Started");

    let mut wifi = nrf_wifi::NrfWifi::new::<CurrentPlatform>();

    wifi.start_scan();

    let mut led = Output::new(p.P1_06, Level::Low, OutputDrive::Standard);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(300)).await;
        led.set_low();
        Timer::after(Duration::from_millis(300)).await;
    }
}
