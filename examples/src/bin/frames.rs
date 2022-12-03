// $ cargo rb frames
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
use nrf_embassy as _; // global logger + panicking-behavior + memory layout

use embassy_executor::Spawner;
use embassy_executor::_export::StaticCell;
use embassy_nrf::{
    gpio::{Level, Output, OutputDrive},
    interrupt,
    peripherals::{P0_20, P0_22, P0_24, SPI3},
    spim::{self, Spim},
};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
use embedded_hal_async::spi::ExclusiveDevice;
use st7735_embassy::{self, Frame, ST7735IF};

const BUF_SIZE: usize = 160 * 128 * 2;
static FRAME_A: StaticCell<Frame<BUF_SIZE>> = StaticCell::new();
static FRAME_B: StaticCell<Frame<BUF_SIZE>> = StaticCell::new();
static NEXT_FRAME: StaticCell<Signal<ThreadModeRawMutex, &'static mut Frame<BUF_SIZE>>> =
    StaticCell::new();
static READY_FRAME: StaticCell<Signal<ThreadModeRawMutex, &'static mut Frame<BUF_SIZE>>> =
    StaticCell::new();

#[embassy_executor::task]
async fn render(
    spi_dev: ExclusiveDevice<Spim<'static, SPI3>, Output<'static, P0_24>>,
    dc: Output<'static, P0_20>,
    rst: Output<'static, P0_22>,
    next_frame: &'static Signal<ThreadModeRawMutex, &'static mut Frame<BUF_SIZE>>,
    ready_frame: &'static Signal<ThreadModeRawMutex, &'static mut Frame<BUF_SIZE>>,
) {
    let mut display = ST7735IF::new(spi_dev, dc, rst, Default::default());
    display.init(&mut Delay).await.unwrap();
    let mut frame = ready_frame.wait().await;
    loop {
        next_frame.signal(frame);
        frame = ready_frame.wait().await;
        display.flush_frame(&frame).await.unwrap();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(config);
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M32;
    let irq = interrupt::take!(SPIM3);
    let spim = spim::Spim::new_txonly(p.SPI3, irq, p.P0_15, p.P0_18, config);
    let cs_pin = Output::new(p.P0_24, Level::Low, OutputDrive::Standard);
    let spi_dev = ExclusiveDevice::new(spim, cs_pin);

    let dc = Output::new(p.P0_20, Level::High, OutputDrive::Standard);
    let rst = Output::new(p.P0_22, Level::High, OutputDrive::Standard);

    let next_frame = NEXT_FRAME.init(Signal::new());
    let frame_a = FRAME_A.init(Default::default());
    next_frame.signal(frame_a);

    let ready_frame = READY_FRAME.init(Signal::new());
    let frame_b = FRAME_B.init(Default::default());
    ready_frame.signal(frame_b);

    defmt::unwrap!(spawner.spawn(render(spi_dev, dc, rst, next_frame, ready_frame)));

    let _backlight = Output::new(p.P0_13, Level::High, OutputDrive::Standard);

    let mut x = 0;
    let mut y = 0;
    loop {
        let frame = next_frame.wait().await;
        // frame.clear(Rgb565::BLACK).unwrap();
        frame.set_pixel(x, y, Rgb565::GREEN);
        ready_frame.signal(frame);
        x = (x + 1) % 160;
        y = (y + 1) % 128;
        Timer::after(Duration::from_millis(10)).await;
    }
}
