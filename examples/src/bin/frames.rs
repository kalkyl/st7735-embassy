// $ cargo rb frames
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
use nrf_embassy as _; // global logger + panicking-behavior + memory layout

use embassy::channel::signal::Signal;
use embassy::executor::Spawner;
use embassy::time::{Delay, Duration, Timer};
use embassy::util::Forever;
use embassy_nrf::{
    gpio::{Level, NoPin, Output, OutputDrive},
    interrupt,
    peripherals::{P0_20, P0_22, SPI3},
    spim::{self, Spim},
    Peripherals,
};
use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
use st7735_embassy::{self, Frame, Orientation, ST7735};

const BUF_SIZE: usize = 160 * 128 * 2;
static FRAME: Forever<Frame<BUF_SIZE>> = Forever::new();
static FRAME2: Forever<Frame<BUF_SIZE>> = Forever::new();
static SIG: Forever<Signal<&'static mut Frame<BUF_SIZE>>> = Forever::new();
static SIG2: Forever<Signal<&'static mut Frame<BUF_SIZE>>> = Forever::new();

#[embassy::task]
async fn render(
    spim: Spim<'static, SPI3>,
    dc: Output<'static, P0_20>,
    rst: Output<'static, P0_22>,
    sig: &'static Signal<&'static mut Frame<BUF_SIZE>>,
    sig2: &'static Signal<&'static mut Frame<BUF_SIZE>>,
) {
    let mut display = ST7735::new(spim, dc, rst, true, false, 160, 128, Orientation::Landscape);
    display.init(&mut Delay).await.unwrap();
    let mut frame = sig2.wait().await;
    loop {
        sig.signal(frame);
        frame = sig2.wait().await;
        display.flush_buffer(&frame.buffer).await.unwrap();
    }
}

#[embassy::main(config = "config()")]
async fn main(spawner: Spawner, p: Peripherals) {
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M32;
    let irq = interrupt::take!(SPIM3);
    let spim = spim::Spim::new(p.SPI3, irq, p.P0_15, NoPin, p.P0_18, config);

    let _cs_pin = Output::new(p.P0_24, Level::Low, OutputDrive::Standard);
    let dc = Output::new(p.P0_20, Level::High, OutputDrive::Standard);
    let rst = Output::new(p.P0_22, Level::High, OutputDrive::Standard);

    let sig = SIG.put(Signal::new());
    let frame_a = FRAME.put(Default::default());
    sig.signal(frame_a);

    let sig2 = SIG2.put(Signal::new());
    let frame_b = FRAME2.put(Default::default());
    sig2.signal(frame_b);

    defmt::unwrap!(spawner.spawn(render(spim, dc, rst, sig, sig2)));

    let _backlight = Output::new(p.P0_13, Level::High, OutputDrive::Standard);

    let mut x = 0;
    let mut y = 0;
    loop {
        let frame = sig.wait().await;
        frame.clear(Rgb565::BLACK).unwrap();
        frame.set_pixel(x, y, Rgb565::GREEN);
        sig2.signal(frame);
        x = (x + 2) % 160;
        y = (y + 2) % 128;
        Timer::after(Duration::from_millis(10)).await;
    }
}

fn config() -> embassy_nrf::config::Config {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config
}
