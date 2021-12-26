// $ cargo rb ferris
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use nrf_embassy as _; // global logger + panicking-behavior + memory layout

use defmt::*;
use embassy::executor::Spawner;
use embassy::time::{Delay, Duration, Timer};
use embassy_nrf::gpio::{Level, NoPin, Output, OutputDrive};
use embassy_nrf::{interrupt, spim, Peripherals};
use embedded_graphics::{
    image::{Image, ImageRaw, ImageRawLE},
    pixelcolor::Rgb565,
    prelude::*,
};
use st7735_embassy::{self, Orientation, ST7735};

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M16;
    let irq = interrupt::take!(SPIM3);
    let spim = spim::Spim::new(p.SPI3, irq, p.P0_15, NoPin, p.P0_18, config);

    let _cs_pin = Output::new(p.P0_24, Level::Low, OutputDrive::Standard);
    let rst = Output::new(p.P0_22, Level::High, OutputDrive::Standard);
    let dc = Output::new(p.P0_20, Level::High, OutputDrive::Standard);

    let mut display = ST7735::new(spim, dc, rst, true, false, 160, 128, Orientation::Landscape);

    display.init(&mut Delay).await.ok();
    display.clear(Rgb565::BLACK).ok();

    let image_raw: ImageRawLE<Rgb565> =
        ImageRaw::new(include_bytes!("../../assets/ferris.raw"), 86);
    let image: Image<_> = Image::new(&image_raw, Point::new(34, 24));
    image.draw(&mut display).ok();
    display.flush().await.ok();

    let mut backlight = Output::new(p.P0_13, Level::Low, OutputDrive::Standard);
    loop {
        backlight.set_high();
        Timer::after(Duration::from_millis(700)).await;
        backlight.set_low();
        Timer::after(Duration::from_millis(300)).await;
    }
}
