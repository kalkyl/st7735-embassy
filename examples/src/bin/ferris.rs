// $ cargo rb ferris
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use nrf_embassy as _; // global logger + panicking-behavior + memory layout
use tinybmp::Bmp;

use embassy::executor::Spawner;
use embassy::time::{Delay, Duration, Timer};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::{interrupt, spim, Peripherals};
use embedded_graphics::{image::Image, pixelcolor::Rgb565, prelude::*};
use embedded_hal_async::spi::ExclusiveDevice;
use st7735_embassy::{self, ST7735};

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M32;
    let irq = interrupt::take!(SPIM3);
    let spim = spim::Spim::new_txonly(p.SPI3, irq, p.P0_15, p.P0_18, config);
    let cs_pin = Output::new(p.P0_24, Level::Low, OutputDrive::Standard);
    let spi_dev = ExclusiveDevice::new(spim, cs_pin);

<<<<<<< HEAD
    // rst:  display reset pin, managed at driver level
    let rst = Output::new(p.P0_31, Level::High, OutputDrive::Standard);
    // dc: data/command selection pin, managed at driver level

    let dc = Output::new(p.P0_29, Level::High, OutputDrive::Standard);
=======
    let rst = Output::new(p.P0_22, Level::High, OutputDrive::Standard);
    let dc = Output::new(p.P0_20, Level::High, OutputDrive::Standard);
>>>>>>> shared-bus

    let mut display = ST7735::new(spi_dev, dc, rst, Default::default(), 160, 128);
    display.init(&mut Delay).await.unwrap();
    display.clear(Rgb565::BLACK).unwrap();

    let raw_image: Bmp<Rgb565> =
        Bmp::from_slice(include_bytes!("../../assets/ferris.bmp")).unwrap();
    let image = Image::new(&raw_image, Point::new(34, 24));

    image.draw(&mut display).unwrap();
    display.flush().await.unwrap();

    let mut backlight = Output::new(p.P0_13, Level::High, OutputDrive::Standard);
    loop {
        backlight.set_high();
        Timer::after(Duration::from_millis(700)).await;
        backlight.set_low();
        Timer::after(Duration::from_millis(300)).await;
    }
}
