#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use nrf_embassy as _; // global logger + panicking-behavior + memory layout

use defmt::*;
use embassy::blocking_mutex::raw::ThreadModeRawMutex;
use embassy::executor::Spawner;
use embassy::mutex::Mutex;
use embassy::time::{Delay, Duration, Timer};
use embassy::util::Forever;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::{
    interrupt,
    peripherals::{P0_20, P0_22, P0_24, SPI3},
    spim, Peripherals,
};
use embedded_graphics::{image::Image, pixelcolor::Rgb565, prelude::*};
use nrf_embassy::shared_spi::SpiDeviceWithCs;
use st7735_embassy::{self, ST7735};
use tinybmp::Bmp;

#[embassy::task]
async fn display_task(
    mut display: ST7735<
        SpiDeviceWithCs<
            'static,
            ThreadModeRawMutex,
            spim::Spim<'static, SPI3>,
            Output<'static, P0_24>,
        >,
        Output<'static, P0_20>,
        Output<'static, P0_22>,
    >,
) {
    display.init(&mut Delay).await.unwrap();
    display.clear(Rgb565::BLACK).unwrap();
    let raw_image: Bmp<Rgb565> =
        Bmp::from_slice(include_bytes!("../../assets/ferris.bmp")).unwrap();
    let image = Image::new(&raw_image, Point::new(34, 24));
    image.draw(&mut display).unwrap();
    display.flush().await.unwrap();
}

#[embassy::main]
async fn main(spawner: Spawner, p: Peripherals) {
    static SPI_BUS: Forever<Mutex<ThreadModeRawMutex, spim::Spim<SPI3>>> = Forever::new();
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M32;
    let irq = interrupt::take!(SPIM3);
    let spi_bus = Mutex::<ThreadModeRawMutex, spim::Spim<SPI3>>::new(spim::Spim::new_txonly(
        p.SPI3, irq, p.P0_15, p.P0_18, config,
    ));
    let spi_bus = SPI_BUS.put(spi_bus);

    let cs_pin = Output::new(p.P0_24, Level::Low, OutputDrive::Standard);
    let spi_dev = SpiDeviceWithCs::new(spi_bus, cs_pin);
    let dc = Output::new(p.P0_20, Level::High, OutputDrive::Standard);
    let rst = Output::new(p.P0_22, Level::High, OutputDrive::Standard);
    let display = ST7735::new(spi_dev, dc, rst, Default::default(), 160, 128);
    unwrap!(spawner.spawn(display_task(display)));

    let mut backlight = Output::new(p.P0_13, Level::High, OutputDrive::Standard);
    loop {
        backlight.set_high();
        Timer::after(Duration::from_millis(700)).await;
        backlight.set_low();
        Timer::after(Duration::from_millis(300)).await;
    }
}
