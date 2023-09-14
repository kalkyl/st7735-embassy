#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use nrf_embassy as _; // global logger + panicking-behavior + memory layout

use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::_export::StaticCell;
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    gpio::{AnyPin, Level, Output, OutputDrive, Pin},
    peripherals::{self, SPI3},
    spim::{self, Config, Spim},
};
use embassy_nrf::spim::Frequency;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::{image::Image, pixelcolor::Rgb565, prelude::*};
use tinybmp::Bmp;

use st7735_embassy::{self, ST7735};

type SpiDev = SpiDevice<'static, ThreadModeRawMutex, Spim<'static, SPI3>, Output<'static, AnyPin>>;

bind_interrupts!(struct Irqs {
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

#[embassy_executor::task]
async fn display_task(mut display: ST7735<SpiDev, Output<'static, AnyPin>, Output<'static, AnyPin>>) {
    display.init(&mut Delay).await.unwrap();
    display.clear(Rgb565::BLACK).unwrap();
    let raw_image: Bmp<Rgb565> =
        Bmp::from_slice(include_bytes!("../../assets/ferris.bmp")).unwrap();
    let image = Image::new(&raw_image, Point::new(34, 24));
    image.draw(&mut display).unwrap();
    display.flush().await.unwrap();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    static SPI_BUS: StaticCell<Mutex<ThreadModeRawMutex, Spim<SPI3>>> = StaticCell::new();
    let mut config = Config::default();
    config.frequency = Frequency::M32;
    let spim = spim::Spim::new_txonly(p.SPI3, Irqs, p.P1_05, p.P1_04, config);
    let spi_bus = Mutex::new(spim);
    let spi_bus = SPI_BUS.init(spi_bus);

    let cs_pin = Output::new(p.P1_03.degrade(), Level::Low, OutputDrive::Standard);
    let spi_dev = SpiDevice::new(spi_bus, cs_pin);
    let dc = Output::new(p.P1_02.degrade(), Level::High, OutputDrive::Standard);
    let rst = Output::new(p.P1_01.degrade(), Level::High, OutputDrive::Standard);
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
