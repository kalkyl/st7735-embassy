#![no_std]

pub mod instruction;
use crate::instruction::Instruction;
use embassy_traits::delay::Delay;
use embedded_hal::digital::v2::OutputPin;

/// 128px x 160px screen with 16 bits (2 bytes) per pixel
const BUF_SIZE: usize = 128 * 160 * 2;

/// Async ST7735 LCD display driver.
pub struct ST7735<SPI, DC, RST, CommE, PinE>
where
    SPI: embassy_traits::spi::Write<u8> + embassy_traits::spi::Spi<u8, Error = CommE>,
    DC: OutputPin<Error = PinE>,
    RST: OutputPin<Error = PinE>,
{
    /// SPI
    spi: SPI,
    /// Data/command pin.
    dc: DC,
    /// Reset pin.
    rst: RST,
    /// Whether the display is RGB (true) or BGR (false)
    rgb: bool,
    /// Whether the colours are inverted (true) or not (false)
    inverted: bool,
    /// Global image offset
    dx: u16,
    dy: u16,
    width: u32,
    height: u32,
    /// Screen orientation
    orientation: Orientation,
    buffer: [u8; BUF_SIZE],
}

/// Display orientation.
#[derive(Clone, Copy)]
pub enum Orientation {
    Portrait = 0x00,
    Landscape = 0x60,
    PortraitSwapped = 0xC0,
    LandscapeSwapped = 0xA0,
}

impl<SPI, DC, RST, CommE, PinE> ST7735<SPI, DC, RST, CommE, PinE>
where
    SPI: embassy_traits::spi::Write<u8> + embassy_traits::spi::Spi<u8, Error = CommE>,
    DC: OutputPin<Error = PinE>,
    RST: OutputPin<Error = PinE>,
{
    /// Creates a new driver instance that uses hardware SPI.
    pub fn new(
        spi: SPI,
        dc: DC,
        rst: RST,
        rgb: bool,
        inverted: bool,
        width: u32,
        height: u32,
        orientation: Orientation,
    ) -> Self {
        Self {
            spi,
            dc,
            rst,
            rgb,
            inverted,
            dx: 0,
            dy: 0,
            width,
            height,
            orientation,
            buffer: [0; BUF_SIZE],
        }
    }

    /// Runs commands to initialize the display.
    pub async fn init<D>(&mut self, delay: &mut D) -> Result<(), Error<CommE, PinE>>
    where
        D: Delay,
    {
        self.hard_reset(delay).await?;
        self.write_command(Instruction::SWRESET, &[]).await?;
        delay.delay_ms(200).await;
        self.write_command(Instruction::SLPOUT, &[]).await?;
        delay.delay_ms(200).await;
        self.write_command(Instruction::FRMCTR1, &[0x01, 0x2C, 0x2D])
            .await?;
        self.write_command(Instruction::FRMCTR2, &[0x01, 0x2C, 0x2D])
            .await?;
        self.write_command(Instruction::FRMCTR3, &[0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D])
            .await?;
        self.write_command(Instruction::INVCTR, &[0x07]).await?;
        self.write_command(Instruction::PWCTR1, &[0xA2, 0x02, 0x84])
            .await?;
        self.write_command(Instruction::PWCTR2, &[0xC5]).await?;
        self.write_command(Instruction::PWCTR3, &[0x0A, 0x00])
            .await?;
        self.write_command(Instruction::PWCTR4, &[0x8A, 0x2A])
            .await?;
        self.write_command(Instruction::PWCTR5, &[0x8A, 0xEE])
            .await?;
        self.write_command(Instruction::VMCTR1, &[0x0E]).await?;
        if self.inverted {
            self.write_command(Instruction::INVON, &[]).await?;
        } else {
            self.write_command(Instruction::INVOFF, &[]).await?;
        }
        if self.rgb {
            self.write_command(Instruction::MADCTL, &[0x00]).await?;
        } else {
            self.write_command(Instruction::MADCTL, &[0x08]).await?;
        }
        self.write_command(Instruction::COLMOD, &[0x05]).await?;
        self.write_command(Instruction::DISPON, &[]).await?;
        delay.delay_ms(200).await;
        self.set_orientation(self.orientation).await?;
        Ok(())
    }

    pub async fn hard_reset<D>(&mut self, delay: &mut D) -> Result<(), Error<CommE, PinE>>
    where
        D: Delay,
    {
        self.rst.set_high().map_err(Error::Pin)?;
        delay.delay_ms(10).await;
        self.rst.set_low().map_err(Error::Pin)?;
        delay.delay_ms(10).await;
        self.rst.set_high().map_err(Error::Pin)
    }

    async fn write_command(
        &mut self,
        command: Instruction,
        params: &[u8],
    ) -> Result<(), Error<CommE, PinE>> {
        self.dc.set_low().map_err(Error::Pin)?;
        let mut data = [0_u8; 1];
        data.copy_from_slice(&[command as u8]);
        embassy_traits::spi::Write::write(&mut self.spi, &data)
            .await
            .map_err(Error::Comm)?;
        if !params.is_empty() {
            self.start_data()?;
            self.write_data(params).await?;
        }
        Ok(())
    }

    fn start_data(&mut self) -> Result<(), Error<CommE, PinE>> {
        self.dc.set_high().map_err(Error::Pin)
    }

    async fn write_data(&mut self, data: &[u8]) -> Result<(), Error<CommE, PinE>> {
        let mut buf = [0_u8; 8];
        buf[..data.len()].copy_from_slice(data);
        embassy_traits::spi::Write::write(&mut self.spi, &buf[..data.len()])
            .await
            .map_err(Error::Comm)
    }

    async fn write_buffer(&mut self) -> Result<(), Error<CommE, PinE>> {
        embassy_traits::spi::Write::write(&mut self.spi, &self.buffer)
            .await
            .map_err(Error::Comm)
    }

    /// Writes a data word to the display.
    async fn write_word(&mut self, value: u16) -> Result<(), Error<CommE, PinE>> {
        self.write_data(&value.to_be_bytes()).await
    }

    pub async fn set_orientation(
        &mut self,
        orientation: Orientation,
    ) -> Result<(), Error<CommE, PinE>> {
        if self.rgb {
            self.write_command(Instruction::MADCTL, &[orientation as u8])
                .await?;
        } else {
            self.write_command(Instruction::MADCTL, &[orientation as u8 | 0x08])
                .await?;
        }
        self.orientation = orientation;
        Ok(())
    }

    /// Sets the global offset of the displayed image
    pub fn set_offset(&mut self, dx: u16, dy: u16) {
        self.dx = dx;
        self.dy = dy;
    }

    /// Sets the address window for the display.
    pub async fn set_address_window(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
    ) -> Result<(), Error<CommE, PinE>> {
        self.write_command(Instruction::CASET, &[]).await?;
        self.start_data()?;
        self.write_word(sx + self.dx).await?;
        self.write_word(ex + self.dx).await?;
        self.write_command(Instruction::RASET, &[]).await?;
        self.start_data()?;
        self.write_word(sy + self.dy).await?;
        self.write_word(ey + self.dy).await
    }

    pub async fn flush(&mut self) -> Result<(), Error<CommE, PinE>> {
        self.set_address_window(0, 0, self.width as u16 - 1, self.height as u16 - 1)
            .await?;
        self.write_command(Instruction::RAMWR, &[]).await?;
        self.start_data()?;
        self.write_buffer().await
    }

    pub async fn flush_buffer(&mut self, buf: &[u8]) -> Result<(), Error<CommE, PinE>> {
        self.set_address_window(0, 0, self.width as u16 - 1, self.height as u16 - 1)
            .await?;
        self.write_command(Instruction::RAMWR, &[]).await?;
        self.start_data()?;
        embassy_traits::spi::Write::write(&mut self.spi, buf)
            .await
            .map_err(Error::Comm)
    }

    /// Sets a pixel color at the given coords.
    pub fn set_pixel(&mut self, x: u16, y: u16, color: u16) {
        let idx = match self.orientation {
            Orientation::Landscape | Orientation::LandscapeSwapped => {
                if x as u32 >= self.width {
                    return;
                }
                ((y as usize) * self.width as usize) + (x as usize)
            }

            Orientation::Portrait | Orientation::PortraitSwapped => {
                if y as u32 >= self.width as u32 {
                    return;
                }
                ((y as usize) * self.height as usize) + (x as usize)
            }
        } * 2;

        // Split 16 bit value into two bytes
        let low = (color & 0xff) as u8;
        let high = ((color & 0xff00) >> 8) as u8;
        if idx >= self.buffer.len() - 1 {
            return;
        }
        self.buffer[idx] = high;
        self.buffer[idx + 1] = low;
    }
}

extern crate embedded_graphics_core;
use self::embedded_graphics_core::{
    draw_target::DrawTarget,
    pixelcolor::{
        raw::{RawData, RawU16},
        Rgb565,
    },
    prelude::*,
};

impl<SPI, DC, RST, CommE, PinE> DrawTarget for ST7735<SPI, DC, RST, CommE, PinE>
where
    SPI: embassy_traits::spi::Write<u8> + embassy_traits::spi::Spi<u8, Error = CommE>,
    DC: OutputPin<Error = PinE>,
    RST: OutputPin<Error = PinE>,
{
    type Error = ();
    type Color = Rgb565;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let bb = self.bounding_box();

        pixels
            .into_iter()
            .filter(|Pixel(pos, _color)| bb.contains(*pos))
            .for_each(|Pixel(pos, color)| {
                self.set_pixel(pos.x as u16, pos.y as u16, RawU16::from(color).into_inner())
            });

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let c = RawU16::from(color).into_inner();
        for i in 0..BUF_SIZE {
            assert!(i < self.buffer.len());
            self.buffer[i] = if i % 2 == 0 {
                ((c & 0xff00) >> 8) as u8
            } else {
                (c & 0xff) as u8
            };
        }
        Ok(())
    }
}

impl<SPI, DC, RST, CommE, PinE> OriginDimensions for ST7735<SPI, DC, RST, CommE, PinE>
where
    SPI: embassy_traits::spi::Write<u8> + embassy_traits::spi::Spi<u8, Error = CommE>,
    DC: OutputPin<Error = PinE>,
    RST: OutputPin<Error = PinE>,
{
    fn size(&self) -> Size {
        Size::new(self.width, self.height)
    }
}

#[derive(Debug)]
pub enum Error<CommE = (), PinE = ()> {
    /// Communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),
}

pub struct Frame<const N: usize> {
    pub width: u32,
    pub height: u32,
    pub orientation: Orientation,
    pub buffer: [u8; N],
}

impl<const N: usize> Frame<N> {
    pub fn new(width: u32, height: u32, orientation: Orientation, buffer: [u8; N]) -> Self {
        Self {
            width,
            height,
            orientation,
            buffer,
        }
    }
    pub fn set_pixel(&mut self, x: u16, y: u16, color: Rgb565) {
        let color = RawU16::from(color).into_inner();
        let idx = match self.orientation {
            Orientation::Landscape | Orientation::LandscapeSwapped => {
                if x as u32 >= self.width {
                    return;
                }
                ((y as usize) * self.width as usize) + (x as usize)
            }

            Orientation::Portrait | Orientation::PortraitSwapped => {
                if y as u32 >= self.width as u32 {
                    return;
                }
                ((y as usize) * self.height as usize) + (x as usize)
            }
        } * 2;

        // Split 16 bit value into two bytes
        let low = (color & 0xff) as u8;
        let high = ((color & 0xff00) >> 8) as u8;
        if idx >= self.buffer.len() - 1 {
            return;
        }
        self.buffer[idx] = high;
        self.buffer[idx + 1] = low;
    }
}
impl<const N: usize> Default for Frame<N> {
    fn default() -> Self {
        Self {
            width: 160,
            height: 128,
            orientation: Orientation::Landscape,
            buffer: [0; N],
        }
    }
}

impl<const N: usize> DrawTarget for Frame<N> {
    type Error = ();
    type Color = Rgb565;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let bb = self.bounding_box();
        pixels
            .into_iter()
            .filter(|Pixel(pos, _color)| bb.contains(*pos))
            .for_each(|Pixel(pos, color)| self.set_pixel(pos.x as u16, pos.y as u16, color));
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let c = RawU16::from(color).into_inner();
        for i in 0..BUF_SIZE {
            assert!(i < self.buffer.len());
            self.buffer[i] = if i % 2 == 0 {
                ((c & 0xff00) >> 8) as u8
            } else {
                (c & 0xff) as u8
            };
        }
        Ok(())
    }
}

impl<const N: usize> OriginDimensions for Frame<N> {
    fn size(&self) -> Size {
        Size::new(self.width, self.height)
    }
}
