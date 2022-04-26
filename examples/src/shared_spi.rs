use core::{fmt::Debug, future::Future};
use embassy::blocking_mutex::raw::RawMutex;
// use embassy::blocking_mutex::raw::ThreadModeRawMutex;
use embassy::mutex::Mutex;

use embedded_hal::digital::blocking::OutputPin;
pub use embedded_hal::spi::{
    blocking, Error, ErrorKind, ErrorType, Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3,
};
use embedded_hal_async::spi;
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum SpiDeviceWithCsError<BUS, CS> {
    #[allow(unused)] // will probably use in the future when adding a flush() to SpiBus
    Spi(BUS),
    Cs(CS),
}

impl<BUS, CS> spi::Error for SpiDeviceWithCsError<BUS, CS>
where
    BUS: spi::Error + Debug,
    CS: Debug,
{
    fn kind(&self) -> spi::ErrorKind {
        match self {
            Self::Spi(e) => e.kind(),
            Self::Cs(_) => spi::ErrorKind::Other,
        }
    }
}

pub struct SpiDeviceWithCs<'a, M: RawMutex, BUS, CS> {
    bus: &'a Mutex<M, BUS>,
    cs: CS,
}

impl<'a, M: RawMutex, BUS, CS> SpiDeviceWithCs<'a, M, BUS, CS> {
    pub fn new(bus: &'a Mutex<M, BUS>, cs: CS) -> Self {
        Self { bus, cs }
    }
}

impl<'a, M: RawMutex, BUS, CS> spi::ErrorType for SpiDeviceWithCs<'a, M, BUS, CS>
where
    BUS: spi::ErrorType,
    CS: OutputPin,
{
    type Error = SpiDeviceWithCsError<BUS::Error, CS::Error>;
}

impl<M, BUS, CS> spi::SpiDevice for SpiDeviceWithCs<'_, M, BUS, CS>
where
    M: RawMutex,
    BUS: spi::SpiBusFlush,
    CS: OutputPin,
{
    type Bus = BUS;

    type TransactionFuture<'a, R, F, Fut> = impl Future<Output = Result<R, Self::Error>> + 'a
    where
        Self: 'a, R: 'a, F: FnOnce(&'a mut Self::Bus) -> Fut + 'a,
        Fut: Future<Output = (&'a mut Self::Bus, Result<R, <Self::Bus as ErrorType>::Error>)> + 'a;
    fn transaction<'a, R, F, Fut>(&'a mut self, f: F) -> Self::TransactionFuture<'a, R, F, Fut>
    where
        R: 'a,
        F: FnOnce(&'a mut Self::Bus) -> Fut + 'a,
        Fut: Future<
                Output = (
                    &'a mut Self::Bus,
                    Result<R, <Self::Bus as ErrorType>::Error>,
                ),
            > + 'a,
    {
        async move {
            let mut bus = self.bus.lock().await;
            self.cs.set_low().map_err(SpiDeviceWithCsError::Cs)?;

            let (bus, f_res) = f(&mut bus).await;

            // On failure, it's important to still flush and deassert CS.
            let flush_res = bus.flush().await;
            let cs_res = self.cs.set_high();

            let f_res = f_res.map_err(SpiDeviceWithCsError::Spi)?;
            flush_res.map_err(SpiDeviceWithCsError::Spi)?;
            cs_res.map_err(SpiDeviceWithCsError::Cs)?;

            Ok(f_res)
        }
    }
}
