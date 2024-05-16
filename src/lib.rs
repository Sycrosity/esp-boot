#![no_std]
#![warn(missing_docs)]
#![doc = include_str!("../README.md")]
mod fmt;

pub use embassy_boot::{
    AlignedBuffer, BlockingFirmwareState, BlockingFirmwareUpdater, BootError, BootLoaderConfig, FirmwareState,
    FirmwareUpdater, FirmwareUpdaterConfig, State,
};
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::peripherals::{FLASH, WATCHDOG};
use embassy_rp::watchdog::Watchdog;

use esp_hal::delay::MicrosDurationU64;
use esp_hal::timer::Wdt;
use esp_storage::FlashStorage;

use embassy_rp::Peripheral;
use embassy_time::Duration;
use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};

use esp_hal::prelude::*;

use esp_hal::timer::TimerGroupInstance;


/// A bootloader for ESP devices.
pub struct BootLoader<const BUFFER_SIZE: usize = {FlashStorage::SECTOR_SIZE as usize}>;

impl<const BUFFER_SIZE: usize> BootLoader<BUFFER_SIZE> {
    /// Inspect the bootloader state and perform actions required before booting, such as swapping firmware
    pub fn prepare<ACTIVE: NorFlash, DFU: NorFlash, STATE: NorFlash>(
        config: BootLoaderConfig<ACTIVE, DFU, STATE>,
    ) -> Self {
        Self::try_prepare::<ACTIVE, DFU, STATE>(config).expect("Boot prepare error")
    }

    /// Inspect the bootloader state and perform actions required before booting, such as swapping firmware
    pub fn try_prepare<ACTIVE: NorFlash, DFU: NorFlash, STATE: NorFlash>(
        config: BootLoaderConfig<ACTIVE, DFU, STATE>,
    ) -> Result<Self, BootError> {
        let mut aligned_buf = AlignedBuffer([0; BUFFER_SIZE]);
        let mut boot = embassy_boot::BootLoader::new(config);
        let _state = boot.prepare_boot(aligned_buf.as_mut())?;
        Ok(Self)
    }

    /// Boots the application.
    ///
    /// # Safety
    ///
    /// This modifies the stack pointer and reset vector and will run code placed in the active partition.
    pub unsafe fn load(self, start: u32) -> ! {
        trace!("Loading app at 0x{:x}", start);
        // #[allow(unused_mut)]
        let mut b = cortex_m::Peripherals::steal();
        #[allow(unused_mut)]
        let mut p = esp_hal::peripherals::Peripherals::take();
        // #[cfg(not(armv6m))]
        b.SCB.invalidate_icache();

        b.SCB.vtor.write(start);

        // esp_hal::reset::software_reset();
        
        cortex_m::asm::bootload(start as *const u32)
    }
}

/// A flash implementation that will feed a watchdog when touching flash.
pub struct WatchdogFlash<TG: TimerGroupInstance> {
    // flash: Flash<'d, FLASH, Blocking, SIZE>,
    flash: FlashStorage,
    watchdog: Wdt<TG, esp_hal::Blocking>,
}

impl<TG: TimerGroupInstance> WatchdogFlash<TG> {
    /// Start a new watchdog with a given flash and watchdog peripheral and a timeout
    pub fn start(watchdog_tg: TG, timeout: Duration) -> Self {
        // let flash = Flash::<_, Blocking, SIZE>::new_blocking(flash);

        let flash = FlashStorage::new();

        let mut watchdog = Wdt::<TG, esp_hal::Blocking>::new();

        watchdog.set_timeout(MicrosDurationU64::from_ticks(timeout.as_ticks()));
        watchdog.enable();

        // let mut watchdog = Watchdog::new(watchdog);
        // watchdog.start(timeout);
        Self { flash, watchdog }
    }
}

impl<TG: TimerGroupInstance> ErrorType for WatchdogFlash<TG> {
    type Error = <FlashStorage as ErrorType>::Error;
}

impl<TG: TimerGroupInstance> NorFlash for WatchdogFlash<TG> {
    const WRITE_SIZE: usize = <FlashStorage as NorFlash>::WRITE_SIZE;
    const ERASE_SIZE: usize = <FlashStorage as NorFlash>::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.watchdog.feed();
        self.flash.erase(from, to)
    }
    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
        self.watchdog.feed();
        self.flash.write(offset, data)
    }
}

impl<TG: TimerGroupInstance> ReadNorFlash for WatchdogFlash<TG> {
    const READ_SIZE: usize = <FlashStorage as ReadNorFlash>::READ_SIZE;
    fn read(&mut self, offset: u32, data: &mut [u8]) -> Result<(), Self::Error> {
        self.watchdog.feed();
        self.flash.read(offset, data)
    }
    fn capacity(&self) -> usize {
        self.flash.capacity()
    }
}
