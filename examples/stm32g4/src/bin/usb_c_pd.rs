#![no_std]
#![no_main]

use defmt::{info, Format};
use embassy_executor::Spawner;
use embassy_stm32::{
    ucpd::{self, CcPull, CcVState, Ucpd},
    Config,
};
use embassy_time::{with_timeout, Duration};
use {defmt_rtt as _, panic_probe as _};

#[derive(Debug, Format)]
enum CableOrientation {
    Normal,
    Flipped,
    DebugAccessoryMode,
}

// Returns true when the cable
async fn wait_attached<'d, T: ucpd::Instance>(ucpd: &mut Ucpd<'d, T>) -> CableOrientation {
    loop {
        let (cc1, cc2) = ucpd.cc_vstate();
        if cc1 == CcVState::LOWEST && cc2 == CcVState::LOWEST {
            // Detached, wait until attached by monitoring the CC lines.
            ucpd.wait_for_cc_change().await;
            continue;
        }

        // Attached, wait for CC lines to be stable for tCCDebounce (100..200ms).
        if with_timeout(Duration::from_millis(100), ucpd.wait_for_cc_change())
            .await
            .is_ok()
        {
            // State has changed, restart detection procedure.
            continue;
        };

        // State was stable for the complete debounce period, check orientation.
        return match (cc1, cc2) {
            (_, CcVState::LOWEST) => CableOrientation::Normal,  // CC1 connected
            (CcVState::LOWEST, _) => CableOrientation::Flipped, // CC2 connected
            _ => CableOrientation::DebugAccessoryMode,          // Both connected (special cable)
        };
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // TODO: Disable DBCC pin functionality by default but have flag in the config to keep it enabled when required.
    let p = embassy_stm32::init(Config::default());

    info!("Hello World!");

    let mut ucpd = Ucpd::new(p.UCPD1, p.PB6, p.PB4, CcPull::Sink);

    info!("Waiting for USB connection...");
    let cable_orientation = wait_attached(&mut ucpd).await;
    info!("USB cable connected, orientation: {}", cable_orientation);

    loop {}
}