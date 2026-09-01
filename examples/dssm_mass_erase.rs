//! Replicate probe-rs's DSSM mass erase on a healthy part, and report what the ROM actually says.
//!
//! Throwaway. The sequence is probe-rs's `dssm_mass_erase`, register for register, driven from
//! outside so it can be run on a part whose AHB-AP answers — that function is only reached when the
//! AHB-AP is dead, so nothing on a working board ever executes it.
//!
//! **This erases MAIN flash.** NONMAIN is untouched, which is the whole point of mass erase.
use std::time::{Duration, Instant};

use probe_rs::architecture::arm::{FullyQualifiedApAddress, Pins};

const SEC_AP: u8 = 2;
const TXDATA: u64 = 0x00;
const TXCTL: u64 = 0x04;
const RXDATA: u64 = 0x08;
const RXCTL: u64 = 0x0C;
const IDR: u64 = 0xFC;
const RXCTL_RX_VALID: u32 = 1 << 0;
const DSSM_MASS_ERASE: u32 = 0x020C;

fn main() -> anyhow::Result<()> {
    let attach = probe_bench::Attach {
        chip: "MSPM0L1306".to_owned(),
        probe: std::env::var("PROBE_RS_PROBE").ok(),
        verify: probe_bench::Verify::Skip,
        ..Default::default()
    };
    let elf = std::path::PathBuf::from(std::env::args().nth(1).expect("elf"));
    let mut bench = probe_bench::Bench::attach(&attach, &elf)?;
    let ap = FullyQualifiedApAddress::v1_with_default_dp(SEC_AP);

    let arm = bench.session().get_arm_interface()?;
    println!("SEC-AP IDR             {:#010x}", arm.read_raw_ap_register(&ap, IDR)?);
    println!("RXCTL before           {:#010x}", arm.read_raw_ap_register(&ap, RXCTL)?);

    // probe-rs's order: command first, then the argument.
    arm.write_raw_ap_register(&ap, TXCTL, DSSM_MASS_ERASE)?;
    arm.write_raw_ap_register(&ap, TXDATA, 0)?;
    println!("staged {DSSM_MASS_ERASE:#06x} in TXCTL, 0 in TXDATA");

    let _ = arm.read_raw_ap_register(&ap, RXDATA);
    let _ = arm.read_raw_ap_register(&ap, RXCTL);
    let _ = arm.flush();
    std::thread::sleep(Duration::from_millis(500));

    // The mailbox is serviced only out of a BOOTRST, so the reset pin is the trigger.
    let mut pins = Pins(0);
    pins.set_nreset(true);
    let low = arm.swj_pins(0, u32::from(pins.0), 0);
    println!("nRESET low  -> {low:?}");
    std::thread::sleep(Duration::from_millis(50));
    let high = arm.swj_pins(u32::from(pins.0), u32::from(pins.0), 0);
    println!("nRESET high -> {high:?}");

    let started = Instant::now();
    loop {
        let rxctl = arm.read_raw_ap_register(&ap, RXCTL).unwrap_or(0);
        if rxctl & RXCTL_RX_VALID != 0 {
            println!("RXVLD after            {:?}", started.elapsed());
            break;
        }
        if started.elapsed() > Duration::from_secs(2) {
            println!("no answer in 2s; last RXCTL {rxctl:#010x}");
            return Ok(());
        }
        std::thread::sleep(Duration::from_millis(1));
    }

    // Reading RXDATA clears RXVLD, so this order matters.
    println!("RXDATA                 {:#010x}", arm.read_raw_ap_register(&ap, RXDATA)?);
    println!("RXCTL                  {:#010x}", arm.read_raw_ap_register(&ap, RXCTL)?);
    Ok(())
}
