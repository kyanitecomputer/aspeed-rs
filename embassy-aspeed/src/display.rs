//! AST2700 DisplayPort MCU bring-up.
//!
//! On the AST2700 the DisplayPort MCU (DPMCU) is a small autonomous core that
//! reads the monitor EDID and performs DP link training. This mirrors the
//! vendor `dp_init` (aspeed-zephyr-project mcu-runtime `soc/ast2700/dp.c`),
//! which runs in the BootMCU **before** the CA35 is released: it resets and
//! clocks the DP/DPMCU, loads the DPMCU firmware into instruction memory,
//! releases the core, enables its interrupts, and sets the DP scratch handshake.
//!
//! Why the BootMCU (and not the CA35/PSP): the DPMCU must be released and allowed
//! to train the link **while the CA35 is still held in reset**. If the core is
//! instead released late from the running CA35, the DPMCU's bring-up bus traffic
//! races the CA35's live DRAM/fabric traffic and intermittently wedges the CA35.
//! Releasing it here — as the vendor does — lets the DPMCU finish (or nearly
//! finish) training during the BootMCU's remaining work (Caliptra auth, payload
//! loads) so it is idle by the time the CA35 runs. The CA35 still owns the
//! *display* side (VLink, CRT timing, framebuffer, scanout).
//!
//! The scanout framebuffer is NOT set up here, so the released DPMCU does not
//! contend for DRAM; it only drives the DP aux/link.
//!
//! `early_crt_clock_select` is a clock-mux select that must be programmed before
//! the SCU register policy locks it so the CA35 can still derive the pixel clock.

use crate::pac;
use crate::prebuilt::{PrebuiltTable, PrebuiltType};
use aspeed_mmio::MmioBlock;

const SCU0_BASE: usize = 0x12c0_2000;
const DPMCU_IMEM_BASE: usize = 0x1102_0000;
// DPMCU interrupt-control register (MCU_INTR_CTRL) and the DP handshake scratch
// registers, addressed raw (as in sdrammc.rs) since they live outside the pac's
// DPMCU_REG block / are plain SCU0 scratch words.
const DPMCU_INT: usize = 0x1101_00e8;
const SCU0_VGA0_SCRATCH1: usize = 0x12c0_2900;
const SCU0_VGA1_SCRATCH1: usize = 0x12c0_2910;

const MCU_CTRL_CONFIG: u32 = 1 << 28;
const MCU_CTRL_IMEM_CLK_OFF: u32 = 1 << 22;
const MCU_CTRL_IMEM_SHUT_DOWN: u32 = 1 << 20;
const MCU_CTRL_DMEM_CLK_OFF: u32 = 1 << 18;
const MCU_CTRL_DMEM_SHUT_DOWN: u32 = 1 << 16;
const MCU_CTRL_CORE_SW_RST: u32 = 1 << 12;
const MCU_CTRL_AHBM_SW_RST: u32 = 1 << 8;
const MCU_CTRL_AHBS_SW_RST: u32 = 1 << 4;
const MCU_CTRL_AHBS_IMEM_EN: u32 = 1 << 0;
// MCU_INTR_CTRL EN field is bits[23:16]; vendor writes 0xff there.
const MCU_INTR_CTRL_EN: u32 = 0xff << 16;
const SCU0_RST1_DP: u32 = 1 << 28;
const SCU0_RST1_DPMCU: u32 = 1 << 29;
const SCU0_CLKGATE1_DP: u32 = 1 << 18;

#[inline]
fn rd32(addr: usize) -> u32 {
    let regs = unsafe { MmioBlock::new(addr) };
    regs.read32(0)
}

#[inline]
fn wr32(addr: usize, val: u32) {
    let mut regs = unsafe { MmioBlock::new(addr) };
    regs.write32(0, val)
}

#[inline]
fn set32(addr: usize, mask: u32) {
    wr32(addr, rd32(addr) | mask);
}

fn delay(mut loops: usize) {
    while loops != 0 {
        core::hint::spin_loop();
        loops -= 1;
    }
}

/// Copy the DisplayPort MCU firmware image into DPMCU instruction memory.
///
/// Uses the silicon-aware prebuilt catalogue: on A2 the DP firmware is an image
/// in the FLSH container (`parse_from_spi` only understands the A1 ASTH header,
/// so it finds nothing on A2). This mirrors the DDR training loader
/// (`sdrammc.rs`), which already uses `parse()`.
fn copy_firmware_to_imem() {
    if let Some(fw) = PrebuiltTable::parse().spi_slice(PrebuiltType::DpFw as u32) {
        for (i, chunk) in fw.chunks(4).enumerate() {
            let mut word = [0u8; 4];
            word[..chunk.len()].copy_from_slice(chunk);
            wr32(DPMCU_IMEM_BASE + i * 4, u32::from_le_bytes(word));
        }
    }
}

/// Bring up the DisplayPort MCU, mirroring the vendor `dp_init`: reset+clock the
/// DP/DPMCU, load the firmware into IMEM, release the core, enable its
/// interrupts, and set the DP scratch handshake. Returns false if the DP
/// controller is absent.
///
/// Call this in the BootMCU **before releasing the CA35** so the DPMCU trains
/// the link while the CA35 is still held in reset (see module docs). The CA35
/// still owns the display side: VLink, CRT timing, framebuffer, and scanout.
pub fn bring_up_dp() -> bool {
    // Reset and clock the DP/DPMCU so instruction memory is accessible.
    pac::SCU0
        .RST_CTRL1()
        .write_value(pac::scu0_ast2700_v1::RST_CTRL1(
            SCU0_RST1_DP | SCU0_RST1_DPMCU,
        ));
    delay(10_000);
    pac::SCU0
        .CLKGATE1_CLR()
        .write_value(pac::scu0_ast2700_v1::CLKGATE1(SCU0_CLKGATE1_DP));
    delay(1_000);
    pac::SCU0
        .RST_CLR1()
        .write_value(pac::scu0_ast2700_v1::RST_CTRL1(
            SCU0_RST1_DP | SCU0_RST1_DPMCU,
        ));
    delay(100);

    if pac::DP.VERSION().read() == 0 {
        return false;
    }
    pac::DP.HANDSHAKE().modify(|r| {
        r.set_HOST_READ_EDID(false);
        r.set_VIDEO_FMT_SRC(false);
    });
    pac::DPMCU_DMEM.DISPLAY_FORMAT().write_value(0);

    // Open the AHB-slave IMEM window, copy the firmware, then close it. The core
    // stays in reset — the PSP releases it.
    let mut ctrl = MCU_CTRL_CONFIG
        | MCU_CTRL_IMEM_CLK_OFF
        | MCU_CTRL_IMEM_SHUT_DOWN
        | MCU_CTRL_DMEM_CLK_OFF
        | MCU_CTRL_DMEM_SHUT_DOWN
        | MCU_CTRL_AHBS_SW_RST;
    pac::DPMCU_REG
        .CTRL()
        .write_value(pac::dp_ast2700_v1::MCU_CTRL(ctrl));
    ctrl &= !(MCU_CTRL_IMEM_SHUT_DOWN | MCU_CTRL_DMEM_SHUT_DOWN);
    pac::DPMCU_REG
        .CTRL()
        .write_value(pac::dp_ast2700_v1::MCU_CTRL(ctrl));
    ctrl &= !(MCU_CTRL_IMEM_CLK_OFF | MCU_CTRL_DMEM_CLK_OFF);
    pac::DPMCU_REG
        .CTRL()
        .write_value(pac::dp_ast2700_v1::MCU_CTRL(ctrl));
    ctrl |= MCU_CTRL_AHBS_IMEM_EN;
    pac::DPMCU_REG
        .CTRL()
        .write_value(pac::dp_ast2700_v1::MCU_CTRL(ctrl));
    copy_firmware_to_imem();
    ctrl &= !MCU_CTRL_AHBS_IMEM_EN;
    pac::DPMCU_REG
        .CTRL()
        .write_value(pac::dp_ast2700_v1::MCU_CTRL(ctrl));

    // Release the DPMCU core (and its AHB master) so it starts executing the
    // firmware, then enable its interrupts. Vendor dp_init order.
    ctrl |= MCU_CTRL_CORE_SW_RST | MCU_CTRL_AHBM_SW_RST;
    pac::DPMCU_REG
        .CTRL()
        .write_value(pac::dp_ast2700_v1::MCU_CTRL(ctrl));
    wr32(DPMCU_INT, MCU_INTR_CTRL_EN);

    // DP scratch handshake: hand control to the DPMCU firmware. Bits [11:9]=7
    // (vendor dp_init) plus bit7|bit12 (vendor vga_init "disable P2A"); the
    // DPMCU firmware polls this to advance from the host-programmed state to
    // link training, and sets bit13 when trained.
    for scratch in [SCU0_VGA0_SCRATCH1, SCU0_VGA1_SCRATCH1] {
        let mut v = rd32(scratch);
        v &= !(0x7 << 9);
        v |= (0x7 << 9) | (1 << 7) | (1 << 12);
        wr32(scratch, v);
    }
    true
}

/// Set the CRT clock source select (SCU0+0x288 bit 14).
///
/// Must be called BEFORE `scu::apply_ibex_default_register_policy()` because the
/// policy locks this register. Programmed here so the PSP can still derive the
/// display pixel clock after the lock.
pub fn early_crt_clock_select() {
    set32(SCU0_BASE + 0x288, 1 << 14);
}
