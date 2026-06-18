//! Physical ↔ virtual address translation for cross-processor DMA buffers.
//!
//! The SSP's CM3 virtual address `0x0000_0000` maps to a physical DRAM
//! address configured by the CA7 in the SSP control register `SCUA04`
//! (`MEM_BASE`, field `BASE[31:20]`).
//!
//! Any buffer passed to the CA7 for DMA or shared-memory IPC must be
//! expressed as a physical (bus) address.
//!
//! # Caching note
//!
//! Place shared-memory buffers in `RAM_NC` (non-cached region, starting at
//! CM3 virtual address `0x0100_0000`) to avoid coherency issues:
//!
//! ```rust,ignore
//! #[link_section = ".ram_nc"]
//! static SHARED: [u8; 256] = [0; 256];
//! let phys = to_phys(SHARED.as_ptr());
//! // Pass `phys` to the CA7 via IPC.
//! ```
//!
//! # Design
//!
//! `phys_base()` reads `SCUA04` once and caches the result in a static.
//! Subsequent calls return the cached value without MMIO access.

use core::sync::atomic::{AtomicU32, Ordering};

use aspeed_mmio::MmioBlock;

// ── SSP control register ──────────────────────────────────────────────────────

/// SSP `MEM_BASE` register address (SCUA04, CM3 view).
const SCUA04: usize = 0x7E6E_2A04;

/// Sentinel meaning "not yet initialised".
const UNSET: u32 = u32::MAX;

static CACHED_PHYS_BASE: AtomicU32 = AtomicU32::new(UNSET);

// ── Public API ────────────────────────────────────────────────────────────────

/// Return the physical base address of the CM3 virtual address space.
///
/// Reads `SCUA04.BASE[31:20]` (1 MB aligned) on the first call; subsequent
/// calls return the cached result.
pub fn phys_base() -> u32 {
    let cached = CACHED_PHYS_BASE.load(Ordering::Relaxed);
    if cached != UNSET {
        return cached;
    }
    let raw = unsafe { MmioBlock::new(SCUA04) }.read32(0);
    // BASE field is bits [31:20]; shift left by 0 — the register already
    // stores the address aligned to 1 MB (bits [31:20] << 20 gives the address,
    // but the field value itself is bits [31:20] of the address, so we mask
    // and keep bits [31:20]).
    let base = raw & 0xFFF0_0000;
    CACHED_PHYS_BASE.store(base, Ordering::Relaxed);
    base
}

/// Translate a CM3 virtual pointer to a physical (bus) address.
///
/// The caller is responsible for ensuring the pointer is within the CM3
/// DRAM window (`0x0000_0000`–`0x1FFF_FFFF`).
pub fn to_phys(virt: *const u8) -> u32 {
    phys_base().wrapping_add(virt as u32)
}

/// Translate a physical (bus) address back to a CM3 virtual pointer.
///
/// # Safety
///
/// The resulting pointer is only valid if `phys` refers to memory within
/// the CM3 DRAM window.
pub unsafe fn to_virt(phys: u32) -> *const u8 {
    phys.wrapping_sub(phys_base()) as *const u8
}

// ── Host-side tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Set a mock phys_base and verify round-trip translation.
    #[test]
    fn round_trip() {
        // Simulate a phys_base of 0x8000_0000 (common AST2600 DDR start).
        CACHED_PHYS_BASE.store(0x8000_0000, Ordering::Relaxed);

        let virt = 0x0100_0000 as *const u8; // typical RAM_NC start
        let phys = to_phys(virt);
        assert_eq!(phys, 0x8100_0000);

        // SAFETY: test only — we know the arithmetic is correct.
        let back = unsafe { to_virt(phys) };
        assert_eq!(back, virt);

        // Reset for other tests.
        CACHED_PHYS_BASE.store(UNSET, Ordering::Relaxed);
    }
}
