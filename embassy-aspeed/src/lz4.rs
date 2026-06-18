//! LZ4 block decompression for AST2700-A2 payload loading.
//!
//! A2 images may carry LZ4-compressed payloads to cut flash read time at boot.
//! This wraps lz4_flex's *safe* (bounds-checked, no `unsafe`) block decoder —
//! appropriate for expanding attacker-reachable flash contents.
//!
//! [`decompress_into`] writes straight into the caller's buffer (e.g. a slice
//! over the DRAM load address) with **no output allocation**, so a payload can
//! be expanded directly to where it runs. lz4_flex nonetheless links `alloc`
//! unconditionally, so the final binary must still provide a global allocator
//! even though this path never allocates. (The `fbrozovic/lz4_flex` heapless
//! fork removes that requirement; revisit when it lands upstream.)

pub use lz4_flex::block::DecompressError;

/// Decompress a raw LZ4 block from `src` into `dst`.
///
/// `dst` must be exactly the uncompressed length. Returns the number of bytes
/// written. No allocation occurs.
pub fn decompress_into(src: &[u8], dst: &mut [u8]) -> Result<usize, DecompressError> {
    lz4_flex::block::decompress_into(src, dst)
}

/// Decompress an LZ4 block whose first 4 little-endian bytes are the
/// uncompressed size (as produced by lz4_flex `compress_prepend_size`) into
/// `dst`, returning the number of bytes written.
///
/// Errors with [`DecompressError::OutputTooSmall`] if `dst` cannot hold the
/// declared size, or [`DecompressError::ExpectedAnotherByte`] if the size
/// prefix is missing.
pub fn decompress_size_prepended_into(
    src: &[u8],
    dst: &mut [u8],
) -> Result<usize, DecompressError> {
    if src.len() < 4 {
        return Err(DecompressError::ExpectedAnotherByte);
    }
    let size = u32::from_le_bytes([src[0], src[1], src[2], src[3]]) as usize;
    if size > dst.len() {
        return Err(DecompressError::OutputTooSmall {
            expected: size,
            actual: dst.len(),
        });
    }
    decompress_into(&src[4..], &mut dst[..size])
}

// ── Global allocator (firmware only) ────────────────────────────────────────
//
// lz4_flex links `alloc` unconditionally, so a `#[global_allocator]` must exist
// in every bootmcu binary that enables `lz4`. We provide it centrally here so
// the bins don't each have to. It is a tiny bump allocator: `decompress_into`
// (the payload path) never allocates, so this only backstops incidental small
// allocations; freed memory is not reclaimed, which is fine for one-shot boot.
//
// Gated to riscv32 + not(test): host unit tests use the std allocator.
#[cfg(all(target_arch = "riscv32", not(test)))]
mod global_alloc {
    use core::alloc::{GlobalAlloc, Layout};
    use core::cell::Cell;
    use critical_section::Mutex;

    /// Backing store for the bump allocator (in SRAM/BSS).
    const HEAP_SIZE: usize = 16 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    /// Next free byte offset into `HEAP`.
    static NEXT: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));

    struct BumpAlloc;

    unsafe impl GlobalAlloc for BumpAlloc {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            let base = core::ptr::addr_of_mut!(HEAP) as usize;
            critical_section::with(|cs| {
                let cur = NEXT.borrow(cs).get();
                let start = (base + cur + layout.align() - 1) & !(layout.align() - 1);
                let new_next = match start.checked_add(layout.size()) {
                    Some(end) => end - base,
                    None => return core::ptr::null_mut(),
                };
                if new_next > HEAP_SIZE {
                    return core::ptr::null_mut();
                }
                NEXT.borrow(cs).set(new_next);
                start as *mut u8
            })
        }

        unsafe fn dealloc(&self, _ptr: *mut u8, _layout: Layout) {
            // Bump allocator: memory is not reclaimed.
        }
    }

    #[global_allocator]
    static ALLOCATOR: BumpAlloc = BumpAlloc;
}

#[cfg(test)]
mod tests {
    use super::*;
    use lz4_flex::block::{compress, compress_prepend_size};

    #[test]
    fn round_trip_into_exact_buffer() {
        // Repetition so the encoder emits back-references, exercising both the
        // literal-copy and match-copy paths of the decoder.
        let original = b"The quick brown fox. The quick brown fox. The quick brown fox.";
        let compressed = compress(original);
        assert!(compressed.len() < original.len(), "should compress");

        let mut out = [0u8; 62];
        assert_eq!(original.len(), out.len());
        let n = decompress_into(&compressed, &mut out).unwrap();
        assert_eq!(n, original.len());
        assert_eq!(&out[..n], original);
    }

    #[test]
    fn round_trip_size_prepended() {
        let original = b"aaaaaaaaaaaaaaaabbbbbbbbbbbbbbbbccccccccccccccccdddd";
        let compressed = compress_prepend_size(original);

        let mut out = [0u8; 128];
        let n = decompress_size_prepended_into(&compressed, &mut out).unwrap();
        assert_eq!(n, original.len());
        assert_eq!(&out[..n], original);
    }

    #[test]
    fn size_prepended_rejects_small_output() {
        let original = b"0123456789ABCDEF0123456789ABCDEF";
        let compressed = compress_prepend_size(original);

        let mut too_small = [0u8; 8];
        assert!(matches!(
            decompress_size_prepended_into(&compressed, &mut too_small),
            Err(DecompressError::OutputTooSmall { .. })
        ));
    }

    #[test]
    fn missing_size_prefix_errors() {
        let mut out = [0u8; 16];
        assert!(matches!(
            decompress_size_prepended_into(&[0x01, 0x02], &mut out),
            Err(DecompressError::ExpectedAnotherByte)
        ));
    }
}
