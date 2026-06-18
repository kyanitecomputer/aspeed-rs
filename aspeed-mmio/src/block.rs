use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::ptr;

/// A handle to a memory-mapped I/O register block.
///
/// Provides safe volatile read/write access following the derive-mmio access
/// model where `&self` methods are pure reads (no side effects assumed) and
/// `&mut self` methods perform writes or read-modify-write operations.
///
/// # `Send` but `!Sync`
///
/// `MmioBlock` is `Send`: an owning task may move a peripheral handle to
/// another thread (e.g., pass it to an Embassy task).
///
/// `MmioBlock` is `!Sync`: a shared reference `&MmioBlock` must **not** be
/// handed to multiple threads simultaneously. `MmioBlock` represents exclusive
/// peripheral ownership — concurrent access from multiple execution contexts
/// would violate MMIO register access semantics.  The `PhantomData<UnsafeCell>`
/// field enforces this at the type level.
///
/// # Safety contract
///
/// The caller of [`MmioBlock::new`] must ensure:
/// - `base` is a valid, aligned MMIO base address for the target peripheral.
/// - Only one `MmioBlock` handle exists per peripheral at a time (the borrow
///   checker enforces exclusive write access through `&mut self`).
///
/// # Example
///
/// ```rust,ignore
/// let mut wdt = unsafe { MmioBlock::new(0x14C3_7000) };
/// wdt.write32(0x0C, 0x4755); // unlock register
/// let status = wdt.read32(0x00);
/// wdt.modify32(0x08, |v| v | (1 << 0)); // set enable bit
/// ```
pub struct MmioBlock {
    base: usize,
    /// Makes `MmioBlock` `!Sync`: `UnsafeCell<T>` is `!Sync`, so
    /// `PhantomData<UnsafeCell<()>>` propagates `!Sync` to `MmioBlock`
    /// without adding any runtime cost or changing alignment.
    _not_sync: PhantomData<UnsafeCell<()>>,
}

impl MmioBlock {
    /// Create a handle to a memory-mapped register block.
    ///
    /// # Safety
    ///
    /// `base` must be a valid MMIO base address. The caller must guarantee
    /// that no other `MmioBlock` or raw pointer aliases this address range
    /// for the lifetime of this handle.
    #[inline(always)]
    pub const unsafe fn new(base: usize) -> Self {
        Self {
            base,
            _not_sync: PhantomData,
        }
    }

    /// Read a 32-bit register at `offset` bytes from base.
    ///
    /// This is a **PureRead** operation (derive-mmio terminology): it only
    /// requires a shared reference because reads are assumed side-effect-free
    /// for most registers.
    #[inline(always)]
    pub fn read32(&self, offset: usize) -> u32 {
        unsafe { ptr::read_volatile((self.base + offset) as *const u32) }
    }

    /// Write a 32-bit register at `offset` bytes from base.
    ///
    /// This is a **Write** operation: requires exclusive `&mut self` access
    /// to prevent concurrent writes through the borrow checker.
    #[inline(always)]
    pub fn write32(&mut self, offset: usize, value: u32) {
        unsafe { ptr::write_volatile((self.base + offset) as *mut u32, value) }
    }

    /// Read-modify-write a 32-bit register.
    ///
    /// This is a **Modify** operation: performs a volatile read, applies `f`,
    /// then a volatile write. Requires exclusive access.
    ///
    /// # Warning
    ///
    /// Not safe for registers where the read value differs from what was
    /// written (e.g., W1C status registers). Use separate `read32`/`write32`
    /// for those.
    #[inline(always)]
    pub fn modify32(&mut self, offset: usize, f: impl FnOnce(u32) -> u32) {
        let val = self.read32(offset);
        self.write32(offset, f(val));
    }

    /// Set bits in a 32-bit register (OR mask).
    #[inline(always)]
    pub fn set_bits32(&mut self, offset: usize, mask: u32) {
        self.modify32(offset, |v| v | mask);
    }

    /// Clear bits in a 32-bit register (AND NOT mask).
    #[inline(always)]
    pub fn clr_bits32(&mut self, offset: usize, mask: u32) {
        self.modify32(offset, |v| v & !mask);
    }

    /// Write a 64-bit value as two 32-bit volatile writes (low word first).
    #[inline(always)]
    pub fn write64(&mut self, offset: usize, value: u64) {
        self.write32(offset, value as u32);
        self.write32(offset + 4, (value >> 32) as u32);
    }

    /// Read a 64-bit value as two 32-bit volatile reads (low word first).
    #[inline(always)]
    pub fn read64(&self, offset: usize) -> u64 {
        let lo = self.read32(offset) as u64;
        let hi = self.read32(offset + 4) as u64;
        (hi << 32) | lo
    }

    /// Read an 8-bit register at `offset` bytes from base.
    #[inline(always)]
    pub fn read8(&self, offset: usize) -> u8 {
        unsafe { ptr::read_volatile((self.base + offset) as *const u8) }
    }

    /// Write an 8-bit register at `offset` bytes from base.
    #[inline(always)]
    pub fn write8(&mut self, offset: usize, value: u8) {
        unsafe { ptr::write_volatile((self.base + offset) as *mut u8, value) }
    }

    /// Read a 16-bit register at `offset` bytes from base.
    #[inline(always)]
    pub fn read16(&self, offset: usize) -> u16 {
        unsafe { ptr::read_volatile((self.base + offset) as *const u16) }
    }

    /// Write a 16-bit register at `offset` bytes from base.
    #[inline(always)]
    pub fn write16(&mut self, offset: usize, value: u16) {
        unsafe { ptr::write_volatile((self.base + offset) as *mut u16, value) }
    }

    /// Get the base address of this register block.
    #[inline(always)]
    pub const fn base(&self) -> usize {
        self.base
    }

    /// Create a sub-block at `offset` bytes from this block's base.
    ///
    /// Useful for nested register regions (derive-mmio Inner pattern).
    ///
    /// # Safety
    ///
    /// The caller must ensure the sub-block does not outlive the parent and
    /// is not used concurrently with the parent or any other `MmioBlock`
    /// that overlaps the same address range.  The borrow checker cannot
    /// enforce the no-alias invariant across two independently owned
    /// `MmioBlock` instances.
    #[inline(always)]
    pub unsafe fn sub_block(&self, offset: usize) -> MmioBlock {
        MmioBlock::new(self.base + offset)
    }
}

/// `MmioBlock` is `Send`: an owning task may transfer a peripheral handle
/// between threads (e.g., move into an Embassy async task).
///
/// Safety: volatile MMIO accesses are inherently single-at-a-time; the `Send`
/// bound says one thread may *own* the block, not that concurrent access is
/// safe.  Concurrent access is prevented by the `!Sync` bound.
unsafe impl Send for MmioBlock {}
