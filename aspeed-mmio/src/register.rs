use core::ptr;

/// A read-only MMIO register (tock-registers `ReadOnly` pattern).
///
/// Only allows volatile reads. The type parameter `T` is the register width
/// (typically `u32` for 32-bit MMIO peripherals).
///
/// # Safety contract
///
/// Created via `unsafe fn new()` — caller must ensure the address is a valid,
/// properly aligned MMIO register.
pub struct ReadOnly<T: Copy> {
    ptr: *const T,
}

impl<T: Copy> ReadOnly<T> {
    /// Create a read-only register handle.
    ///
    /// # Safety
    ///
    /// `addr` must be a valid, aligned MMIO register address.
    #[inline(always)]
    pub const unsafe fn new(addr: usize) -> Self {
        Self {
            ptr: addr as *const T,
        }
    }

    /// Volatile read of the register value.
    #[inline(always)]
    pub fn read(&self) -> T {
        unsafe { ptr::read_volatile(self.ptr) }
    }

    /// Get the raw address of this register.
    #[inline(always)]
    pub fn addr(&self) -> usize {
        self.ptr as usize
    }
}

unsafe impl<T: Copy> Send for ReadOnly<T> {}

/// A write-only MMIO register (tock-registers `WriteOnly` pattern).
///
/// Only allows volatile writes. Reading is not permitted at the type level.
pub struct WriteOnly<T: Copy> {
    ptr: *mut T,
}

impl<T: Copy> WriteOnly<T> {
    /// Create a write-only register handle.
    ///
    /// # Safety
    ///
    /// `addr` must be a valid, aligned MMIO register address.
    #[inline(always)]
    pub const unsafe fn new(addr: usize) -> Self {
        Self {
            ptr: addr as *mut T,
        }
    }

    /// Volatile write to the register.
    #[inline(always)]
    pub fn write(&mut self, value: T) {
        unsafe { ptr::write_volatile(self.ptr, value) }
    }

    /// Get the raw address of this register.
    #[inline(always)]
    pub fn addr(&self) -> usize {
        self.ptr as usize
    }
}

unsafe impl<T: Copy> Send for WriteOnly<T> {}

/// A read-write MMIO register (tock-registers `ReadWrite` pattern).
///
/// Allows volatile reads via `&self` and volatile writes via `&mut self`,
/// enforcing exclusive write access through the borrow checker.
pub struct ReadWrite<T: Copy> {
    ptr: *mut T,
}

impl<T: Copy> ReadWrite<T> {
    /// Create a read-write register handle.
    ///
    /// # Safety
    ///
    /// `addr` must be a valid, aligned MMIO register address.
    #[inline(always)]
    pub const unsafe fn new(addr: usize) -> Self {
        Self {
            ptr: addr as *mut T,
        }
    }

    /// Volatile read of the register value.
    ///
    /// PureRead: only requires shared reference.
    #[inline(always)]
    pub fn read(&self) -> T {
        unsafe { ptr::read_volatile(self.ptr as *const T) }
    }

    /// Volatile write to the register.
    ///
    /// Write: requires exclusive reference.
    #[inline(always)]
    pub fn write(&mut self, value: T) {
        unsafe { ptr::write_volatile(self.ptr, value) }
    }

    /// Read-modify-write the register.
    ///
    /// Modify: requires exclusive reference. Performs a volatile read,
    /// applies the closure, then a volatile write.
    #[inline(always)]
    pub fn modify(&mut self, f: impl FnOnce(T) -> T) {
        let val = self.read();
        self.write(f(val));
    }

    /// Get the raw address of this register.
    #[inline(always)]
    pub fn addr(&self) -> usize {
        self.ptr as usize
    }
}

unsafe impl<T: Copy> Send for ReadWrite<T> {}
