#![no_std]
#![doc = "Safe MMIO primitives for ASPEED HAL drivers."]
#![doc = ""]
#![doc = "Inspired by patterns from `derive-mmio` (knurling-rs) and `tock-registers`:"]
#![doc = ""]
#![doc = "- **`MmioBlock`**: A handle to a memory-mapped register region with"]
#![doc = "  `&self` for reads and `&mut self` for writes (derive-mmio access model)."]
#![doc = "- **`ReadOnly`/`WriteOnly`/`ReadWrite`**: Typed single-register wrappers"]
#![doc = "  with compile-time access control (tock-registers model)."]
#![doc = "- **`poll_until`/`poll_until_async`**: Sync and async polling helpers"]
#![doc = "  for status register waiting patterns."]

mod block;
mod polling;
mod register;

pub use block::MmioBlock;
pub use polling::{poll_until, TimeoutError};
#[cfg(feature = "async")]
pub use polling::poll_until_async;
pub use register::{ReadOnly, ReadWrite, WriteOnly};
