//! Executor-free blocking drivers for ASPEED SoCs.
//!
//! Firmware that owns the SoC before any runtime exists (a reset-vector
//! bootblock, a library linked into foreign firmware) drives the hardware
//! through these cores. `embassy-aspeed` keeps the async wrappers.
//!
//! # Feature flags
//!
//! - `ast2400` — AST2400/AST1250 ARM926 main core (`aspeed-pac/ast2400-arm`)
//! - `ast2500` — AST2500 ARM1176 main core (`aspeed-pac/ast2500-arm`)
//!
//! One SoC feature at a time: the PAC exposes one chip.

#![no_std]

#[cfg(all(feature = "ast2400", feature = "ast2500"))]
compile_error!("aspeed-hal builds for one SoC: enable `ast2400` or `ast2500`");

#[cfg(feature = "ast2400")]
pub mod ast2400;
#[cfg(feature = "ast2500")]
pub mod ast2500;
#[cfg(any(feature = "ast2400", feature = "ast2500"))]
pub mod fmc;
#[cfg(any(feature = "ast2400", feature = "ast2500"))]
pub mod timer;
