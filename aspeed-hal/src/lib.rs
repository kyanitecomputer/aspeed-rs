//! Executor-free blocking drivers for ASPEED SoCs.
//!
//! Firmware that owns the SoC before any runtime exists (a reset-vector
//! bootblock, a library linked into foreign firmware) drives the hardware
//! through these cores. `embassy-aspeed` keeps the async wrappers.
//!
//! # Feature flags
//!
//! - `ast2500` — AST2500 ARM1176 main core (`aspeed-pac/ast2500-arm`)

#![no_std]

#[cfg(feature = "ast2500")]
pub mod ast2500;
