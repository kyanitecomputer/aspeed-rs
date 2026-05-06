//! Build script for embassy-aspeed.
//!
//! Responsibilities:
//!   1. Select the correct linker memory layout for the target chip.
//!   2. Copy the selected layout file to `OUT_DIR/memory.x` so that
//!      cortex-m-rt can find and `INCLUDE` it (see cortex-m-rt's link.x).
//!   3. Add `OUT_DIR` to the linker search path.
//!   4. Re-run when any layout file or this script changes.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let target_arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();

    // Host test builds (x86_64): no linker setup needed.
    if target_arch != "arm" && target_arch != "riscv32" {
        println!("cargo:rerun-if-changed=build.rs");
        return;
    }

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // ── Select memory layout by chip feature ─────────────────────────────────
    let layout_src = if env::var("CARGO_FEATURE_AST2600_SSP").is_ok() {
        "link/ast2600-ssp.x"
    } else if env::var("CARGO_FEATURE_AST1060").is_ok() {
        "link/ast1060.x"
    } else if env::var("CARGO_FEATURE_AST2700_BOOTMCU").is_ok() {
        "link/ast2700-bootmcu.x"
    } else {
        // No chip feature selected — lib.rs will also emit a compile_error!.
        panic!(
            "embassy-aspeed build.rs: no chip feature selected. \
             Enable exactly one of: ast2600-ssp, ast1060, ast2700-bootmcu"
        );
    };

    // Copy the layout file to OUT_DIR/memory.x.
    // cortex-m-rt's link.x starts with `INCLUDE memory.x`.
    let dest = out_dir.join("memory.x");
    fs::copy(layout_src, &dest)
        .unwrap_or_else(|e| panic!("failed to copy {} → {}: {}", layout_src, dest.display(), e));

    // Make OUT_DIR a linker search directory so `memory.x` is found.
    println!("cargo:rustc-link-search={}", out_dir.display());

    // Re-run the build script if the layout file or this script change.
    println!("cargo:rerun-if-changed={}", layout_src);
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=link/");
}
