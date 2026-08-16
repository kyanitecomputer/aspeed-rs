# aspeed-rs

Async Rust HAL for ASPEED SoC coprocessors and standalone SoCs, built on the [Embassy](https://embassy.dev/) framework.

Part of the [Kyanite](https://github.com/kyanitecomputer) stack.

> **Status:** experimental — expect breaking changes.

```
https://github.com/kyanitecomputer/aspeed-rs
```

## Supported chips

| Feature flag | SoC | Core | Target triple | HAL status |
|---|---|---|---|---|
| `ast2600-ssp` | AST2600 SSP | Cortex-M3 | `thumbv7m-none-eabi` | ✅ UART / GPIO / IPC / WDT / Timer / Clock |
| `ast1060` | AST1060 | Cortex-M4F | `thumbv7em-none-eabihf` | ✅ UART / GPIO / I2C / SPI / WDT / Timer / Clock |
| `ast2700-bootmcu` | AST2700 BootMCU | RV32IMC (ibex) | `riscv32imc-unknown-none-elf` | ✅ UART / IPC1 / Timer |
| `ast2700-ssp` | AST2700 SSP | Cortex-M4F | `thumbv7em-none-eabihf` | ❌ Not started |
| `ast2700-tsp` | AST2700 TSP | Cortex-M4F | `thumbv7em-none-eabihf` | ❌ Not started |

## Driver inventory

| Module | Chips | Description |
|--------|-------|-------------|
| `boot` | ast2600-ssp, ast1060 | `pre_init`: VTOR, FPU, cache, SCU unlock |
| `boot_rv` | ast2700-bootmcu | RV32 `pre_init` |
| `clock` | ast2600-ssp, ast1060 | `ClockGate` enable/disable |
| `uart` | all three | 16550 async TX/RX |
| `gpio` | ast2600-ssp, ast1060 | v1 GPIO (async edge wait) |
| `ipc` | ast2600-ssp | 15-channel IPC doorbell |
| `ipc1` | ast2700-bootmcu | Data-channel polling IPC |
| `i2c` | ast1060 | Async I2C controller |
| `spi` | ast1060 | SPI/FMC |
| `wdt` | ast2600-ssp, ast1060 | WDT + software reboot |
| `timer` | ast2600-ssp, ast1060 | 8-channel countdown |
| `addr` | all | Address translation (virtual ↔ physical) |
| `time_driver` | ast2600-ssp, ast1060 | SysTick Embassy time driver |
| `time_driver_rv` | ast2700-bootmcu | 64-bit custom timer |

## Dependencies

- [`aspeed-data`](https://github.com/kyanitecomputer/aspeed-data) — provides the generated `aspeed-pac` Peripheral Access Crate. Required as a sibling directory at `../aspeed-data` during local development. Post-push: switches to a `git =` Cargo dependency.
- [Embassy](https://github.com/embassy-rs/embassy) — async embedded runtime (git dependency, pinned rev)

## Build and check

All automation uses [Dagger](https://dagger.io). `aspeed-data` must be a sibling directory.

```sh
# Check all chip targets compile
dagger call check --aspeed-data ../aspeed-data

# Run host-side unit tests
dagger call test --aspeed-data ../aspeed-data

# Full CI pipeline
dagger call ci --aspeed-data ../aspeed-data
```

## Using in firmware

Firmware binaries (examples, applications) live in [`aspeed-mcu-runtime`](https://github.com/kyanitecomputer/aspeed-mcu-runtime). This crate is a library only — no `[[bin]]` entries.

Quick start (from aspeed-mcu-runtime):

```rust
#![no_std]
#![no_main]

use embassy_aspeed as hal;
use embassy_executor::Spawner;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    hal::init(hal::Config::default());
    let mut uart = hal::uart::Uart::new_uart11();
    uart.write_all(b"Hello from AST2600 SSP!\r\n").await.unwrap();
}
```

## Contributing

See the org-wide [CONTRIBUTING guide](https://github.com/kyanitecomputer/.github/blob/main/CONTRIBUTING.md).
Contributions are dual-licensed.

## Security

See the org-wide [SECURITY policy](https://github.com/kyanitecomputer/.github/blob/main/SECURITY.md).

## License

Dual-licensed under either of Apache-2.0 ([LICENSE-APACHE](LICENSE-APACHE)) or
MIT ([LICENSE-MIT](LICENSE-MIT)) at your option.
