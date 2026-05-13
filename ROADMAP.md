# aspeed-rs — Roadmap

## Current state

AST2600 SSP, AST1060, and AST2700 BootMCU HAL complete. AST2700 SSP/TSP not started.

## Phase P: AST2700 SSP/TSP HAL — basic vertical slice
*Requires `aspeed-data` Phase A (AST2700 YAML expansion) first.*

| Task | Description | Size |
|------|-------------|------|
| P-1 | Feature flags + linker scripts (`ast2700-ssp`, `ast2700-tsp`) | M |
| P-2 | AST2700 SSP boot module (FPU, D/I-cache via `ssp_v2::CACHE_FUNC`, SCU unlock) | M |
| P-3 | UART / WDT / Timer for AST2700 SSP/TSP (extend `#[cfg]` guards, same IP) | M |
| P-4 | GPIO for AST2700 SSP/TSP | S |
| P-5 | Clock HAL for AST2700 (`clock_ast2700_v1::SCU`) | M |

## Phase Q: AST2700 INTC + IPC0

| Task | Description | Size |
|------|-------------|------|
| Q-1 | `ipc0.rs` — async SSP↔CA35 send/receive via IPC0 bus | L |
| Q-2 | INTC0 wiring in `platform_init()` for all AST2700 peripheral IRQs | M |

## Phase R: AST2600 driver expansion

| Task | Description | Size |
|------|-------------|------|
| R-1 | Enable I2C for `ast2600-ssp` (identical register layout to ast1060) | S |
| R-2 | HACE HAL driver (SHA-256/512 via DMA descriptor chain) | L |

## Phase S: Remaining driver gaps

| Task | Description | Chips | Size |
|------|-------------|-------|------|
| S-1 | SGPIO HAL | AST2600, AST1060 | M |
| S-2 | UART DMA HAL | AST1060 | M |
| S-3 | I3C HAL (SDR mode: ENTDAA, write, read) | AST1060 | XL |
| S-4 | WDT for BootMCU | AST2700 BootMCU | S |

## Phase T: New peripheral drivers

| Task | Description | Size |
|------|-------------|------|
| T-1 | PWM/Tachometer (`pwm_v1`) | L |
| T-2 | ADC single-shot (`adc_v1`) | M |

## embedded-hal trait coverage

| Trait | Status | Notes |
|-------|--------|-------|
| `embedded_hal::serial::Write` | ✅ | UART |
| `embedded_io::Write` | ✅ | UART |
| `embedded_io_async::Write` | ✅ | UART |
| `embedded_hal::i2c::I2c` | ❌ | I2C uses custom interface |
| `embedded_hal_async::i2c::I2c` | ❌ | |
| `embedded_hal::spi::SpiDevice` | ❌ | SPI uses custom interface |
| `embedded_hal::pwm::SetDutyCycle` | — | Planned in T-1 |

## Post-push dependency migration

Once pushed to `github.com/kyanitecomputer/aspeed-rs`, switch `aspeed-pac` dep from:
```toml
aspeed-pac = { path = "../../aspeed-data/aspeed-pac" }
```
to:
```toml
aspeed-pac = { git = "https://github.com/kyanitecomputer/aspeed-data", rev = "<sha>" }
```
