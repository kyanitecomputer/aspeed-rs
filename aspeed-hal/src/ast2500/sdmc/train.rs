//! Pure decision logic of the AST2500 DRAM training, kept free of MMIO so
//! the host can test it.

/// Sweeps per Vref scan before the attempt fails.
pub const SWEEP_ATTEMPTS: usize = 4;

/// Minimum number of consecutive passing Vref steps for a usable window.
pub const MIN_PASSES: u8 = 16;

/// Contiguous run of passing settings found by one sweep.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Window {
    /// First passing setting.
    pub first: u8,
    /// Last passing setting.
    pub last: u8,
    /// Number of passing settings.
    pub passes: u8,
    /// Passing setting with the largest metric (first one on ties).
    pub best: u8,
    /// Metric of `best`.
    pub best_metric: u8,
}

impl Window {
    /// Midpoint of the window, rounded up: the DRAM-side Vref to program.
    pub const fn center(&self) -> u8 {
        (self.first as u16 + self.last as u16).div_ceil(2) as u8
    }
}

/// Why a Vref scan was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanError<E> {
    /// No setting passed in any sweep.
    NoPass,
    /// The passing run was shorter than [`MIN_PASSES`].
    Margin(Window),
    /// The probe itself failed (PHY initialisation did not complete).
    Probe(E),
}

/// One sweep: probe each setting in order, stop at the first failure after
/// a pass. `probe` returns `Some(metric)` on pass. `None` if nothing passed.
pub fn sweep<E>(
    settings: impl IntoIterator<Item = u8>,
    mut probe: impl FnMut(u8) -> Result<Option<u8>, E>,
) -> Result<Option<Window>, E> {
    let mut window: Option<Window> = None;
    for setting in settings {
        match (probe(setting)?, window.as_mut()) {
            (Some(metric), None) => {
                window = Some(Window {
                    first: setting,
                    last: setting,
                    passes: 1,
                    best: setting,
                    best_metric: metric,
                })
            }
            (Some(metric), Some(w)) => {
                w.last = setting;
                w.passes += 1;
                if metric > w.best_metric {
                    w.best = setting;
                    w.best_metric = metric;
                }
            }
            (None, Some(_)) => break,
            (None, None) => {}
        }
    }
    Ok(window)
}

/// Up to [`SWEEP_ATTEMPTS`] sweeps until one passes, then the margin check.
pub fn scan<E, S: IntoIterator<Item = u8>>(
    settings: impl Fn() -> S,
    mut probe: impl FnMut(u8) -> Result<Option<u8>, E>,
) -> Result<Window, ScanError<E>> {
    for _ in 0..SWEEP_ATTEMPTS {
        if let Some(w) = sweep(settings(), &mut probe).map_err(ScanError::Probe)? {
            return if w.passes < MIN_PASSES {
                Err(ScanError::Margin(w))
            } else {
                Ok(w)
            };
        }
    }
    Err(ScanError::NoPass)
}

/// Smaller of the DQL and DQH read-eye pass windows in MCR3D0.
pub const fn eye_metric(mcr3d0: u32) -> u8 {
    let dql = mcr3d0 as u8;
    let dqh = (mcr3d0 >> 8) as u8;
    if dql < dqh {
        dql
    } else {
        dqh
    }
}

/// Final read-training margin check on MCR3A0 (gate) and MCR3D0 (eye),
/// both byte lanes of each.
pub const fn margins_ok(mcr3a0: u32, mcr3d0: u32) -> bool {
    (mcr3a0 & 0xFFFF) >= 0x150
        && (mcr3a0 >> 16) >= 0x150
        && (mcr3d0 & !0xFF00) >= 0x90
        && (mcr3d0 >> 8) >= 0x90
}

/// DRAM capacity, as encoded in MCR04 bits 1:0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Capacity {
    Mib128 = 0,
    Mib256 = 1,
    Mib512 = 2,
    Gib1 = 3,
}

impl Capacity {
    /// Capacity from MCR04 bits 1:0.
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 3 {
            0 => Capacity::Mib128,
            1 => Capacity::Mib256,
            2 => Capacity::Mib512,
            _ => Capacity::Gib1,
        }
    }

    /// Size in bytes.
    pub const fn bytes(self) -> u32 {
        (128 << 20) << (self as u32)
    }

    /// Size from the alias probe: whether the patterns written at
    /// `0x8810_0000`, `0x9010_0000` and `0xA010_0000` survived the later
    /// writes to the addresses below them.
    pub const fn from_probe(alive_256m: bool, alive_512m: bool, alive_1g: bool) -> Self {
        if alive_1g {
            Capacity::Gib1
        } else if alive_512m {
            Capacity::Mib512
        } else if alive_256m {
            Capacity::Mib256
        } else {
            Capacity::Mib128
        }
    }

    /// tRFC byte for this capacity from the timing table's packed word.
    pub const fn trfc(self, packed: u32) -> u8 {
        (packed >> (8 * self as u32)) as u8
    }
}

/// MCR54 value protecting `[0, bytes)`: the range field holds the last
/// MiB index, bits 29:20.
pub const fn ecc_range(bytes: u32) -> u32 {
    (bytes - (1 << 20)) & 0x3FF0_0000
}

/// Whether ECC over `protected` bytes, plus its 1/8 code storage, fits in
/// the `available` bytes below the VGA carve-out.
pub const fn ecc_fits(protected: u32, available: u32) -> bool {
    protected.is_multiple_of(1 << 20)
        && protected != 0
        && protected as u64 + protected as u64 / 8 <= available as u64
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;

    fn pass_in(lo: u8, hi: u8) -> impl FnMut(u8) -> Result<Option<u8>, Infallible> {
        move |v| Ok((lo..=hi).contains(&v).then_some(v))
    }

    #[test]
    fn sweep_stops_at_first_failure_after_a_pass() {
        let mut probed = [false; 0x80];
        let w = sweep(0x40..0x80, |v: u8| {
            probed[v as usize] = true;
            Ok::<_, Infallible>(((0x48..=0x5F).contains(&v) || v == 0x70).then_some(1))
        })
        .unwrap()
        .unwrap();
        assert_eq!((w.first, w.last, w.passes), (0x48, 0x5F, 24));
        assert!(probed[0x60] && !probed[0x61] && !probed[0x70]);
    }

    #[test]
    fn sweep_keeps_first_best_on_ties() {
        let w = sweep(1..0x40, |v: u8| {
            Ok::<_, Infallible>(
                (10..=40)
                    .contains(&v)
                    .then_some(if v >= 20 { 9 } else { 3 }),
            )
        })
        .unwrap()
        .unwrap();
        assert_eq!((w.best, w.best_metric), (20, 9));
    }

    #[test]
    fn scan_retries_empty_sweeps_then_gives_up() {
        let mut sweeps = 0;
        let r = scan(
            || 1u8..4,
            |v| {
                if v == 1 {
                    sweeps += 1;
                }
                Ok::<_, Infallible>(None)
            },
        );
        assert_eq!(r, Err(ScanError::NoPass));
        assert_eq!(sweeps, SWEEP_ATTEMPTS);
    }

    #[test]
    fn scan_rejects_short_runs_and_centers_long_ones() {
        assert!(matches!(
            scan(|| 1u8..0x40, pass_in(10, 24)),
            Err(ScanError::Margin(Window { passes: 15, .. }))
        ));
        let w = scan(|| 1u8..0x40, pass_in(10, 25)).unwrap();
        assert_eq!((w.passes, w.center()), (16, 18));
    }

    #[test]
    fn eye_and_margin_checks_cover_both_lanes() {
        assert_eq!(eye_metric(0x0000_A0B0), 0xA0);
        assert!(margins_ok(0x0150_0150, 0x0000_9090));
        assert!(!margins_ok(0x014F_0150, 0x0000_9090));
        assert!(!margins_ok(0x0150_0150, 0x0000_908F));
    }

    #[test]
    fn capacity_and_trfc_follow_the_alias_probe() {
        let c = Capacity::from_probe(true, true, false);
        assert_eq!(c, Capacity::Mib512);
        assert_eq!(c.bytes(), 512 << 20);
        assert_eq!(c.trfc(0x9971_4545), 0x71);
        assert_eq!(
            Capacity::from_probe(false, false, false).trfc(0x9971_4545),
            0x45
        );
    }

    #[test]
    fn ecc_range_covers_424_mib_below_a_32_mib_carve_out() {
        assert_eq!(ecc_range(424 << 20), 0x1A70_0000);
        assert!(ecc_fits(424 << 20, (512 - 32) << 20));
        assert!(!ecc_fits(427 << 20, (512 - 32) << 20));
    }
}
