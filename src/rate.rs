//! Timing of the PIO program as pure functions of the rate and clk_sys.
//!
//! Everything here derives from the measurement program `dshot_m1_probe` (its PIO program and
//! its divider table) and from the capture windows measured with it on 2026-11-12. None of it
//! is a hot path: the back-end calls these once when it is built.

/// The DShot rates an AM32 ESC accepts. DShot150 is not here: AM32 rejects it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Rate {
    /// 300 kbit/s, 3.333 us per bit.
    Dshot300,
    /// 600 kbit/s, 1.667 us per bit.
    Dshot600,
}

/// A PIO clock divider: `integer + fraction / 256`, the format of the PIO `CLKDIV` register.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ClockDivider {
    /// The integer part, 1 or more.
    pub integer: u16,
    /// The fraction in 256ths.
    pub fraction: u8,
}

impl ClockDivider {
    /// The divider times 256.
    #[must_use]
    pub const fn as_q8(self) -> u32 {
        ((self.integer as u32) << 8) | self.fraction as u32
    }
}

/// Why [`Rate::clock_divider`] gave no divider.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum DividerError {
    /// clk_sys is too slow: the divider would be below 1.0 and the best one (1.0) is more than
    /// 0.5 % off the rate. Rounding to 1/256 alone costs at most 0.2 % at a divider of 1.0 and
    /// less above it, so this is the only way to miss the tolerance.
    ClockTooSlow,
}

impl Rate {
    /// PIO cycles in one bit, the same at both rates: 15 low, 15 data, 10 high.
    pub const CYCLES_PER_BIT: u32 = 40;
    /// PIO cycles of one reply sample (`in pins, 4` and `jmp x--`).
    pub const REPLY_SAMPLE_CYCLES: u32 = 2;
    /// Samples in one RX word (autopush after 8 four-bit samples).
    pub const SAMPLES_PER_WORD: u32 = 8;
    /// The accepted error of the bit rate, in thousandths: 5 is 0.5 %.
    const TOLERANCE_PER_MILLE: u64 = 5;

    /// Bits per second on the wire: 300 000 or 600 000.
    #[must_use]
    pub const fn bits_per_second(self) -> u32 {
        match self {
            Rate::Dshot300 => 300_000,
            Rate::Dshot600 => 600_000,
        }
    }

    /// PIO cycles in one bit: 40.
    #[must_use]
    pub const fn cycles_per_bit(self) -> u32 {
        Self::CYCLES_PER_BIT
    }

    /// The three phases of one bit in PIO cycles, `[low, data, high]` = `[15, 15, 10]`.
    ///
    /// All lines are low in the first, carry the data level in the second, and are high in the
    /// third. A 0 bit is therefore low for 15 of 40 cycles (37.5 %, T0H of the DShot spec is
    /// 37.5 %), a 1 bit for 30 of 40 (75 %). AM32 reads a 1 above half a bit (`Src/dshot.c:75-82`).
    #[must_use]
    pub const fn phase_cycles(self) -> [u32; 3] {
        [15, 15, 10]
    }

    /// The PIO clock the program needs: bits per second times 40, 12 MHz at DShot300 and 24 MHz
    /// at DShot600.
    #[must_use]
    pub const fn pio_clock_hz(self) -> u32 {
        self.bits_per_second() * Self::CYCLES_PER_BIT
    }

    /// The clock divider for a system clock of `clk_sys_hz`: clk_sys over [`Rate::pio_clock_hz`],
    /// rounded to the nearest 1/256 (12.5 at DShot300 and 6.25 at DShot600 for 150 MHz).
    ///
    /// The bit rate that results is within 0.5 % of the nominal one, or this returns an error.
    ///
    /// # Errors
    ///
    /// [`DividerError::ClockTooSlow`] when clk_sys is so low that the divider would have to be
    /// below 1.0 by more than 0.5 %.
    pub const fn clock_divider(self, clk_sys_hz: u32) -> Result<ClockDivider, DividerError> {
        let pio = self.pio_clock_hz() as u64;
        let clk = clk_sys_hz as u64;
        // Divider times 256, rounded to nearest, not below 1.0. At most 4.29 GHz / 12 MHz * 256
        // fits a u16 integer part, so the result always fits the register.
        let exact_num = clk * 256;
        let mut q = (exact_num + pio / 2) / pio;
        if q < 256 {
            q = 256;
        }
        // |exact - q| / q within the tolerance, in integers: |clk*256 - q*pio| * 1000 <= 5 * q * pio.
        let achieved = q * pio;
        let diff = exact_num.abs_diff(achieved);
        if diff * 1000 > Self::TOLERANCE_PER_MILLE * achieved {
            return Err(DividerError::ClockTooSlow);
        }
        Ok(ClockDivider {
            integer: (q >> 8) as u16,
            fraction: (q & 0xFF) as u8,
        })
    }

    /// Nominal reply samples per reply bit: 16 at both rates.
    ///
    /// The ESC's reply bit is 4/5 of a command bit (it is the 5/4 rate), that is 32 PIO cycles;
    /// one sample takes [`Rate::REPLY_SAMPLE_CYCLES`] = 2 cycles. (AM32's reply is about 3.5 %
    /// faster than nominal, measured; see the README.)
    #[must_use]
    pub const fn reply_samples_per_bit(self) -> u32 {
        Self::CYCLES_PER_BIT * 4 / 5 / Self::REPLY_SAMPLE_CYCLES
    }

    /// PIO cycles between two reply samples: 2, so 6.0 MS/s at DShot300 and 12.0 MS/s at DShot600.
    #[must_use]
    pub const fn reply_sample_cycles(self) -> u32 {
        Self::REPLY_SAMPLE_CYCLES
    }

    /// The capture window after the lines are released, in nanoseconds: 89 500 at DShot300 and
    /// 60 400 at DShot600.
    ///
    /// Measured on 2026-11-12 with the probe on the four ESCs of the board: the latest reply
    /// end seen in 1000 captures per line (80.4 us and 50.8 us after the last rising edge of the
    /// frame; the lines are released 10 PIO cycles later) plus a margin of 10 us.
    #[must_use]
    pub const fn capture_window_ns(self) -> u32 {
        match self {
            Rate::Dshot300 => 89_500,
            Rate::Dshot600 => 60_400,
        }
    }

    /// RX FIFO words in one capture: the window of [`Rate::capture_window_ns`] in whole words,
    /// rounded up, which is 68 at DShot300 and 91 at DShot600.
    ///
    /// One word is 8 samples of 2 cycles = 16 PIO cycles, and the PIO clock is fixed by the rate
    /// (the divider of [`Rate::clock_divider`]), so the count does not depend on clk_sys. The 10
    /// us of margin inside the window is the only margin, plus the rounding up (at most one word,
    /// 1.33 us or 0.67 us). The samples are `8 * words`; the program is loaded with that minus 1.
    #[must_use]
    pub const fn capture_words(self) -> u32 {
        // window_ns * pio_clock_khz / (1e6 * 16 cycles per word); the product is at most
        // 1.45e9, inside a u32.
        let num = self.capture_window_ns() * (self.pio_clock_hz() / 1000);
        let den = 1_000_000 * Self::REPLY_SAMPLE_CYCLES * Self::SAMPLES_PER_WORD;
        num.div_ceil(den)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    const BOTH: [Rate; 2] = [Rate::Dshot300, Rate::Dshot600];

    #[test]
    fn bits_per_second_is_300k_and_600k() {
        assert_eq!(Rate::Dshot300.bits_per_second(), 300_000);
        assert_eq!(Rate::Dshot600.bits_per_second(), 600_000);
    }

    #[test]
    fn cycles_per_bit_is_40() {
        for r in BOTH {
            assert_eq!(r.cycles_per_bit(), 40);
        }
    }

    #[test]
    fn phase_cycles_are_15_15_10_and_make_a_bit() {
        for r in BOTH {
            let [low, data, high] = r.phase_cycles();
            assert_eq!([low, data, high], [15, 15, 10]);
            assert_eq!(low + data + high, r.cycles_per_bit());
            // T0H 37.5 %, T1H 75 % of the bit.
            assert_eq!(low * 1000 / r.cycles_per_bit(), 375);
            assert_eq!((low + data) * 100 / r.cycles_per_bit(), 75);
        }
    }

    #[test]
    fn pio_clock_hz_is_12_and_24_mhz() {
        assert_eq!(Rate::Dshot300.pio_clock_hz(), 12_000_000);
        assert_eq!(Rate::Dshot600.pio_clock_hz(), 24_000_000);
    }

    #[test]
    fn clock_divider_at_150_mhz_is_12_5_and_6_25_exactly() {
        // The probe's table: div_q8 3200 and 1600.
        let d300 = Rate::Dshot300.clock_divider(150_000_000).unwrap();
        assert_eq!(
            d300,
            ClockDivider {
                integer: 12,
                fraction: 128
            }
        );
        assert_eq!(d300.as_q8(), 3200);
        let d600 = Rate::Dshot600.clock_divider(150_000_000).unwrap();
        assert_eq!(
            d600,
            ClockDivider {
                integer: 6,
                fraction: 64
            }
        );
        assert_eq!(d600.as_q8(), 1600);
    }

    /// Relative error of the bit rate that divider `d` gives at `clk`, by f64.
    fn rate_error(rate: Rate, clk: u32, d: ClockDivider) -> f64 {
        let div = f64::from(d.as_q8()) / 256.0;
        let pio = f64::from(clk) / div;
        let want = f64::from(rate.pio_clock_hz());
        (pio - want).abs() / want
    }

    #[test]
    fn clock_divider_at_125_and_133_mhz_is_within_half_a_percent() {
        for clk in [125_000_000, 133_000_000] {
            for r in BOTH {
                let d = r.clock_divider(clk).unwrap();
                assert!(rate_error(r, clk, d) <= 0.005, "{clk} {r:?}");
            }
        }
        // 125 MHz / 12 MHz = 10.41667 -> 10 + 107/256; 133 MHz / 12 MHz = 11.08333 -> 11 + 21/256.
        let d = Rate::Dshot300.clock_divider(125_000_000).unwrap();
        assert_eq!((d.integer, d.fraction), (10, 107));
        let d = Rate::Dshot300.clock_divider(133_000_000).unwrap();
        assert_eq!((d.integer, d.fraction), (11, 21));
    }

    #[test]
    fn clock_divider_refuses_a_clock_too_slow_for_the_rate() {
        assert_eq!(
            Rate::Dshot300.clock_divider(0),
            Err(DividerError::ClockTooSlow)
        );
        assert_eq!(
            Rate::Dshot300.clock_divider(5_000_000),
            Err(DividerError::ClockTooSlow)
        );
        assert_eq!(
            Rate::Dshot600.clock_divider(12_000_000),
            Err(DividerError::ClockTooSlow)
        );
        // 1.0 exactly is fine, and so is 0.41 % short of it (clamped to 1.0), but not 0.8 % short.
        assert_eq!(
            Rate::Dshot300.clock_divider(12_000_000),
            Ok(ClockDivider {
                integer: 1,
                fraction: 0
            })
        );
        assert!(Rate::Dshot300.clock_divider(11_950_000).is_ok());
        assert_eq!(
            Rate::Dshot300.clock_divider(11_900_000),
            Err(DividerError::ClockTooSlow)
        );
        // The fastest clock a u32 can hold still fits the register.
        assert!(Rate::Dshot300.clock_divider(u32::MAX).is_ok());
    }

    #[test]
    fn as_q8_is_the_divider_times_256() {
        assert_eq!(
            ClockDivider {
                integer: 1,
                fraction: 0
            }
            .as_q8(),
            256
        );
        assert_eq!(
            ClockDivider {
                integer: 65535,
                fraction: 255
            }
            .as_q8(),
            0xFFFFFF
        );
    }

    #[test]
    fn reply_samples_per_bit_is_16_and_reply_sample_cycles_is_2() {
        for r in BOTH {
            assert_eq!(r.reply_samples_per_bit(), 16);
            assert_eq!(r.reply_sample_cycles(), 2);
            // The reply bit is 4/5 of a command bit.
            assert_eq!(
                r.reply_samples_per_bit() * r.reply_sample_cycles() * 5,
                r.cycles_per_bit() * 4
            );
        }
    }

    #[test]
    fn capture_window_ns_is_the_measured_89_5_and_60_4_us() {
        assert_eq!(Rate::Dshot300.capture_window_ns(), 89_500);
        assert_eq!(Rate::Dshot600.capture_window_ns(), 60_400);
    }

    #[test]
    fn capture_words_cover_the_window_with_less_than_one_word_extra() {
        assert_eq!(Rate::Dshot300.capture_words(), 68);
        assert_eq!(Rate::Dshot600.capture_words(), 91);
        for r in BOTH {
            // A word is 16 PIO cycles, in nanoseconds at the PIO clock.
            let word_ns = 16.0e9 / f64::from(r.pio_clock_hz());
            let covered = f64::from(r.capture_words()) * word_ns;
            let window = f64::from(r.capture_window_ns());
            assert!(covered >= window, "{r:?}");
            assert!(covered - window < word_ns, "{r:?}");
        }
    }

    proptest! {
        #[test]
        fn clock_divider_accepted_dividers_are_within_half_a_percent(clk in any::<u32>()) {
            for r in BOTH {
                match r.clock_divider(clk) {
                    Ok(d) => {
                        prop_assert!(d.integer >= 1);
                        prop_assert!(rate_error(r, clk, d) <= 0.005, "{:?} {}", r, clk);
                    }
                    Err(DividerError::ClockTooSlow) => {
                        // The nearest allowed divider (1.0) misses by more than 0.5 %.
                        prop_assert!(f64::from(clk) < f64::from(r.pio_clock_hz()) * 0.996);
                    }
                }
            }
        }

        #[test]
        fn clock_divider_rounds_to_the_nearest_256th(clk in 12_000_000u32..=400_000_000) {
            for r in BOTH {
                if let Ok(d) = r.clock_divider(clk) {
                    let exact = f64::from(clk) / f64::from(r.pio_clock_hz());
                    let got = f64::from(d.as_q8()) / 256.0;
                    prop_assert!((exact - got).abs() <= 0.5 / 256.0 + 1e-12);
                }
            }
        }
    }
}
