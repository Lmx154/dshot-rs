//! Bench program: the first that sends a throttle above zero. It spins ONE motor at a time at a
//! low, capped throttle and reports the eRPM each ESC replies (decision D4, "Motor phase").
//!
//! Built only with the feature `motor-test` and two build-time variables:
//! `DSHOT_SPIN_CAP` (the throttle cap, 1..=200 on the 1..=2000 scale) and `DSHOT_SPIN_ERPM_MAX`
//! (the eRPM above which the run stops). Both are set by the owner; neither has a default.
//!
//! Guards, all in this file (the driver holds no policy):
//! - G3: every throttle is built by `capped`, and `Bench::exchange` refuses a level above the cap.
//! - G4: `Bench::exchange` refuses a throttle frame that would make its burst longer than 3 s or
//!   the throttle phase longer than 60 s, or that would start a burst less than 1 s after the
//!   last frame that was not Stop on all four lines.
//! - G5: before the first throttle frame, P1 (stopped-motor replies, every line past its arming
//!   tune), P2 (command 13 on one line at a time, its acknowledge on that line only) and the
//!   battery on ADC2 in range; otherwise no throttle is sent.
//! - G6: from the first throttle frame on, 10 missing replies in a row on any line, or 10 eRPM
//!   replies in a row above `DSHOT_SPIN_ERPM_MAX` on any line, stop the run. A reading above
//!   1 000 000 eRPM is not a speed: on its first start after a reset AM32 2.20 reports a period
//!   of 2 us (30 000 000 eRPM) until its first commutation (`main.c:1840` with
//!   `commutation_intervals` still zero and `running` set at `main.c:1165`). Such readings are
//!   counted apart as "not started yet", and 300 of them in a row on a line (300 ms) stop the run.
//!
//! Every path after `start()` ends with Stop frames on all four lines for 1 s or more, the last
//! eRPM per line, and `release()`. Frames go out at 1 kHz from `start()` to the end; the log
//! lines between frames are one per frame. Touches PIO2, DMA channel 1, GPIO36-39, the ADC and
//! GPIO42.
#![no_std]
#![no_main]
#![allow(clippy::needless_range_loop)]

use adc_battery::{AdcConfig, Reading, VoltageInput};
use defmt_rtt as _;
use dshot::{
    Command, Erpm, ExtendedTelemetry, LINES, LineCapture, LineFrame, Parts, PioDshot, Rate, Reply,
    Throttle, Value,
};
use embassy_executor::Spawner;
use embassy_rp::adc::{self, Adc, Channel};
use embassy_rp::gpio::Pull;
use embassy_rp::peripherals::{DMA_CH1, PIO2};
use embassy_rp::pio::{self, Pio};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Delay, Duration, Instant, Timer};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    PIO2_IRQ_0 => pio::InterruptHandler<PIO2>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>;
    ADC_IRQ_FIFO => adc::InterruptHandler;
});

macro_rules! check {
    ($fails:expr, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {
        if $cond {
            defmt::info!($fmt, "PASS" $(, $arg)*);
        } else {
            $fails += 1;
            defmt::error!($fmt, "FAIL" $(, $arg)*);
        }
    };
}

// ---------------------------------------------------------------------------------------------
// Build-time limits (G3, G6).

/// A decimal number from a build-time variable; anything else is a compile error.
const fn decimal(s: &str) -> u32 {
    let b = s.as_bytes();
    assert!(!b.is_empty(), "a DSHOT_SPIN_* build variable is empty");
    let mut v: u32 = 0;
    let mut i = 0;
    while i < b.len() {
        assert!(
            b[i].is_ascii_digit(),
            "a DSHOT_SPIN_* build variable is not a decimal number"
        );
        assert!(
            v <= 100_000_000,
            "a DSHOT_SPIN_* build variable is too large"
        );
        v = v * 10 + (b[i] - b'0') as u32;
        i += 1;
    }
    v
}

const CAP_SET: u32 = decimal(env!(
    "DSHOT_SPIN_CAP",
    "DSHOT_SPIN_CAP is not set: the owner sets the throttle cap at build time (1..=200 on the 1..=2000 scale, for example DSHOT_SPIN_CAP=100)"
));
const _: () = assert!(
    CAP_SET <= 200,
    "DSHOT_SPIN_CAP is above 200 (10 % throttle): refused"
);
const _: () = assert!(
    CAP_SET >= 1,
    "DSHOT_SPIN_CAP is 0: a throttle level is 1..=2000"
);
/// G3: the highest throttle level this program may send.
const CAP: u16 = CAP_SET as u16;

/// G6: an eRPM reply above this stops the run.
const ERPM_MAX: u32 = decimal(env!(
    "DSHOT_SPIN_ERPM_MAX",
    "DSHOT_SPIN_ERPM_MAX is not set: the owner sets, at build time, the eRPM above which the run stops (no default)"
));

// ---------------------------------------------------------------------------------------------
// Lines, rate, timing.

const RATE: Rate = Rate::Dshot300;
/// Line 0 is GPIO36 (as `dshot_smoke` names them).
const NAMES: [&str; LINES] = ["M4/G36", "M3/G37", "M2/G38", "M1/G39"];
/// M1, M2, M3, M4.
const SPIN_ORDER: [usize; LINES] = [3, 2, 1, 0];
const STOP4: [LineFrame; LINES] = [LineFrame::STOP; LINES];

/// Frame period: 1 kHz.
const PERIOD: Duration = Duration::from_micros(1000);
/// Never two frames closer than this (when catching up after a late frame).
const MIN_SPACING: Duration = Duration::from_micros(500);
/// A push-to-push interval over this is counted as late in the report.
const LATE_US: u64 = 1100;

/// P1: every line past its arming tune within this time of `start()` returning.
const ARMING_MAX: Duration = Duration::from_millis(10_000);
/// P2: frames of command 13 on the line under test (six or more identical frames in a row).
const CMD_FRAMES: u32 = 10;
const _: () = assert!(CMD_FRAMES >= Command::ExtendedTelemetryOn.repeats() as u32);
/// P2: frames watched per line, from the first command frame.
const P2_FRAMES: u32 = 200;
/// ADC2: frames during which the battery input is read, and conversions after each.
const ADC_FRAMES: u32 = 64;
const ADC_PER_FRAME: u32 = 16;
/// ADC2: the battery range of `adc-battery/hil/src/bin/adc_battery_smoke.rs` (`BAT_RANGE_MV`).
const BAT_RANGE_MV: core::ops::RangeInclusive<u32> = 9_000..=26_000;

/// Throttle steps per motor: cap/3, 2*cap/3, cap.
const STEPS: u16 = 3;
/// Frames per burst (2 s at 1 kHz); G4 refuses anything past `BURST_MAX_US`.
const BURST_FRAMES: u32 = 2000;
/// eRPM statistics over the last half of a burst.
const TAIL: usize = (BURST_FRAMES / 2) as usize;
/// G4: no burst longer than 3 s, the throttle phase no longer than 60 s, 1 s or more of Stop on
/// all four lines before a burst.
const BURST_MAX_US: u64 = 3_000_000;
const PHASE_MAX_US: u64 = 60_000_000;
const GAP_MIN_US: u64 = 1_000_000;
/// The Stop between bursts and at the end goes on past 1 s until every line's latest eRPM reply
/// is "stopped", at most this long.
const GAP_MAX_US: u64 = 2_500_000;
/// Time allowed between a limit check and the push it guards.
const PUSH_MARGIN_US: u64 = 1000;
/// G6: missing replies in a row that stop the run.
const MISSING_MAX: u32 = 10;
/// G6: eRPM replies in a row above the limit that stop the run.
const ERPM_OVER_MAX: u32 = 10;
/// Above this an eRPM reply is not a speed (143 000 rpm on 14 magnets): the ESC has been told to
/// run and has not commutated yet.
const UNSTARTED_ERPM: u32 = 1_000_000;
/// G6: "not started yet" replies in a row that stop the run (300 ms at 1 kHz).
const UNSTARTED_MAX: u32 = 300;
/// Exchanges per line kept for the report of an automatic stop.
const HISTORY: usize = 20;

// ---------------------------------------------------------------------------------------------
// The guards as plain functions.

/// G3: the one place a throttle is built. A level above the cap (or outside 1..=2000) is
/// refused, and the refusal ends the run with Stop frames.
fn capped(line: usize, level: u16) -> Result<Throttle, Halt> {
    if !within_cap(level) {
        return Err(Halt::Cap { line, level });
    }
    Throttle::new(level).ok_or(Halt::Cap { line, level })
}

/// G3: whether a level is at or below the cap.
const fn within_cap(level: u16) -> bool {
    level <= CAP
}

/// G4: whether a frame pushed `elapsed_us` after the start of its burst (or of the throttle
/// phase) stays within `limit_us`, with the margin for the push.
const fn within_limit(elapsed_us: u64, limit_us: u64) -> bool {
    elapsed_us.saturating_add(PUSH_MARGIN_US) <= limit_us
}

/// G6: missing replies in a row after this capture. A reply of either kind (eRPM or extended
/// telemetry) is present; `Silent`, `Invalid` and `Foreign` are missing.
const fn missing_after(count: u32, capture: &LineCapture) -> u32 {
    match capture {
        LineCapture::Reply(_) => 0,
        _ => count.saturating_add(1),
    }
}

/// Throttle level of step `k` (0-based): cap/3, 2*cap/3, cap, each at least 1.
const fn step_level(cap: u16, k: u16) -> u16 {
    let v = cap * (k + 1) / STEPS;
    if v == 0 { 1 } else { v }
}

fn stopped(c: &LineCapture) -> bool {
    matches!(c, LineCapture::Reply(Reply::Erpm(e)) if e.is_stopped())
}

fn us(later: Instant, earlier: Instant) -> u64 {
    later.saturating_duration_since(earlier).as_micros()
}

// ---------------------------------------------------------------------------------------------

/// Why the run stopped by itself.
#[derive(Clone, Copy)]
enum Halt {
    /// G3: a throttle level above the cap.
    Cap { line: usize, level: u16 },
    /// G4: the burst would pass 3 s.
    Burst { line: usize, ms: u64 },
    /// G4: the throttle phase would pass 60 s.
    Phase { line: usize, ms: u64 },
    /// G4: a burst would start less than 1 s after the last frame that was not all Stop.
    Gap { line: usize, ms: u64 },
    /// G6: 10 missing replies in a row.
    Missing { line: usize },
    /// G6: an eRPM above the limit.
    Erpm { line: usize, erpm: u32 },
    /// G6: the ESC reported "not started yet" for 300 ms.
    NotStarted { line: usize },
}

impl Halt {
    fn line(&self) -> usize {
        match *self {
            Halt::Cap { line, .. }
            | Halt::Burst { line, .. }
            | Halt::Phase { line, .. }
            | Halt::Gap { line, .. }
            | Halt::Missing { line }
            | Halt::NotStarted { line }
            | Halt::Erpm { line, .. } => line,
        }
    }

    fn report(&self) {
        let name = NAMES[self.line()];
        match *self {
            Halt::Cap { level, .. } => defmt::error!(
                "[FAIL] automatic stop (G3) on {=str}: throttle level {=u16} refused, the cap is {=u16}",
                name,
                level,
                CAP
            ),
            Halt::Burst { ms, .. } => defmt::error!(
                "[FAIL] automatic stop (G4) on {=str}: the burst would pass 3000 ms ({=u64} ms since its first throttle frame)",
                name,
                ms
            ),
            Halt::Phase { ms, .. } => defmt::error!(
                "[FAIL] automatic stop (G4) on {=str}: the throttle phase would pass 60000 ms ({=u64} ms since the first throttle frame)",
                name,
                ms
            ),
            Halt::Gap { ms, .. } => defmt::error!(
                "[FAIL] automatic stop (G4) on {=str}: a burst would start {=u64} ms after the last frame that was not Stop on all four lines (1000 ms or more required)",
                name,
                ms
            ),
            Halt::Missing { .. } => defmt::error!(
                "[FAIL] automatic stop (G6) on {=str}: {=u32} missing replies in a row (Silent, Invalid or Foreign) during the throttle phase",
                name,
                MISSING_MAX
            ),
            Halt::NotStarted { .. } => defmt::error!(
                "[FAIL] automatic stop (G6) on {=str}: {=u32} replies in a row above {=u32} eRPM (the ESC was told to run and had not commutated): the motor did not start",
                name,
                UNSTARTED_MAX,
                UNSTARTED_ERPM
            ),
            Halt::Erpm { erpm, .. } => defmt::error!(
                "[FAIL] automatic stop (G6) on {=str}: {=u32} eRPM replies in a row above the limit {=u32}, the latest {=u32}",
                name,
                ERPM_OVER_MAX,
                ERPM_MAX,
                erpm
            ),
        }
    }
}

#[derive(Clone, Copy)]
struct Entry {
    frame: u32,
    sent: Value,
    got: LineCapture,
}

/// The last `HISTORY` exchanges of one line.
#[derive(Clone, Copy)]
struct History {
    e: [Entry; HISTORY],
    next: usize,
    len: usize,
}

impl History {
    const NEW: Self = Self {
        e: [Entry {
            frame: 0,
            sent: Value::Stop,
            got: LineCapture::Silent,
        }; HISTORY],
        next: 0,
        len: 0,
    };

    fn push(&mut self, entry: Entry) {
        if let Some(slot) = self.e.get_mut(self.next) {
            *slot = entry;
        }
        self.next = (self.next + 1) % HISTORY;
        self.len = (self.len + 1).min(HISTORY);
    }

    /// The `k`-th oldest entry kept.
    fn get(&self, k: usize) -> Option<Entry> {
        if k >= self.len {
            return None;
        }
        self.e
            .get((self.next + HISTORY - self.len + k) % HISTORY)
            .copied()
    }
}

/// The driver, the 1 kHz cadence and the guards. Every frame of this program goes through
/// `Bench::exchange`.
struct Bench {
    drv: PioDshot<'static, PIO2>,
    fails: u32,
    /// When the next frame is due.
    next: Instant,
    last_push: Option<Instant>,
    /// Frames since `start()`.
    frames: u32,
    /// Longest push-to-push interval since `start()`, and how many were over `LATE_US`.
    max_period_us: u64,
    late: u32,
    hist: [History; LINES],
    /// Each line's latest eRPM reply (extended telemetry replies leave it unchanged).
    last_erpm: [Option<Erpm>; LINES],
    /// The latest frame that was not Stop on all four lines.
    last_not_stop: Option<Instant>,
    /// The first throttle frame of the run.
    phase_start: Option<Instant>,
    /// The first throttle frame of the current burst.
    burst_start: Instant,
    /// The latest frame carried a throttle.
    in_burst: bool,
    /// G6 counts from the first throttle frame until the ending.
    guarded: bool,
    missing: [u32; LINES],
    /// eRPM replies in a row above the limit, per line (other replies leave the count as it is).
    over: [u32; LINES],
    /// "Not started yet" replies in a row, per line, and the longest such run so far.
    unstarted: [u32; LINES],
    unstarted_longest: [u32; LINES],
}

impl Bench {
    fn new(drv: PioDshot<'static, PIO2>) -> Self {
        Self {
            drv,
            fails: 0,
            next: Instant::now(),
            last_push: None,
            frames: 0,
            max_period_us: 0,
            late: 0,
            hist: [History::NEW; LINES],
            last_erpm: [None; LINES],
            last_not_stop: None,
            phase_start: None,
            burst_start: Instant::now(),
            in_burst: false,
            guarded: false,
            missing: [0; LINES],
            over: [0; LINES],
            unstarted: [0; LINES],
            unstarted_longest: [0; LINES],
        }
    }

    /// One frame at the next 1 kHz slot, with G3 and G4 checked before the push and G6 after it.
    /// A refused frame is not sent.
    async fn exchange(&mut self, frames: [LineFrame; LINES]) -> Result<dshot::Exchange, Halt> {
        Timer::at(self.next).await;
        let now = Instant::now();
        let throttle_line = frames
            .iter()
            .position(|f| matches!(f.value, Value::Throttle(_)));
        if let Some(line) = throttle_line {
            for f in &frames {
                if let Value::Throttle(t) = f.value
                    && !within_cap(t.level())
                {
                    return Err(Halt::Cap {
                        line,
                        level: t.level(),
                    });
                }
            }
            if !self.in_burst {
                if let Some(t) = self.last_not_stop {
                    let gap = us(now, t);
                    if gap < GAP_MIN_US {
                        return Err(Halt::Gap {
                            line,
                            ms: gap / 1000,
                        });
                    }
                }
                self.burst_start = now;
                if self.phase_start.is_none() {
                    self.phase_start = Some(now);
                    self.guarded = true;
                }
            }
            let burst = us(now, self.burst_start);
            if !within_limit(burst, BURST_MAX_US) {
                return Err(Halt::Burst {
                    line,
                    ms: burst / 1000,
                });
            }
            let phase = self.phase_start.map_or(0, |t| us(now, t));
            if !within_limit(phase, PHASE_MAX_US) {
                return Err(Halt::Phase {
                    line,
                    ms: phase / 1000,
                });
            }
        }

        let x = self.drv.exchange(frames).await;

        if let Some(p) = self.last_push {
            let period = us(now, p);
            self.max_period_us = self.max_period_us.max(period);
            if period > LATE_US {
                self.late += 1;
            }
        }
        self.last_push = Some(now);
        self.next = (self.next + PERIOD).max(now + MIN_SPACING);
        self.frames += 1;
        self.in_burst = throttle_line.is_some();
        if frames.iter().any(|f| f.value != Value::Stop) {
            self.last_not_stop = Some(now);
        }
        let mut halt = None;
        for l in 0..LINES {
            let got = x.captures[l];
            self.hist[l].push(Entry {
                frame: self.frames,
                sent: frames[l].value,
                got,
            });
            if let LineCapture::Reply(Reply::Erpm(e)) = got {
                self.last_erpm[l] = Some(e);
                if self.guarded && e.erpm() > UNSTARTED_ERPM {
                    self.unstarted[l] = self.unstarted[l].saturating_add(1);
                    self.unstarted_longest[l] = self.unstarted_longest[l].max(self.unstarted[l]);
                    if self.unstarted[l] >= UNSTARTED_MAX && halt.is_none() {
                        halt = Some(Halt::NotStarted { line: l });
                    }
                } else if self.guarded {
                    self.unstarted[l] = 0;
                    self.over[l] = if e.erpm() > ERPM_MAX {
                        self.over[l].saturating_add(1)
                    } else {
                        0
                    };
                    if self.over[l] >= ERPM_OVER_MAX && halt.is_none() {
                        halt = Some(Halt::Erpm {
                            line: l,
                            erpm: e.erpm(),
                        });
                    }
                }
            }
            if self.guarded {
                self.missing[l] = missing_after(self.missing[l], &got);
                if self.missing[l] >= MISSING_MAX && halt.is_none() {
                    halt = Some(Halt::Missing { line: l });
                }
            }
        }
        match halt {
            Some(h) => Err(h),
            None => Ok(x),
        }
    }

    /// Microseconds since the latest frame that was not Stop on all four lines.
    fn since_not_stop_us(&self) -> u64 {
        self.last_not_stop
            .map_or(u64::MAX, |t| us(Instant::now(), t))
    }

    fn all_stopped(&self) -> bool {
        self.last_erpm
            .iter()
            .all(|e| e.is_some_and(|e| e.is_stopped()))
    }

    /// One Stop frame in the ending (G6 is off there and a Stop frame is never refused).
    async fn stop(&mut self) {
        if let Err(h) = self.exchange(STOP4).await {
            h.report();
        }
    }

    /// The ending, on every path after `start()`: Stop on all four lines for 1 s or more (on
    /// until every line's latest eRPM reply is "stopped", 2.5 s at most), the report of an
    /// automatic stop one line per frame, the last eRPM per line, then `release()`. Returns
    /// the number of failed checks.
    async fn finish(mut self, halt: Option<(Halt, [History; LINES])>) -> u32 {
        self.guarded = false;
        self.stop().await;
        let t0 = self.last_push.unwrap_or_else(Instant::now);
        let mut said = 0usize;
        let el = loop {
            if let Some((h, hist)) = &halt {
                if said == 0 {
                    h.report();
                } else if let Some(e) = hist[h.line()].get(said - 1) {
                    defmt::info!(
                        "[INFO] {=str} exchange {=u32} from start(): sent {}, got {}",
                        NAMES[h.line()],
                        e.frame,
                        e.sent,
                        e.got
                    );
                }
                said += 1;
            }
            self.stop().await;
            let el = self.last_push.map_or(0, |t| us(t, t0));
            let pending = halt.is_some() && said <= HISTORY;
            if el >= GAP_MIN_US && !pending && (self.all_stopped() || el >= GAP_MAX_US) {
                break el;
            }
        };
        defmt::info!(
            "[STAT] ending: Stop frames on all four lines for {=u64} ms (first to latest)",
            el / 1000
        );
        self.stop().await;
        for l in 0..LINES {
            match self.last_erpm[l] {
                Some(e) if e.is_stopped() => {
                    defmt::info!("[INFO] last eRPM reply on {=str}: stopped", NAMES[l])
                }
                Some(e) => defmt::info!(
                    "[INFO] last eRPM reply on {=str}: {=u32} eRPM, NOT stopped",
                    NAMES[l],
                    e.erpm()
                ),
                None => defmt::info!("[INFO] last eRPM reply on {=str}: none", NAMES[l]),
            }
            self.stop().await;
        }
        let all = self.all_stopped();
        check!(
            self.fails,
            all,
            "[{=str}] after the final Stop every line's latest eRPM reply is stopped"
        );
        self.stop().await;
        defmt::info!(
            "[STAT] longest run of \"not started yet\" replies (above {=u32} eRPM) per line M4/G36, M3/G37, M2/G38, M1/G39, in frames: {}",
            UNSTARTED_ERPM,
            self.unstarted_longest
        );
        self.stop().await;
        defmt::info!(
            "[STAT] cadence: {=u32} frames since start(), longest push-to-push {=u64} us, {=u32} over {=u64} us",
            self.frames,
            self.max_period_us,
            self.late,
            LATE_US
        );
        self.stop().await;
        let Bench { drv, mut fails, .. } = self;
        match drv.release() {
            Ok(_parts) => defmt::info!("[PASS] release() accepted after the Stop frames"),
            Err(drv) => {
                fails += 1;
                defmt::error!("[FAIL] release() refused after the Stop frames");
                drop(drv);
            }
        }
        fails
    }
}

// ---------------------------------------------------------------------------------------------
// G5.

/// Stop frames until every line is past its arming tune, as `dshot_smoke`'s `past_arming`: after
/// its first stopped-motor reply, a block of 250 or more exchanges without one, then stopped
/// replies unbroken for 1500 ms. Returns per line the ms from the call (0: not within 10 s).
async fn past_arming(b: &mut Bench) -> Result<[u32; LINES], Halt> {
    let t0 = Instant::now();
    let (mut live, mut blocked, mut nongood) = ([false; LINES], [false; LINES], [0u32; LINES]);
    let mut since = [None::<Instant>; LINES];
    let mut done = [0u32; LINES];
    while done.contains(&0) && t0.elapsed() < ARMING_MAX {
        let x = b.exchange(STOP4).await?;
        let now = Instant::now();
        for l in 0..LINES {
            if done[l] != 0 {
                continue;
            }
            if stopped(&x.captures[l]) {
                blocked[l] |= live[l] && nongood[l] >= 250;
                live[l] = true;
                nongood[l] = 0;
                if blocked[l] && now - *since[l].get_or_insert(now) >= Duration::from_millis(1500) {
                    done[l] = t0.elapsed().as_millis() as u32;
                }
            } else {
                nongood[l] += u32::from(live[l]);
                since[l] = None;
            }
        }
    }
    Ok(done)
}

/// P2 on one line: command 13 in `CMD_FRAMES` identical frames on that line, Stop on the others,
/// then Stop on all four; the acknowledges 0xE00 per line in the `P2_FRAMES` frames.
async fn p2_line(b: &mut Bench, line: usize) -> Result<[u32; LINES], Halt> {
    let cmd = LineFrame::new(Value::Command(Command::ExtendedTelemetryOn), false);
    let mut frames = STOP4;
    let mut acks = [0u32; LINES];
    for k in 0..P2_FRAMES {
        frames[line] = if k < CMD_FRAMES { cmd } else { LineFrame::STOP };
        let x = b.exchange(frames).await?;
        for (n, c) in acks.iter_mut().zip(&x.captures) {
            if matches!(
                c,
                LineCapture::Reply(Reply::Extended(ExtendedTelemetry::EnabledAck))
            ) {
                *n += 1;
            }
        }
    }
    Ok(acks)
}

/// G5: P1, P2 on each line in turn, ADC2. Returns whether all passed.
async fn preflight(
    b: &mut Bench,
    adc: &mut Adc<'_, adc::Async>,
    vbat: &mut Channel<'_>,
) -> Result<bool, Halt> {
    let done = past_arming(b).await?;
    let p1 = !done.contains(&0);
    check!(
        b.fails,
        p1,
        "[{=str}] G5 P1: every line gave stopped-motor replies and is past its arming tune, at ms {=[?]} after start() returned (lines M4/G36, M3/G37, M2/G38, M1/G39; 0: not within 10 s)",
        &done[..]
    );
    b.exchange(STOP4).await?;
    if !p1 {
        return Ok(false);
    }

    let mut p2 = true;
    for &line in &SPIN_ORDER {
        let acks = p2_line(b, line).await?;
        let ok = acks
            .iter()
            .enumerate()
            .all(|(l, &n)| if l == line { n >= 1 } else { n == 0 });
        check!(
            b.fails,
            ok,
            "[{=str}] G5 P2 on {=str}: command 13 in {=u32} identical frames on this line only, Stop on the others; acknowledges 0xE00 per line (M4/G36, M3/G37, M2/G38, M1/G39) in {=u32} frames: {=[?]}",
            NAMES[line],
            CMD_FRAMES,
            P2_FRAMES,
            &acks[..]
        );
        b.exchange(STOP4).await?;
        p2 &= ok;
    }
    if !p2 {
        return Ok(false);
    }

    let (mut sum, mut count, mut errors) = (0u32, 0u32, 0u32);
    for _ in 0..ADC_FRAMES {
        b.exchange(STOP4).await?;
        for _ in 0..ADC_PER_FRAME {
            match adc.read(vbat).await {
                Ok(v) => {
                    sum += u32::from(v);
                    count += 1;
                }
                Err(_) => errors += 1,
            }
        }
    }
    let cfg = AdcConfig::new();
    let (mv, full) = match Reading::from_sum(sum, count) {
        Some(r) => (
            VoltageInput::new(100_000, 10_000).millivolts(cfg.pin_microvolts(r)),
            cfg.is_at_full_scale(r),
        ),
        None => (0, false),
    };
    let bat = errors == 0 && count > 0 && !full && BAT_RANGE_MV.contains(&mv);
    check!(
        b.fails,
        bat,
        "[{=str}] G5 ADC2: battery {=u32} mV (GPIO42 through 100k/10k, {=u32} conversions over {=u32} frames, {=u32} ADC errors, at full scale {=bool}) is in 9000..=26000 mV",
        mv,
        count,
        ADC_FRAMES,
        errors,
        full
    );
    b.exchange(STOP4).await?;
    Ok(bat)
}

// ---------------------------------------------------------------------------------------------
// The spin.

#[derive(Clone, Copy)]
struct Span {
    min: u32,
    max: u32,
    n: u32,
}

impl Span {
    const NEW: Self = Self {
        min: u32::MAX,
        max: 0,
        n: 0,
    };

    fn add(&mut self, v: u32) {
        self.min = self.min.min(v);
        self.max = self.max.max(v);
        self.n += 1;
    }

    fn lo(&self) -> u32 {
        if self.n == 0 { 0 } else { self.min }
    }
}

/// What one step showed.
struct Step {
    frames: u32,
    ms: u64,
    erpm: u32,
    ext: u32,
    silent: u32,
    invalid: u32,
    foreign: u32,
    /// eRPM of the driven line over the last half of the burst, stopped as 0.
    tail: [u32; TAIL],
    tail_n: usize,
    volt: Span,
    amps: Span,
    temp: Span,
    /// The other lines: eRPM replies, and those not "stopped".
    others_erpm: [u32; LINES],
    others_moving: [u32; LINES],
}

impl Step {
    const NEW: Self = Self {
        frames: 0,
        ms: 0,
        erpm: 0,
        ext: 0,
        silent: 0,
        invalid: 0,
        foreign: 0,
        tail: [0; TAIL],
        tail_n: 0,
        volt: Span::NEW,
        amps: Span::NEW,
        temp: Span::NEW,
        others_erpm: [0; LINES],
        others_moving: [0; LINES],
    };

    fn record(&mut self, line: usize, tail: bool, captures: &[LineCapture; LINES]) {
        for (l, c) in captures.iter().enumerate() {
            if l == line {
                match c {
                    LineCapture::Reply(Reply::Erpm(e)) => {
                        self.erpm += 1;
                        if tail && let Some(slot) = self.tail.get_mut(self.tail_n) {
                            *slot = e.erpm();
                            self.tail_n += 1;
                        }
                    }
                    LineCapture::Reply(Reply::Extended(x)) => {
                        self.ext += 1;
                        match x {
                            ExtendedTelemetry::Voltage { millivolts } => self.volt.add(*millivolts),
                            ExtendedTelemetry::Current { milliamps } => self.amps.add(*milliamps),
                            ExtendedTelemetry::Temperature { celsius } => {
                                self.temp.add(u32::from(*celsius))
                            }
                            _ => {}
                        }
                    }
                    LineCapture::Silent => self.silent += 1,
                    LineCapture::Invalid(_) => self.invalid += 1,
                    LineCapture::Foreign => self.foreign += 1,
                }
            } else if let LineCapture::Reply(Reply::Erpm(e)) = c {
                self.others_erpm[l] += 1;
                if !e.is_stopped() {
                    self.others_moving[l] += 1;
                }
            }
        }
    }

    /// Min, median and max of the tail (all 0 when it is empty).
    fn tail_stats(&mut self) -> (u32, u32, u32) {
        let n = self.tail_n.min(TAIL);
        let Some(t) = self.tail.get_mut(..n) else {
            return (0, 0, 0);
        };
        t.sort_unstable();
        (
            t.first().copied().unwrap_or(0),
            t.get(n / 2).copied().unwrap_or(0),
            t.last().copied().unwrap_or(0),
        )
    }
}

/// One burst of `BURST_FRAMES` frames: the throttle on `line`, Stop on the others.
async fn burst(b: &mut Bench, line: usize, t: Throttle, s: &mut Step) -> Result<(), Halt> {
    let mut frames = STOP4;
    frames[line] = LineFrame::new(Value::Throttle(t), false);
    for k in 0..BURST_FRAMES {
        let x = b.exchange(frames).await?;
        s.record(line, k >= BURST_FRAMES / 2, &x.captures);
        s.frames += 1;
    }
    if let Some(last) = b.last_push {
        s.ms = us(last, b.burst_start) / 1000;
    }
    // One Stop before any log line: a log call must never hold back the frame that ends a burst.
    b.exchange(STOP4).await?;
    Ok(())
}

/// G4: Stop on all four lines for 1 s or more after the last frame that was not all Stop, on
/// until every line's latest eRPM reply is "stopped", 2.5 s at most.
async fn stop_gap(b: &mut Bench) -> Result<(), Halt> {
    loop {
        b.exchange(STOP4).await?;
        let el = b.since_not_stop_us();
        if el >= GAP_MIN_US && (b.all_stopped() || el >= GAP_MAX_US) {
            break;
        }
    }
    defmt::info!(
        "[STAT] Stop on all four lines for {=u64} ms after the last frame that was not all Stop; every line's latest eRPM reply stopped: {=bool}",
        b.since_not_stop_us() / 1000,
        b.all_stopped()
    );
    b.exchange(STOP4).await?;
    Ok(())
}

/// The report of one step, one line per Stop frame. Returns the median eRPM.
async fn report_step(
    b: &mut Bench,
    m: usize,
    line: usize,
    k: u16,
    level: u16,
    s: &mut Step,
    prev: Option<u32>,
) -> Result<u32, Halt> {
    let name = NAMES[line];
    let motor = m + 1;
    let step = k + 1;
    let (lo, med, hi) = s.tail_stats();
    defmt::info!(
        "[STAT] M{=usize} ({=str}) step {=u16} of {=u16}: throttle {=u16} of 2000 (cap {=u16}), {=u32} frames, {=u64} ms from the first throttle frame to the last",
        motor,
        name,
        step,
        STEPS,
        level,
        CAP,
        s.frames,
        s.ms
    );
    b.exchange(STOP4).await?;
    defmt::info!(
        "[STAT] M{=usize} step {=u16}: replies on {=str}: eRPM {=u32}, extended telemetry {=u32}, Silent {=u32}, Invalid {=u32}, Foreign {=u32}",
        motor,
        step,
        name,
        s.erpm,
        s.ext,
        s.silent,
        s.invalid,
        s.foreign
    );
    b.exchange(STOP4).await?;
    defmt::info!(
        "[STAT] M{=usize} step {=u16}: eRPM over the last {=usize} frames of the burst ({=usize} eRPM replies, stopped counted as 0): min {=u32} median {=u32} max {=u32}",
        motor,
        step,
        TAIL,
        s.tail_n,
        lo,
        med,
        hi
    );
    b.exchange(STOP4).await?;
    defmt::info!(
        "[STAT] M{=usize} step {=u16}: extended telemetry on {=str}: voltage {=u32} to {=u32} mV ({=u32} frames), current {=u32} to {=u32} mA ({=u32}), temperature {=u32} to {=u32} degC ({=u32})",
        motor,
        step,
        name,
        s.volt.lo(),
        s.volt.max,
        s.volt.n,
        s.amps.lo(),
        s.amps.max,
        s.amps.n,
        s.temp.lo(),
        s.temp.max,
        s.temp.n
    );
    b.exchange(STOP4).await?;
    check!(
        b.fails,
        med > 0,
        "[{=str}] M{=usize} step {=u16}: median eRPM {=u32} on {=str} is above 0",
        motor,
        step,
        med,
        name
    );
    b.exchange(STOP4).await?;
    let others = s.others_moving.iter().all(|&n| n == 0);
    check!(
        b.fails,
        others,
        "[{=str}] M{=usize} step {=u16}: the other three lines reported stopped in every eRPM reply (per line M4/G36, M3/G37, M2/G38, M1/G39, driven line 0: not stopped {=[?]} of {=[?]} eRPM replies)",
        motor,
        step,
        &s.others_moving[..],
        &s.others_erpm[..]
    );
    b.exchange(STOP4).await?;
    if let Some(p) = prev {
        if med > p {
            defmt::info!(
                "[INFO] M{=usize} step {=u16}: median eRPM rose from {=u32} to {=u32}",
                motor,
                step,
                p,
                med
            );
        } else {
            defmt::info!(
                "[INFO] M{=usize} step {=u16}: median eRPM did not rise ({=u32} after {=u32}); reported, not a failure",
                motor,
                step,
                med,
                p
            );
        }
        b.exchange(STOP4).await?;
    }
    Ok(med)
}

/// M1, M2, M3, M4, one at a time: three steps rising to the cap, each a burst and then Stop.
async fn spin(b: &mut Bench) -> Result<(), Halt> {
    for (m, &line) in SPIN_ORDER.iter().enumerate() {
        let name = NAMES[line];
        defmt::info!(
            "[INFO] ===== M{=usize} ({=str}): {=u16} steps up to the cap; Stop on the other three lines =====",
            m + 1,
            name,
            STEPS
        );
        b.exchange(STOP4).await?;
        defmt::info!(
            "[INFO] the program cannot see the direction of rotation: the person at the bench notes it for M{=usize} ({=str})",
            m + 1,
            name
        );
        b.exchange(STOP4).await?;
        let mut prev = None;
        for k in 0..STEPS {
            let level = step_level(CAP, k);
            let t = capped(line, level)?;
            let mut s = Step::NEW;
            burst(b, line, t, &mut s).await?;
            prev = Some(report_step(b, m, line, k, level, &mut s, prev).await?);
            stop_gap(b).await?;
        }
    }
    Ok(())
}

async fn run(
    b: &mut Bench,
    adc: &mut Adc<'_, adc::Async>,
    vbat: &mut Channel<'_>,
) -> Result<(), Halt> {
    if !preflight(b, adc, vbat).await? {
        defmt::info!("[INFO] G5 not met: no throttle frame is sent");
        b.exchange(STOP4).await?;
        return Ok(());
    }
    defmt::info!("[INFO] G5 met: P1, P2 on all four lines and ADC2 in this boot");
    b.exchange(STOP4).await?;
    stop_gap(b).await?;
    spin(b).await
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!(
        "[INFO] dshot_spin: throttle cap {=u16} of 2000 ({=u16}.{=u16} %, DSHOT_SPIN_CAP at build time); steps {=u16}, {=u16}, {=u16}",
        CAP,
        CAP / 20,
        CAP % 20 / 2,
        step_level(CAP, 0),
        step_level(CAP, 1),
        step_level(CAP, 2)
    );
    defmt::info!(
        "[INFO] git commit of this build: not known (the hil crate has no way to record it)"
    );
    defmt::info!(
        "[INFO] eRPM limit {=u32} (DSHOT_SPIN_ERPM_MAX at build time): from the first throttle frame on, {=u32} eRPM replies in a row above it, {=u32} missing replies in a row, or {=u32} \"not started yet\" replies in a row (above {=u32} eRPM), on any line, stop the run",
        ERPM_MAX,
        ERPM_OVER_MAX,
        MISSING_MAX,
        UNSTARTED_MAX,
        UNSTARTED_ERPM
    );
    defmt::info!(
        "[INFO] lines on PIO2 at DShot300: M1 = GPIO39, M2 = GPIO38, M3 = GPIO37, M4 = GPIO36; one motor at a time in that order"
    );
    defmt::info!(
        "[INFO] limits: bursts of {=u32} frames at 1 kHz, refused past {=u64} ms; {=u64} ms or more of Stop on all four lines before each burst; throttle phase refused past {=u64} ms",
        BURST_FRAMES,
        BURST_MAX_US / 1000,
        GAP_MIN_US / 1000,
        PHASE_MAX_US / 1000
    );
    defmt::info!(
        "[INFO] on a panic frames stop (panic-probe; the hil crate has no hook to send Stop first) and each ESC cuts its motor by itself about 0.5 s after the last frame"
    );

    let p = embassy_rp::init(Default::default());

    // ADC2 first: its pad comes out of reset with the pull-down on, and the 100 nF needs time
    // to charge once that is off (adc-battery README, rule 6: 150 ms is the wait it tested).
    let mut adc = Adc::new(p.ADC, Irqs, adc::Config::default());
    let mut vbat = Channel::new_pin(p.PIN_42, Pull::None);
    Timer::after_millis(150).await;

    let Pio {
        common: mut c2,
        sm0,
        ..
    } = Pio::new(p.PIO2, Irqs);
    let pins = [
        c2.make_pio_pin(p.PIN_36),
        c2.make_pio_pin(p.PIN_37),
        c2.make_pio_pin(p.PIN_38),
        c2.make_pio_pin(p.PIN_39),
    ];
    let dma = dma::Channel::new(p.DMA_CH1, Irqs);
    let parts = Parts {
        common: c2,
        sm0,
        pins,
        dma,
    };
    let drv = match PioDshot::new(parts, RATE, embassy_rp::clocks::clk_sys_freq()) {
        Ok(d) => d,
        Err(e) => {
            defmt::error!("[FAIL] PioDshot::new: {}; no frame was sent", e.reason);
            hil_common::exit(1)
        }
    };
    let mut b = Bench::new(drv);
    b.drv.start(&mut Delay).await;
    b.next = Instant::now();

    let halt = match run(&mut b, &mut adc, &mut vbat).await {
        Ok(()) => None,
        Err(h) => {
            b.fails += 1;
            Some((h, b.hist))
        }
    };
    let fails = b.finish(halt).await;
    hil_common::exit(fails)
}
