//! On-board test of `dshot::PioDshot` on the four AM32 controllers (GPIO36-39, PIO2), at
//! DShot300 and DShot600, with an independent witness of the lines: four run-length observers on
//! PIO0 (13.3 ns resolution, DMA rings on channels 4-7), as in `dshot_m8_startup`.
//!
//! SAFETY OF THE BENCH: every frame this program sends is `LineFrame::STOP` on all four lines;
//! it builds no `Throttle` and no `Command`. It touches only PIO0, PIO2, DMA channels 1 and 4-7
//! and GPIO36-39.
//!
//! Checks, per rate (numbering of the packet):
//! 1. `start()`, then STOP at 1 kHz: every line Live within 2 s.
//! 2. 3000 exchanges after all lines are Live (seventh amendment): our frame strict on each; G and
//!    Q against the driver; anything else only in the line's transient zone at arming, 3 at most.
//!    Self-tests of the classifier run before any frame.
//! 3. `DBG_PADOE` only during frames and the break; the break 25 ms or more on all four together;
//!    addendum C: no run under 300 ns with both edges within 200 ns of our drive interval or of a
//!    break edge, no edge inside our drive interval more than 53 ns off our bit grid; other runs
//!    under 300 ns reported. Self-tests before any frame.
//! 6. Fifth amendment item 3, as the sixth amendment reads it. S1, warm: a fresh `start()`, STOP
//!    at 1 kHz until every line is past its arming, one exchange dropped mid-capture, `release()`
//!    refused, 1 ms, the next exchange returns its own frame's result, `release()` accepted. S2,
//!    parked: 3.5 s without frames, 100 ms with no edge, `release()` accepted, 100 build/release
//!    cycles with no edge, then `start()` shows its break and 100 consecutive good replies.
//!    Then the cancel sweep (design invariant 3.4): past arming again, at least 1000 exchanges
//!    dropped 1 us apart from before the push to after the capture window, `release()` after
//!    each; frames seen = pushes, the exchange after each cancel returns its own result. Then
//!    exchanges cancelled during the drain of a cancelled one, dropped 1 us apart.
#![no_std]
#![no_main]
#![allow(clippy::needless_range_loop)]

use core::future::poll_fn;
use core::pin::pin;
use core::task::Poll;

use defmt_rtt as _;
use dshot::{
    Exchange, ExtendedTelemetry, LineCapture, LineFrame, LineState, Parts, PioDshot, Rate, Reply,
    decode_reply, program,
};
use embassy_executor::Spawner;
use embassy_rp::peripherals::{DMA_CH1, PIO0, PIO2};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{self, Config, FifoJoin, Pio, ShiftConfig, ShiftDirection, StateMachine};
use embassy_rp::{bind_interrupts, dma, pac};
use embassy_time::{Delay, Duration, Instant, Timer};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    PIO2_IRQ_0 => pio::InterruptHandler<PIO2>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>;
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

const CLK_HZ: u32 = 150_000_000;
const LINES: usize = 4;
/// GPIO36 is PIO pin 20 with GPIOBASE 16.
const OE_MASK: u32 = 0xF << 20;
const NAMES: [&str; LINES] = ["M4/G36", "M3/G37", "M2/G38", "M1/G39"];
const STOP4: [LineFrame; LINES] = [LineFrame::STOP; LINES];
/// A run shorter than this (45 system cycles, 300 ns) is a spike or a glitch: our shortest run is
/// the 417 ns high of a 1 bit at DShot600 (10 PIO cycles), a reply symbol is 1.28 us or more.
const SPIKE_SYS: u64 = 45;

// ---------------------------------------------------------------------------------------------
// The observers (PIO0 state machines 0-3, one per line), as in dshot_m8_startup.

const RING_N: usize = 1024;
#[repr(C, align(4096))]
struct RingMem([u32; RING_N]);
static mut RINGS: [RingMem; LINES] = [const { RingMem([0; RING_N]) }; LINES];
const DMA_FIRST: usize = 4;
const EDGE_CAP: usize = 512;

fn ring_base(i: usize) -> usize {
    (&raw mut RINGS as *mut RingMem as usize) + i * core::mem::size_of::<RingMem>()
}

#[derive(Clone, Copy)]
struct Line {
    /// Time of the last edge, system cycles since the observers started.
    t: u64,
    high: bool,
    skip: bool,
    /// Edges of the current window: time << 1 | rising.
    e: [u64; EDGE_CAP],
    n: usize,
    over: u32,
    total: u32,
    /// Level of the line and time of its last edge when the window was last emptied (`mark`).
    h0: bool,
    t_h0: u64,
}

impl Line {
    const NEW: Line = Line {
        t: 0,
        high: true,
        skip: true,
        e: [0; EDGE_CAP],
        n: 0,
        over: 0,
        total: 0,
        h0: true,
        t_h0: 0,
    };
}

struct Obs {
    /// CPU time at which the observers started (their time 0).
    t0: Instant,
    l: [Line; LINES],
    qh: [u32; LINES],
    qt: [u32; LINES],
    ring_over: u32,
    spike_check: bool,
    spikes: u32,
    min_run: u64,
    /// Addendum C: runs under 300 ns not judged yet; our drive intervals of the last exchanges
    /// (first falling edge, bit time in quarter cycles); the break edges of the last `start()`.
    shorts: [Short; 32],
    shorts_n: usize,
    shorts_lost: u32,
    drive: [(u64, i64); 4],
    drive_i: usize,
    drive_n: usize,
    breaks: [u64; 2 * LINES],
    breaks_n: usize,
    sp_ours: u32,
    sp_rep: u32,
    off_grid: u32,
    /// Seventh amendment item 4, in check 2: runs under 300 ns at our release held until their
    /// exchange is classified, with the first falling edge of its frame.
    defer_rel: bool,
    held: [(Short, u64); 8],
    held_n: usize,
    /// For the records: the check, the exchange, the driver's captures.
    stage: &'static str,
    xk: u32,
    caps: Option<[LineCapture; LINES]>,
}

impl Obs {
    fn sync(&mut self) {
        for i in 0..LINES {
            let wp = pac::DMA.ch(DMA_FIRST + i).write_addr().read() as usize;
            let idx = ((wp - ring_base(i)) >> 2) as u32 & (RING_N as u32 - 1);
            let delta = idx.wrapping_sub(self.qh[i]) & (RING_N as u32 - 1);
            self.qh[i] = self.qh[i].wrapping_add(delta);
            if self.qh[i].wrapping_sub(self.qt[i]) > RING_N as u32 {
                self.ring_over += 1;
                self.qt[i] = self.qh[i].wrapping_sub(RING_N as u32);
            }
        }
        for i in 0..LINES {
            while self.qt[i] != self.qh[i] {
                let k = self.qt[i] as usize & (RING_N - 1);
                // SAFETY: the index is inside the ring; the DMA channel only writes it.
                let w = unsafe { ((ring_base(i) as *const u32).add(k)).read_volatile() };
                self.qt[i] = self.qt[i].wrapping_add(1);
                self.word(i, w);
            }
        }
    }

    fn word(&mut self, i: usize, w: u32) {
        let ln = &mut self.l[i];
        if ln.skip {
            ln.skip = false;
            return;
        }
        let was_high = ln.high;
        let run = if was_high { 4 } else { 6 } + 2 * u64::from(w);
        ln.t += run;
        ln.high = !was_high;
        ln.total += 1;
        if self.spike_check && run < SPIKE_SYS {
            self.spikes += 1;
            if self.shorts_n < self.shorts.len() {
                self.shorts[self.shorts_n] = Short {
                    l: i as u8,
                    a: ln.t - run,
                    b: ln.t,
                    high: was_high,
                };
                self.shorts_n += 1;
            } else {
                self.shorts_lost += 1;
            }
        }
        if self.spike_check {
            self.min_run = self.min_run.min(run);
        }
        if ln.n < EDGE_CAP {
            ln.e[ln.n] = (ln.t << 1) | u64::from(!was_high);
            ln.n += 1;
        } else {
            ln.over += 1;
        }
    }

    /// Processes what has arrived and empties the windows.
    fn mark(&mut self) {
        self.sync();
        self.settle();
        for ln in &mut self.l {
            ln.n = 0;
            ln.h0 = ln.high;
            ln.t_h0 = ln.t;
        }
    }

    fn edges_in_window(&self) -> u32 {
        self.l.iter().map(|l| l.n as u32).sum()
    }

    /// Now, in observer time (system cycles), from the CPU clock.
    fn now(&self) -> u64 {
        self.t0.elapsed().as_micros() * 150
    }

    /// Our frame of one exchange (its 32 edges on the anchor line): its drive interval for
    /// addendum C; C's grid test on every line's edges inside it.
    fn register(&mut self, a: &[u64; 32], rate: Rate) {
        let (t0, b4) = (a[0] >> 1, bit4(rate));
        self.drive[self.drive_i] = (t0, b4);
        self.drive_i = (self.drive_i + 1) % self.drive.len();
        self.drive_n = (self.drive_n + 1).min(self.drive.len());
        if !self.spike_check {
            return;
        }
        let rel = t0 + 4 * b4 as u64;
        for l in 0..LINES {
            for k in 0..self.l[l].n {
                let x = self.l[l].e[k];
                if (t0..=rel).contains(&(x >> 1)) && !on_grid(x >> 1, t0, b4) {
                    self.off_grid += 1;
                    self.record(
                        "an edge inside our drive interval more than 53 ns off our bit grid",
                        l,
                        x,
                        x,
                        Ec::OffGrid,
                        Ec::OffGrid,
                    );
                }
            }
        }
    }

    /// Addendum C: judges the runs under 300 ns seen so far: ours when both edges lie within
    /// 200 ns of one of our drive intervals or of a break edge, reported otherwise.
    fn settle(&mut self) {
        for k in 0..self.shorts_n {
            let s = self.shorts[k];
            let (c1, c2) = (self.class(s.a), self.class(s.b));
            let ours = c1 != Ec::Outside && c2 != Ec::Outside;
            // Seventh amendment item 4: a run at our release waits for its exchange's class.
            if ours
                && self.defer_rel
                && self.held_n < self.held.len()
                && let Some(t0) = touches_release(s.a, s.b, &self.drive[..self.drive_n])
            {
                self.held[self.held_n] = (s, t0);
                self.held_n += 1;
                continue;
            }
            if ours {
                self.sp_ours += 1;
            } else {
                self.sp_rep += 1;
            }
            let what = if ours {
                "a run under 300 ns attributed to us"
            } else {
                "a run under 300 ns with an edge outside our intervals (reported)"
            };
            self.record(
                what,
                s.l as usize,
                (s.a << 1) | u64::from(s.high),
                (s.b << 1) | u64::from(!s.high),
                c1,
                c2,
            );
        }
        self.shorts_n = 0;
    }

    /// Judges the held runs at the release of the exchange whose frame began at `t0`;
    /// `transient[l]`: line l spent that exchange in its transient.
    fn resolve(&mut self, t0: u64, transient: [bool; LINES]) {
        let mut k = 0;
        while k < self.held_n {
            let (s, d) = self.held[k];
            if d == t0 {
                self.held[k] = self.held[self.held_n - 1];
                self.held_n -= 1;
                self.judge_held(s, transient[s.l as usize]);
            } else {
                k += 1;
            }
        }
    }

    /// The runs still held at the end of check 2: judged as in a clean exchange.
    fn flush_held(&mut self) {
        while self.held_n > 0 {
            self.held_n -= 1;
            let (s, _) = self.held[self.held_n];
            self.judge_held(s, false);
        }
    }

    fn judge_held(&mut self, s: Short, transient: bool) {
        let ours = short_ours(
            s.a,
            s.b,
            &self.drive[..self.drive_n],
            &self.breaks[..self.breaks_n],
            transient,
        );
        if ours {
            self.sp_ours += 1;
        } else {
            self.sp_rep += 1;
        }
        let what = if ours {
            "a run under 300 ns attributed to us"
        } else {
            "a run under 300 ns at our release in a transient exchange (reported, seventh amendment item 4)"
        };
        let (c1, c2) = (self.class(s.a), self.class(s.b));
        self.record(
            what,
            s.l as usize,
            (s.a << 1) | u64::from(s.high),
            (s.b << 1) | u64::from(!s.high),
            c1,
            c2,
        );
    }

    fn class(&self, t: u64) -> Ec {
        edge_class(
            t,
            &self.drive[..self.drive_n],
            &self.breaks[..self.breaks_n],
        )
    }

    /// Addendum C's record of one run or edge (the first 8 of the program run in full).
    fn record(&mut self, what: &'static str, l: usize, x1: u64, x2: u64, c1: Ec, c2: Ec) {
        let sl = slog();
        sl.n += 1;
        let Some(r) = sl.r.get_mut(sl.n as usize - 1) else {
            return;
        };
        let t = x1 >> 1;
        let t0 = self.drive[..self.drive_n]
            .iter()
            .map(|d| d.0)
            .min_by_key(|&d| d.abs_diff(t));
        *r = SpikeRec {
            what,
            stage: self.stage,
            l: l as u8,
            xk: self.xk,
            t_obs: t,
            phase_ns: t0.map_or(i32::MIN, |d| sns(t, d)),
            run_ns: ns((x2 >> 1) - t),
            rising1: x1 & 1 == 1,
            c1,
            c2,
            padoe: padoe(),
            caps: self.caps,
            ..SpikeRec::NONE
        };
        for m in 0..LINES {
            let ln = &self.l[m];
            for &y in &ln.e[..ln.n] {
                let d = sns(y >> 1, t);
                if d.abs() <= 60_000 && usize::from(r.win_n[m]) < r.win[m].len() {
                    r.win[m][usize::from(r.win_n[m])] = (d << 1) | (y & 1) as i32;
                    r.win_n[m] += 1;
                }
            }
        }
    }
}

fn dma_start() {
    for i in 0..LINES {
        let ch = pac::DMA.ch(DMA_FIRST + i);
        ch.read_addr().write_value(pac::PIO0.rxf(i).as_ptr() as u32);
        ch.write_addr().write_value(ring_base(i) as u32);
        ch.trans_count().write(|w| {
            w.set_mode(0.into());
            w.set_count(0x0FFF_FFFF);
        });
        ch.ctrl_trig().write(|w| {
            w.set_treq_sel(pac::dma::vals::TreqSel::from(4 + i as u8));
            w.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
            w.set_incr_read(false);
            w.set_incr_write(true);
            w.set_ring_size(12);
            w.set_ring_sel(true);
            w.set_chain_to((DMA_FIRST + i) as u8);
            w.set_en(true);
        });
    }
}

fn obs_cfg(origin: &pio::LoadedProgram<'static, PIO0>, gpio: u8) -> Config<'static, PIO0> {
    let mut cfg = Config::default();
    cfg.use_program(origin, &[]);
    cfg.fifo_join = FifoJoin::RxOnly;
    cfg.shift_in = ShiftConfig {
        threshold: 32,
        direction: ShiftDirection::Right,
        auto_fill: false,
    };
    // No pin of PIO0 is made: the observers only read the inputs (jmp pin), which every PIO block
    // sees whatever the pad's function. One dummy out pin puts the block in the GPIO16-47
    // window; nothing in the program writes pins.
    let mut pins = cfg.get_pins();
    pins.out_base = 36;
    pins.out_count = 1;
    // SAFETY: only the window and the JMP pin (counted from GPIOBASE, written by hand) change.
    unsafe { cfg.set_pins(pins) };
    let mut exec = cfg.get_exec();
    exec.jmp_pin = gpio - 16;
    // SAFETY: as above; the wrap settings are the ones use_program wrote.
    unsafe { cfg.set_exec(exec) };
    cfg
}

// ---------------------------------------------------------------------------------------------
// What the observer saw in one window

/// Bit time of the rate in quarter system cycles.
fn bit4(rate: Rate) -> i64 {
    (4 * CLK_HZ / rate.bits_per_second()) as i64
}

#[derive(Clone, Copy, Default)]
struct ReplyObs {
    present: bool,
    delay_ns: u32,
    /// Reply bit time in tenths of a nanosecond.
    bit_dns: u32,
    value: Option<Reply>,
}

fn ns(sys: u64) -> u32 {
    (sys * 20 / 3) as u32
}

/// Signed nanoseconds from `b` to `a` (system cycles), saturated.
fn sns(a: u64, b: u64) -> i32 {
    let d = (a.abs_diff(b) * 20 / 3).min(i32::MAX as u64) as i32;
    if a >= b { d } else { -d }
}

/// Addendum C: one run under 300 ns (line, its two edges, its level).
#[derive(Clone, Copy)]
struct Short {
    l: u8,
    a: u64,
    b: u64,
    high: bool,
}

impl Short {
    const NONE: Short = Short {
        l: 0,
        a: 0,
        b: 0,
        high: false,
    };
}

/// Addendum C: where an edge lies: inside our drive interval on our bit grid or off it, within
/// 200 ns outside it, within 200 ns of a break edge, or outside all of these.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Ec {
    Drive,
    OffGrid,
    Margin,
    Break,
    Outside,
}

/// 200 ns in system cycles.
const MARGIN_SYS: u64 = 30;

/// `drive`: our frames as (first falling edge, bit time in quarter cycles), 16 bits to our release.
fn edge_class(t: u64, drive: &[(u64, i64)], breaks: &[u64]) -> Ec {
    let mut c = Ec::Outside;
    for &(t0, b4) in drive {
        let rel = t0 + 4 * b4 as u64;
        if (t0..=rel).contains(&t) {
            return if on_grid(t, t0, b4) {
                Ec::Drive
            } else {
                Ec::OffGrid
            };
        }
        if t + MARGIN_SYS >= t0 && t <= rel + MARGIN_SYS {
            c = Ec::Margin;
        }
    }
    if c == Ec::Outside && breaks.iter().any(|&b| b.abs_diff(t) <= MARGIN_SYS) {
        c = Ec::Break;
    }
    c
}

/// Within 53 ns of one of our bit phases from `t0`: a falling edge, the rise of a 0 or of a 1.
fn on_grid(t: u64, t0: u64, b4: i64) -> bool {
    // 32-bit (t lies inside one frame): a 64-bit division here cost check 2 its 1 kHz cadence.
    let (p, b4) = ((((t - t0) * 4) as u32 % b4 as u32) as i32, b4 as i32);
    [0, b4 * 15 / 40, b4 * 30 / 40, b4]
        .iter()
        .any(|&g| (p - g).abs() <= 4 * TOL_SYS as i32)
}

/// Addendum C on one line's synthetic edges, our frame from `t0` (system cycles): runs under
/// 300 ns attributed to us, runs reported, edges off our bit grid inside the drive interval.
fn spike_verdict(e: &[u64], t0: u64, b4: i64) -> (u32, u32, u32) {
    let d = [(t0, b4)];
    let (mut ours, mut rep) = (0u32, 0u32);
    for w in e.windows(2) {
        if (w[1] >> 1) - (w[0] >> 1) < SPIKE_SYS {
            if edge_class(w[0] >> 1, &d, &[]) != Ec::Outside
                && edge_class(w[1] >> 1, &d, &[]) != Ec::Outside
            {
                ours += 1;
            } else {
                rep += 1;
            }
        }
    }
    let off = e
        .iter()
        .filter(|&&x| edge_class(x >> 1, &d, &[]) == Ec::OffGrid)
        .count() as u32;
    (ours, rep, off)
}

/// Our drive interval (first falling edge `t0`) whose release lies within 200 ns of the run `a`-`b`.
fn touches_release(a: u64, b: u64, drive: &[(u64, i64)]) -> Option<u64> {
    drive
        .iter()
        .find(|&&(t0, b4)| {
            let rel = t0 + 4 * b4 as u64;
            a <= rel + MARGIN_SYS && b + MARGIN_SYS >= rel
        })
        .map(|&(t0, _)| t0)
}

/// Addendum C with the seventh amendment's item 4: a run under 300 ns with both edges near our drive
/// interval or a break edge is ours, unless it touches our release in a transient exchange.
fn short_ours(a: u64, b: u64, drive: &[(u64, i64)], breaks: &[u64], transient: bool) -> bool {
    edge_class(a, drive, breaks) != Ec::Outside
        && edge_class(b, drive, breaks) != Ec::Outside
        && !(transient && touches_release(a, b, drive).is_some())
}

/// Addendum C's record of one run under 300 ns or one edge off our grid.
#[derive(Clone, Copy)]
struct SpikeRec {
    what: &'static str,
    stage: &'static str,
    l: u8,
    xk: u32,
    t_obs: u64,
    phase_ns: i32,
    run_ns: u32,
    rising1: bool,
    c1: Ec,
    c2: Ec,
    padoe: u32,
    caps: Option<[LineCapture; LINES]>,
    /// The edges of all four lines within 60 us, (ns from the recorded edge) << 1 | rising.
    win: [[i32; 40]; LINES],
    win_n: [u8; LINES],
}

impl SpikeRec {
    const NONE: SpikeRec = SpikeRec {
        what: "",
        stage: "",
        l: 0,
        xk: 0,
        t_obs: 0,
        phase_ns: 0,
        run_ns: 0,
        rising1: false,
        c1: Ec::Outside,
        c2: Ec::Outside,
        padoe: 0,
        caps: None,
        win: [[0; 40]; LINES],
        win_n: [0; LINES],
    };
}

struct SpikeLog {
    n: u32,
    r: [SpikeRec; 8],
}

static mut SLOG: SpikeLog = SpikeLog {
    n: 0,
    r: [SpikeRec::NONE; 8],
};

fn slog() -> &'static mut SpikeLog {
    // SAFETY: one executor thread; the log is touched by `Obs::record` and the final print only,
    // never from an interrupt, and no reference outlives the call that took it.
    let p = &raw mut SLOG;
    unsafe { &mut *p }
}

fn print_spikes() {
    let sl = slog();
    let m = (sl.n as usize).min(sl.r.len());
    defmt::info!(
        "[INFO] addendum C: {=u32} runs under 300 ns or edges off our grid recorded in this run, the first {=usize} in full",
        sl.n,
        m
    );
    for r in &sl.r[..m] {
        defmt::info!(
            "[INFO] addendum C record: {=str}; {=str}, {=str}, exchange {=u32}, observer cycle {=u64}, phase {=i32} ns from t0 (-2147483648: no frame); run {=u32} ns; first edge rising {=bool}, class {}, second edge class {}; DBG_PADOE {=u32:#x}; driver {}; edges within 60 us on M4, M3, M2, M1, (ns from the recorded edge) << 1 | rising: {=[?]} {=[?]} {=[?]} {=[?]}",
            r.what,
            NAMES[usize::from(r.l)],
            r.stage,
            r.xk,
            r.t_obs,
            r.phase_ns,
            r.run_ns,
            r.rising1,
            r.c1,
            r.c2,
            r.padoe,
            r.caps,
            &r.win[0][..usize::from(r.win_n[0])],
            &r.win[1][..usize::from(r.win_n[1])],
            &r.win[2][..usize::from(r.win_n[2])],
            &r.win[3][..usize::from(r.win_n[3])]
        );
    }
}

/// Is `e[..32]` one whole STOP frame (12 short lows, 4 long ones) at the bit time of `rate`?
fn is_stop_frame(e: &[u64], rate: Rate) -> bool {
    if e.len() < 32 {
        return false;
    }
    let b = bit4(rate);
    let (t0h, t1h) = (b * 15 / 40, b * 30 / 40);
    let tol = 4 * 8; // 8 system cycles, 53 ns
    let t = |k: usize| ((e[k] >> 1) * 4) as i64;
    let rising = |k: usize| e[k] & 1 == 1;
    let t0 = t(0);
    for k in 0..16 {
        let (f, r) = (2 * k, 2 * k + 1);
        if rising(f) || !rising(r) {
            return false;
        }
        if (t(f) - t0 - k as i64 * b).abs() > tol {
            return false;
        }
        let want = if k < 12 { t0h } else { t1h };
        if (t(r) - t(f) - want).abs() > tol {
            return false;
        }
    }
    true
}

/// The reply in the edges after our frame: start delay, bit time and the value, decoded from
/// the edges against the nominal reply bit (4/5 of the command bit).
fn reply_of(after: &[u64], last_rise: u64, rate: Rate) -> ReplyObs {
    let mut o = ReplyObs::default();
    if after.is_empty() {
        return o;
    }
    o.present = true;
    let t = |k: usize| after[k] >> 1;
    o.delay_ns = ns(t(0) - (last_rise >> 1));
    let nominal4 = bit4(rate) * 4 / 5;
    let mut levels = [true; 21];
    let mut pos = 0usize;
    let mut level = false; // the first symbol is low
    let mut symbols = 0i64;
    for k in 1..after.len() {
        let run4 = ((t(k) - t(k - 1)) * 4) as i64;
        let n = ((run4 + nominal4 / 2) / nominal4) as usize;
        for _ in 0..n {
            if pos < 21 {
                levels[pos] = level;
            }
            pos += 1;
        }
        symbols += n as i64;
        level = !level;
    }
    let span = (t(after.len() - 1) - t(0)) as i64;
    if symbols > 0 {
        o.bit_dns = (span * 200 / 3 / symbols) as u32;
    }
    if after.len().is_multiple_of(2) && pos <= 21 {
        let mut word = 0u32;
        for &l in &levels {
            word = (word << 1) | u32::from(l);
        }
        o.value = decode_reply(word).ok();
    }
    o
}

/// Diagnostic: the edges of one line's window, as (ns from its first edge) << 1 | rising.
fn dump_window(obs: &Obs, l: usize, what: &str, k: u32) {
    let ln = &obs.l[l];
    let e = &ln.e[..ln.n];
    let e0 = e.first().map_or(0, |&x| x >> 1);
    let mut rel = [0u32; 48];
    let m = e.len().min(48);
    for j in 0..m {
        rel[j] = (ns((e[j] >> 1) - e0) << 1) | (e[j] & 1) as u32;
    }
    defmt::info!(
        "[INFO] diag {=str} {=str} at {=u32}: {=usize} edges (over {=u32}), first at observer cycle {=u64}, {=u64} us before now: {=[?]}",
        what,
        NAMES[l],
        k,
        ln.n,
        ln.over,
        e0,
        obs.now().saturating_sub(e0) / 150,
        &rel[..m]
    );
}

// ---------------------------------------------------------------------------------------------

fn padoe() -> u32 {
    pac::PIO2.dbg_padoe().read() & OE_MASK
}

/// Diagnostic: (levels of GPIO39..36, PIO2 DBG_PADOE of them, SIO output enables of them, pads ok).
fn lines_now() -> (u32, u32, u32, bool) {
    (
        pac::SIO.gpio_in(1).read() >> 4 & 0xF,
        padoe() >> 20,
        pac::SIO.gpio_oe(1).value().read() >> 4 & 0xF,
        pads_ok(),
    )
}

/// Every motor line an input with its pull-up: pad pull-up on, pull-down off, input enabled,
/// output not disabled by the pad.
fn pads_ok() -> bool {
    (36..40).all(|g| {
        let p = pac::PADS_BANK0.gpio(g).read();
        p.pue() && !p.pde() && p.ie() && !p.od()
    })
}

fn stopped(c: &LineCapture) -> bool {
    matches!(c, LineCapture::Reply(Reply::Erpm(e)) if e.is_stopped())
}

// ---------------------------------------------------------------------------------------------
// Check 2 (seventh amendment): the classifier of one exchange on one line and the transient zone.

/// The 21 reply levels of a stopped motor (eRPM 0xFFF).
const STOPPED_WORD: u32 = 0x52951;
/// The 21 reply levels of the extended-telemetry acknowledge 0xE00 (self-test only).
const ACK_WORD: u32 = 0x5BA2D;
/// The frame test's tolerance: 8 system cycles, 53 ns.
const TOL_SYS: u64 = 8;

/// The class of one exchange on one line.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Cls {
    /// Clean: our whole frame, then one stopped reply in the reply slot; driver `Reply(stopped)`.
    G,
    /// Clean: our whole frame and nothing else; driver `Silent`.
    Q,
    /// Not clean, our frame strict: the controller's transient (allowed only in the zone).
    T,
    /// A clean window the driver does not agree with.
    D,
    /// Our frame not strict.
    X,
}

#[derive(Clone, Copy)]
struct Ex {
    cls: Cls,
    /// Why not G or Q (diagnostic).
    why: &'static str,
    reply: ReplyObs,
}

impl Ex {
    const NONE: Ex = Ex {
        cls: Cls::X,
        why: "",
        reply: ReplyObs {
            present: false,
            delay_ns: 0,
            bit_dns: 0,
            value: None,
        },
    };
}

/// Our whole STOP frame anywhere in `e`: the first index where the unchanged 32-edge test holds
/// (M10's `find_frame` without its held-low form).
fn find_frame(e: &[u64], rate: Rate) -> Option<usize> {
    (0..e.len().saturating_sub(31)).find(|&j| is_stop_frame(&e[j..], rate))
}

/// A gap longer than this ends a group: 3.2 command bits (4 reply bits), system cycles (as M10).
fn gap_sys(rate: Rate) -> u64 {
    (bit4(rate) * 32 / 40) as u64
}

/// The end of the group that starts at `e[i]`.
fn group_end(e: &[u64], i: usize, gap: u64) -> usize {
    let mut k = i + 1;
    while k < e.len() && (e[k] >> 1) - (e[k - 1] >> 1) <= gap {
        k += 1;
    }
    k
}

/// Check 2 for one exchange on one line: G and Q are the clean windows, each against the driver's
/// result (D when it disagrees); T anything else with our frame strict, whole on this line or, where
/// a controller group overlaps it, addendum A's a1-a3 against `anchor` (the 32 edges of our frame on
/// a line where it is whole in the same exchange); X our frame not strict.
fn classify(e: &[u64], c: &LineCapture, rate: Rate, anchor: Option<&[u64]>) -> Ex {
    let x = Ex::NONE;
    let Some(j) = find_frame(e, rate) else {
        return match anchor.map(|a| a_form(e, a)) {
            Some(Ok(())) => Ex {
                cls: Cls::T,
                why: "our first falling edge masked by a controller group (A)",
                ..x
            },
            Some(Err(why)) => Ex { why, ..x },
            None => Ex {
                why: "no whole frame, and no line with one in this exchange (a1)",
                ..x
            },
        };
    };
    let (before, after) = (&e[..j], &e[j + 32..]);
    let r = reply_of(after, e[j + 31], rate);
    let obs_q = before.is_empty() && after.is_empty();
    let obs_g = before.is_empty()
        && matches!(r.value, Some(Reply::Erpm(v)) if v.is_stopped())
        && after[0] & 1 == 0
        && group_end(after, 0, gap_sys(rate)) == after.len()
        && (20_000..=30_000).contains(&r.delay_ns);
    if obs_g && stopped(c) {
        Ex {
            cls: Cls::G,
            reply: r,
            ..x
        }
    } else if obs_q && *c == LineCapture::Silent {
        Ex { cls: Cls::Q, ..x }
    } else if obs_g || obs_q {
        Ex {
            cls: Cls::D,
            why: "a clean window the driver does not agree with",
            ..x
        }
    } else {
        Ex {
            cls: Cls::T,
            why: "not a clean window",
            reply: r,
        }
    }
}

/// Addendum A's a1-a3: our edges 2-32 each within 53 ns of the whole line's `a`, the line low at t0
/// after a falling edge, the merged low at least our first bit's low.
fn a_form(e: &[u64], a: &[u64]) -> Result<(), &'static str> {
    let near = |p: u64, q: u64| p & 1 == q & 1 && (p >> 1).abs_diff(q >> 1) <= TOL_SYS;
    let Some(m) = e.iter().position(|&p| near(p, a[1])) else {
        return Err("no whole frame; A (a2): no first rising edge on the whole line's");
    };
    if m + 31 > e.len() || !(1..32).all(|k| near(e[m + k - 1], a[k])) {
        return Err("no whole frame; A (a2): our edges 2-32 not within 53 ns of the whole line's");
    }
    let t0 = a[0] >> 1;
    if m == 0 || e[m - 1] & 1 != 0 || e[m - 1] >> 1 >= t0 {
        return Err("A (a3): the line not low at t0 after a falling edge");
    }
    if (e[m] >> 1) - (e[m - 1] >> 1) < (a[1] >> 1) - (a[0] >> 1) {
        return Err("A (a3): merged low shorter than our first bit's low");
    }
    Ok(())
}

/// Our frame of the next exchange: the first line where it is whole after `after` (the previous
/// frame's last rising edge); its 32 edges.
fn find_anchor(obs: &Obs, after: Option<u64>, rate: Rate) -> Option<[u64; 32]> {
    obs.l.iter().find_map(|ln| {
        let e = &ln.e[..ln.n];
        let from = after.map_or(0, |a| e.iter().position(|&x| x >> 1 > a).unwrap_or(e.len()));
        find_frame(&e[from..], rate).map(|j| core::array::from_fn(|k| e[from + j + k]))
    })
}

/// Where the next exchange's window begins on one line: at its frame (first falling edge `t0` on
/// the anchor line), moved back over a group that runs into it, never before `after`.
fn cut_at(e: &[u64], after: Option<u64>, t0: u64, gap: u64) -> usize {
    let from = after.map_or(0, |a| e.iter().position(|&x| x >> 1 > a).unwrap_or(e.len()));
    let mut c = from
        + e[from..]
            .iter()
            .position(|&x| (x >> 1) + TOL_SYS >= t0)
            .unwrap_or(e.len() - from);
    if c < e.len() && c > from && e[c - 1] & 1 == 0 {
        c -= 1;
    }
    while c > from && c < e.len() && (e[c] >> 1) - (e[c - 1] >> 1) <= gap {
        c -= 1;
    }
    c
}

/// Check 2's bookkeeping for one line: the counts; the arming block (the first run of 250 or more
/// exchanges without a good reply); its transient zone (from the block's first exchange to 3 after
/// its last); what lies outside it; the first 4 transient exchanges and the driver's results.
#[derive(Clone, Copy)]
struct Zone {
    g: u32,
    q: u32,
    t: u32,
    d: u32,
    x: u32,
    run_start: u32,
    run_len: u32,
    run_t: u32,
    run_fi: u32,
    block: Option<(u32, u32)>,
    t_zone: u32,
    outside: u32,
    fi_out: u32,
    t_i: [u32; 4],
    t_cap: [LineCapture; 4],
    t_n: usize,
}

impl Zone {
    const NEW: Zone = Zone {
        g: 0,
        q: 0,
        t: 0,
        d: 0,
        x: 0,
        run_start: 0,
        run_len: 0,
        run_t: 0,
        run_fi: 0,
        block: None,
        t_zone: 0,
        outside: 0,
        fi_out: 0,
        t_i: [0; 4],
        t_cap: [LineCapture::Silent; 4],
        t_n: 0,
    };

    /// Exchange `i` of the stream, its class and the driver's result.
    fn push(&mut self, i: u32, cls: Cls, cap: LineCapture) {
        let fi = matches!(cap, LineCapture::Foreign | LineCapture::Invalid(_));
        match cls {
            Cls::G => self.g += 1,
            Cls::Q => self.q += 1,
            Cls::T => self.t += 1,
            Cls::D => self.d += 1,
            Cls::X => self.x += 1,
        }
        if cls == Cls::T {
            if self.t_n < self.t_i.len() {
                self.t_i[self.t_n] = i;
                self.t_cap[self.t_n] = cap;
            }
            self.t_n += 1;
        }
        if cls == Cls::G {
            self.close();
            return;
        }
        match self.block {
            Some((_, e)) if i <= e + 3 => self.t_zone += u32::from(cls == Cls::T),
            Some(_) => {
                self.outside += 1;
                self.fi_out += u32::from(fi);
            }
            None => {
                if self.run_len == 0 {
                    self.run_start = i;
                }
                self.run_len += 1;
                self.run_t += u32::from(cls == Cls::T);
                self.run_fi += u32::from(fi);
            }
        }
    }

    /// Ends the current run without a good reply (at a G or the end of the stream).
    fn close(&mut self) {
        if self.run_len >= 250 {
            self.block = Some((self.run_start, self.run_start + self.run_len - 1));
            self.t_zone += self.run_t;
        } else {
            self.outside += self.run_len;
            self.fi_out += self.run_fi;
        }
        self.run_len = 0;
        self.run_t = 0;
        self.run_fi = 0;
    }

    /// No disagreement, our frame strict, nothing but G outside the zone, 3 transients at most in
    /// it, one block of 500 or less, 1000 G or more.
    fn passes(&self) -> bool {
        self.d == 0
            && self.x == 0
            && self.outside == 0
            && self.t_zone <= 3
            && self.g >= 1000
            && self.block.is_some_and(|(s, e)| e - s < 500)
    }
}

/// Diagnostic of check 2, kept during the stream and printed after it (a print inside the loop can
/// stall it): one line's window as (ns from its first edge) << 1 | rising.
#[derive(Clone, Copy)]
struct Diag {
    l: usize,
    k: u32,
    why: &'static str,
    cap: LineCapture,
    n: usize,
    m: usize,
    rel: [u32; 64],
}

impl Diag {
    fn new(e: &[u64], l: usize, k: u32, why: &'static str, cap: LineCapture) -> Diag {
        let e0 = e.first().map_or(0, |&x| x >> 1);
        let mut rel = [0u32; 64];
        let m = e.len().min(64);
        for j in 0..m {
            rel[j] = (ns((e[j] >> 1) - e0) << 1) | (e[j] & 1) as u32;
        }
        Diag {
            l,
            k,
            why,
            cap,
            n: e.len(),
            m,
            rel,
        }
    }

    fn print(&self) {
        defmt::info!(
            "[INFO] diag check 2 {=str} at exchange {=u32} from start(): {=str}; driver {}; {=usize} edges, (ns from the first) << 1 | rising: {=[?]}",
            NAMES[self.l],
            self.k,
            self.why,
            self.cap,
            self.n,
            &self.rel[..self.m]
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Self-tests, before the first frame

/// Synthetic observer edges.
#[derive(Clone, Copy)]
struct Syn {
    e: [u64; 128],
    n: usize,
}

impl Syn {
    const NEW: Syn = Syn { e: [0; 128], n: 0 };

    fn edge(&mut self, t4: i64, rising: bool) {
        if self.n < self.e.len() {
            self.e[self.n] = (((t4 / 4) as u64) << 1) | u64::from(rising);
            self.n += 1;
        }
    }

    /// Our STOP frame from `t4` (quarter system cycles), its first falling edge left out when
    /// `masked`; returns its last rising edge.
    fn frame(&mut self, t4: i64, rate: Rate, masked: bool) -> i64 {
        let b = bit4(rate);
        let mut last = t4;
        for k in 0..16 {
            let f = t4 + k * b;
            if k > 0 || !masked {
                self.edge(f, false);
            }
            last = f + if k < 12 { b * 15 / 40 } else { b * 30 / 40 };
            self.edge(last, true);
        }
        last
    }

    /// The reply levels `word` from `t4` at the reply bit `rb4`, the edges before `until4` only.
    fn group(&mut self, t4: i64, word: u32, rb4: i64, until4: i64) {
        let mut level = true;
        for k in 0..21i64 {
            let bit = (word >> (20 - k)) & 1 == 1;
            if bit != level && t4 + k * rb4 < until4 {
                self.edge(t4 + k * rb4, bit);
            }
            level = bit;
        }
        if !level && t4 + 21 * rb4 < until4 {
            self.edge(t4 + 21 * rb4, true);
        }
    }

    fn get(&self) -> &[u64] {
        &self.e[..self.n]
    }
}

/// 3000 exchanges through `Zone`.
fn zone_run(f: impl Fn(u32) -> Cls) -> Zone {
    let mut z = Zone::NEW;
    for i in 0..3000 {
        z.push(i, f(i), LineCapture::Silent);
    }
    z.close();
    z
}

/// Check 2's classifier and zone rules, addendum C's attribution and check 6's comparators on
/// synthetic edges and captures; returns the number of failures.
fn self_tests() -> u32 {
    let mut f = 0u32;
    let sil = LineCapture::Silent;
    let Ok(st) = decode_reply(STOPPED_WORD) else {
        defmt::error!("[FAIL] self-test: 0x52951 does not decode");
        return 1;
    };
    let stop = LineCapture::Reply(st);
    check!(
        f,
        matches!(st, Reply::Erpm(e) if e.is_stopped())
            && matches!(
                decode_reply(ACK_WORD),
                Ok(Reply::Extended(ExtendedTelemetry::EnabledAck))
            ),
        "[{=str}] self-test: the crate's decoder reads 0x52951 as the stopped motor and 0x5BA2D as the acknowledge 0xE00"
    );
    for rate in [Rate::Dshot300, Rate::Dshot600] {
        let label = match rate {
            Rate::Dshot300 => "DShot300",
            Rate::Dshot600 => "DShot600",
        };
        let b = bit4(rate);
        // The reply bit 4 % fast, as on the bench.
        let rb4 = b * 4 / 5 * 96 / 100;
        let us4 = 600i64;
        let t0 = 400_000i64;
        let dly4 = match rate {
            Rate::Dshot300 => 26 * us4,
            Rate::Dshot600 => 235 * us4 / 10,
        };
        let late4 = t0 + 600 * us4;
        let cl = |s: &Syn, c: &LineCapture, a: Option<&[u64]>| classify(s.get(), c, rate, a).cls;

        let mut g_win = Syn::NEW;
        let last = g_win.frame(t0, rate, false);
        g_win.group(last + dly4, STOPPED_WORD, rb4, i64::MAX);
        let mut anchor = Syn::NEW;
        anchor.frame(t0, rate, false);
        let a = &anchor.e[..32];
        check!(
            f,
            cl(&g_win, &stop, None) == Cls::G && cl(&anchor, &sil, None) == Cls::Q,
            "[{=str}] self-test {=str}: our frame and the stopped reply, driver Reply(stopped): G; our frame alone, driver Silent: Q",
            label
        );
        let mut late = g_win;
        late.group(late4, STOPPED_WORD, rb4, i64::MAX);
        let mut at_rel = Syn::NEW;
        let last = at_rel.frame(t0, rate, false);
        at_rel.group(last + 10 * b / 40 + 12, ACK_WORD, rb4, i64::MAX);
        let mut a_win = Syn::NEW;
        a_win.group(t0 - 21 * rb4 / 2, STOPPED_WORD, rb4, t0);
        a_win.frame(t0, rate, true);
        check!(
            f,
            cl(&late, &stop, Some(a)) == Cls::T
                && cl(&at_rel, &LineCapture::Foreign, Some(a)) == Cls::T
                && cl(&a_win, &sil, Some(a)) == Cls::T,
            "[{=str}] self-test {=str}: a late group after the reply, a group from our release, a group still low at t0 with our other 31 edges on the whole line's: transient",
            label
        );
        // Negative controls of the seventh amendment.
        check!(
            f,
            cl(&g_win, &sil, None) == Cls::D && cl(&anchor, &stop, None) == Cls::D,
            "[{=str}] self-test {=str}: a driver/observer disagreement on a clean window (Silent against the stopped reply, Reply against nothing): flagged",
            label
        );
        let mut moved = late;
        moved.e[10] += 12 << 1;
        let mut a_moved = a_win;
        let m = a_moved.n - 31;
        a_moved.e[m + 5] += 12 << 1;
        check!(
            f,
            cl(&moved, &stop, Some(a)) == Cls::X
                && cl(&a_moved, &sil, Some(a)) == Cls::X
                && cl(&a_win, &sil, None) == Cls::X,
            "[{=str}] self-test {=str}: our frame with one edge 80 ns off on a whole and on an overlapped line, an overlapped line with no whole line in the exchange: our frame not strict",
            label
        );

        // Addendum C: the spike check's attribution.
        let (sys4, ts) = (4i64, (t0 / 4) as u64);
        let mut s = Syn::NEW;
        s.edge(t0 - 20 * us4, false);
        s.edge(t0 - 12 * sys4, true);
        s.frame(t0, rate, false);
        check!(
            f,
            spike_verdict(s.get(), ts, b).0 >= 1,
            "[{=str}] self-test {=str}: C, 80 ns high before our first fall: attributed to us (fails the run)",
            label
        );
        let mut s = Syn::NEW;
        s.frame(t0, rate, false);
        s.edge(t0 + 5 * b + 6 * b / 10, false);
        s.edge(t0 + 5 * b + 6 * b / 10 + 15 * sys4, true);
        s.e[..s.n].sort_unstable();
        let v = spike_verdict(s.get(), ts, b);
        check!(
            f,
            v.0 >= 1 && v.2 >= 1,
            "[{=str}] self-test {=str}: C, a 100 ns pulse between two grid edges in our frame: attributed to us and off the grid (fails the run)",
            label
        );
        let mut s = Syn::NEW;
        s.frame(t0, rate, false);
        s.edge(t0 + 16 * b + 9 * sys4, false);
        s.edge(t0 + 16 * b + 24 * sys4, true);
        check!(
            f,
            spike_verdict(s.get(), ts, b).0 >= 1,
            "[{=str}] self-test {=str}: C, a 100 ns low at our release: attributed to us (fails the run)",
            label
        );
        let mut s = Syn::NEW;
        s.edge(t0 - 20 * us4, false);
        s.edge(t0 - 42 * sys4, true);
        s.frame(t0, rate, false);
        check!(
            f,
            spike_verdict(s.get(), ts, b) == (0, 1, 0),
            "[{=str}] self-test {=str}: C, a controller rise 280 ns before our first fall: reported only",
            label
        );

        let mut s = Syn::NEW;
        s.frame(t0, rate, false);
        s.edge(t0 + 16 * b + 9 * sys4, false);
        s.edge(t0 + 16 * b + 24 * sys4, true);
        let (d, ra, rb) = ([(ts, b)], s.e[32] >> 1, s.e[33] >> 1);
        check!(
            f,
            short_ours(ra, rb, &d, &[], false) && !short_ours(ra, rb, &d, &[], true),
            "[{=str}] self-test {=str}: C item 4, a short run at our release: attributed to us in a clean exchange, reported in a transient one",
            label
        );

        // Check 6 and the cancel sweep: the comparator of "own" and the dropped exchange's window.
        check!(
            f,
            own_ok(g_win.get(), &stop, rate)
                && dropped_ok(g_win.get(), rate)
                && dropped_ok(anchor.get(), rate),
            "[{=str}] self-test {=str}: check 6: our frame and the stopped reply, driver Reply(stopped): own; our frame with or without the reply: a whole dropped frame",
            label
        );
        check!(
            f,
            !own_ok(g_win.get(), &sil, rate),
            "[{=str}] self-test {=str}: check 6 comparator: driver Silent against an observer that saw a stopped reply: flagged",
            label
        );
        check!(
            f,
            !own_ok(anchor.get(), &stop, rate),
            "[{=str}] self-test {=str}: check 6 comparator: driver Reply(stopped) against an observer that saw no reply: flagged",
            label
        );
        let mut two = g_win;
        two.frame(late4, rate, false);
        let mut cut = g_win;
        cut.e[10] += 12 << 1;
        let none = Syn::NEW;
        check!(
            f,
            !own_ok(two.get(), &stop, rate)
                && !dropped_ok(two.get(), rate)
                && !own_ok(cut.get(), &stop, rate)
                && !dropped_ok(cut.get(), rate)
                && !dropped_ok(none.get(), rate),
            "[{=str}] self-test {=str}: check 6 and the sweep: two whole frames in one window, our frame with one falling edge moved 80 ns, no frame: each flagged",
            label
        );
    }

    // The zone on 3000 exchanges: G, a block of 300 Q from 1200, G.
    let base = |i: u32| {
        if (1200..1500).contains(&i) {
            Cls::Q
        } else {
            Cls::G
        }
    };
    let z = zone_run(|i| match i {
        1499 | 1501 | 1502 => Cls::T,
        _ => base(i),
    });
    check!(
        f,
        z.passes() && z.t_zone == 3,
        "[{=str}] self-test: transients at the block's last exchange and 2 and 3 after it: pass"
    );
    let z = zone_run(|i| if i == 600 { Cls::T } else { base(i) });
    let z2 = zone_run(|i| if i == 1503 { Cls::T } else { base(i) });
    check!(
        f,
        !z.passes() && !z2.passes() && z.outside == 1 && z2.outside == 1,
        "[{=str}] self-test: a transient outside the zone (exchange 600; 4 after the block): fails"
    );
    let z = zone_run(|i| match i {
        1497..=1499 | 1501 => Cls::T,
        _ => base(i),
    });
    check!(
        f,
        !z.passes() && z.t_zone == 4,
        "[{=str}] self-test: a fourth transient inside the zone: fails"
    );
    let z = zone_run(|i| if i == 1499 { Cls::X } else { base(i) });
    let z2 = zone_run(|i| if i == 1499 { Cls::D } else { base(i) });
    check!(
        f,
        !z.passes() && !z2.passes(),
        "[{=str}] self-test: our frame not strict, and a disagreeing clean window, inside the zone: fail"
    );
    let z = zone_run(|i| if i == 2000 { Cls::Q } else { base(i) });
    check!(
        f,
        !z.passes(),
        "[{=str}] self-test: a missed reply outside the zone (a second block): fails"
    );
    f
}

struct Ctx {
    drv: Option<PioDshot<'static, PIO2>>,
    obs: Obs,
    fails: u32,
    /// The next shot may first finish a cancelled capture: its first poll is not the push.
    no_oe_during: bool,
    oe_bad_during: u32,
    oe_bad_after: u32,
    oe_samples: u32,
    /// Check 2: the observer's windows run on from one exchange to the next.
    keep: bool,
    /// Exchanges sent since `start()` last returned.
    sent: u32,
    /// When `shot` last polled its exchange the first time (at or before its push).
    t_push: Instant,
}

struct Shot {
    r: Exchange,
}

impl Ctx {
    fn drv(&mut self) -> &mut PioDshot<'static, PIO2> {
        match self.drv.as_mut() {
            Some(d) => d,
            None => {
                defmt::error!("[FAIL] no driver");
                hil_common::exit(self.fails + 1)
            }
        }
    }

    /// One exchange of four STOP frames, with DBG_PADOE read during the frame (after the first
    /// poll, which pushes) and after it, and the observer's window of this exchange.
    async fn shot(&mut self) -> Shot {
        if self.keep {
            self.obs.sync();
        } else {
            self.obs.mark();
        }
        let rate = self.drv().rate();
        let (r, during) = {
            let Some(drv) = self.drv.as_mut() else {
                hil_common::exit(self.fails + 1)
            };
            let mut fut = pin!(drv.exchange(STOP4));
            self.t_push = Instant::now();
            let first = poll_fn(|cx| match fut.as_mut().poll(cx) {
                Poll::Ready(r) => Poll::Ready(Some(r)),
                Poll::Pending => Poll::Ready(None),
            })
            .await;
            match first {
                Some(r) => (r, None),
                None => {
                    let oe = padoe();
                    (fut.await, Some(oe))
                }
            }
        };
        let after = padoe();
        self.oe_samples += 1;
        self.obs.xk = self.oe_samples;
        if let Some(oe) = during
            && oe != OE_MASK
            && !self.no_oe_during
        {
            self.oe_bad_during += 1;
        }
        self.no_oe_during = false;
        if after != 0 {
            self.oe_bad_after += 1;
        }
        self.obs.sync();
        self.sent += 1;
        // Check 2 classifies its own windows (they run on from one exchange to the next).
        if self.keep {
            return Shot { r };
        }
        let anchor = find_anchor(&self.obs, None, rate);
        if let Some(a) = &anchor {
            self.obs.register(a, rate);
        }
        self.obs.caps = Some(r.captures);
        self.obs.settle();
        Shot { r }
    }
}

// ---------------------------------------------------------------------------------------------

/// Runs `start()` with the observer watching; checks the break (check 3). Returns whether the
/// break was seen on all four lines.
async fn start_checked(ctx: &mut Ctx, label: &str) -> bool {
    ctx.obs.mark();
    ctx.obs.stage = "start() and its break";
    let t0 = Instant::now();
    let Some(drv) = ctx.drv.as_mut() else {
        hil_common::exit(ctx.fails + 1)
    };
    let mut delay = Delay;
    drv.start(&mut delay).await;
    ctx.sent = 0;
    let took = t0.elapsed().as_millis() as u32;
    ctx.obs.sync();
    let now = ctx.obs.now();
    // The break is the last falling edge of each line in the window; the line comes back up at
    // our release or, when a controller holds it in its start-up (M8), later: then it is still
    // low now. The lows of all four overlap for the length of the break or more.
    let mut fall = [0u64; LINES];
    let mut end = [0u64; LINES];
    let mut found = true;
    let mut park_edges = 0u32;
    for (l, ln) in ctx.obs.l.iter().enumerate() {
        let e = &ln.e[..ln.n];
        match e.iter().rposition(|&x| x & 1 == 0) {
            Some(k) => {
                fall[l] = e[k] >> 1;
                end[l] = e.get(k + 1).map_or(now, |&r| r >> 1);
                park_edges += k as u32;
            }
            None => found = false,
        }
    }
    for l in 0..LINES {
        ctx.obs.breaks[2 * l] = fall[l];
        ctx.obs.breaks[2 * l + 1] = end[l];
    }
    ctx.obs.breaks_n = 2 * LINES;
    ctx.obs.settle();
    let together_start = fall.iter().copied().max().unwrap_or(0);
    let together_end = end.iter().copied().min().unwrap_or(0);
    let together_us = together_end.saturating_sub(together_start) / 150;
    let spread_ns = ns(together_start - fall.iter().copied().min().unwrap_or(0));
    check!(
        ctx.fails,
        found && together_us >= 25_000,
        "[{=str}] {=str}: start() took {=u32} ms; the break held all four lines low together for {=u64} us (25000 or more; a controller's start-up hold may lengthen it), falls within {=u32} ns",
        label,
        took,
        together_us,
        spread_ns
    );
    defmt::info!(
        "[STAT] {=str}: edges on the lines during the 3000 ms park before the break: {=u32}",
        label,
        park_edges
    );
    found && together_us >= 25_000
}

/// Exchanges at 1 kHz until every line is Live or 2 s pass (check 1). Returns, per line, the
/// frame from `start()` at which it became Live.
async fn until_live(ctx: &mut Ctx, label: &str) -> [Option<u32>; LINES] {
    ctx.obs.stage = "check 1";
    let t0 = Instant::now();
    let mut live_at = [None::<u32>; LINES];
    let mut next = Instant::now();
    let fs0 = ctx.drv().monitor().foreign_while_starting();
    let mut k = 0u32;
    while Instant::now() - t0 < Duration::from_millis(2000) {
        Timer::at(next).await;
        next += Duration::from_micros(1000);
        let s = ctx.shot().await;
        k += 1;
        for l in 0..LINES {
            if live_at[l].is_none() && s.r.states[l] == LineState::Live {
                live_at[l] = Some(k);
            }
        }
        if live_at.iter().all(Option::is_some) {
            break;
        }
    }
    let ms = t0.elapsed().as_millis() as u32;
    let fs = ctx.drv().monitor().foreign_while_starting();
    let all = live_at.iter().all(Option::is_some);
    check!(
        ctx.fails,
        all,
        "[{=str}] {=str}: every line Live within 2 s of start() returning ({=u32} ms, {=u32} frames)",
        label,
        ms,
        k
    );
    for l in 0..LINES {
        defmt::info!(
            "[STAT] {=str}: {=str} Live at frame {=u32}; Foreign captures while Starting {=u32}",
            label,
            NAMES[l],
            live_at[l].unwrap_or(0),
            fs[l] - fs0[l]
        );
    }
    live_at
}

async fn run_rate(ctx: &mut Ctx, rate: Rate) {
    let label: &str = match rate {
        Rate::Dshot300 => "DShot300",
        Rate::Dshot600 => "DShot600",
    };
    defmt::info!("[INFO] ===== {=str} =====", label);
    ctx.obs.spike_check = true;

    // ---- 1
    start_checked(ctx, label).await;
    until_live(ctx, label).await;

    // ---- 2 (seventh amendment): 3000 exchanges after all lines are Live, and one more that only
    // closes the last window. Each window is a whole frame period: it begins at our frame (or at a
    // group that runs into it) and ends where the next frame begins, so it is classified one
    // exchange late. Our frame strict on each; G and Q against the driver; anything else only in the
    // line's transient zone, 3 at most.
    ctx.obs.stage = "check 2";
    let fl0 = ctx.drv().monitor().foreign_while_live();
    let k0 = ctx.sent;
    let gap = gap_sys(rate);
    let mut zone = [Zone::NEW; LINES];
    let (mut shown_x, mut shown_t) = ([0u32; LINES], [0u32; LINES]);
    let mut diags: [[Option<Diag>; 7]; LINES] = [[None; 7]; LINES];
    let mut proc_max = 0u64;
    let (mut shot_sum, mut proc_sum) = (0u64, 0u64);
    let t_loop = Instant::now();
    let mut caps = [LineCapture::Silent; LINES];
    let mut anchor: Option<[u64; 32]> = None;
    let (mut dmin, mut dmax, mut dsum, mut dn) = ([u32::MAX; 4], [0u32; 4], [0u64; 4], [0u32; 4]);
    let (mut bmin, mut bmax, mut bsum) = ([u32::MAX; 4], [0u32; 4], [0u64; 4]);
    ctx.obs.mark();
    ctx.keep = true;
    ctx.obs.defer_rel = true;
    // Scheduled from the push with a floor, no catch-up: each frame goes out 1000 us or more after
    // the previous one, later when the loop is late. A fixed 1 ms grid caught up after a
    // classification over 1 ms with a frame that came early, and the controller, just out of its
    // arming tune, answered that frame at our release (run 20261112-221711, M3, exchange 1866). One
    // late frame does not disturb a controller whose timeout is 510 ms; an early one did: that is
    // the measured evidence for this schedule and for the precondition below. The push lies tens of
    // us after the wake, inside the exchange's first poll; the observer's frame spacing against the
    // CPU's wake spacing gives the change of that delay; a fixed margin of 25 us covers it
    // (seventh amendment).
    let mut c_next: Option<u32> = None;
    let mut per = Periods::NEW;
    for i in 0..=3000u32 {
        // The edges so far first: between the wake and the push only the poll.
        ctx.obs.sync();
        if let Some(c) = c_next {
            wait_cyc(c).await;
        }
        let c_go = cyc();
        let t_shot = Instant::now();
        let s = ctx.shot().await;
        let t_proc = Instant::now();
        shot_sum += (t_proc - t_shot).as_micros();
        let prev = caps;
        caps = s.r.captures;
        if i == 0 {
            anchor = find_anchor(&ctx.obs, None, rate);
            if let Some(a) = &anchor {
                ctx.obs.register(a, rate);
            }
            c_next = Some(per.frame(0, anchor.map(|a| a[0] >> 1), c_go));
            continue;
        }
        // Close the window of exchange i - 1 (exchange k from start()).
        let after = anchor.map(|a| a[31] >> 1);
        let next_anchor = find_anchor(&ctx.obs, after, rate);
        c_next = Some(per.frame(i, next_anchor.map(|a| a[0] >> 1), c_go));
        if let Some(a) = &next_anchor {
            ctx.obs.register(a, rate);
        }
        ctx.obs.caps = Some(caps);
        ctx.obs.settle();
        let k = k0 + i;
        let mut ex = [Ex::NONE; LINES];
        let mut cuts = [0usize; LINES];
        for l in 0..LINES {
            let ln = &ctx.obs.l[l];
            let n = ln.n;
            cuts[l] = next_anchor.map_or(n, |a| cut_at(&ln.e[..n], after, a[0] >> 1, gap));
            let w = &ln.e[..cuts[l]];
            ex[l] = classify(w, &prev[l], rate, anchor.as_ref().map(|a| &a[..]));
            if ln.over > 0 {
                ex[l].cls = Cls::X;
                ex[l].why = "observer edge buffer full";
            }
            match ex[l].cls {
                Cls::G => {
                    let r = ex[l].reply;
                    dmin[l] = dmin[l].min(r.delay_ns);
                    dmax[l] = dmax[l].max(r.delay_ns);
                    dsum[l] += u64::from(r.delay_ns);
                    dn[l] += 1;
                    bmin[l] = bmin[l].min(r.bit_dns);
                    bmax[l] = bmax[l].max(r.bit_dns);
                    bsum[l] += u64::from(r.bit_dns);
                }
                Cls::T if shown_t[l] < 4 => {
                    diags[l][3 + shown_t[l] as usize] =
                        Some(Diag::new(w, l, k, ex[l].why, prev[l]));
                    shown_t[l] += 1;
                }
                Cls::D | Cls::X if shown_x[l] < 3 => {
                    diags[l][shown_x[l] as usize] = Some(Diag::new(w, l, k, ex[l].why, prev[l]));
                    shown_x[l] += 1;
                }
                _ => {}
            }
            zone[l].push(i - 1, ex[l].cls, prev[l]);
        }
        if let Some(a) = &anchor {
            ctx.obs
                .resolve(a[0] >> 1, core::array::from_fn(|l| ex[l].cls == Cls::T));
        }
        for l in 0..LINES {
            let ln = &mut ctx.obs.l[l];
            let (n, c) = (ln.n, cuts[l]);
            if next_anchor.is_some() {
                ln.e.copy_within(c..n, 0);
                ln.n = n - c;
            } else {
                ln.n = 0;
            }
            ln.over = 0;
        }
        anchor = next_anchor;
        let p = t_proc.elapsed().as_micros();
        proc_max = proc_max.max(p);
        proc_sum += p;
    }
    ctx.obs.defer_rel = false;
    ctx.obs.flush_held();
    let total_us = t_loop.elapsed().as_micros();
    defmt::info!(
        "[STAT] {=str}: check 2 stream: 3001 frames in {=u64} us (CPU); classification at most {=u64} us per exchange, mean {=u64} us; exchange (shot) mean {=u64} us; pacing margin 25 us (fixed); largest drop of the wake-to-push delay {=u32} ns",
        label,
        total_us,
        proc_max,
        proc_sum / 3001,
        shot_sum / 3001,
        ns((-per.dl_min).max(0) as u64)
    );
    check!(
        ctx.fails,
        per.valid(),
        "[{=str}] {=str}: check 2 precondition (otherwise the stream is not valid): frame spacing from the observer {=u32} to {=u32} ns over {=u32} periods ({=u32} unmeasured); under 1000 us {=u32} (0), over 2000 us {=u32} (0), over 1250 us {=u32} (3 at most); the 3000 periods in {=u64} us (3000 x 1025 = 3075000 within 2 %)",
        label,
        ns(per.min.min(per.max)),
        ns(per.max),
        per.n,
        per.unmeasured,
        per.under,
        per.over2000,
        per.over1250,
        per.total() / 150
    );
    for d in diags.iter().flatten().flatten() {
        d.print();
    }
    for z in &mut zone {
        z.close();
    }
    ctx.keep = false;
    ctx.obs.mark();
    let fl = ctx.drv().monitor().foreign_while_live();
    let nominal_dns = 10 * 4 * 1_000_000_000u64 / 5 / u64::from(rate.bits_per_second());
    for l in 0..LINES {
        let z = zone[l];
        let fl_d = fl[l] - fl0[l];
        let (bs, be) = z.block.unwrap_or((0, 0));
        check!(
            ctx.fails,
            z.passes(),
            "[{=str}] {=str} {=str}: 3000 exchanges: {=u32} G, {=u32} Q, {=u32} transient, {=u32} clean windows the driver disagrees with, {=u32} with our frame not strict; non-reply block exchanges {=u32} to {=u32} from start() (one, 250 to 500 long), zone to {=u32}; transients in the zone {=u32} (3 at most), not G outside it {=u32} (0); 1000 G or more",
            label,
            NAMES[l],
            z.g,
            z.q,
            z.t,
            z.d,
            z.x,
            k0 + 1 + bs,
            k0 + 1 + be,
            k0 + 4 + be,
            z.t_zone,
            z.outside
        );
        check!(
            ctx.fails,
            z.fi_out == 0,
            "[{=str}] {=str} {=str}: 2.9: Foreign and Invalid on the Live line only in the transient zone ({=u32} outside it); the driver's Foreign-while-Live count {=u32} (reported)",
            label,
            NAMES[l],
            z.fi_out,
            fl_d
        );
        check!(
            ctx.fails,
            z.d == 0,
            "[{=str}] {=str} {=str}: the observer agrees with the driver on every clean window ({=u32} disagree)",
            label,
            NAMES[l],
            z.d
        );
        let tk: [u32; 4] = core::array::from_fn(|j| k0 + 1 + z.t_i[j]);
        let tn = z.t_n.min(4);
        defmt::info!(
            "[STAT] {=str} {=str}: transient exchanges {=[?]} from start() (first 4 of {=usize}), driver {=[?]}",
            label,
            NAMES[l],
            &tk[..tn],
            z.t_n,
            &z.t_cap[..tn]
        );
        let n = u64::from(dn[l].max(1));
        let bmean = bsum[l] / n;
        defmt::info!(
            "[STAT] {=str} {=str}: observer, {=u32} replies: start {=u32}/{=u64}/{=u32} ns after our last rising edge (min/mean/max); reply bit {=u32}/{=u64}/{=u32} x0.1 ns, mean {=i64} per mille from nominal {=u64} x0.1 ns",
            label,
            NAMES[l],
            dn[l],
            dmin[l],
            dsum[l] / n,
            dmax[l],
            bmin[l],
            bmean,
            bmax[l],
            (bmean as i64 - nominal_dns as i64) * 1000 / nominal_dns as i64,
            nominal_dns
        );
    }
    check!(
        ctx.fails,
        ctx.oe_bad_during == 0 && ctx.oe_bad_after == 0,
        "[{=str}] {=str}: PIO2 DBG_PADOE all four set during each frame and 0 after it ({=u32} exchanges so far: {=u32} not set during, {=u32} not 0 after)",
        label,
        ctx.oe_samples,
        ctx.oe_bad_during,
        ctx.oe_bad_after
    );

    // ---- 2.8 control: 3 s with the lines high, then 5 frames through check 2's classifier.
    Timer::after_millis(3000).await;
    ctx.obs.spike_check = false;
    let mut live = 0;
    let mut foreign = 0;
    let mut ctl_x = [0u32; LINES];
    let mut next = Instant::now();
    for _ in 0..5 {
        Timer::at(next).await;
        next += Duration::from_micros(1000);
        let x = ctx.shot().await.r;
        live += x.states.iter().filter(|&&st| st == LineState::Live).count() as u32;
        foreign += x
            .captures
            .iter()
            .filter(|c| **c == LineCapture::Foreign)
            .count() as u32;
        let a = find_anchor(&ctx.obs, None, rate);
        for l in 0..LINES {
            let ln = &ctx.obs.l[l];
            let ex = classify(
                &ln.e[..ln.n],
                &x.captures[l],
                rate,
                a.as_ref().map(|a| &a[..]),
            );
            ctl_x[l] += u32::from(!matches!(ex.cls, Cls::G | Cls::Q));
        }
    }
    ctx.obs.mark();
    ctx.obs.spike_check = true;
    let ctl: u32 = ctl_x.iter().sum();
    check!(
        ctx.fails,
        ctl >= 1,
        "[{=str}] {=str}: board control of check 2: 5 exchanges after 3 s of high lines, through check 2's classifier, hold {=u32} line-exchanges that are neither G nor Q (1 or more); per line {=[?]}; Foreign captures {=u32}, line-frames Live {=u32} (reported)",
        label,
        ctl,
        &ctl_x[..],
        foreign,
        live
    );

    // ---- 3: summary of the spike watch for this rate (the 2.8 control is excluded)
    ctx.obs.settle();
    check!(
        ctx.fails,
        ctx.obs.sp_ours == 0
            && ctx.obs.off_grid == 0
            && ctx.obs.shorts_lost == 0
            && ctx.obs.ring_over == 0,
        "[{=str}] {=str}: addendum C, from the run's start: runs under 300 ns attributed to us (both edges within 200 ns of our drive interval or of a break edge) {=u32}; edges inside our drive interval more than 53 ns off our bit grid {=u32}; reported (an edge outside those) {=u32}; all runs under 300 ns {=u32}, not judged (queue full) {=u32}; shortest run {=u32} ns; ring overflows {=u32}",
        label,
        ctx.obs.sp_ours,
        ctx.obs.off_grid,
        ctx.obs.sp_rep,
        ctx.obs.spikes,
        ctx.obs.shorts_lost,
        ns(ctx.obs.min_run),
        ctx.obs.ring_over
    );

    // ---- 6: release, then the cancel sweep on the stream check 6 left running
    check_release(ctx, rate, label).await;
    cancel_sweep(ctx, rate, label).await;
}

async fn check_release(ctx: &mut Ctx, rate: Rate, label: &str) {
    let (ls, rel, lam) = (last_sample_sys(rate), release_sys(rate), lam_sys(rate));

    // ---- S1, warm: a fresh start(), STOP at 1 kHz until every line is past its arming.
    start_checked(ctx, label).await;
    let sp0 = ctx.obs.sp_ours + ctx.obs.off_grid;
    let (armed, mut sp_min, last) = past_arming(ctx, label, "check 6 S1").await;
    ctx.obs.stage = "check 6 S1";
    // One exchange dropped mid-capture, 1 ms or more after the previous frame.
    ctx.obs.mark();
    for ln in &mut ctx.obs.l {
        ln.over = 0;
    }
    // No processing between this wake and the push, unlike in `shot`: 5 us instead of 1.
    Timer::at(ctx.t_push + Duration::from_micros(1005)).await;
    let (t_bp, t_ap, t_drop, ready, i_ap);
    {
        let drv = ctx.drv();
        let mut fut = pin!(drv.exchange(STOP4));
        t_bp = cyc();
        ready = poll_fn(|cx| Poll::Ready(fut.as_mut().poll(cx).is_ready())).await;
        t_ap = cyc();
        i_ap = Instant::now();
        // Midway between the latest possible release and the earliest possible last sample.
        let mid = (t_ap.wrapping_sub(t_bp) + lam + rel + ls) / 2;
        while cyc().wrapping_sub(t_bp) < mid {}
        t_drop = cyc();
    }
    let ds = t_drop.wrapping_sub(t_bp);
    // Mid-capture for certain: after the latest possible release (the push at t_ap at the latest)
    // and before the earliest possible last sample (the push at t_bp at the earliest).
    let mid_ok = !ready && ds > t_ap.wrapping_sub(t_bp) + lam + rel && ds < ls;
    let refused = match ctx.drv.take().map(PioDshot::release) {
        Some(Err(d)) => {
            ctx.drv = Some(d);
            true
        }
        Some(Ok(parts)) => {
            ctx.drv = PioDshot::new(parts, rate, CLK_HZ).ok();
            false
        }
        None => false,
    };
    // 1 ms. The dropped exchange's window: our frame whole and nothing of ours besides it.
    Timer::at(i_ap + Duration::from_micros(1001)).await;
    ctx.obs.sync();
    let a_drop = find_anchor(&ctx.obs, None, rate);
    if let Some(a) = &a_drop {
        ctx.obs.register(a, rate);
    }
    ctx.obs.settle();
    let drop_ok: [bool; LINES] = core::array::from_fn(|l| {
        let ln = &ctx.obs.l[l];
        ln.over == 0 && dropped_ok(&ln.e[..ln.n], rate)
    });
    for l in 0..LINES {
        if !drop_ok[l] {
            dump_window(&ctx.obs, l, "check 6 S1 dropped exchange", 0);
        }
    }
    for ln in &mut ctx.obs.l {
        ln.over = 0;
    }
    // The next exchange finishes the dropped capture and returns its own frame's result.
    ctx.no_oe_during = true;
    let s = ctx.shot().await;
    let a_own = find_anchor(&ctx.obs, None, rate);
    let own: [bool; LINES] = core::array::from_fn(|l| {
        let ln = &ctx.obs.l[l];
        ln.over == 0 && own_ok(&ln.e[..ln.n], &s.r.captures[l], rate)
    });
    for l in 0..LINES {
        if !own[l] {
            dump_window(&ctx.obs, l, "check 6 S1 next exchange", 0);
        }
    }
    let f_drop = a_drop.map(|a| a[0] >> 1);
    for (p, q) in [(last, f_drop), (f_drop, a_own.map(|a| a[0] >> 1))] {
        sp_min = match (p, q) {
            (Some(p), Some(q)) => sp_min.min(q.saturating_sub(p)),
            _ => 0,
        };
    }
    let parts = match ctx.drv.take().map(PioDshot::release) {
        Some(Ok(p)) => Some(p),
        Some(Err(d)) => {
            ctx.drv = Some(d);
            None
        }
        None => None,
    };
    let accepted = parts.is_some();
    let idle = padoe() == 0 && pads_ok();
    ctx.obs.settle();
    let sp = ctx.obs.sp_ours + ctx.obs.off_grid - sp0;
    defmt::info!(
        "[INFO] diag check 6 S1: next exchange {}; (levels G39..36, PIO2 PADOE, SIO OE, pads ok) after the accepted release {=?}",
        s.r,
        lines_now()
    );
    check!(
        ctx.fails,
        armed && sp_min >= 150_000,
        "[{=str}] {=str}: check 6 S1 entry: a fresh start(), every line past its arming, every frame 1 ms or more after the previous (shortest {=u64} us, observer)",
        label,
        sp_min / 150
    );
    check!(
        ctx.fails,
        mid_ok
            && refused
            && own.iter().all(|&o| o)
            && drop_ok.iter().all(|&o| o)
            && accepted
            && idle
            && sp == 0,
        "[{=str}] {=str}: check 6 S1: an exchange dropped mid-capture for certain {=bool} ({=u32} ns after its first poll began, which took {=u32} ns; our release {=u32} ns, the last sample {=u32} ns after our first falling edge); release() refused {=bool}; 1 ms later the next exchange returns its own frame's result per line {=[?]} (driver = observer, a good reply, one whole frame in its window); the dropped exchange's frame whole, nothing else of ours, per line {=[?]}; release() accepted {=bool}; then DBG_PADOE 0 and pads pulled up {=bool}; runs under 300 ns attributed to us or off-grid edges in S1 {=u32}",
        label,
        mid_ok,
        ns(u64::from(ds)),
        ns(u64::from(t_ap.wrapping_sub(t_bp))),
        ns(u64::from(rel)),
        ns(u64::from(ls)),
        refused,
        &own[..],
        &drop_ok[..],
        accepted,
        idle,
        sp
    );

    // ---- S2, parked: no frame for 3.5 s (the controllers reset after about 0.51 s and sit
    // silent in their bootloaders), with the driver built.
    ctx.obs.stage = "check 6 S2";
    if let Some(p) = parts {
        match PioDshot::new(p, rate, CLK_HZ) {
            Ok(d) => ctx.drv = Some(d),
            Err(e) => {
                defmt::error!("[FAIL] rebuild after check 6 S1: {}", e.reason);
                hil_common::exit(ctx.fails + 1)
            }
        }
    }
    Timer::after_millis(3400).await;
    ctx.obs.mark();
    Timer::after_millis(100).await;
    ctx.obs.sync();
    let pre = ctx.obs.edges_in_window();
    let run = pre == 0;
    check!(
        ctx.fails,
        run,
        "[{=str}] {=str}: check 6 S2 precondition: 3.5 s without frames, no edge on any line in the last 100 ms ({=u32}); otherwise S2 not run",
        label,
        pre
    );
    if run {
        // The window runs on from the precondition's 100 ms.
        let mut parts = match ctx.drv.take().map(PioDshot::release) {
            Some(Ok(p)) => Some(p),
            Some(Err(d)) => {
                ctx.drv = Some(d);
                None
            }
            None => None,
        };
        let rel_ok = parts.is_some() && padoe() == 0 && pads_ok();
        let (mut built, mut released, mut idle_bad) = (0u32, 0u32, 0u32);
        let mut first_edge_iter = u32::MAX;
        for it in 0..100u32 {
            ctx.obs.sync();
            if first_edge_iter == u32::MAX && ctx.obs.edges_in_window() > 0 {
                first_edge_iter = it;
            }
            let Some(p) = parts.take() else { break };
            match PioDshot::new(p, rate, CLK_HZ) {
                Ok(d) => {
                    built += 1;
                    idle_bad += u32::from(padoe() != 0 || !pads_ok());
                    match d.release() {
                        Ok(p) => {
                            released += 1;
                            idle_bad += u32::from(padoe() != 0 || !pads_ok());
                            parts = Some(p);
                        }
                        Err(d) => {
                            ctx.drv = Some(d);
                            break;
                        }
                    }
                }
                Err(e) => {
                    parts = Some(e.parts);
                    break;
                }
            }
        }
        Timer::after_millis(100).await;
        ctx.obs.sync();
        let edges = ctx.obs.edges_in_window();
        if edges != 0 {
            defmt::info!(
                "[INFO] diag check 6 S2: first edge seen before cycle {=u32}; lines now {=?}",
                first_edge_iter,
                lines_now()
            );
            for l in 0..LINES {
                if ctx.obs.l[l].n > 0 {
                    dump_window(&ctx.obs, l, "check 6 S2", 0);
                }
            }
        }
        check!(
            ctx.fails,
            rel_ok && built == 100 && released == 100 && idle_bad == 0 && edges == 0,
            "[{=str}] {=str}: check 6 S2: release() of the built driver accepted, then DBG_PADOE 0 and pads pulled up {=bool}; 100 times built and released ({=u32}), idle each time ({=u32} not); edges from the precondition's 100 ms through the cycles and 100 ms after {=u32}",
            label,
            rel_ok,
            released,
            idle_bad,
            edges
        );
        if let Some(p) = parts {
            ctx.drv = PioDshot::new(p, rate, CLK_HZ).ok();
        }
    }
    // The next start() in the same observer session: its break on all four makes S2's 0 edges count.
    let brk = start_checked(ctx, label).await;
    check!(
        ctx.fails,
        run && brk,
        "[{=str}] {=str}: check 6 S2 valid: S2 run {=bool}, and the next start() in the same observer session showed its break on all four lines (25 ms or more together) {=bool}",
        label,
        run,
        brk
    );
    let mut streak = [0u32; LINES];
    let mut reach = [0u32; LINES];
    let mut other_live = [0u32; LINES];
    let (t0, mut k) = (Instant::now(), 0u32);
    while streak.iter().any(|&g| g < 100) && t0.elapsed() < Duration::from_millis(3000) {
        Timer::at(ctx.t_push + Duration::from_micros(1001)).await;
        let x = ctx.shot().await.r;
        k += 1;
        for l in 0..LINES {
            if streak[l] >= 100 {
                continue;
            }
            if stopped(&x.captures[l]) {
                streak[l] += 1;
                if streak[l] == 100 {
                    reach[l] = k;
                }
            } else {
                streak[l] = 0;
                if x.states[l] == LineState::Live && x.captures[l] != LineCapture::Silent {
                    other_live[l] += 1;
                }
            }
        }
    }
    check!(
        ctx.fails,
        streak.iter().all(|&g| g >= 100) && other_live.iter().all(|&o| o == 0),
        "[{=str}] {=str}: check 6 S2: after start(), 100 consecutive good replies on every line (reached at frame {=[?]} from start(); other captures on a Live line {=u32})",
        label,
        &reach[..],
        other_live.iter().sum::<u32>()
    );
}

/// Check 2's frame spacing from the observer (first falling edges) and the pacing it drives.
struct Periods {
    f_prev: Option<u64>,
    c_prev: u32,
    /// The largest drop seen in the delay from the wake to the push (cycles).
    dl_min: i64,
    n: u32,
    unmeasured: u32,
    min: u64,
    max: u64,
    under: u32,
    over1250: u32,
    over2000: u32,
    f_first: Option<u64>,
    f_last: u64,
}

impl Periods {
    const NEW: Periods = Periods {
        f_prev: None,
        c_prev: 0,
        dl_min: 0,
        n: 0,
        unmeasured: 0,
        min: u64::MAX,
        max: 0,
        under: 0,
        over1250: 0,
        over2000: 0,
        f_first: None,
        f_last: 0,
    };

    /// Frame `k` (0 first): its first falling edge `f` (observer) and the CPU cycle `c` of the
    /// wake before it. Returns the cycle at which the next wake may come.
    fn frame(&mut self, k: u32, f: Option<u64>, c: u32) -> u32 {
        match (self.f_prev, f) {
            (Some(p), Some(f)) => {
                let d = f.saturating_sub(p);
                self.n += 1;
                self.min = self.min.min(d);
                self.max = self.max.max(d);
                self.under += u32::from(d < 150_000);
                self.over1250 += u32::from(d > 187_500);
                self.over2000 += u32::from(d > 300_000);
                self.dl_min = self
                    .dl_min
                    .min(d as i64 - i64::from(c.wrapping_sub(self.c_prev)));
            }
            _ if k > 0 => self.unmeasured += 1,
            _ => {}
        }
        if k == 0 {
            self.f_first = f;
        }
        if let Some(f) = f {
            self.f_last = f;
        }
        self.f_prev = f;
        self.c_prev = c;
        // The fixed margin of 25 us (seventh amendment).
        c.wrapping_add(150_000 + 3750)
    }

    fn total(&self) -> u64 {
        self.f_last
            .saturating_sub(self.f_first.unwrap_or(self.f_last))
    }

    fn valid(&self) -> bool {
        self.n == 3000
            && self.unmeasured == 0
            && self.under == 0
            && self.over2000 == 0
            && self.over1250 <= 3
            && self.total().abs_diff(461_250_000) <= 9_225_000
    }
}

/// Waits until the CPU cycle counter reaches `c`: a timer for the most of it, then a spin.
async fn wait_cyc(c: u32) {
    let left = c.wrapping_sub(cyc()) as i32;
    if left > 50 * 150 {
        Timer::after_micros(((left - 30 * 150) / 150) as u64).await;
    }
    while (c.wrapping_sub(cyc()) as i32) > 0 {}
}

/// CPU cycles (DWT CYCCNT, clk_sys).
fn cyc() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}

/// `n` PIO cycles of the driver, in system cycles.
fn pio_sys(n: u32, rate: Rate) -> u32 {
    (u64::from(n) * bit4(rate) as u64 / 160) as u32
}

/// From our first falling edge to the capture's last sample, system cycles.
fn last_sample_sys(rate: Rate) -> u32 {
    pio_sys(
        program::FRAME_CYCLES + program::RELEASE_TO_FIRST_SAMPLE + 4 * (program::samples(rate) - 1),
        rate,
    )
}

/// From our first falling edge to the release of the lines, system cycles.
fn release_sys(rate: Rate) -> u32 {
    pio_sys(program::FRAME_CYCLES, rate)
}

/// The longest from the push to our first falling edge: the status read every 2 PIO cycles, 4
/// more to the edge, 2 system cycles of input synchroniser.
fn lam_sys(rate: Rate) -> u32 {
    pio_sys(program::STATUS_TO_FIRST_EDGE + 2, rate) + 2
}

/// Whole STOP frames in a window, not overlapping.
fn frames_in(e: &[u64], rate: Rate) -> u32 {
    let (mut n, mut i) = (0, 0);
    while let Some(j) = find_frame(&e[i..], rate) {
        n += 1;
        i += j + 32;
    }
    n
}

/// The start of the second whole frame in a window.
fn second_frame(e: &[u64], rate: Rate) -> Option<usize> {
    let k = find_frame(e, rate)? + 32;
    find_frame(&e[k..], rate).map(|j| k + j)
}

/// Check 6 "own" for one line: one whole frame in the exchange's window, and the driver and the
/// observer agree on a good reply after it.
fn own_ok(e: &[u64], c: &LineCapture, rate: Rate) -> bool {
    stopped(c) && frames_in(e, rate) == 1 && classify(e, c, rate, None).cls == Cls::G
}

/// A dropped exchange's window on one line: our frame whole and nothing of ours besides it (the
/// stopped reply after it, or nothing, as the controller answered).
fn dropped_ok(e: &[u64], rate: Rate) -> bool {
    let Ok(st) = decode_reply(STOPPED_WORD) else {
        return false;
    };
    frames_in(e, rate) == 1
        && (classify(e, &LineCapture::Reply(st), rate, None).cls == Cls::G
            || classify(e, &LineCapture::Silent, rate, None).cls == Cls::Q)
}

/// STOP, each frame 1001 us or more after the previous push, until every line is past its arming
/// (sixth amendment): after its first good reply, a block of 250 or more exchanges without one,
/// then good replies unbroken for 1500 ms; 8 s at most. Returns whether all four got there, the
/// shortest frame spacing the observer saw (system cycles) and the observer time of the last
/// frame.
async fn past_arming(ctx: &mut Ctx, label: &str, what: &str) -> (bool, u64, Option<u64>) {
    let rate = ctx.drv().rate();
    let t0 = Instant::now();
    let (mut live, mut blocked, mut nongood) = ([false; LINES], [false; LINES], [0u32; LINES]);
    let mut since = [None::<Instant>; LINES];
    let mut done = [0u32; LINES];
    let (mut k, mut sp_min, mut prev) = (0u32, u64::MAX, None::<u64>);
    while done.contains(&0) && t0.elapsed() < Duration::from_millis(8000) {
        Timer::at(ctx.t_push + Duration::from_micros(1001)).await;
        let s = ctx.shot().await;
        k += 1;
        let f = find_anchor(&ctx.obs, None, rate).map(|a| a[0] >> 1);
        if let (Some(p), Some(q)) = (prev, f) {
            sp_min = sp_min.min(q.saturating_sub(p));
        }
        prev = f;
        let now = Instant::now();
        for l in 0..LINES {
            if done[l] != 0 {
                continue;
            }
            if stopped(&s.r.captures[l]) {
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
    defmt::info!(
        "[STAT] {=str} {=str}: past arming (after the first good reply a block of 250 or more without one, then 1500 ms of good replies) at ms {=[?]} of this stream (0: not within 8 s), {=u32} frames",
        label,
        what,
        &done[..],
        k
    );
    (!done.contains(&0), sp_min, prev)
}

/// What a window of the cancel sweep must show.
#[derive(Clone, Copy)]
enum Pend {
    /// The exchange after a cancel, with the driver's captures.
    Own([LineCapture; LINES]),
    /// A cancelled exchange; `acc`: the window's edge counts when `release()` was accepted.
    Dropped {
        pushed: bool,
        acc: Option<[usize; LINES]>,
    },
    /// A cancelled exchange and the one cancelled during its drain (frame seen or not).
    Double { acc: Option<[usize; LINES]> },
}

struct Sweep {
    pushes: u32,
    frames: [u32; LINES],
    own_bad: u32,
    drop_bad: u32,
    unsent: u32,
    shown: u32,
    prev: Option<u64>,
    sp_min: u64,
    sp_max: u64,
}

/// Judges the window collected since the last mark (cancel sweep), then empties it.
fn close_window(ctx: &mut Ctx, rate: Rate, pend: Pend, d: i32, sw: &mut Sweep) {
    let obs = &mut ctx.obs;
    obs.sync();
    if let Some(a) = find_anchor(obs, None, rate) {
        obs.register(&a, rate);
        let f = a[0] >> 1;
        if let Some(p) = sw.prev {
            let g = f.saturating_sub(p);
            sw.sp_min = sw.sp_min.min(g);
            sw.sp_max = sw.sp_max.max(g);
        }
        sw.prev = Some(f);
    }
    if let Pend::Double { .. } = pend {
        let a2 = obs.l.iter().find_map(|ln| {
            let e = &ln.e[..ln.n];
            second_frame(e, rate).map(|j| core::array::from_fn::<u64, 32, _>(|k| e[j + k]))
        });
        if let Some(a2) = a2 {
            obs.register(&a2, rate);
        }
    }
    if let Pend::Own(c) = pend {
        obs.caps = Some(c);
    }
    obs.settle();
    for l in 0..LINES {
        let ln = &obs.l[l];
        let e = &ln.e[..ln.n];
        sw.frames[l] += frames_in(e, rate);
        let (bad, what) = match pend {
            Pend::Own(c) => (
                ln.over > 0 || !own_ok(e, &c[l], rate),
                "cancel sweep, the exchange after a cancel",
            ),
            Pend::Dropped { pushed: false, .. } => (
                ln.n > 0 || ln.over > 0,
                "cancel sweep, an exchange never polled",
            ),
            Pend::Dropped { pushed: true, acc } => (
                ln.over > 0 || !dropped_ok(e, rate) || acc.is_some_and(|n| n[l] != ln.n),
                "cancel sweep, a cancelled exchange",
            ),
            Pend::Double { acc } => {
                let ok = match second_frame(e, rate) {
                    Some(j) => dropped_ok(&e[..j], rate) && dropped_ok(&e[j..], rate),
                    None => dropped_ok(e, rate),
                };
                (
                    ln.over > 0
                        || !ok
                        || frames_in(e, rate) > 2
                        || acc.is_some_and(|n| n[l] != ln.n),
                    "cancel sweep, two cancelled exchanges",
                )
            }
        };
        match pend {
            Pend::Own(_) => sw.own_bad += u32::from(bad),
            Pend::Dropped { pushed: false, .. } => sw.unsent += u32::from(bad),
            Pend::Dropped { pushed: true, .. } | Pend::Double { .. } => {
                sw.drop_bad += u32::from(bad)
            }
        }
        if bad && sw.shown < 6 {
            sw.shown += 1;
            let cap = match pend {
                Pend::Own(c) => Some(c[l]),
                Pend::Dropped { .. } | Pend::Double { .. } => None,
            };
            defmt::info!(
                "[INFO] diag {=str} {=str}: dropped {=i32} us after its first poll; driver {}",
                what,
                NAMES[l],
                d,
                cap
            );
            dump_window(obs, l, what, 0);
        }
    }
    for ln in &mut obs.l {
        ln.over = 0;
    }
    obs.mark();
}

/// The cancel sweep (design invariant 3.4, fifth amendment item 3 (ii)): on a stream past arming,
/// an exchange dropped at a delay swept in 1 us steps from before its push (never polled) to after
/// its capture window, `release()` after each (rebuilt when accepted), then 1 ms later an exchange
/// that must return its own result; 1000 cancels or more.
async fn cancel_sweep(ctx: &mut Ctx, rate: Rate, label: &str) {
    let (armed, _, _) = past_arming(ctx, label, "cancel sweep").await;
    check!(
        ctx.fails,
        armed,
        "[{=str}] {=str}: cancel sweep entry: the stream after check 6's start() past arming on all four",
        label
    );
    ctx.obs.stage = "cancel sweep";
    let (ls, lam) = (last_sample_sys(rate), lam_sys(rate));
    let (d_min, d_max) = (-2i32, (ls / 150) as i32 + 8);
    let passes = 1000u32.div_ceil((d_max - d_min + 1) as u32);
    let sp0 = ctx.obs.sp_ours + ctx.obs.off_grid;
    let mut sw = Sweep {
        pushes: 0,
        frames: [0; LINES],
        own_bad: 0,
        drop_bad: 0,
        unsent: 0,
        shown: 0,
        prev: None,
        sp_min: u64::MAX,
        sp_max: 0,
    };
    let (mut cancels, mut refused, mut accepted, mut acc_unpolled) = (0u32, 0u32, 0u32, 0u32);
    let (mut acc_early, mut acc_unseen, mut ref_late, mut between) = (0u32, 0u32, 0u32, 0u32);
    let (mut ready_n, mut oe_bad, mut pads_bad) = (0u32, 0u32, 0u32);
    let (mut acc_first, mut ref_last) = (u32::MAX, 0u32);
    let (mut poll_min, mut poll_max) = (u32::MAX, 0u32);
    ctx.obs.mark();
    for ln in &mut ctx.obs.l {
        ln.over = 0;
    }
    let mut next = ctx.t_push + Duration::from_micros(1001);
    let mut own: Option<([LineCapture; LINES], i32)> = None;
    for _ in 0..passes {
        for d in d_min..=d_max {
            Timer::at(next).await;
            match own.take() {
                Some((c, dd)) => close_window(ctx, rate, Pend::Own(c), dd, &mut sw),
                None => ctx.obs.mark(),
            }
            cancels += 1;
            let polled = d >= 0;
            let (t_bp, t_ap, t_drop, ready, i_ap);
            {
                let drv = ctx.drv();
                let mut fut = pin!(drv.exchange(STOP4));
                t_bp = cyc();
                ready = polled && poll_fn(|cx| Poll::Ready(fut.as_mut().poll(cx).is_ready())).await;
                t_ap = cyc();
                i_ap = Instant::now();
                while polled && cyc().wrapping_sub(t_ap) < d as u32 * 150 {}
                t_drop = cyc();
            }
            sw.pushes += u32::from(polled);
            ready_n += u32::from(ready);
            if polled {
                poll_min = poll_min.min(t_ap.wrapping_sub(t_bp));
                poll_max = poll_max.max(t_ap.wrapping_sub(t_bp));
            }
            let ds = t_drop.wrapping_sub(t_bp);
            // From here on the capture has certainly ended and been read by the DMA.
            let sure = t_ap.wrapping_sub(t_bp) + lam + ls + 450;
            between += u32::from(polled && (ls..sure).contains(&ds));
            let mut acc = None;
            match ctx.drv.take().map(PioDshot::release) {
                Some(Ok(parts)) => {
                    accepted += 1;
                    acc_unpolled += u32::from(!polled);
                    ctx.obs.sync();
                    acc = Some(core::array::from_fn(|l| ctx.obs.l[l].n));
                    if polled {
                        acc_first = acc_first.min(ds);
                        acc_early += u32::from(ds < ls);
                        acc_unseen += u32::from(!(0..LINES).all(|l| {
                            let ln = &ctx.obs.l[l];
                            frames_in(&ln.e[..ln.n], rate) == 1
                        }));
                    }
                    pads_bad += u32::from(padoe() != 0 || !pads_ok());
                    match PioDshot::new(parts, rate, CLK_HZ) {
                        Ok(dv) => ctx.drv = Some(dv),
                        Err(e) => {
                            defmt::error!("[FAIL] rebuild in the cancel sweep: {}", e.reason);
                            hil_common::exit(ctx.fails + 1)
                        }
                    }
                }
                Some(Err(dv)) => {
                    refused += 1;
                    ctx.drv = Some(dv);
                    if polled {
                        ref_last = ref_last.max(ds);
                    }
                    ref_late += u32::from(!polled || ds >= sure);
                }
                None => hil_common::exit(ctx.fails + 1),
            }
            next = i_ap + Duration::from_micros(1001);
            Timer::at(next).await;
            close_window(
                ctx,
                rate,
                Pend::Dropped {
                    pushed: polled,
                    acc,
                },
                d,
                &mut sw,
            );
            oe_bad += u32::from(padoe() != 0);
            let x = ctx.drv().exchange(STOP4).await;
            let i_ret = Instant::now();
            sw.pushes += 1;
            oe_bad += u32::from(padoe() != 0);
            own = Some((x.captures, d));
            // The return follows the push by the frame and the window or more.
            next = i_ret + Duration::from_micros(1001) - Duration::from_micros(u64::from(ls / 150));
        }
    }
    Timer::at(next).await;
    if let Some((c, dd)) = own.take() {
        close_window(ctx, rate, Pend::Own(c), dd, &mut sw);
    }
    ctx.obs.settle();
    let sp = ctx.obs.sp_ours + ctx.obs.off_grid - sp0;
    let acc_first_ns = if acc_first == u32::MAX {
        0
    } else {
        ns(u64::from(acc_first))
    };
    check!(
        ctx.fails,
        cancels >= 1000 && acc_early == 0 && acc_unseen == 0 && ref_late == 0 && ready_n == 0,
        "[{=str}] {=str}: cancel sweep: {=u32} cancels, each exchange dropped {=i32} to {=i32} us after its first poll returned (the push is in that poll; it took {=u32} to {=u32} ns), in 1 us steps ({=u32} passes; below 0 never polled); release() after each: refused {=u32}, accepted {=u32} ({=u32} never polled); accepted before the capture's last sample could have been taken {=u32}; accepted without our frame already seen whole on all four {=u32}; refused with nothing in flight (never polled, or dropped 3 us or more after the latest possible last sample) {=u32}; dropped in the band between (either allowed) {=u32}; earliest drop accepted {=u32} ns, latest refused {=u32} ns after the first poll began (the last sample {=u32} ns after our first falling edge); first poll already ready {=u32}",
        label,
        cancels,
        d_min,
        d_max,
        ns(u64::from(poll_min.min(poll_max))),
        ns(u64::from(poll_max)),
        passes,
        refused,
        accepted,
        acc_unpolled,
        acc_early,
        acc_unseen,
        ref_late,
        between,
        acc_first_ns,
        ns(u64::from(ref_last)),
        ns(u64::from(ls)),
        ready_n
    );
    check!(
        ctx.fails,
        sw.frames.iter().all(|&f| f == sw.pushes)
            && sw.own_bad == 0
            && sw.drop_bad == 0
            && sw.unsent == 0
            && sp == 0
            && oe_bad == 0
            && pads_bad == 0
            && sw.sp_min >= 150_000,
        "[{=str}] {=str}: cancel sweep: whole frames the observer saw per line {=[?]} = pushes {=u32} (no lost, extra or partial frame); line-exchanges after a cancel not returning their own result {=u32}; cancelled exchanges' line-windows with anything but our whole frame (and the reply), or edges after an accepted release {=u32}; line-windows with edges where nothing was pushed {=u32}; runs under 300 ns attributed to us or off-grid edges {=u32}; DBG_PADOE not 0 after a cancel or an exchange {=u32}; pads not pulled up after an accepted release {=u32}; frame spacing {=u64} to {=u64} us (1000 or more)",
        label,
        &sw.frames[..],
        sw.pushes,
        sw.own_bad,
        sw.drop_bad,
        sw.unsent,
        sp,
        oe_bad,
        pads_bad,
        sw.sp_min / 150,
        sw.sp_max / 150
    );
    defmt::info!(
        "[STAT] {=str}: cancel sweep frame spacing {=u64} to {=u64} us (observer, first falling edge to first falling edge)",
        label,
        sw.sp_min / 150,
        sw.sp_max / 150
    );

    // ---- Cancels during a drain: an exchange dropped just after our release (most of its words
    // owed, release() must refuse), then the next one, which must first drain those words,
    // busy-polled as its waker would have it polled and dropped `d` us after its first poll began,
    // swept in 1 us steps across the drain and its own frame and capture; release() after it; 1 ms
    // later an exchange that must return its own result. The second exchange's push is witnessed by
    // DBG_PADOE after each poll (our frame drives the pads); frames seen must equal pushes witnessed.
    ctx.obs.stage = "cancel sweep, cancels during a drain";
    let rel = release_sys(rate);
    let db_max = (2 * ls / 150) as i32 + 100;
    let dpasses = 300u32.div_ceil((db_max + 1) as u32);
    let sp0 = ctx.obs.sp_ours + ctx.obs.off_grid;
    let mut dw = Sweep {
        pushes: 0,
        frames: [0; LINES],
        own_bad: 0,
        drop_bad: 0,
        unsent: 0,
        shown: 0,
        prev: None,
        sp_min: u64::MAX,
        sp_max: 0,
    };
    let (mut dbl, mut a_bad, mut b_ref, mut b_acc, mut b_acc_early, mut b_ref_late) =
        (0u32, 0u32, 0u32, 0u32, 0u32, 0u32);
    let (mut b_wit, mut b_ready, mut oe_bad, mut pads_bad) = (0u32, 0u32, 0u32, 0u32);
    for _ in 0..dpasses {
        for d in 0..=db_max {
            Timer::at(next).await;
            match own.take() {
                Some((c, dd)) => close_window(ctx, rate, Pend::Own(c), dd, &mut dw),
                None => ctx.obs.mark(),
            }
            dbl += 1;
            let (a_bp, a_ap, a_drop, a_ready);
            {
                let drv = ctx.drv();
                let mut fut = pin!(drv.exchange(STOP4));
                a_bp = cyc();
                a_ready = poll_fn(|cx| Poll::Ready(fut.as_mut().poll(cx).is_ready())).await;
                a_ap = cyc();
                while cyc().wrapping_sub(a_ap) < lam + rel + 450 {}
                a_drop = cyc();
            }
            dw.pushes += 1;
            let ads = a_drop.wrapping_sub(a_bp);
            let a_mid = !a_ready && ads > a_ap.wrapping_sub(a_bp) + lam + rel && ads < ls;
            let a_refused = match ctx.drv.take().map(PioDshot::release) {
                Some(Err(dv)) => {
                    ctx.drv = Some(dv);
                    true
                }
                Some(Ok(parts)) => {
                    ctx.drv = PioDshot::new(parts, rate, CLK_HZ).ok();
                    false
                }
                None => false,
            };
            a_bad += u32::from(!(a_mid && a_refused));
            // The second exchange 250 us or more after the first push (M1's shortest tested period).
            while cyc().wrapping_sub(a_ap) < 250 * 150 {}
            let (b0, b_drop, seen, ready);
            {
                let drv = ctx.drv();
                let mut fut = pin!(drv.exchange(STOP4));
                b0 = cyc();
                let mut s = None;
                ready = loop {
                    let r = poll_fn(|cx| Poll::Ready(fut.as_mut().poll(cx).is_ready())).await;
                    if s.is_none() && padoe() != 0 {
                        s = Some(cyc());
                    }
                    if r || cyc().wrapping_sub(b0) >= d as u32 * 150 {
                        break r;
                    }
                };
                b_drop = cyc();
                seen = s;
            }
            b_wit += u32::from(seen.is_some());
            b_ready += u32::from(ready);
            dw.pushes += u32::from(seen.is_some());
            let mut acc = None;
            match ctx.drv.take().map(PioDshot::release) {
                Some(Ok(parts)) => {
                    b_acc += 1;
                    ctx.obs.sync();
                    acc = Some(core::array::from_fn(|l| ctx.obs.l[l].n));
                    // The pads are driven only during our frame: its first falling edge is no
                    // earlier than `seen` less the frame, its last sample `ls` after that edge.
                    if let Some(s) = seen {
                        b_acc_early += u32::from(b_drop.wrapping_sub(s) + rel < ls);
                    }
                    pads_bad += u32::from(padoe() != 0 || !pads_ok());
                    match PioDshot::new(parts, rate, CLK_HZ) {
                        Ok(dv) => ctx.drv = Some(dv),
                        Err(e) => {
                            defmt::error!("[FAIL] rebuild in the cancel sweep: {}", e.reason);
                            hil_common::exit(ctx.fails + 1)
                        }
                    }
                }
                Some(Err(dv)) => {
                    b_ref += 1;
                    ctx.drv = Some(dv);
                    b_ref_late += u32::from(
                        ready || seen.is_some_and(|s| b_drop.wrapping_sub(s) >= ls + 450),
                    );
                }
                None => hil_common::exit(ctx.fails + 1),
            }
            next = Instant::now() + Duration::from_micros(1001);
            Timer::at(next).await;
            close_window(ctx, rate, Pend::Double { acc }, d, &mut dw);
            oe_bad += u32::from(padoe() != 0);
            let x = ctx.drv().exchange(STOP4).await;
            let i_ret = Instant::now();
            dw.pushes += 1;
            oe_bad += u32::from(padoe() != 0);
            own = Some((x.captures, d));
            next = i_ret + Duration::from_micros(1001) - Duration::from_micros(u64::from(ls / 150));
        }
    }
    Timer::at(next).await;
    if let Some((c, dd)) = own.take() {
        close_window(ctx, rate, Pend::Own(c), dd, &mut dw);
    }
    ctx.obs.settle();
    let sp = ctx.obs.sp_ours + ctx.obs.off_grid - sp0;
    check!(
        ctx.fails,
        dbl >= 300 && a_bad == 0 && b_acc_early == 0 && b_ref_late == 0,
        "[{=str}] {=str}: cancels during a drain: {=u32}; the first exchange dropped mid-capture for certain and refused, failures {=u32}; the second dropped 0 to {=i32} us after its first poll began, 1 us steps ({=u32} passes): release() refused {=u32}, accepted {=u32}; accepted before its last sample could have been taken {=u32}; refused with nothing in flight (completed, or dropped 3 us or more after its last sample) {=u32}; its push witnessed {=u32}, completed before the drop {=u32}",
        label,
        dbl,
        a_bad,
        db_max,
        dpasses,
        b_ref,
        b_acc,
        b_acc_early,
        b_ref_late,
        b_wit,
        b_ready
    );
    check!(
        ctx.fails,
        dw.frames.iter().all(|&f| f == dw.pushes)
            && dw.own_bad == 0
            && dw.drop_bad == 0
            && sp == 0
            && oe_bad == 0
            && pads_bad == 0
            && dw.sp_min >= 150_000,
        "[{=str}] {=str}: cancels during a drain: whole frames the observer saw per line {=[?]} = pushes {=u32} (the second exchange's counted when witnessed); line-exchanges after a double cancel not returning their own result {=u32}; line-windows of the two cancelled exchanges with anything but whole frames and replies, or edges after an accepted release {=u32}; runs under 300 ns attributed to us or off-grid edges {=u32}; DBG_PADOE not 0 after a cancel or an exchange {=u32}; pads not pulled up after an accepted release {=u32}; spacing of the first frames {=u64} to {=u64} us (1000 or more)",
        label,
        &dw.frames[..],
        dw.pushes,
        dw.own_bad,
        dw.drop_bad,
        sp,
        oe_bad,
        pads_bad,
        dw.sp_min / 150,
        dw.sp_max / 150
    );
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("[INFO] dshot_smoke: start");
    let p = embassy_rp::init(Default::default());
    let hz = embassy_rp::clocks::clk_sys_freq();
    let mut fails = 0u32;
    check!(
        fails,
        hz == CLK_HZ,
        "[{=str}] clk_sys is 150 MHz ({=u32})",
        hz
    );
    // The CPU cycle counter times the drops of check 6 and the cancel sweep (clk_sys, as the
    // observers).
    // SAFETY: only the trace enable and the cycle counter are touched; nothing else uses them.
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
    let c0 = cyc();
    Timer::after_millis(1).await;
    let dc = cyc().wrapping_sub(c0);
    check!(
        fails,
        (150_000..=160_000).contains(&dc),
        "[{=str}] the CPU cycle counter runs at clk_sys ({=u32} cycles over a 1 ms timer)",
        dc
    );
    // Check 2's classifier on synthetic input, before any frame.
    let st = self_tests();
    check!(
        fails,
        st == 0,
        "[{=str}] self-tests of check 2's classifier and zone, addendum C and check 6, before the first frame: {=u32} failed",
        st
    );
    if st != 0 {
        hil_common::exit(fails)
    }

    // The driver's program words are what the assembler makes of the documented source.
    let drv_src = pio_asm!(
        ".wrap_target",
        "idle:",
        "    mov x, status",
        "    jmp x-- idle",
        "    set pins, 0",
        "    set x, 15",
        "    set pindirs, 15 [1]",
        "bit:",
        "    pull ifempty block [12]",
        "    out pins, 4 [14]",
        "    set pins, 15 [8]",
        "    jmp x-- low",
        "    set pindirs, 0",
        "    mov x, y [1]",
        "smp:",
        "    in pins, 4 [2]",
        "    jmp x-- smp",
        ".wrap",
        "low:",
        "    set pins, 0",
        "    jmp bit",
    );
    let same = drv_src.program.code.as_slice() == &program::DRIVER[..]
        && (drv_src.program.wrap.source, drv_src.program.wrap.target) == program::DRIVER_WRAP;
    check!(
        fails,
        same,
        "[{=str}] the driver's 15 instruction words equal pio_asm! of its documented source, wrap included"
    );

    // Observers on PIO0.
    let Pio {
        common: mut c0,
        sm0: o0,
        sm1: o1,
        sm2: o2,
        sm3: o3,
        ..
    } = Pio::new(p.PIO0, Irqs);
    c0.set_input_sync_bypass(OE_MASK, OE_MASK);
    let obs_prog = pio_asm!(
        "lo_start:",
        "    mov x, ~null",
        "lo_loop:",
        "    jmp pin hi_push",
        "    jmp x-- lo_loop",
        "    jmp lo_start",
        "hi_push:",
        "    mov isr, ~x",
        "    push block",
        "    mov x, ~null",
        "hi_loop:",
        "    jmp pin hi_dec",
        "    jmp lo_push",
        "hi_dec:",
        "    jmp x-- hi_loop",
        "    jmp hi_loop",
        "lo_push:",
        "    mov isr, ~x",
        "    push block",
        "    jmp lo_start",
    );
    let obs_loaded = c0.load_program(&obs_prog.program);
    fn arm<const S: usize>(sm: &mut StateMachine<'static, PIO0, S>, cfg: &Config<'static, PIO0>) {
        sm.set_enable(false);
        sm.set_config(cfg);
        sm.restart();
        sm.clear_fifos();
    }
    let (mut o0, mut o1, mut o2, mut o3) = (o0, o1, o2, o3);
    arm(&mut o0, &obs_cfg(&obs_loaded, 36));
    arm(&mut o1, &obs_cfg(&obs_loaded, 37));
    arm(&mut o2, &obs_cfg(&obs_loaded, 38));
    arm(&mut o3, &obs_cfg(&obs_loaded, 39));
    let _dma_reserved = (p.DMA_CH4, p.DMA_CH5, p.DMA_CH6, p.DMA_CH7);

    // The driver on PIO2.
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
    let drv = match PioDshot::new(parts, Rate::Dshot300, CLK_HZ) {
        Ok(d) => d,
        Err(e) => {
            defmt::error!("[FAIL] PioDshot::new: {}", e.reason);
            hil_common::exit(fails + 1)
        }
    };
    // Start the observers with the line levels as they are now.
    let levels = pac::SIO.gpio_in(1).read() >> 4;
    let mut obs = Obs {
        t0: Instant::now(),
        l: [Line::NEW; LINES],
        qh: [0; LINES],
        qt: [0; LINES],
        ring_over: 0,
        spike_check: false,
        spikes: 0,
        min_run: u64::MAX,
        shorts: [Short::NONE; 32],
        shorts_n: 0,
        shorts_lost: 0,
        drive: [(0, 0); 4],
        drive_i: 0,
        drive_n: 0,
        breaks: [0; 2 * LINES],
        breaks_n: 0,
        sp_ours: 0,
        sp_rep: 0,
        off_grid: 0,
        defer_rel: false,
        held: [(Short::NONE, 0); 8],
        held_n: 0,
        stage: "built",
        xk: 0,
        caps: None,
    };
    for (i, ln) in obs.l.iter_mut().enumerate() {
        ln.high = levels >> i & 1 != 0;
        ln.skip = ln.high;
    }
    dma_start();
    obs.t0 = Instant::now();
    pac::PIO0.ctrl().modify(|w| w.set_sm_enable(0xF));
    let mut ctx = Ctx {
        drv: Some(drv),
        obs,
        fails,
        no_oe_during: false,
        oe_bad_during: 0,
        oe_bad_after: 0,
        oe_samples: 0,
        keep: false,
        sent: 0,
        t_push: Instant::now(),
    };
    // Built: lines released. (A controller may be holding its line low in its start-up: only our
    // drive and our pads are judged here, not the line level.)
    Timer::after_millis(10).await;
    check!(
        ctx.fails,
        padoe() == 0 && pads_ok(),
        "[{=str}] built: DBG_PADOE 0, every pad an input with its pull-up"
    );

    for rate in [Rate::Dshot300, Rate::Dshot600] {
        if ctx.drv().rate() != rate {
            let drv = ctx.drv.take();
            match drv.map(PioDshot::release) {
                Some(Ok(parts)) => match PioDshot::new(parts, rate, CLK_HZ) {
                    Ok(d) => ctx.drv = Some(d),
                    Err(e) => {
                        defmt::error!("[FAIL] rebuild at {}: {}", rate, e.reason);
                        hil_common::exit(ctx.fails + 1)
                    }
                },
                _ => {
                    defmt::error!("[FAIL] release before {} refused", rate);
                    hil_common::exit(ctx.fails + 1)
                }
            }
        }
        run_rate(&mut ctx, rate).await;
    }

    // Drop: the lines released, the state machines stopped.
    ctx.obs.mark();
    drop(ctx.drv.take());
    let oe = padoe();
    let pads = pads_ok();
    Timer::after_millis(100).await;
    ctx.obs.sync();
    let after_drop_edges = ctx.obs.edges_in_window();
    check!(
        ctx.fails,
        oe == 0 && pads && after_drop_edges == 0,
        "[{=str}] after Drop: DBG_PADOE 0, every pad an input with its pull-up, no edge on the lines in the next 100 ms ({=u32})",
        after_drop_edges
    );
    ctx.obs.settle();
    let sp_total = ctx.obs.sp_ours + ctx.obs.off_grid;
    check!(
        ctx.fails,
        sp_total == 0,
        "[{=str}] addendum C over the whole run, after the last segment and the final drop: runs under 300 ns attributed to us plus edges off our bit grid inside our drive interval {=u32} (0 required; transient-exempt runs reported, not counted)",
        sp_total
    );
    defmt::info!("[INFO] controllers left parked in their bootloaders once frames stop (inert)");
    print_spikes();
    hil_common::exit(ctx.fails)
}
