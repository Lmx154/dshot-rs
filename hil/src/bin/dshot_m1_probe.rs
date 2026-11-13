//! M1 measurement for the AM32 bidirectional DShot link: one PIO state machine (PIO2, state
//! machine 0) puts the STOP frame on GPIO36-39 in step, releases the lines, and samples them four
//! wide for a fixed window through DMA; the CPU decodes the replies. A second state machine of the
//! same block (PIO2, state machine 1) only watches the pins at 150 MS/s, for the frame check and
//! as a cross-check of the timing.
//!
//! The only thing this program ever puts on a line is the bidirectional STOP frame, the constant
//! `STOP_IMAGE`. Between frames, and at the start and the end, the four pins are inputs with the
//! pad pull-up on. Touches only GPIO36-39, PIO2, DMA channels 1 and 2.
//!
//! PIO program (state machine 0), `X` = bit counter, `Y` = number of samples in the window - 1:
//!
//! ```text
//! idle:  mov x, status        ; all ones while the TX FIFO holds fewer than 2 words
//!        jmp x-- idle         ; (falls through, with x = 0, once both words are in)
//!        set x, 15
//!        set pindirs, 15      ; the four lines are driven from here, latch high
//! bit:   set pins, 0          ; phase 1, 15 cycles: all low
//!        pull ifempty block [13]  ; (the next word, after 8 bits; the TX FIFO holds both words)
//!        out pins, 4   [14]   ; phase 2, 15 cycles: 0xF for a 0 bit (high), 0x0 for a 1 bit (low)
//!        set pins, 15  [8]    ; phase 3, 10 cycles with the jmp: all high
//!        jmp x-- bit
//!        set pindirs, 0       ; released: inputs, the pad pull-up holds the line high
//!        mov x, y
//! smp:   in pins, 4           ; autopush every 8 samples
//!        jmp x-- smp          ; 2 cycles per sample
//! ```
//!
//! One bit is 40 cycles: divider 12.5 at DShot300 (500 cycles of clk_sys = 3.333 us), 6.25 at
//! DShot600 (1.667 us). Low time 15 + 15 cycles = 37.5 % for a 0 bit, 30 cycles of 40 = 75 % for a 1.
#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Pull};
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2, PIO2};
use embassy_rp::pio::program::{InSource, InstructionOperands, OutDestination, pio_asm};
use embassy_rp::pio::{
    self, Config, Pio, ShiftConfig, ShiftDirection, StateMachine, StatusN, StatusSource,
};
use embassy_rp::{bind_interrupts, dma, pac};
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
use fixed::types::U24F8;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    PIO2_IRQ_0 => pio::InterruptHandler<PIO2>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
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
const PIN_BASE: u32 = 20;
const OE_MASK: u32 = 0xF << PIN_BASE;
const NAMES: [&str; LINES] = ["M4/G36", "M3/G37", "M2/G38", "M1/G39"];

/// THE frame: the bidirectional STOP frame 0x000F (value 0, telemetry bit 0, inverted checksum
/// 0xF), most significant bit first, as the pin levels of the data phase, one nibble per bit, the
/// first bit in the lowest nibble of word 0. A 0 bit is 0xF (the line goes high after the first
/// phase), a 1 bit is 0x0 (it stays low). Bits 15..8 are all 0, bits 7..4 are 0, bits 3..0 are 1.
/// Nothing else is ever written to the TX FIFO of the state machine that drives the lines.
const STOP_IMAGE: [u32; 2] = [0xFFFF_FFFF, 0x0000_FFFF];

/// Reply code table of facts-am32-link.md section 5, nibble 0..F to 5-bit code.
const GCR: [u8; 16] = [
    0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
];
const GCR_INV: [u8; 32] = {
    let mut t = [0xFFu8; 32];
    let mut i = 0;
    while i < 16 {
        t[GCR[i] as usize] = i as u8;
        i += 1;
    }
    t
};

#[derive(Clone, Copy)]
struct Rate {
    label: u32,
    /// Nominal command bit time in ps.
    bit_ps: u32,
    /// PIO clock divider times 256.
    div_q8: u32,
}

const RATES: [Rate; 2] = [
    Rate {
        label: 300,
        bit_ps: 3_333_333,
        div_q8: 3200,
    },
    Rate {
        label: 600,
        bit_ps: 1_666_667,
        div_q8: 1600,
    },
];

impl Rate {
    /// One PIO cycle in ps.
    fn c_ps(self) -> u32 {
        (self.div_q8 as u64 * 1_000_000_000_000 / CLK_HZ as u64 / 256) as u32
    }
    /// Time of one RX word (8 samples of 2 cycles) in ps.
    fn word_ps(self) -> u32 {
        16 * self.c_ps()
    }
    /// From the first bit to the release of the lines, in ps.
    fn frame_ps(self) -> u32 {
        16 * self.bit_ps
    }
    fn words_for(self, window_ps: u32) -> usize {
        ((window_ps as usize).div_ceil(self.word_ps() as usize)).clamp(8, MAX_WORDS)
    }
}

/// The first sample comes 12 cycles after our last rising edge, then one every 2 cycles
/// (set pins [8] + jmp = 10 cycles to `set pindirs`, `mov x, y`, then the first `in`).
const FIRST_SAMPLE_CYCLES: u32 = 12;
/// The release comes 10 cycles after our last rising edge.
const RELEASE_CYCLES: u32 = 10;
const MAX_WORDS: usize = 256;
const OBS_WORDS: usize = 1408;
/// Sample period of the observer state machine (2 cycles at divider 1) in ps.
const OBS_PERIOD_PS: u32 = (2_000_000_000_000u64 / CLK_HZ as u64) as u32;
/// Frame 0 comes out of the low pause and has no first falling edge; frames 1 to 5 are checked.
const OBS_FRAMES: u32 = 6;
const N: usize = 1000;
const MARGIN_PS: u32 = 10_000_000;
const DEFAULT_WINDOW_PS: u32 = 150_000_000;
const FIND_LIMIT: u32 = 10_000;
const CALIB_FRAMES: u32 = 50;
/// Frames sent, and not counted, between the first 100 decodable replies and the 1000 measured captures.
const WARMUP_FRAMES: u32 = 2000;

// ---------------------------------------------------------------------------------------------
// Capture analysis

const MAX_EDGES: usize = 64;

#[derive(Clone, Copy)]
struct Edges {
    n: usize,
    k: [u32; MAX_EDGES],
    over: bool,
}

impl Edges {
    const EMPTY: Edges = Edges {
        n: 0,
        k: [0; MAX_EDGES],
        over: false,
    };

    fn push(&mut self, v: u32) {
        if self.n < MAX_EDGES {
            self.k[self.n] = v;
            self.n += 1;
        } else {
            self.over = true;
        }
    }

    fn list(&self) -> &[u32] {
        &self.k[..self.n]
    }
}

/// Sample index of every level change, per line. Eight samples per word, the oldest in the
/// highest nibble, line 0 (GPIO36) in bit 0 of the nibble. The level before the first sample is
/// high.
fn extract(buf: &[u32]) -> [Edges; LINES] {
    let mut out = [Edges::EMPTY; LINES];
    let mut prev: u32 = 0xF;
    for (wi, &w) in buf.iter().enumerate() {
        let before = (w >> 4) | (prev << 28);
        let mut d = w ^ before;
        prev = w & 0xF;
        while d != 0 {
            let p = 31 - d.leading_zeros();
            d &= !(1 << p);
            let sample = wi as u32 * 8 + (31 - p) / 4;
            out[(p & 3) as usize].push(sample);
        }
    }
    out
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Silent,
    Truncated,
    Shape,
    Gcr,
    Checksum,
    Ok,
}

#[derive(Clone, Copy)]
struct Reply {
    kind: Kind,
    /// First falling edge, ps after our last rising edge.
    t0: u32,
    /// Last edge, same reference.
    t_last: u32,
    /// Reply bit time from the total length, ps.
    fit: u32,
    /// Shortest run, ps.
    short: u32,
    value: u16,
}

impl Reply {
    const SILENT: Reply = Reply {
        kind: Kind::Silent,
        t0: 0,
        t_last: 0,
        fit: 0,
        short: 0,
        value: 0,
    };
}

/// Decodes one line. `times` are the edge times in ps, rising and falling alternately, the first
/// one falling. 21 symbols: one leading, then 20 NRZI symbols (a 1 toggles the line) that are
/// four 5-bit GCR codes, most significant first.
fn decode(times: &[u32], over: bool, cmd_bit_ps: u32) -> Reply {
    let n = times.len();
    if n == 0 {
        return Reply::SILENT;
    }
    let mut r = Reply {
        kind: Kind::Shape,
        t0: times[0],
        t_last: times[n - 1],
        fit: 0,
        short: 0,
        value: 0,
    };
    if over {
        return r;
    }
    if n < 2 || n % 2 == 1 {
        r.kind = Kind::Truncated;
        return r;
    }
    let nominal = cmd_bit_ps / 5 * 4;
    let mut est = (nominal as u64 * 9625 / 10_000) as u32;
    let mut slots = [0u32; MAX_EDGES];
    let mut fit = est;
    for _ in 0..2 {
        for (j, w) in times.windows(2).enumerate() {
            let run = w[1] - w[0];
            slots[j + 1] = slots[j] + ((2 * run + est) / (2 * est)).max(1);
        }
        let mut num = 0u64;
        let mut den = 0u64;
        for (&s, &t) in slots[1..n].iter().zip(&times[1..]) {
            num += s as u64 * (t - times[0]) as u64;
            den += s as u64 * s as u64;
        }
        fit = (num / den) as u32;
        est = fit.max(1);
    }
    let short = times.windows(2).map(|w| w[1] - w[0]).min().unwrap_or(0);
    r.fit = fit;
    r.short = short;
    if slots[n - 1] > 21 || fit * 4 < nominal * 3 || fit * 4 > nominal * 5 {
        return r;
    }
    // Level of each of the 21 symbols (true = high); the leading symbol is low.
    let mut lvl = [false; 21];
    for (i, l) in lvl.iter_mut().enumerate() {
        let toggles = slots[..n].iter().filter(|&&s| s as usize <= i).count();
        *l = toggles % 2 == 0;
    }
    let mut code = 0u32;
    let mut prev = lvl[0];
    for &l in &lvl[1..] {
        code = code << 1 | u32::from(l != prev);
        prev = l;
    }
    let mut word = 0u16;
    for g in 0..4 {
        let nib = GCR_INV[((code >> (15 - 5 * g)) & 0x1F) as usize];
        if nib == 0xFF {
            r.kind = Kind::Gcr;
            return r;
        }
        word = word << 4 | nib as u16;
    }
    let v = word >> 4;
    let want = !(v ^ (v >> 4) ^ (v >> 8)) & 0xF;
    r.value = v;
    r.kind = if word & 0xF == want {
        Kind::Ok
    } else {
        Kind::Checksum
    };
    r
}

/// Times in ps after our last rising edge of the edges seen by the sampling state machine.
fn main_times(e: &Edges, c_ps: u32, out: &mut [u32; MAX_EDGES]) -> usize {
    for (o, &k) in out.iter_mut().zip(e.list()) {
        *o = (FIRST_SAMPLE_CYCLES + 2 * k - 1) * c_ps;
    }
    e.n
}

fn decode_main(edges: &[Edges; LINES], rate: Rate) -> [Reply; LINES] {
    let mut out = [Reply::SILENT; LINES];
    let mut t = [0u32; MAX_EDGES];
    for (o, e) in out.iter_mut().zip(edges) {
        let n = main_times(e, rate.c_ps(), &mut t);
        *o = decode(&t[..n], e.over, rate.bit_ps);
    }
    out
}

/// What the observer saw of our frame on one line.
#[derive(Clone, Copy)]
struct FrameLine {
    bit_avg: u32,
    per_min: u32,
    per_max: u32,
    lo0_min: u32,
    lo0_max: u32,
    lo1_min: u32,
    lo1_max: u32,
    total: u32,
}

/// Index of the first edge of our own frame in an observer capture: the first falling edge that
/// starts 16 low pulses and periods of about one bit (the line may be low or noisy before it).
fn find_frame(e: &Edges, bit_ps: u32) -> Option<usize> {
    let t = |i: usize| e.k[i] * OBS_PERIOD_PS;
    let mut j = 0;
    while j + 32 <= e.n {
        let good = (0..16).all(|i| {
            let lo = t(j + 2 * i + 1) - t(j + 2 * i);
            let lo_ok = lo * 10 > bit_ps * 2 && lo * 10 < bit_ps * 9;
            let per_ok = i == 15 || {
                let per = t(j + 2 * i + 2) - t(j + 2 * i);
                per * 10 > bit_ps * 8 && per * 10 < bit_ps * 12
            };
            lo_ok && per_ok
        });
        if good {
            return Some(j);
        }
        j += 2;
    }
    None
}

fn analyze_frame(e: &Edges, j: usize) -> FrameLine {
    let t = |i: usize| e.k[i] * OBS_PERIOD_PS;
    let f = |i: usize| t(j + 2 * i);
    let r = |i: usize| t(j + 2 * i + 1);
    let mut fl = FrameLine {
        bit_avg: (f(15) - f(0)) / 15,
        per_min: u32::MAX,
        per_max: 0,
        lo0_min: u32::MAX,
        lo0_max: 0,
        lo1_min: u32::MAX,
        lo1_max: 0,
        total: r(15) - f(0),
    };
    for i in 0..15 {
        let p = f(i + 1) - f(i);
        fl.per_min = fl.per_min.min(p);
        fl.per_max = fl.per_max.max(p);
    }
    for i in 0..16 {
        let lo = r(i) - f(i);
        if i < 12 {
            fl.lo0_min = fl.lo0_min.min(lo);
            fl.lo0_max = fl.lo0_max.max(lo);
        } else {
            fl.lo1_min = fl.lo1_min.min(lo);
            fl.lo1_max = fl.lo1_max.max(lo);
        }
    }
    fl
}

/// Observer result for the frame check over the first frames.
#[derive(Clone, Copy)]
struct FrameAcc {
    frames: u32,
    bad_count: u32,
    bit_min: u32,
    bit_max: u32,
    per_min: u32,
    per_max: u32,
    lo0_min: u32,
    lo0_max: u32,
    lo1_min: u32,
    lo1_max: u32,
    total_min: u32,
    total_max: u32,
}

impl FrameAcc {
    const NEW: FrameAcc = FrameAcc {
        frames: 0,
        bad_count: 0,
        bit_min: u32::MAX,
        bit_max: 0,
        per_min: u32::MAX,
        per_max: 0,
        lo0_min: u32::MAX,
        lo0_max: 0,
        lo1_min: u32::MAX,
        lo1_max: 0,
        total_min: u32::MAX,
        total_max: 0,
    };

    fn add(&mut self, f: Option<FrameLine>) {
        self.frames += 1;
        let Some(f) = f else {
            self.bad_count += 1;
            return;
        };
        self.bit_min = self.bit_min.min(f.bit_avg);
        self.bit_max = self.bit_max.max(f.bit_avg);
        self.per_min = self.per_min.min(f.per_min);
        self.per_max = self.per_max.max(f.per_max);
        self.lo0_min = self.lo0_min.min(f.lo0_min);
        self.lo0_max = self.lo0_max.max(f.lo0_max);
        self.lo1_min = self.lo1_min.min(f.lo1_min);
        self.lo1_max = self.lo1_max.max(f.lo1_max);
        self.total_min = self.total_min.min(f.total);
        self.total_max = self.total_max.max(f.total);
    }
}

// ---------------------------------------------------------------------------------------------
// Statistics

struct Lane {
    replies: u32,
    silent: u32,
    truncated: u32,
    shape: u32,
    gcr: u32,
    checksum: u32,
    ok: u32,
    fff: u32,
    other_value: Option<u16>,
    start: [u32; N],
    n_start: usize,
    fit: [u32; N],
    short: [u32; N],
    n_fit: usize,
    end_edge_max: u32,
    end21_max: u32,
}

impl Lane {
    fn new() -> Self {
        Lane {
            replies: 0,
            silent: 0,
            truncated: 0,
            shape: 0,
            gcr: 0,
            checksum: 0,
            ok: 0,
            fff: 0,
            other_value: None,
            start: [0; N],
            n_start: 0,
            fit: [0; N],
            short: [0; N],
            n_fit: 0,
            end_edge_max: 0,
            end21_max: 0,
        }
    }

    fn reset(&mut self) {
        self.replies = 0;
        self.silent = 0;
        self.truncated = 0;
        self.shape = 0;
        self.gcr = 0;
        self.checksum = 0;
        self.ok = 0;
        self.fff = 0;
        self.other_value = None;
        self.n_start = 0;
        self.n_fit = 0;
        self.end_edge_max = 0;
        self.end21_max = 0;
    }

    fn add(&mut self, r: &Reply) {
        match r.kind {
            Kind::Silent => self.silent += 1,
            Kind::Truncated => self.truncated += 1,
            Kind::Shape => self.shape += 1,
            Kind::Gcr => self.gcr += 1,
            Kind::Checksum => self.checksum += 1,
            Kind::Ok => self.ok += 1,
        }
        if r.kind == Kind::Silent {
            return;
        }
        self.replies += 1;
        if self.n_start < N {
            self.start[self.n_start] = r.t0;
            self.n_start += 1;
        }
        self.end_edge_max = self.end_edge_max.max(r.t_last);
        if matches!(r.kind, Kind::Gcr | Kind::Checksum | Kind::Ok) {
            if self.n_fit < N {
                self.fit[self.n_fit] = r.fit;
                self.short[self.n_fit] = r.short;
                self.n_fit += 1;
            }
            self.end21_max = self.end21_max.max(r.t0 + 21 * r.fit);
        }
        if r.kind == Kind::Ok {
            if r.value == 0xFFF {
                self.fff += 1;
            } else {
                self.other_value = Some(r.value);
            }
        }
    }
}

struct Stats {
    lane: [Lane; LINES],
    spread_max: u32,
    captures: u32,
}

impl Stats {
    fn new() -> Self {
        Stats {
            lane: core::array::from_fn(|_| Lane::new()),
            spread_max: 0,
            captures: 0,
        }
    }

    fn reset(&mut self) {
        for l in &mut self.lane {
            l.reset();
        }
        self.spread_max = 0;
        self.captures = 0;
    }

    fn add(&mut self, rs: &[Reply; LINES]) {
        self.captures += 1;
        for (l, r) in self.lane.iter_mut().zip(rs) {
            l.add(r);
        }
        let mut lo = u32::MAX;
        let mut hi = 0;
        let mut n = 0;
        for r in rs.iter().filter(|r| r.kind != Kind::Silent) {
            lo = lo.min(r.t0);
            hi = hi.max(r.t0);
            n += 1;
        }
        if n >= 2 {
            self.spread_max = self.spread_max.max(hi - lo);
        }
    }
}

fn min_med_max(v: &mut [u32]) -> (u32, u32, u32) {
    if v.is_empty() {
        return (0, 0, 0);
    }
    v.sort_unstable();
    (v[0], v[v.len() / 2], v[v.len() - 1])
}

fn ns(ps: u32) -> u32 {
    (ps + 500) / 1000
}

// ---------------------------------------------------------------------------------------------
// The hardware

#[derive(Default)]
struct Health {
    stalls: u32,
    leftovers: u32,
    timeouts: u32,
    push_failed: u32,
    oe_after_frame: u32,
    oe_during_frame: u32,
    observed: u32,
    obs_stalls: u32,
    frames: u32,
}

struct Ctx<'d> {
    sm0: StateMachine<'d, PIO2, 0>,
    sm1: StateMachine<'d, PIO2, 1>,
    dma0: dma::Channel<'d>,
    dma1: dma::Channel<'d>,
    main_origin: u8,
    obs_origin: u8,
    buf: [u32; MAX_WORDS],
    words: usize,
    obs: [u32; OBS_WORDS],
    h: Health,
    last_start: Option<Instant>,
    spacing_min: u32,
    spacing_max: u32,
    last_gap: u32,
    fails: u32,
}

fn settle() {
    cortex_m::asm::delay(2_000);
}

/// Loads Y of a disabled state machine from a word pushed to its TX FIFO, by an explicit `pull`
/// and `out y, 32`, so that the state of the OSR does not matter.
fn load_y<const SM: usize>(sm: &mut StateMachine<'_, PIO2, SM>, v: u32) {
    sm.tx().push(v);
    // SAFETY: the state machine is disabled; the two instructions only move the pushed word to Y.
    unsafe {
        sm.exec_instr(PULL_32);
        settle();
        sm.exec_instr(OUT_Y_32);
        settle();
    }
}

const IN_Y_32: u16 = InstructionOperands::IN {
    source: InSource::Y,
    bit_count: 32,
}
.encode();
const PULL_32: u16 = InstructionOperands::PULL {
    if_empty: false,
    block: true,
}
.encode();
const OUT_Y_32: u16 = InstructionOperands::OUT {
    destination: OutDestination::Y,
    bit_count: 32,
}
.encode();
impl Ctx<'_> {
    /// Reloads state machine 0 with the divider of `rate` and a window of `words` RX words, and
    /// parks it at the top of the program with the lines released, or with `low` driven low.
    fn configure(&mut self, rate: Rate, words: usize, low: bool) {
        self.words = words;
        self.sm0.set_enable(false);
        self.sm0.restart();
        self.sm0.clear_fifos();
        self.sm0.set_clock_divider(U24F8::from_bits(rate.div_q8));
        load_y(&mut self.sm0, 8 * words as u32 - 1);
        // SAFETY: the state machine is disabled; these only set the output latch and the direction
        // of the four lines: driven low (the ESC reset pause) or released with the latch high.
        unsafe {
            if low {
                self.sm0.set_pin(0);
                settle();
                self.sm0.set_pindir(15);
            } else {
                self.sm0.set_pin(15);
                settle();
                self.sm0.set_pindir(0);
            }
        }
        settle();
        // The window count must have gone into Y and nothing may be left in the TX FIFO: a stale
        // word there would be taken for part of a frame.
        // SAFETY: the state machine is disabled; `in y, 32` autopushes Y into the RX FIFO.
        unsafe { self.sm0.exec_instr(IN_Y_32) };
        settle();
        let y = self.sm0.rx().try_pull();
        if y != Some(8 * words as u32 - 1) || !self.sm0.tx().empty() || !self.sm0.rx().empty() {
            defmt::error!(
                "Y read back {=u32} (want {=u32}), TX FIFO empty {=bool}, RX FIFO empty {=bool}",
                y.unwrap_or(u32::MAX),
                8 * words as u32 - 1,
                self.sm0.tx().empty(),
                self.sm0.rx().empty()
            );
            self.abort("state machine 0 not loaded as expected");
        }
        // SAFETY: as above.
        unsafe { self.sm0.exec_jmp(self.main_origin) };
        settle();
        self.sm0.clkdiv_restart();
        self.sm0.set_enable(true);
    }

    fn obs_prepare(&mut self) {
        self.sm1.set_enable(false);
        self.sm1.restart();
        self.sm1.clear_fifos();
        load_y(&mut self.sm1, 8 * OBS_WORDS as u32 - 1);
        // SAFETY: the observer state machine is disabled and only jumps to its first instruction.
        unsafe { self.sm1.exec_jmp(self.obs_origin) };
        settle();
    }

    fn obs_enable(on: bool) {
        pac::PIO2.ctrl().modify(|w| {
            let v = w.sm_enable();
            w.set_sm_enable(if on { v | 0b10 } else { v & !0b10 });
        });
    }

    fn release_lines(&mut self) {
        Self::obs_enable(false);
        self.sm0.set_enable(false);
        // SAFETY: both state machines are disabled; this only releases the lines.
        unsafe { self.sm0.set_pindir(0) };
        settle();
    }

    /// Watches the pins for 150 us with no frame sent.
    async fn capture_idle(&mut self) -> bool {
        self.obs_prepare();
        let t = self.sm1.rx().dma_pull(&mut self.dma1, &mut self.obs, false);
        Self::obs_enable(true);
        let ok = with_timeout(Duration::from_millis(3), t).await.is_ok();
        Self::obs_enable(false);
        if self.sm1.rx().stalled() {
            self.h.obs_stalls += 1;
        }
        ok
    }

    /// One frame and its capture. With `observe` the observer watches the pins as well. False
    /// when a DMA transfer timed out.
    async fn frame(&mut self, observe: bool) -> bool {
        let words = self.words;
        if !self.sm0.tx().empty() {
            self.abort("TX FIFO not empty before a frame");
        }
        if observe {
            self.obs_prepare();
        }
        let Ctx {
            sm0,
            sm1,
            dma0,
            dma1,
            buf,
            obs,
            h,
            last_start,
            spacing_min,
            spacing_max,
            last_gap,
            ..
        } = self;
        let ok = {
            let (rx0, tx0) = sm0.rx_tx();
            let t0 = rx0.dma_pull(dma0, &mut buf[..words], false);
            let t1 = if observe {
                Some(sm1.rx().dma_pull(dma1, &mut obs[..], false))
            } else {
                None
            };
            if observe {
                Self::obs_enable(true);
            }
            let now = Instant::now();
            // Both words in with no await between: the program starts only when both are there.
            let a = tx0.try_push(STOP_IMAGE[0]);
            let b = tx0.try_push(STOP_IMAGE[1]);
            if !(a && b) {
                h.push_failed += 1;
            }
            cortex_m::asm::delay(300);
            if observe {
                h.observed += 1;
            }
            if observe && pac::PIO2.dbg_padoe().read() & OE_MASK == OE_MASK {
                h.oe_during_frame += 1;
            }
            if let Some(prev) = *last_start {
                let d = (now - prev).as_micros() as u32;
                *last_gap = d;
                *spacing_min = (*spacing_min).min(d);
                *spacing_max = (*spacing_max).max(d);
            }
            *last_start = Some(now);
            let mut ok = with_timeout(Duration::from_millis(3), t0).await.is_ok();
            if let Some(t) = t1 {
                ok &= with_timeout(Duration::from_millis(3), t).await.is_ok();
            }
            ok
        };
        if observe {
            Self::obs_enable(false);
            if sm1.rx().stalled() {
                h.obs_stalls += 1;
            }
        }
        let rx0 = sm0.rx();
        if !ok {
            h.timeouts += 1;
        }
        if rx0.stalled() {
            h.stalls += 1;
        }
        if !rx0.empty() {
            h.leftovers += 1;
            while rx0.try_pull().is_some() {}
        }
        if pac::PIO2.dbg_padoe().read() & OE_MASK != 0 {
            h.oe_after_frame += 1;
        }
        h.frames += 1;
        ok
    }

    fn reset_spacing(&mut self) {
        self.last_start = None;
        self.spacing_min = u32::MAX;
        self.spacing_max = 0;
    }

    fn abort(&mut self, why: &str) -> ! {
        self.release_lines();
        defmt::error!("[FAIL] aborted: {=str}", why);
        self.fails += 1;
        hil_common::exit(self.fails)
    }
}

// ---------------------------------------------------------------------------------------------

/// Replies as seen by the observer: the edges after the 32 edges of our own frame.
fn decode_observer(e: &[Edges; LINES], rate: Rate) -> [Reply; LINES] {
    let mut out = [Reply::SILENT; LINES];
    let mut t = [0u32; MAX_EDGES];
    for (o, e) in out.iter_mut().zip(e) {
        let Some(j) = find_frame(e, rate.bit_ps) else {
            continue;
        };
        let last = e.k[j + 31];
        let mut n = 0;
        for &k in &e.k[j + 32..e.n] {
            t[n] = (k - last) * OBS_PERIOD_PS;
            n += 1;
        }
        *o = decode(&t[..n], e.over, rate.bit_ps);
    }
    out
}

fn self_test(fails: &mut u32) {
    // Reply 0xFFF as hand-derived from the code table: toggles at these symbol numbers.
    const SLOTS: [u32; 16] = [0, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 16, 17, 20];
    let mut t = [0u32; 16];
    let mut ok_all = true;
    for (bit_ps, t_ps) in [(3_333_333u32, 2_567_000u32), (1_666_667, 1_283_000)] {
        for (s, o) in SLOTS.iter().zip(t.iter_mut()) {
            *o = 20_000_000 + s * t_ps;
        }
        let r = decode(&t, false, bit_ps);
        ok_all &= r.kind == Kind::Ok && r.value == 0xFFF;
        // The same with the timing noise of one sample.
        for (i, o) in t.iter_mut().enumerate() {
            *o = (*o + 40_000 * (i as u32 % 3)).saturating_sub(40_000);
        }
        let r = decode(&t, false, bit_ps);
        ok_all &= r.kind == Kind::Ok && r.value == 0xFFF;
    }
    // Negative controls: a toggle moved by one symbol must not decode to a good 0xFFF.
    let mut moved = SLOTS;
    moved[15] = 19;
    let mut neg_ok = true;
    for (s, o) in moved.iter().zip(t.iter_mut()) {
        *o = 20_000_000 + s * 2_567_000;
    }
    let r = decode(&t, false, 3_333_333);
    neg_ok &= !(r.kind == Kind::Ok && r.value == 0xFFF);
    let mut moved = SLOTS;
    moved[4] = 6;
    moved[5] = 7;
    for (s, o) in moved.iter().zip(t.iter_mut()) {
        *o = 20_000_000 + s * 2_567_000;
    }
    let r = decode(&t, false, 3_333_333);
    neg_ok &= !(r.kind == Kind::Ok && r.value == 0xFFF);
    check!(
        *fails,
        ok_all,
        "[{=str}] decoder self-test: synthetic 0xFFF reply decodes, both rates, with noise"
    );
    check!(
        *fails,
        neg_ok,
        "[{=str}] decoder self-test: two corrupted replies do not decode to 0xFFF"
    );
}

fn lines_high() -> [bool; LINES] {
    let v = pac::SIO.gpio_in(1).read() >> 4;
    core::array::from_fn(|i| v >> i & 1 != 0)
}

struct Dump {
    idx: u32,
    obs: Edges,
    smp0: Edges,
    smp3: Edges,
}

fn print_dumps(label: u32, dumps: &[Dump], from: usize, to: usize) {
    for d in &dumps[from..to] {
        if d.obs.n == 0 && d.smp0.n == 0 && d.smp3.n == 0 {
            continue;
        }
        defmt::info!(
            "[INFO] DShot{=u32} raw frame {=u32}: observer line 0 edges (13.3 ns samples) {=[?]}; sampler line 0 edges {=[?]}; sampler line 3 edges {=[?]}",
            label,
            d.idx,
            &d.obs.k[..d.obs.n],
            &d.smp0.k[..d.smp0.n],
            &d.smp3.k[..d.smp3.n]
        );
    }
}

struct BlockOut {
    window_ps: u32,
    fit_median: u32,
    worked_period_us: Option<u32>,
    first_ok_ms: [Option<u32>; LINES],
    none_replied: bool,
}

fn print_lane(rate: Rate, i: usize, l: &mut Lane, fails: &mut u32) {
    let label = rate.label;
    let (smin, smed, smax) = min_med_max(&mut l.start[..l.n_start]);
    defmt::info!(
        "[STAT] DShot{=u32} {=str}: reply start after our last rising edge, {=usize} of {=u32} captures: min {=u32} median {=u32} max {=u32} ns",
        label,
        NAMES[i],
        l.n_start,
        l.replies + l.silent,
        ns(smin),
        ns(smed),
        ns(smax)
    );
    let (fmin, fmed, fmax) = min_med_max(&mut l.fit[..l.n_fit]);
    let (_, shmed, _) = min_med_max(&mut l.short[..l.n_fit]);
    let nominal = rate.bit_ps / 5 * 4;
    defmt::info!(
        "[STAT] DShot{=u32} {=str}: reply bit time from total length min {=u32} median {=u32} max {=u32} ns, from shortest run median {=u32} ns (nominal 5/4: {=u32} ns), {=usize} replies",
        label,
        NAMES[i],
        ns(fmin),
        ns(fmed),
        ns(fmax),
        ns(shmed),
        ns(nominal),
        l.n_fit
    );
    defmt::info!(
        "[STAT] DShot{=u32} {=str}: reply end after our last rising edge, max: last edge {=u32} ns, start + 21 bits {=u32} ns",
        label,
        NAMES[i],
        ns(l.end_edge_max),
        ns(l.end21_max)
    );
    defmt::info!(
        "[STAT] DShot{=u32} {=str}: of {=u32} captures: good checksum {=u32}, 0xFFF {=u32}, silent {=u32}, truncated {=u32}, bad shape {=u32}, bad GCR {=u32}, bad checksum {=u32}; other value {=u16}",
        label,
        NAMES[i],
        l.silent + l.replies,
        l.ok,
        l.fff,
        l.silent,
        l.truncated,
        l.shape,
        l.gcr,
        l.checksum,
        l.other_value.unwrap_or(0)
    );
    check!(
        *fails,
        l.fff >= 990,
        "[{=str}] DShot{=u32} {=str}: at least 990 of 1000 captures decode to 0xFFF ({=u32})",
        label,
        NAMES[i],
        l.fff
    );
}

#[allow(clippy::too_many_lines)]
async fn block(ctx: &mut Ctx<'_>, rate: Rate, stats: &mut Stats) -> BlockOut {
    let label = rate.label;
    let default_words = rate.words_for(DEFAULT_WINDOW_PS);
    let c_ps = rate.c_ps();
    let sample_ps = 2 * c_ps;
    defmt::info!(
        "[INFO] DShot{=u32}: divider {=f32}, PIO cycle {=u32} ps, sample period {=u32} ps = {=f32} samples per nominal reply bit, window {=usize} words = {=u32} ns",
        label,
        rate.div_q8 as f32 / 256.0,
        c_ps,
        sample_ps,
        (rate.bit_ps / 5 * 4) as f32 / sample_ps as f32,
        default_words,
        ns(default_words as u32 * rate.word_ps())
    );
    ctx.h = Health::default();
    ctx.configure(rate, default_words, false);

    // ---- 0. levels with the pins as inputs with pull-ups, and a capture with no frame.
    let lv = lines_high();
    defmt::info!(
        "[STAT] DShot{=u32}: line levels before any frame (inputs, pull-ups): {=str}={=bool} {=str}={=bool} {=str}={=bool} {=str}={=bool}",
        label,
        NAMES[0],
        lv[0],
        NAMES[1],
        lv[1],
        NAMES[2],
        lv[2],
        NAMES[3],
        lv[3]
    );
    if !lv.iter().all(|&b| b) {
        defmt::info!(
            "[INFO] DShot{=u32}: NOT all lines high before the first frame (expected high); the ESC is holding a line low",
            label
        );
    }
    if !ctx.capture_idle().await {
        ctx.abort("idle capture DMA timed out");
    }
    let ie = extract(&ctx.obs);
    let idle_edges: u32 = ie.iter().map(|e| e.n as u32).sum();
    let first_nibble = ctx.obs[0] >> 28;
    defmt::info!(
        "[STAT] DShot{=u32}: idle capture {=u32} ns ({=usize} samples, no frame sent): edges {=usize} {=usize} {=usize} {=usize}, first sample 0x{=u32:x}, observer stalls {=u32}",
        label,
        ns(OBS_WORDS as u32 * 8 * OBS_PERIOD_PS),
        OBS_WORDS * 8,
        ie[0].n,
        ie[1].n,
        ie[2].n,
        ie[3].n,
        first_nibble,
        ctx.h.obs_stalls
    );
    check!(
        ctx.fails,
        idle_edges == 0 && ctx.h.obs_stalls == 0,
        "[{=str}] DShot{=u32}: 150 us capture with no frame shows no edge on any line",
        label
    );

    // ---- The ESC reset pause: all four lines driven LOW for 3 s (a high pause puts an ESC that
    // resets into its bootloader), then straight into the frame stream.
    ctx.configure(rate, default_words, true);
    Timer::after_millis(1500).await;
    let lv_mid = lines_high();
    Timer::after_millis(1500).await;
    let lv_end = lines_high();
    defmt::info!(
        "[STAT] DShot{=u32}: lines driven low for 3000 ms, no frame sent; levels at 1.5 s {=bool} {=bool} {=bool} {=bool}, just before the first frame {=bool} {=bool} {=bool} {=bool} (true = high)",
        label,
        lv_mid[0],
        lv_mid[1],
        lv_mid[2],
        lv_mid[3],
        lv_end[0],
        lv_end[1],
        lv_end[2],
        lv_end[3]
    );

    // ---- 1-5. frames at 1 kHz.
    stats.reset();
    let mut frame_acc = [FrameAcc::NEW; LINES];
    let mut first_reply: [Option<u32>; LINES] = [None; LINES];
    let mut first_ok: [Option<u32>; LINES] = [None; LINES];
    let mut first_ok_ms: [Option<u32>; LINES] = [None; LINES];
    let mut consec = [0u32; LINES];
    let mut warm_first = [u32::MAX; LINES];
    let mut warm_last = [0u32; LINES];
    let mut warm_silent = [0u32; LINES];
    let mut mask = [0u64; LINES];
    let mut gaps = [0u32; 40];
    let mut gap_n = [0u32; 8];
    let mut gap_silent = [0u32; 8];
    let mut dumps: [Dump; 6] = core::array::from_fn(|_| Dump {
        idx: 0,
        obs: Edges::EMPTY,
        smp0: Edges::EMPTY,
        smp3: Edges::EMPTY,
    });
    let mut edge_totals = [0u32; LINES];
    // 0 = find replies (100 decodable in a row on every line), 2 = measure, 3 = calibrate against the observer.
    let mut phase = 0u8;
    let mut left = 0u32;
    let mut idx = 0u32;
    let mut t_first = Instant::now();
    let mut calib_n = 0u32;
    let mut calib_min = i32::MAX;
    let mut calib_max = i32::MIN;
    let mut calib_sum = 0i64;
    let mut calib_fit_sum = 0i64;
    let mut calib_obs_start_sum = 0i64;
    let mut none_replied = false;
    ctx.reset_spacing();
    let mut ticker = Ticker::every(Duration::from_micros(1000));
    loop {
        ticker.next().await;
        let observe = idx < OBS_FRAMES || phase == 3 || (5000..5003).contains(&idx);
        if idx == 0 {
            t_first = Instant::now();
        }
        if !ctx.frame(observe).await {
            ctx.abort("capture DMA timed out");
        }
        let edges = extract(&ctx.buf[..ctx.words]);
        let replies = decode_main(&edges, rate);
        let obs_edges = if observe {
            Some(extract(&ctx.obs))
        } else {
            None
        };
        if let Some(oe) = &obs_edges
            && (1..OBS_FRAMES).contains(&idx)
        {
            for (a, e) in frame_acc.iter_mut().zip(oe) {
                a.add(find_frame(e, rate.bit_ps).map(|j| analyze_frame(e, j)));
            }
        }
        for (t, e) in edge_totals.iter_mut().zip(&edges) {
            *t += e.n as u32;
        }
        if idx < 3 || (5000..5003).contains(&idx) {
            let slot = if idx < 3 {
                idx as usize
            } else {
                3 + (idx - 5000) as usize
            };
            if let Some(oe) = &obs_edges {
                dumps[slot] = Dump {
                    idx,
                    obs: oe[0],
                    smp0: edges[0],
                    smp3: edges[3],
                };
            }
        }
        match phase {
            0 => {
                for (i, r) in replies.iter().enumerate() {
                    if r.kind != Kind::Silent && first_reply[i].is_none() {
                        first_reply[i] = Some(idx);
                    }
                    if r.kind == Kind::Ok && r.value == 0xFFF {
                        consec[i] += 1;
                        if first_ok[i].is_none() {
                            first_ok[i] = Some(idx);
                            first_ok_ms[i] = Some((Instant::now() - t_first).as_millis() as u32);
                        }
                    } else {
                        consec[i] = 0;
                    }
                }
                let all = consec.iter().all(|&c| c >= 100);
                if all || idx + 1 >= FIND_LIMIT {
                    none_replied = first_ok.iter().all(Option::is_none);
                    phase = 1;
                    left = WARMUP_FRAMES;
                    if none_replied {
                        break;
                    }
                }
            }
            1 => {
                for (i, r) in replies.iter().enumerate() {
                    if r.kind == Kind::Silent {
                        warm_silent[i] += 1;
                        warm_first[i] = warm_first[i].min(WARMUP_FRAMES - left);
                        warm_last[i] = WARMUP_FRAMES - left;
                    }
                }
                left -= 1;
                if left == 0 {
                    phase = 2;
                    left = N as u32;
                }
            }
            2 => {
                stats.add(&replies);
                let bin = ((ctx.last_gap.clamp(850, 1249) - 850) / 50) as usize;
                gap_n[bin] += 1;
                let c = stats.captures as usize - 1;
                if c < 64 {
                    for (m, r) in mask.iter_mut().zip(&replies) {
                        *m |= u64::from(r.kind == Kind::Silent) << c;
                    }
                }
                if c < 40 {
                    gaps[c] = ctx.last_gap;
                }
                gap_silent[bin] += u32::from(replies[0].kind == Kind::Silent);
                left -= 1;
                if left == 0 {
                    phase = 3;
                    left = CALIB_FRAMES;
                }
            }
            _ => {
                if let Some(oe) = &obs_edges {
                    let orp = decode_observer(oe, rate);
                    for (m, o) in replies.iter().zip(&orp) {
                        if m.kind != Kind::Silent && o.kind != Kind::Silent {
                            let d = ns(m.t0) as i32 - ns(o.t0) as i32;
                            calib_min = calib_min.min(d);
                            calib_max = calib_max.max(d);
                            calib_sum += d as i64;
                            calib_obs_start_sum += ns(o.t0) as i64;
                            calib_fit_sum += ns(m.fit) as i64 - ns(o.fit) as i64;
                            calib_n += 1;
                        }
                    }
                }
                left -= 1;
                if left == 0 {
                    break;
                }
            }
        }
        idx += 1;
    }

    defmt::info!(
        "[STAT] DShot{=u32} 1 kHz: frames by start-to-start time (bins of 50 us from 850 us, the first and last open): count {=[?]}, of which silent on M4 {=[?]}",
        label,
        &gap_n[..],
        &gap_silent[..]
    );

    defmt::info!(
        "[STAT] DShot{=u32} 1 kHz: during the {=u32} warm-up frames after the first 100 decodable replies, frames with no reply: M4 {=u32} (frames {=u32}..{=u32}) M3 {=u32} ({=u32}..{=u32}) M2 {=u32} ({=u32}..{=u32}) M1 {=u32} ({=u32}..{=u32})",
        label,
        WARMUP_FRAMES,
        warm_silent[0],
        warm_first[0],
        warm_last[0],
        warm_silent[1],
        warm_first[1],
        warm_last[1],
        warm_silent[2],
        warm_first[2],
        warm_last[2],
        warm_silent[3],
        warm_first[3],
        warm_last[3]
    );
    defmt::info!(
        "[STAT] DShot{=u32} 1 kHz: silent captures among the first 64 (bit n = capture n): M4 0x{=u64:x} M3 0x{=u64:x} M2 0x{=u64:x} M1 0x{=u64:x}; start-to-start of the first 40 {=[?]}",
        label,
        mask[0],
        mask[1],
        mask[2],
        mask[3],
        &gaps[..]
    );

    // Step 1: our own frame as seen on the pins.
    let nominal_bit = rate.bit_ps;
    let lo0_nom = nominal_bit / 8 * 3;
    let lo1_nom = nominal_bit / 4 * 3;
    for (i, a) in frame_acc.iter().enumerate() {
        defmt::info!(
            "[STAT] DShot{=u32} {=str}: own frame on the pin, {=u32} frames: bit time {=u32}..{=u32} ns (nominal {=u32}), single periods {=u32}..{=u32} ns, low of 0 bits {=u32}..{=u32} ns (nominal {=u32}), low of 1 bits {=u32}..{=u32} ns (nominal {=u32}), first fall to last rise {=u32}..{=u32} ns (nominal {=u32}), frames not 32 edges {=u32}",
            label,
            NAMES[i],
            a.frames,
            ns(a.bit_min),
            ns(a.bit_max),
            ns(nominal_bit),
            ns(a.per_min),
            ns(a.per_max),
            ns(a.lo0_min),
            ns(a.lo0_max),
            ns(lo0_nom),
            ns(a.lo1_min),
            ns(a.lo1_max),
            ns(lo1_nom),
            ns(a.total_min),
            ns(a.total_max),
            ns(nominal_bit / 4 * 63),
            a.bad_count
        );
        let bit_ok = a.bit_min as u64 * 100 >= nominal_bit as u64 * 99
            && a.bit_max as u64 * 100 <= nominal_bit as u64 * 101;
        let lo0_ok = a.lo0_min as u64 * 100 >= lo0_nom as u64 * 95
            && a.lo0_max as u64 * 100 <= lo0_nom as u64 * 105;
        let lo1_ok = a.lo1_min as u64 * 100 >= lo1_nom as u64 * 95
            && a.lo1_max as u64 * 100 <= lo1_nom as u64 * 105;
        check!(
            ctx.fails,
            a.bad_count == 0 && bit_ok && lo0_ok && lo1_ok,
            "[{=str}] DShot{=u32} {=str}: own frame bit time within 1 % and low times within 5 % of 37.5 % and 75 %",
            label,
            NAMES[i]
        );
    }
    defmt::info!(
        "[STAT] DShot{=u32}: output enable seen during {=u32} of {=u32} observed frames (positive control), outside a frame in {=u32} of {=u32} frames",
        label,
        ctx.h.oe_during_frame,
        ctx.h.observed,
        ctx.h.oe_after_frame,
        ctx.h.frames
    );
    check!(
        ctx.fails,
        ctx.h.oe_during_frame == ctx.h.observed && ctx.h.oe_after_frame == 0,
        "[{=str}] DShot{=u32}: lines driven during every observed frame, never after a frame (PIO2 DBG_PADOE)",
        label
    );

    if none_replied {
        defmt::info!(
            "[INFO] DShot{=u32}: no line replied in {=u32} frames. Edges seen in the reply windows over all frames: {=u32} {=u32} {=u32} {=u32}",
            label,
            idx + 1,
            edge_totals[0],
            edge_totals[1],
            edge_totals[2],
            edge_totals[3]
        );
        print_dumps(label, &dumps, 0, 6);
        check!(
            ctx.fails,
            false,
            "[{=str}] DShot{=u32}: some line gave a decodable reply within 10000 frames",
            label
        );
        return BlockOut {
            window_ps: DEFAULT_WINDOW_PS,
            fit_median: 0,
            worked_period_us: None,
            first_ok_ms,
            none_replied: true,
        };
    }

    // Steps 2 and 3.
    defmt::info!(
        "[STAT] DShot{=u32}: receive FIFO stalls {=u32}, leftover words {=u32}, DMA timeouts {=u32}, TX pushes refused {=u32}, observer stalls {=u32}; frame start to start {=u32}..{=u32} us over {=u32} frames",
        label,
        ctx.h.stalls,
        ctx.h.leftovers,
        ctx.h.timeouts,
        ctx.h.push_failed,
        ctx.h.obs_stalls,
        ctx.spacing_min,
        ctx.spacing_max,
        ctx.h.frames
    );
    check!(
        ctx.fails,
        ctx.h.stalls == 0
            && ctx.h.leftovers == 0
            && ctx.h.timeouts == 0
            && ctx.h.push_failed == 0
            && ctx.h.obs_stalls == 0,
        "[{=str}] DShot{=u32}: receive FIFO never overflowed, every capture exactly {=usize} words",
        label,
        ctx.words
    );
    for i in 0..LINES {
        defmt::info!(
            "[STAT] DShot{=u32} {=str}: first reply edge in frame {=u32}, first reply decoding to 0xFFF in frame {=u32} (that many frames were sent before it; {=u32} ms after the first frame)",
            label,
            NAMES[i],
            first_reply[i].unwrap_or(u32::MAX),
            first_ok[i].unwrap_or(u32::MAX),
            first_ok_ms[i].unwrap_or(u32::MAX)
        );
    }

    // Steps 4 and 5.
    for (i, l) in stats.lane.iter_mut().enumerate() {
        print_lane(rate, i, l, &mut ctx.fails);
    }
    defmt::info!(
        "[STAT] DShot{=u32}: spread of the reply start between lines within one capture, max {=u32} ns over {=u32} captures",
        label,
        ns(stats.spread_max),
        stats.captures
    );
    if calib_n > 0 {
        let n = calib_n as i64;
        defmt::info!(
            "[STAT] DShot{=u32}: cross-check with the observer (13 ns samples), {=u32} line replies: reply start main sampler minus observer min {=i32} mean {=i32} max {=i32} ns (observer mean start {=i32} ns), reply bit time main minus observer mean {=i32} ns",
            label,
            calib_n,
            calib_min,
            (calib_sum / n) as i32,
            calib_max,
            (calib_obs_start_sum / n) as i32,
            (calib_fit_sum / n) as i32
        );
    }

    // Step 8, first part: the window and the bit time.
    let mut end_max = 0;
    let mut meds = [0u32; LINES];
    for (m, l) in meds.iter_mut().zip(stats.lane.iter_mut()) {
        end_max = end_max.max(l.end21_max);
        *m = min_med_max(&mut l.fit[..l.n_fit]).1;
    }
    let mut sorted = meds;
    sorted.sort_unstable();
    let fit_median = sorted[LINES / 2];
    let release_ps = RELEASE_CYCLES * c_ps;
    let window_ps = if end_max == 0 {
        DEFAULT_WINDOW_PS
    } else {
        end_max.saturating_sub(release_ps) + MARGIN_PS
    };
    let worked_1k = stats.lane.iter().all(|l| l.fff >= 990);

    // ---- 6. higher frame rates with the shortened window.
    let words = rate.words_for(window_ps);
    let win_ps = words as u32 * rate.word_ps();
    let mut worked_us = if worked_1k { Some(1000) } else { None };
    for hz in [2000u32, 4000, 8000] {
        let period_us = 1_000_000 / hz;
        let need_ps = rate.frame_ps() + win_ps + 10_000_000;
        let fits = need_ps <= period_us * 1_000_000;
        defmt::info!(
            "[STAT] DShot{=u32} {=u32} Hz: frame {=u32} ns + window {=u32} ns + 10 us turnaround = {=u32} ns against a period of {=u32} ns: {=str}",
            label,
            hz,
            ns(rate.frame_ps()),
            ns(win_ps),
            ns(need_ps),
            period_us * 1000,
            if fits { "fits" } else { "does not fit" }
        );
        if !fits {
            defmt::info!(
                "[INFO] DShot{=u32} {=u32} Hz: skipped, the frame and the reply do not fit in the period",
                label,
                hz
            );
            continue;
        }
        ctx.configure(rate, words, false);
        ctx.reset_spacing();
        let before = (
            ctx.h.stalls,
            ctx.h.leftovers,
            ctx.h.timeouts,
            ctx.h.push_failed,
            ctx.h.oe_after_frame,
        );
        stats.reset();
        let mut ticker = Ticker::every(Duration::from_micros(period_us as u64));
        for f in 0..(50 + N as u32) {
            ticker.next().await;
            if !ctx.frame(false).await {
                ctx.abort("capture DMA timed out");
            }
            if f >= 50 {
                let e = extract(&ctx.buf[..ctx.words]);
                stats.add(&decode_main(&e, rate));
            }
        }
        let after = (
            ctx.h.stalls,
            ctx.h.leftovers,
            ctx.h.timeouts,
            ctx.h.push_failed,
            ctx.h.oe_after_frame,
        );
        let clean = before == after;
        defmt::info!(
            "[STAT] DShot{=u32} {=u32} Hz: replies decoded to 0xFFF out of {=u32}: {=str} {=u32}, {=str} {=u32}, {=str} {=u32}, {=str} {=u32}; good checksum {=u32} {=u32} {=u32} {=u32}; silent {=u32} {=u32} {=u32} {=u32}; frame start to start {=u32}..{=u32} us; FIFO stalls/leftovers/timeouts/refused/OE {=u32} {=u32} {=u32} {=u32} {=u32}",
            label,
            hz,
            stats.captures,
            NAMES[0],
            stats.lane[0].fff,
            NAMES[1],
            stats.lane[1].fff,
            NAMES[2],
            stats.lane[2].fff,
            NAMES[3],
            stats.lane[3].fff,
            stats.lane[0].ok,
            stats.lane[1].ok,
            stats.lane[2].ok,
            stats.lane[3].ok,
            stats.lane[0].silent,
            stats.lane[1].silent,
            stats.lane[2].silent,
            stats.lane[3].silent,
            ctx.spacing_min,
            ctx.spacing_max,
            after.0 - before.0,
            after.1 - before.1,
            after.2 - before.2,
            after.3 - before.3,
            after.4 - before.4
        );
        check!(
            ctx.fails,
            clean,
            "[{=str}] DShot{=u32} {=u32} Hz: receive FIFO never overflowed and the lines were released after every frame",
            label,
            hz
        );
        if stats.lane.iter().all(|l| l.fff >= 990) && worked_us.is_none_or(|w| period_us < w) {
            worked_us = Some(period_us);
        }
    }

    print_dumps(label, &dumps, 0, 3);
    BlockOut {
        window_ps,
        fit_median,
        worked_period_us: worked_us,
        first_ok_ms,
        none_replied: false,
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("[INFO] dshot_m1_probe: start");
    let p = embassy_rp::init(Default::default());
    let hz = embassy_rp::clocks::clk_sys_freq();
    defmt::info!("[INFO] clk_sys {=u32} Hz", hz);
    let mut fails = 0u32;
    check!(fails, hz == CLK_HZ, "[{=str}] clk_sys is 150 MHz");
    self_test(&mut fails);
    if fails != 0 {
        hil_common::exit(fails);
    }

    let Pio {
        mut common,
        mut sm0,
        mut sm1,
        ..
    } = Pio::new(p.PIO2, Irqs);
    let mut pins = [
        common.make_pio_pin(p.PIN_36),
        common.make_pio_pin(p.PIN_37),
        common.make_pio_pin(p.PIN_38),
        common.make_pio_pin(p.PIN_39),
    ];
    for pin in &mut pins {
        pin.set_pull(Pull::Up);
    }
    let [a, b, c, d] = &pins;
    let line_refs = [a, b, c, d];
    // The pins are sampled with no synchroniser flip-flops: PIO pin numbers 20-23.
    common.set_input_sync_bypass(OE_MASK, OE_MASK);

    let main_prog = pio_asm!(
        ".wrap_target",
        "idle:",
        "    mov x, status",
        "    jmp x-- idle",
        "    set x, 15",
        "    set pindirs, 15",
        "bit:",
        "    set pins, 0",
        "    pull ifempty block [13]",
        "    out pins, 4 [14]",
        "    set pins, 15 [8]",
        "    jmp x-- bit",
        "    set pindirs, 0",
        "    mov x, y",
        "smp:",
        "    in pins, 4",
        "    jmp x-- smp",
        ".wrap",
    );
    let obs_prog = pio_asm!(
        "mov x, y",
        "smp:",
        "    in pins, 4",
        "    jmp x-- smp",
        "    pull block"
    );
    let main_loaded = common.load_program(&main_prog.program);
    let obs_loaded = common.load_program(&obs_prog.program);

    let mut cfg0 = Config::default();
    cfg0.use_program(&main_loaded, &[]);
    cfg0.clock_divider = U24F8::from_bits(RATES[0].div_q8);
    cfg0.shift_out = ShiftConfig {
        threshold: 32,
        direction: ShiftDirection::Right,
        auto_fill: false,
    };
    cfg0.shift_in = ShiftConfig {
        threshold: 32,
        direction: ShiftDirection::Left,
        auto_fill: true,
    };
    cfg0.status_sel = StatusSource::TxFifoLevel;
    cfg0.status_n = StatusN::This(2);
    cfg0.set_set_pins(&line_refs);
    cfg0.set_out_pins(&line_refs);
    cfg0.set_in_pins(&line_refs);

    let mut cfg1 = Config::default();
    cfg1.use_program(&obs_loaded, &[]);
    cfg1.shift_in = ShiftConfig {
        threshold: 32,
        direction: ShiftDirection::Left,
        auto_fill: true,
    };
    cfg1.set_in_pins(&line_refs);

    sm0.set_enable(false);
    sm1.set_enable(false);
    sm0.set_config(&cfg0);
    sm1.set_config(&cfg1);
    // Latch high, direction input: the pad pull-up holds the lines until the first frame.
    sm0.set_pins(Level::High, &line_refs);
    sm0.set_pin_dirs(pio::Direction::In, &line_refs);

    let dma0 = dma::Channel::new(p.DMA_CH1, Irqs);
    let dma1 = dma::Channel::new(p.DMA_CH2, Irqs);
    let mut ctx = Ctx {
        sm0,
        sm1,
        dma0,
        dma1,
        main_origin: main_loaded.origin,
        obs_origin: obs_loaded.origin,
        buf: [0; MAX_WORDS],
        words: 0,
        obs: [0; OBS_WORDS],
        h: Health::default(),
        last_start: None,
        spacing_min: u32::MAX,
        spacing_max: 0,
        last_gap: 0,
        fails,
    };
    let mut stats = Stats::new();
    defmt::info!(
        "[INFO] PIO2: main program {=u8} at {=u8}, observer at {=u8}; pad pull-ups on, GPIO36-39 are PIO pins 20-23 (GPIOBASE 16)",
        main_prog.program.code.len() as u8,
        main_loaded.origin,
        obs_loaded.origin
    );

    let mut outs: [Option<BlockOut>; 2] = [None, None];
    for (bi, rate) in RATES.iter().enumerate() {
        outs[bi] = Some(block(&mut ctx, *rate, &mut stats).await);
        if bi == 0 && outs[0].as_ref().is_some_and(|o| o.none_replied) {
            break;
        }
    }

    // ---- 7 and 8: summary per rate.
    for (rate, out) in RATES.iter().zip(&outs) {
        let Some(o) = out else { continue };
        if o.none_replied {
            continue;
        }
        let nominal = rate.bit_ps / 5 * 4;
        let ratio = o.fit_median as f32 / nominal as f32;
        defmt::info!(
            "[STAT] DShot{=u32} summary: capture window needed {=u32} ns after the release of the lines (latest reply end + 10 us margin), shortest frame period that worked {=u32} us, reply bit time {=u32} ns = {=f32} of the nominal 5/4 of the command rate ({=f32} % {=str}), first reply decoding to 0xFFF {=u32}/{=u32}/{=u32}/{=u32} ms after the first frame",
            rate.label,
            ns(o.window_ps),
            o.worked_period_us.unwrap_or(0),
            ns(o.fit_median),
            ratio,
            (ratio - 1.0) * 100.0,
            if ratio < 1.0 { "faster" } else { "slower" },
            o.first_ok_ms[0].unwrap_or(u32::MAX),
            o.first_ok_ms[1].unwrap_or(u32::MAX),
            o.first_ok_ms[2].unwrap_or(u32::MAX),
            o.first_ok_ms[3].unwrap_or(u32::MAX)
        );
    }

    // ---- Control (a report, not a pass criterion): the old procedure, 3 s of released HIGH lines.
    if !outs
        .iter()
        .any(|o| o.as_ref().is_some_and(|o| o.none_replied))
    {
        let rate = RATES[1];
        ctx.configure(rate, rate.words_for(DEFAULT_WINDOW_PS), false);
        defmt::info!(
            "[INFO] control: lines released HIGH for 3000 ms with no frames (the old procedure), then 5 STOP frames at DShot600, 1 kHz"
        );
        Timer::after_secs(3).await;
        let mut ticker = Ticker::every(Duration::from_micros(1000));
        let mut bad_frames = 0u32;
        for f in 0..5u32 {
            ticker.next().await;
            if !ctx.frame(true).await {
                ctx.abort("control capture DMA timed out");
            }
            let oe = extract(&ctx.obs);
            let mp = extract(&ctx.buf[..ctx.words]);
            let rs = decode_main(&mp, rate);
            let nib = ctx.obs[0] >> 28;
            let mut pre = [false; LINES];
            let mut post_us = [0u32; LINES];
            let mut decodable = 0u32;
            for i in 0..LINES {
                pre[i] = nib >> i & 1 == 0;
                if let Some(j) = find_frame(&oe[i], rate.bit_ps)
                    && j + 32 < oe[i].n
                {
                    let k0 = oe[i].k[j + 32];
                    let k1 = if j + 33 < oe[i].n {
                        oe[i].k[j + 33]
                    } else {
                        OBS_WORDS as u32 * 8
                    };
                    post_us[i] = (k1 - k0) * OBS_PERIOD_PS / 1_000_000;
                }
                decodable += u32::from(rs[i].kind == Kind::Ok && rs[i].value == 0xFFF);
            }
            let old_pattern =
                (pre.iter().any(|&b| b) || post_us.iter().any(|&u| u >= 30)) && decodable == 0;
            bad_frames += u32::from(old_pattern);
            defmt::info!(
                "[INFO] control frame {=u32}: line low before the frame {=bool} {=bool} {=bool} {=bool}; first low after the frame lasts {=u32} {=u32} {=u32} {=u32} us; lines with a reply decoding to 0xFFF {=u32}; old pattern {=bool}",
                f,
                pre[0],
                pre[1],
                pre[2],
                pre[3],
                post_us[0],
                post_us[1],
                post_us[2],
                post_us[3],
                decodable,
                old_pattern
            );
        }
        defmt::info!(
            "[INFO] control: the old pattern (low before a frame or a low of 30 us or more after it, with no decodable reply) was seen in {=u32} of 5 frames",
            bad_frames
        );
        // Then all four lines low for 25 ms, then inputs with pull-ups.
        ctx.configure(rate, rate.words_for(DEFAULT_WINDOW_PS), true);
        Timer::after_millis(25).await;
    }

    // Lines released: inputs with pull-ups, nothing driven.
    ctx.release_lines();
    let oe = pac::PIO2.dbg_padoe().read() & OE_MASK;
    let lv = lines_high();
    defmt::info!(
        "[STAT] end: PIO2 DBG_PADOE for GPIO36-39 = 0x{=u32:x}, levels {=bool} {=bool} {=bool} {=bool}",
        oe >> PIN_BASE,
        lv[0],
        lv[1],
        lv[2],
        lv[3]
    );
    check!(
        ctx.fails,
        oe == 0,
        "[{=str}] end: no line driven, all inputs with pull-ups"
    );
    let fails = ctx.fails;
    hil_common::exit(fails)
}
