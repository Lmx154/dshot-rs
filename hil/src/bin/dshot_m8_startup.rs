//! M8 measurement: what the four AM32 controllers do to their signal lines during the first 800 ms
//! after they leave their bootloaders, with and without DShot frames arriving.
//!
//! The only frame ever put on a line is the bidirectional STOP frame `STOP_IMAGE`, and the only other
//! thing driven is the 25 ms break (all four lines low). Between them the four pins are inputs with
//! the pad pull-up on. Touches only GPIO36-39, PIO0 and PIO2.
//!
//! - PIO2 state machine 0 (the M1 program without the sampling part) sends the STOP frame on
//!   GPIO36-39 once both of its words are in the TX FIFO. The break is driven and released by
//!   executing `set pins` / `set pindirs` on it.
//! - PIO0 state machines 0-3: one run-length observer per line. Each pushes the length of every
//!   high and low run; the CPU drains the four FIFOs in a polling loop and rebuilds the edge times.
//!   The four are enabled by one write, so they share a time base. Resolution 2 PIO cycles = 13.3 ns:
//!   a high run of n counts lasted 4 + 2n cycles, a low run 6 + 2n cycles (cycles of 6.67 ns).
//!   An overflow of the 8-entry RX FIFO stalls the state machine (`push block`) and sets RXSTALL.
//!
//! Observer program, per line:
//!
//! ```text
//! lo_start: mov x, ~null           ; low-run counter
//! lo_loop:  jmp pin hi_push        ; 2 cycles per count
//!           jmp x-- lo_loop
//!           jmp lo_start
//! hi_push:  mov isr, ~x ; push block ; mov x, ~null
//! hi_loop:  jmp pin hi_dec
//!           jmp lo_push
//! hi_dec:   jmp x-- hi_loop
//!           jmp hi_loop
//! lo_push:  mov isr, ~x ; push block ; jmp lo_start
//! ```
//!
//! The CPU time base of a run is the observer's: time 0 is the observer start; "zero" of the case is
//! the release of the break, taken as the first falling edge of GPIO36 (our own drive) plus the
//! break length measured by the CPU timer.
//!
//! While frames are sent, the edges inside a window around each of the 800 frames are checked against
//! the pattern of the frame as they arrive, and only the windows that fail (and a few kept for the
//! raw dump) are stored; the edges between the windows are stored unless they decode as a valid reply.
#![no_std]
#![no_main]
#![allow(clippy::needless_range_loop)]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Pull};
use embassy_rp::peripherals::{PIO0, PIO2};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    self, Config, FifoJoin, Pin, Pio, ShiftConfig, ShiftDirection, StateMachine, StatusN,
    StatusSource,
};
use embassy_rp::{bind_interrupts, pac};
use embassy_time::{Instant, Timer};
use fixed::types::U24F8;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    PIO2_IRQ_0 => pio::InterruptHandler<PIO2>;
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

/// THE frame: the bidirectional STOP frame, as in the M1 probe. Nothing else is ever written to the
/// TX FIFO of the state machine that drives the lines.
const STOP_IMAGE: [u32; 2] = [0xFFFF_FFFF, 0x0000_FFFF];
/// DShot300: PIO divider 12.5, one bit = 40 PIO cycles = 500 cycles of clk_sys.
const DIV_Q8: u32 = 3200;

const PARK_MS: u64 = 3500;
const BREAK_US: u64 = 25_000;
const LEAD_US: u64 = 1_000;
const RECORD_US: u64 = 800_000;
const FIRST_FRAME_US: u64 = 330;
const FRAME_PERIOD_US: u64 = 1_000;
const FRAMES: u32 = 800;

/// Cycles (6.67 ns) per microsecond.
const CPU_US: u32 = 150;
const PERIOD_C: u32 = FRAME_PERIOD_US as u32 * CPU_US;
/// Frames are told apart in cells of one period that start 250 us after the release; the window of
/// a frame starts 2 us before the moment the CPU pushed it and lasts 60 us.
const CELL_OFF_C: u32 = 250 * CPU_US;
const WIN_PRE_C: u32 = 2 * CPU_US;
const WIN_LEN_C: u32 = 9_000;
const QN: usize = 1024;
const MAXW: usize = 64;
const EV_CAP: usize = 24_576;
const LOW_CAP: usize = 512;

const FLAG_GAP: u32 = 0;
const FLAG_KEEP: u32 = 1;
const FLAG_ANOM: u32 = 2;

/// The ideal edge times of the STOP frame on a line that idles high, from its first falling edge, in
/// half cycles of clk_sys: falling edge i at 500 i cycles, rising edge after 187.5 cycles (bits 0..11)
/// or 375 (bits 12..15); with a 33rd edge, a fall at the release, when something pulls the line low.
const IDEAL_A: [u32; 33] = {
    let mut t = [0u32; 33];
    let mut i = 0;
    while i < 16 {
        t[2 * i] = 1000 * i as u32;
        t[2 * i + 1] = 1000 * i as u32 + if i < 12 { 375 } else { 750 };
        i += 1;
    }
    t[32] = 16000;
    t
};
/// The same frame on a line that something else holds low: the first edge is the spike of one PIO
/// cycle (12.5 cycles = 25 half cycles) that our output makes when it is enabled with the latch high,
/// then the highs, and the line falls at the start of each next bit and at the release of the line.
const IDEAL_B: [u32; 34] = {
    let mut t = [0u32; 34];
    t[1] = 25;
    let mut i = 0;
    while i < 16 {
        t[2 + 2 * i] = 25 + 1000 * i as u32 + if i < 12 { 375 } else { 750 };
        t[3 + 2 * i] = 25 + 1000 * (i as u32 + 1);
        i += 1;
    }
    t
};

#[repr(C, align(4096))]
struct RingMem([u32; QN]);
/// The DMA rings, one per observer: each channel writes the words of its observer here.
static mut RINGS: [RingMem; LINES] = [const { RingMem([0; QN]) }; LINES];
/// DMA channels 4 to 7 serve the four observers.
const DMA_FIRST: usize = 4;

fn ring_base(i: usize) -> usize {
    (&raw mut RINGS as *mut RingMem as usize) + i * core::mem::size_of::<RingMem>()
}

fn ns(c: i64) -> i32 {
    (c * 20 / 3) as i32
}

fn now_us() -> u64 {
    Instant::now().as_micros()
}

fn settle() {
    cortex_m::asm::delay(2_000);
}

// ---------------------------------------------------------------------------------------------
// Reply decoder, as in the M1 probe

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
    fit: u32,
    value: u16,
}

/// Decodes one line. `times` are the edge times in ps, rising and falling alternately, the first
/// one falling. 21 symbols: one leading, then 20 NRZI symbols (a 1 toggles the line) that are
/// four 5-bit GCR codes, most significant first.
fn decode(times: &[u32], over: bool, cmd_bit_ps: u32) -> Reply {
    let n = times.len();
    let mut r = Reply {
        kind: Kind::Silent,
        fit: 0,
        value: 0,
    };
    if n == 0 {
        return r;
    }
    r.kind = Kind::Shape;
    if over {
        return r;
    }
    if n < 2 || n % 2 == 1 {
        r.kind = Kind::Truncated;
        return r;
    }
    let nominal = cmd_bit_ps / 5 * 4;
    let mut est = (nominal as u64 * 9625 / 10_000) as u32;
    let mut slots = [0u32; MAXW];
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
    r.fit = fit;
    if slots[n - 1] > 21 || fit * 4 < nominal * 3 || fit * 4 > nominal * 5 {
        return r;
    }
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

fn self_test(fails: &mut u32) {
    const SLOTS: [u32; 16] = [0, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 16, 17, 20];
    let mut t = [0u32; 16];
    for (s, o) in SLOTS.iter().zip(t.iter_mut()) {
        *o = 20_000_000 + s * 2_567_000;
    }
    let r = decode(&t, false, 3_333_333);
    let ok = r.kind == Kind::Ok && r.value == 0xFFF;
    let mut moved = SLOTS;
    moved[15] = 19;
    for (s, o) in moved.iter().zip(t.iter_mut()) {
        *o = 20_000_000 + s * 2_567_000;
    }
    let r = decode(&t, false, 3_333_333);
    let neg = !(r.kind == Kind::Ok && r.value == 0xFFF);
    check!(
        *fails,
        ok,
        "[{=str}] decoder self-test: synthetic 0xFFF reply decodes"
    );
    check!(
        *fails,
        neg,
        "[{=str}] decoder self-test: a corrupted reply does not decode to 0xFFF"
    );
}

// ---------------------------------------------------------------------------------------------
// The recorder: run words in, edges out

/// Reply classes of a gap: 0 truncated, 1 shape, 2 GCR, 3 checksum, 4 good, 5 low at the start of
/// the gap, 6 no window before it.
const KINDS: usize = 7;

#[derive(Clone, Copy)]
struct Ln {
    t: u32,
    high: bool,
    skip: bool,
    first_t: u32,
    first_rising: bool,
    edges: u32,
    ws: u32,
    we: u32,
    wk: u32,
    wn: usize,
    wover: bool,
    wfirst_high: bool,
    wbuf: [u32; MAXW],
    gn: usize,
    gk: u32,
    gover: bool,
    gfirst_high: bool,
    gref_t: u32,
    gref_ok: bool,
    gbuf: [u32; MAXW],
    ref_t: u32,
    ref_k: u32,
    win_edges: u32,
    win_seen: u32,
    clean: u32,
    bad_fewer: u32,
    bad_more: u32,
    bad_dev: u32,
    first_rise: u32,
    end_low: u32,
    hist: [u32; 5],
    dev_min: [i32; 2],
    dev_max: [i32; 2],
    anom_stored: u32,
    forms: [u32; 3],
    other_n: [u32; 4],
    last_end_low_k: i32,
    first_end_high_k: i32,
    last_first_rise_k: i32,
    kinds: [u32; KINDS],
    gaps: u32,
    first_ok: u32,
    first_fff: u32,
}

impl Ln {
    const NEW: Ln = Ln {
        t: 0,
        high: true,
        skip: true,
        first_t: 0,
        first_rising: false,
        edges: 0,
        ws: 0,
        we: 0,
        wk: 0,
        wn: 0,
        wover: false,
        wfirst_high: false,
        wbuf: [0; MAXW],
        gn: 0,
        gk: 0,
        gover: false,
        gfirst_high: false,
        gref_t: 0,
        gref_ok: false,
        gbuf: [0; MAXW],
        ref_t: 0,
        ref_k: u32::MAX,
        win_edges: 0,
        win_seen: 0,
        clean: 0,
        bad_fewer: 0,
        bad_more: 0,
        bad_dev: 0,
        first_rise: 0,
        end_low: 0,
        hist: [0; 5],
        dev_min: [i32::MAX; 2],
        dev_max: [i32::MIN; 2],
        anom_stored: 0,
        forms: [0; 3],
        other_n: [0; 4],
        last_end_low_k: -1,
        first_end_high_k: -1,
        last_first_rise_k: -1,
        kinds: [0; KINDS],
        gaps: 0,
        first_ok: u32::MAX,
        first_fff: u32::MAX,
    };
}

struct Rec {
    l: [Ln; LINES],
    ev: [u32; EV_CAP],
    ev_n: usize,
    ev_drop: u32,
    frames: bool,
    armed: bool,
    zero: u32,
    pushed: u32,
    fpush: [u32; FRAMES as usize],
    qh: [u32; LINES],
    qt: [u32; LINES],
    q_over: u32,
}

fn ev_t(e: u32) -> u32 {
    e >> 5
}
fn ev_flag(e: u32) -> u32 {
    (e >> 3) & 3
}
fn ev_line(e: u32) -> usize {
    ((e >> 1) & 3) as usize
}
fn ev_high(e: u32) -> bool {
    e & 1 != 0
}

fn kept(k: u32) -> bool {
    k < 5 || (300..305).contains(&k) || (600..605).contains(&k)
}

impl Rec {
    fn new() -> Rec {
        Rec {
            l: [Ln::NEW; LINES],
            ev: [0; EV_CAP],
            ev_n: 0,
            ev_drop: 0,
            frames: false,
            armed: false,
            zero: 0,
            pushed: 0,
            fpush: [0; FRAMES as usize],
            qh: [0; LINES],
            qt: [0; LINES],
            q_over: 0,
        }
    }

    fn reset(&mut self, frames: bool) {
        self.l.fill(Ln::NEW);
        self.ev_n = 0;
        self.ev_drop = 0;
        self.frames = frames;
        self.armed = false;
        self.zero = 0;
        self.pushed = 0;
        self.qh = [0; LINES];
        self.qt = [0; LINES];
        self.q_over = 0;
    }

    fn push_ev(&mut self, i: usize, t: u32, high: bool, flag: u32) {
        if self.ev_n < EV_CAP {
            self.ev[self.ev_n] = t << 5 | flag << 3 | (i as u32) << 1 | u32::from(high);
            self.ev_n += 1;
        } else {
            self.ev_drop += 1;
        }
    }

    /// One word from the observer of line `i`.
    #[inline]
    fn word(&mut self, i: usize, w: u32) {
        let ln = &mut self.l[i];
        if ln.skip {
            ln.skip = false;
            return;
        }
        let rising = !ln.high;
        ln.t =
            ln.t.wrapping_add(if ln.high { 4 } else { 6 })
                .wrapping_add(w << 1);
        ln.high = rising;
        let t = ln.t;
        ln.edges += 1;
        if t >= ln.ws && t < ln.we {
            if ln.wn == 0 {
                ln.wfirst_high = rising;
                ln.wover = false;
            }
            if ln.wn < MAXW {
                ln.wbuf[ln.wn] = t;
                ln.wn += 1;
            } else {
                ln.wover = true;
            }
            return;
        }
        self.slow(i, t, rising);
    }

    /// Window of frame `m` in observer cycles, once the CPU has pushed that frame.
    fn win_bounds(&self, m: u32) -> Option<(u32, u32)> {
        if m >= self.pushed {
            return None;
        }
        let ws = self.zero + self.fpush[m as usize] * CPU_US - WIN_PRE_C;
        Some((ws, ws + WIN_LEN_C))
    }

    fn slow(&mut self, i: usize, t: u32, rising: bool) {
        if self.l[i].edges == 1 {
            self.l[i].first_t = t;
            self.l[i].first_rising = rising;
        }
        if !self.frames || !self.armed || t < self.zero + CELL_OFF_C {
            self.push_ev(i, t, rising, FLAG_GAP);
            return;
        }
        let m = (t - self.zero - CELL_OFF_C) / PERIOD_C;
        let (inwin, gm) = match self.win_bounds(m) {
            Some((ws, we)) if t >= ws => (t < we, m),
            _ => (false, m.wrapping_sub(1)),
        };
        self.close_window(i);
        if inwin {
            let (ws, we) = self.win_bounds(m).unwrap_or((0, 0));
            let ln = &mut self.l[i];
            ln.wk = m;
            ln.ws = ws;
            ln.we = we;
            ln.wn = 1;
            ln.wbuf[0] = t;
            ln.wfirst_high = rising;
            ln.wover = false;
        } else if gm == u32::MAX {
            self.push_ev(i, t, rising, FLAG_GAP);
        } else {
            let (ws, we) = self.win_bounds(gm + 1).unwrap_or((0, 0));
            {
                let ln = &mut self.l[i];
                ln.wk = gm + 1;
                ln.ws = ws;
                ln.we = we;
            }
            if self.l[i].gn > 0 && self.l[i].gk != gm {
                self.gap_flush(i);
            }
            let ln = &mut self.l[i];
            if ln.gn == 0 {
                ln.gk = gm;
                ln.gfirst_high = rising;
                ln.gover = false;
                ln.gref_t = ln.ref_t;
                ln.gref_ok = ln.ref_k == gm;
            }
            if ln.gn < MAXW {
                ln.gbuf[ln.gn] = t;
                ln.gn += 1;
            } else {
                ln.gover = true;
            }
        }
    }

    fn close_window(&mut self, i: usize) {
        if self.l[i].wn == 0 {
            return;
        }
        self.eval_window(i);
        let ln = &mut self.l[i];
        ln.ref_t = ln.wbuf[ln.wn - 1];
        ln.ref_k = ln.wk;
        ln.wn = 0;
    }

    fn eval_window(&mut self, i: usize) {
        let buf = self.l[i].wbuf;
        let n = self.l[i].wn;
        let over = self.l[i].wover;
        let k = self.l[i].wk;
        let first_high = self.l[i].wfirst_high;
        let last_high = first_high ^ ((n - 1) & 1 == 1);
        let ln = &mut self.l[i];
        ln.win_seen += 1;
        ln.win_edges += n as u32;
        if first_high {
            ln.first_rise += 1;
        }
        if !last_high {
            ln.end_low += 1;
        }
        let mut clean = false;
        let (ideal, form): (&[u32], usize) = match (n, first_high) {
            (32, false) => (&IDEAL_A[..32], 0),
            (33, false) => (&IDEAL_A[..33], 1),
            (34, true) => (&IDEAL_B[..], 2),
            _ => (&[], 3),
        };
        if form == 3 || over {
            let c = if over {
                3
            } else if n < 32 {
                0
            } else if n > 34 {
                1
            } else {
                2
            };
            ln.other_n[c] += 1;
            if n < 32 {
                ln.bad_fewer += 1;
            } else {
                ln.bad_more += 1;
            }
        } else {
            ln.forms[form] += 1;
            let f0 = buf[0];
            let mut maxd = 0u32;
            for (j, (&b, &id)) in buf.iter().zip(ideal.iter()).enumerate() {
                let d = 2 * (b as i32 - f0 as i32) - id as i32;
                maxd = maxd.max(d.unsigned_abs());
                let ty = usize::from(first_high ^ (j & 1 == 1));
                ln.dev_min[ty] = ln.dev_min[ty].min(d);
                ln.dev_max[ty] = ln.dev_max[ty].max(d);
            }
            let bucket = match maxd {
                0..=4 => 0,
                5..=8 => 1,
                9..=16 => 2,
                17..=32 => 3,
                _ => 4,
            };
            ln.hist[bucket] += 1;
            if maxd <= 8 {
                clean = true;
                ln.clean += 1;
            } else {
                ln.bad_dev += 1;
            }
        }
        if !last_high {
            ln.last_end_low_k = k as i32;
        } else if ln.first_end_high_k < 0 {
            ln.first_end_high_k = k as i32;
        }
        if first_high {
            ln.last_first_rise_k = k as i32;
        }
        let flag = if !clean && ln.anom_stored < 20 {
            ln.anom_stored += 1;
            Some(FLAG_ANOM)
        } else if kept(k) {
            Some(FLAG_KEEP)
        } else {
            None
        };
        if let Some(flag) = flag {
            for (j, &t) in buf[..n].iter().enumerate() {
                self.push_ev(i, t, first_high ^ (j & 1 == 1), flag);
            }
        }
    }

    fn gap_flush(&mut self, i: usize) {
        let n = self.l[i].gn;
        if n == 0 {
            return;
        }
        let buf = self.l[i].gbuf;
        let over = self.l[i].gover;
        let first_high = self.l[i].gfirst_high;
        let gk = self.l[i].gk;
        let gref = self.l[i].gref_t;
        let mut kind = 6;
        let mut value = 0;
        if self.l[i].gref_ok {
            if first_high {
                kind = 5;
            } else {
                let mut times = [0u32; MAXW];
                for (o, &t) in times.iter_mut().zip(&buf[..n]) {
                    *o = (t - gref) * 6667;
                }
                let r = decode(&times[..n], over, 3_333_333);
                value = r.value;
                kind = match r.kind {
                    Kind::Silent | Kind::Truncated => 0,
                    Kind::Shape => 1,
                    Kind::Gcr => 2,
                    Kind::Checksum => 3,
                    Kind::Ok => 4,
                };
            }
        }
        let ln = &mut self.l[i];
        ln.gaps += 1;
        ln.kinds[kind] += 1;
        ln.gn = 0;
        if kind == 4 {
            ln.first_ok = ln.first_ok.min(gk);
            if value == 0xFFF {
                ln.first_fff = ln.first_fff.min(gk);
            }
        } else {
            for (j, &t) in buf[..n].iter().enumerate() {
                self.push_ev(i, t, first_high ^ (j & 1 == 1), FLAG_GAP);
            }
        }
    }

    /// Runs the window, gap and decode code once on made-up edges, so that it is in the flash cache
    /// before the real run (a first call from flash takes about 100 us, longer than the FIFOs hold).
    fn warm_up(&mut self) {
        self.reset(true);
        self.armed = true;
        self.zero = 1_000_000;
        self.pushed = 40;
        for (m, f) in self.fpush.iter_mut().enumerate().take(40) {
            *f = 330 + 1000 * m as u32;
        }
        for round in 0..2 {
            for form in 0..2 {
                {
                    let ln = &mut self.l[0];
                    ln.wk = 0;
                    ln.wn = if form == 0 { 34 } else { 32 };
                    ln.wfirst_high = form == 0;
                    ln.wover = false;
                    for (j, b) in ln.wbuf.iter_mut().enumerate().take(34) {
                        *b = 2_000_000 + IDEAL_B[j.min(33)] / 2 + round;
                    }
                }
                self.eval_window(0);
            }
            {
                let ln = &mut self.l[0];
                ln.gn = 16;
                ln.gk = 0;
                ln.gfirst_high = false;
                ln.gref_ok = true;
                ln.gref_t = 3_000_000;
                for (j, b) in ln.gbuf.iter_mut().enumerate().take(16) {
                    *b = 3_000_000 + 4_000 + j as u32 * 400 + (j as u32 & 3) * 7;
                }
            }
            self.gap_flush(0);
            let c = 1_000_000 + CELL_OFF_C;
            self.slow(0, c + 12 * PERIOD_C + 5_000, true);
            self.slow(0, c + 12 * PERIOD_C + 20_000, false);
            self.slow(0, c + 13 * PERIOD_C + 95_000, true);
            self.slow(0, c + 13 * PERIOD_C + 45_000 + 330 * CPU_US, false);
        }
        self.finish();
    }

    fn arm(&mut self, zero: u32) {
        self.zero = zero;
        self.armed = true;
        for ln in &mut self.l {
            ln.wk = 0;
            ln.ws = 0;
            ln.we = 0;
            ln.wn = 0;
        }
    }

    /// Notes the words the DMA channels have written to the rings.
    fn sync_rings(&mut self) {
        for i in 0..LINES {
            let wp = pac::DMA.ch(DMA_FIRST + i).write_addr().read() as usize;
            let idx = ((wp - ring_base(i)) >> 2) as u32 & (QN as u32 - 1);
            let delta = idx.wrapping_sub(self.qh[i]) & (QN as u32 - 1);
            self.qh[i] = self.qh[i].wrapping_add(delta);
            if self.qh[i].wrapping_sub(self.qt[i]) > QN as u32 {
                self.q_over += 1;
                self.qt[i] = self.qh[i].wrapping_sub(QN as u32);
            }
        }
    }

    /// Processes up to `max` staged words.
    fn process(&mut self, max: usize) {
        let mut n = 0;
        for i in 0..LINES {
            while n < max && self.qt[i] != self.qh[i] {
                let t = self.qt[i];
                // SAFETY: the index is inside the ring; the DMA channel only writes it.
                let w = unsafe {
                    ((ring_base(i) as *const u32).add(t as usize & (QN - 1))).read_volatile()
                };
                self.qt[i] = t.wrapping_add(1);
                self.word(i, w);
                n += 1;
            }
        }
    }

    fn finish(&mut self) {
        self.sync_rings();
        self.process(usize::MAX);
        if self.frames && self.armed {
            for i in 0..LINES {
                self.close_window(i);
                self.gap_flush(i);
            }
        }
        self.ev[..self.ev_n].sort_unstable();
    }
}

// ---------------------------------------------------------------------------------------------
// The hardware

struct Ctx {
    sm0: StateMachine<'static, PIO2, 0>,
    o0: StateMachine<'static, PIO0, 0>,
    o1: StateMachine<'static, PIO0, 1>,
    o2: StateMachine<'static, PIO0, 2>,
    o3: StateMachine<'static, PIO0, 3>,
    main_origin: u8,
    obs_origin: u8,
    push_failed: u32,
}

fn arm_obs<const S: usize>(sm: &mut StateMachine<'static, PIO0, S>, origin: u8) {
    sm.set_enable(false);
    sm.restart();
    sm.clear_fifos();
    // SAFETY: the state machine is disabled; the jump only sets its program counter.
    unsafe { sm.exec_jmp(origin) };
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

fn dma_stop() {
    for i in 0..LINES {
        pac::DMA
            .ch(DMA_FIRST + i)
            .ctrl_trig()
            .modify(|w| w.set_en(false));
    }
}

fn obs_enable(on: bool) {
    pac::PIO0
        .ctrl()
        .modify(|w| w.set_sm_enable(if on { 0xF } else { 0 }));
}

impl Ctx {
    /// SM0 of PIO2 at the top of the frame program, lines released, latch high, enabled: it waits
    /// for two words in its TX FIFO.
    fn frame_sm_idle(&mut self) {
        self.sm0.set_enable(false);
        self.sm0.restart();
        self.sm0.clear_fifos();
        // SAFETY: the state machine is disabled; these only set the output latch and direction of
        // the four lines (latch high, direction input) and jump to the top of the program.
        unsafe {
            self.sm0.set_pin(15);
            settle();
            self.sm0.set_pindir(0);
            settle();
            self.sm0.exec_jmp(self.main_origin);
            settle();
        }
        self.sm0.clkdiv_restart();
        self.sm0.set_enable(true);
    }

    fn release_lines(&mut self) {
        self.sm0.set_enable(false);
        // SAFETY: the state machine is disabled; this only releases the lines.
        unsafe { self.sm0.set_pindir(0) };
        settle();
    }

    fn arm_all_obs(&mut self) {
        obs_enable(false);
        arm_obs(&mut self.o0, self.obs_origin);
        arm_obs(&mut self.o1, self.obs_origin);
        arm_obs(&mut self.o2, self.obs_origin);
        arm_obs(&mut self.o3, self.obs_origin);
        settle();
        pac::PIO0.fdebug().write(|w| w.set_rxstall(0xF));
    }
}

#[derive(Clone, Copy, Default)]
struct Run {
    /// Zero of the case in observer cycles (the release of the break).
    zero: u32,
    /// Break length measured by the CPU timer, us.
    d_us: u32,
    /// RXSTALL mask of the observers at the end.
    stall: u8,
    /// Frames pushed.
    pushed: u32,
    /// False if GPIO36 never showed our break.
    got_break: bool,
    /// Observed time between the first falling edge of G36 and the end of the run, us.
    span_us: u32,
    /// Largest lateness of a frame push against its schedule, us.
    late_max: u32,
    /// Longest `process(1)` call and longest loop iteration, us.
    proc_max: u32,
    iter_max: u32,
}

/// Records for `idle_us` with no break (when `brk` is false), or one break and then 800 ms. With
/// `frames` the STOP frame is sent 330 us after the release and every millisecond after.
fn record(ctx: &mut Ctx, rec: &mut Rec, frames: bool, brk: bool, idle_us: u64) -> Run {
    rec.warm_up();
    rec.reset(frames);
    let levels = pac::SIO.gpio_in(1).read() >> 4;
    for (i, ln) in rec.l.iter_mut().enumerate() {
        // The first word is the length of the run the line is in at the start: if it is high, the
        // program pushes a dummy word first; if it is low, the word is that low run.
        ln.high = levels >> i & 1 != 0;
        ln.skip = ln.high;
    }
    let mut run = Run::default();
    ctx.frame_sm_idle();
    ctx.arm_all_obs();
    dma_start();
    let t_en = now_us();
    obs_enable(true);
    let mut stage = 0u8;
    let mut t_drive = 0u64;
    let mut t_rel = 0u64;
    let mut next_frame = 0u64;
    let mut k = 0u32;
    let mut end_at = if brk { u64::MAX } else { t_en + idle_us };
    let mut late_max = 0u64;
    let mut proc_max = 0u64;
    let mut iter_max = 0u64;
    loop {
        rec.sync_rings();
        let now = now_us();
        if stage == 0 && brk && now >= t_en + LEAD_US {
            // SAFETY: SM0 is idling at the top of its program with nothing in its TX FIFO; these
            // only set the latch low and then drive the four lines (the break).
            unsafe { ctx.sm0.set_pin(0) };
            settle();
            t_drive = now_us();
            // SAFETY: as above.
            unsafe { ctx.sm0.set_pindir(15) };
            stage = 1;
        }
        if stage == 1 && now >= t_drive + BREAK_US {
            t_rel = now_us();
            // SAFETY: as above; this releases the lines.
            unsafe { ctx.sm0.set_pindir(0) };
            run.d_us = (t_rel - t_drive) as u32;
            let l0 = &rec.l[0];
            if l0.edges >= 1 && !l0.first_rising {
                run.got_break = true;
                run.zero = l0.first_t + run.d_us * CPU_US;
            }
            if frames && run.got_break {
                rec.arm(run.zero);
            }
            next_frame = t_rel + FIRST_FRAME_US;
            end_at = t_rel + RECORD_US;
            stage = 2;
        }
        if stage == 2 && now >= t_rel + 20 {
            // SAFETY: as above; the latch goes back high while the lines are inputs.
            unsafe { ctx.sm0.set_pin(15) };
            stage = 3;
        }
        if frames && stage >= 2 && k < FRAMES && now >= next_frame {
            let a = ctx.sm0.tx().try_push(STOP_IMAGE[0]);
            let b = ctx.sm0.tx().try_push(STOP_IMAGE[1]);
            if !(a && b) {
                ctx.push_failed += 1;
            }
            late_max = late_max.max(now_us() - next_frame);
            if let Some(f) = rec.fpush.get_mut(k as usize) {
                *f = (now_us() - t_rel) as u32;
            }
            k += 1;
            rec.pushed = k;
            next_frame += FRAME_PERIOD_US;
        }
        let t0 = now_us();
        rec.process(1);
        proc_max = proc_max.max(now_us() - t0);
        iter_max = iter_max.max(now_us() - now);
        if now >= end_at {
            break;
        }
    }
    obs_enable(false);
    cortex_m::asm::delay(3_000);
    rec.sync_rings();
    dma_stop();
    run.late_max = late_max as u32;
    run.proc_max = proc_max as u32;
    run.iter_max = iter_max as u32;
    run.pushed = k;
    run.stall = pac::PIO0.fdebug().read().rxstall();
    ctx.release_lines();
    rec.finish();
    run.span_us = (rec.l[0].t.wrapping_sub(rec.l[0].first_t)) / CPU_US;
    run
}

// ---------------------------------------------------------------------------------------------
// Reporting

struct Lows {
    n: usize,
    total: u32,
    start: [i32; LOW_CAP],
    dur: [u32; LOW_CAP],
    have_brk: bool,
    brk_end: i32,
    orphan_rise: u32,
    open_from: i32,
    last_end: i32,
}

/// Lows of one line from the stored edges (gap and anomalous-window edges), in cycles from `zero`.
/// With `brk` the first low is the break.
fn lows(rec: &Rec, i: usize, zero: u32, brk: bool) -> Lows {
    let mut o = Lows {
        n: 0,
        total: 0,
        start: [0; LOW_CAP],
        dur: [0; LOW_CAP],
        have_brk: false,
        brk_end: 0,
        orphan_rise: 0,
        open_from: i32::MIN,
        last_end: i32::MIN,
    };
    let mut fall: Option<u32> = None;
    for &e in &rec.ev[..rec.ev_n] {
        if ev_line(e) != i || ev_flag(e) == FLAG_KEEP {
            continue;
        }
        let t = ev_t(e);
        if !ev_high(e) {
            if ev_flag(e) == FLAG_GAP && fall.is_none() {
                fall = Some(t);
            }
        } else if let Some(f) = fall.take() {
            if brk && !o.have_brk {
                o.have_brk = true;
                o.brk_end = t as i32 - zero as i32;
            } else {
                o.total += 1;
                if o.n < LOW_CAP {
                    o.start[o.n] = f as i32 - zero as i32;
                    o.dur[o.n] = t - f;
                    o.n += 1;
                }
                o.last_end = t as i32 - zero as i32;
            }
        } else {
            o.orphan_rise += 1;
        }
    }
    if let Some(f) = fall {
        o.open_from = f as i32 - zero as i32;
    }
    o
}

fn min_med_max(v: &mut [u32]) -> (u32, u32, u32) {
    if v.is_empty() {
        return (0, 0, 0);
    }
    v.sort_unstable();
    (v[0], v[v.len() / 2], v[v.len() - 1])
}

fn report_lows(tag: &str, rep: u32, i: usize, rec: &Rec, run: &Run, with_pos: bool) {
    let o = lows(rec, i, run.zero, true);
    let mut d = [0u32; LOW_CAP];
    let mut sp = [0u32; LOW_CAP];
    let mut mult = 0u32;
    let mut total_low = 0i64;
    let mut over_1ms = 0u32;
    for j in 0..o.n {
        d[j] = o.dur[j];
        total_low += o.dur[j] as i64;
        let dn = ns(o.dur[j] as i64);
        if dn > 1_000_000 {
            over_1ms += 1;
        }
        let m = (dn + 26_041) / 52_083;
        if m >= 1 && (dn - m * 52_083).abs() <= 2_000 {
            mult += 1;
        }
        if j > 0 {
            sp[j - 1] = (o.start[j] - o.start[j - 1]) as u32;
        }
    }
    let (dmin, dmed, dmax) = min_med_max(&mut d[..o.n]);
    let nsp = o.n.saturating_sub(1);
    let (smin, smed, smax) = min_med_max(&mut sp[..nsp]);
    defmt::info!(
        "[STAT] {=str} rep {=u32} {=str}: break low ends {=i32} ns after the release (have {=bool}); {=u32} lows after it (kept {=usize}, orphan rises {=u32}); duration min/median/max {=i32} {=i32} {=i32} ns; lows over 1 ms {=u32}; multiples of 52.083 us within 2 us {=u32}; start-to-start min/median/max {=i32} {=i32} {=i32} ns; first start {=i32} ns, last low ends {=i32} ns after the release; total low {=i32} ns; open low from {=i32} ns; level at the end high {=bool}",
        tag,
        rep,
        NAMES[i],
        ns(o.brk_end as i64),
        o.have_brk,
        o.total,
        o.n,
        o.orphan_rise,
        ns(dmin as i64),
        ns(dmed as i64),
        ns(dmax as i64),
        over_1ms,
        mult,
        ns(smin as i64),
        ns(smed as i64),
        ns(smax as i64),
        if o.n > 0 { ns(o.start[0] as i64) } else { 0 },
        if o.last_end == i32::MIN {
            0
        } else {
            ns(o.last_end as i64)
        },
        ns(total_low),
        if o.open_from == i32::MIN {
            0
        } else {
            ns(o.open_from as i64)
        },
        rec.l[i].high
    );
    let mut pairs = [0i32; 48];
    let mut np = 0;
    let first = o.n.min(24);
    for j in 0..first {
        pairs[np] = ns(o.start[j] as i64);
        pairs[np + 1] = ns(o.dur[j] as i64);
        np += 2;
    }
    defmt::info!(
        "[STAT] {=str} rep {=u32} {=str}: first {=usize} lows, start and duration in ns after the release: {=[?]}",
        tag,
        rep,
        NAMES[i],
        first,
        &pairs[..np]
    );
    if o.n > 24 {
        let from = o.n.saturating_sub(8).max(24);
        np = 0;
        for j in from..o.n {
            pairs[np] = ns(o.start[j] as i64);
            pairs[np + 1] = ns(o.dur[j] as i64);
            np += 2;
        }
        defmt::info!(
            "[STAT] {=str} rep {=u32} {=str}: last lows (from index {=usize}): {=[?]}",
            tag,
            rep,
            NAMES[i],
            from,
            &pairs[..np]
        );
    }
    if with_pos {
        let mut bins = [0u32; 4];
        for j in 0..o.n {
            let us = ns(o.start[j] as i64) / 1000;
            let pos = (us - FIRST_FRAME_US as i32).rem_euclid(1000);
            let b = if !(57..992).contains(&pos) {
                0
            } else if pos < 200 {
                1
            } else if pos < 500 {
                2
            } else {
                3
            };
            bins[b] += 1;
        }
        defmt::info!(
            "[STAT] {=str} rep {=u32} {=str}: foreign lows by start relative to the frame (inside the frame window / 57-200 us after the frame start / 200-500 / 500-992 us): {=u32} {=u32} {=u32} {=u32}",
            tag,
            rep,
            NAMES[i],
            bins[0],
            bins[1],
            bins[2],
            bins[3]
        );
    }
}

/// Last stored non-reply edge of a line, ns after the release.
fn last_foreign(rec: &Rec, i: usize, zero: u32) -> i32 {
    let mut last = i32::MIN;
    for &e in &rec.ev[..rec.ev_n] {
        if ev_line(e) == i && ev_flag(e) != FLAG_KEEP {
            last = last.max(ns(ev_t(e) as i64 - zero as i64));
        }
    }
    last
}

fn dump_window(tag: &str, rep: u32, rec: &Rec, run: &Run, from_ns: i32, len_ns: i32) {
    for i in 0..LINES {
        let mut buf = [0i32; 40];
        let mut n = 0;
        for &e in &rec.ev[..rec.ev_n] {
            if ev_line(e) != i {
                continue;
            }
            let t = ns(ev_t(e) as i64 - run.zero as i64);
            if t < from_ns || t >= from_ns + len_ns {
                continue;
            }
            buf[n] = if ev_high(e) { t.max(1) } else { -t.max(1) };
            n += 1;
            if n == buf.len() {
                defmt::info!(
                    "[RAW] {=str} rep {=u32} {=str} from {=i32} ns: {=[?]}",
                    tag,
                    rep,
                    NAMES[i],
                    from_ns,
                    &buf[..n]
                );
                n = 0;
            }
        }
        if n > 0 {
            defmt::info!(
                "[RAW] {=str} rep {=u32} {=str} from {=i32} ns: {=[?]}",
                tag,
                rep,
                NAMES[i],
                from_ns,
                &buf[..n]
            );
        }
    }
}

fn report_b(rep: u32, rec: &Rec, run: &Run, fails: &mut u32) {
    for i in 0..LINES {
        report_lows("b", rep, i, rec, run, false);
        defmt::info!(
            "[STAT] b rep {=u32} {=str}: all edges {=u32}, non-frame activity ends (last stored edge) {=i32} ns after the release",
            rep,
            NAMES[i],
            rec.l[i].edges,
            last_foreign(rec, i, run.zero)
        );
    }
    check!(
        *fails,
        run.got_break && run.stall == 0,
        "[{=str}] b rep {=u32}: break seen on G36 and no observer FIFO overflow (stall mask 0x{=u8:x})",
        rep,
        run.stall
    );
    check!(
        *fails,
        rec.ev_drop == 0 && rec.q_over == 0,
        "[{=str}] b rep {=u32}: event list not full ({=u32} dropped), staging queues never full ({=u32} lost)",
        rep,
        rec.ev_drop,
        rec.q_over
    );
    defmt::info!(
        "[STAT] b rep {=u32}: break {=u32} us by the CPU timer, observed run {=u32} us, events stored {=usize}",
        rep,
        run.d_us,
        run.span_us,
        rec.ev_n
    );
    dump_window("b", rep, rec, run, -50_000, 5_050_000);
    let mut bins = [0u32; 170];
    for &e in &rec.ev[..rec.ev_n] {
        let t = ns(ev_t(e) as i64 - run.zero as i64);
        if t >= 5_000_000 {
            let b = (t / 5_000_000) as usize;
            if b < bins.len() {
                bins[b] += 1;
            }
        }
    }
    let mut best = 1;
    for (b, &c) in bins.iter().enumerate().skip(1) {
        if c > bins[best] {
            best = b;
        }
    }
    if bins[best] > 0 {
        dump_window("b", rep, rec, run, best as i32 * 5_000_000, 5_000_000);
    }
}

fn report_a(rep: u32, rec: &Rec, run: &Run, fails: &mut u32) {
    for i in 0..LINES {
        let l = &rec.l[i];
        defmt::info!(
            "[STAT] a rep {=u32} {=str}: our frames: windows with edges {=u32} of {=u32} (none: {=u32}); edges in windows {=u32}; forms: 32 edges from a line idling high {=u32}, 33 (line pulled low at the release) {=u32}, 34 (line held low, 80 ns spike first) {=u32}; of these within 2 samples (8 half cycles) of the ideal {=u32}, off by more {=u32}; other edge counts: fewer than 32 {=u32}, more than 34 or odd shape {=u32}, over 64 {=u32}; windows starting with a rise {=u32}, ending low {=u32}",
            rep,
            NAMES[i],
            l.win_seen,
            FRAMES,
            FRAMES - l.win_seen.min(FRAMES),
            l.win_edges,
            l.forms[0],
            l.forms[1],
            l.forms[2],
            l.clean,
            l.bad_dev,
            l.other_n[0],
            l.other_n[1] + l.other_n[2],
            l.other_n[3],
            l.first_rise,
            l.end_low
        );
        defmt::info!(
            "[STAT] a rep {=u32} {=str}: max deviation of a window (half cycles of 3.33 ns) <=4 / <=8 / <=16 / <=32 / more: {=u32} {=u32} {=u32} {=u32} {=u32}; falling edges {=i32}..{=i32} ns, rising edges {=i32}..{=i32} ns from the ideal; windows ending low: last frame {=i32}, windows ending high: first frame {=i32}; windows starting with a rise: last frame {=i32}",
            rep,
            NAMES[i],
            l.hist[0],
            l.hist[1],
            l.hist[2],
            l.hist[3],
            l.hist[4],
            if l.dev_min[0] == i32::MAX {
                0
            } else {
                ns(l.dev_min[0] as i64 / 2)
            },
            if l.dev_max[0] == i32::MIN {
                0
            } else {
                ns(l.dev_max[0] as i64 / 2)
            },
            if l.dev_min[1] == i32::MAX {
                0
            } else {
                ns(l.dev_min[1] as i64 / 2)
            },
            if l.dev_max[1] == i32::MIN {
                0
            } else {
                ns(l.dev_max[1] as i64 / 2)
            },
            l.last_end_low_k,
            l.first_end_high_k,
            l.last_first_rise_k
        );
        defmt::info!(
            "[STAT] a rep {=u32} {=str}: replies in the gaps: {=u32} gaps; truncated {=u32}, bad shape {=u32}, bad GCR {=u32}, bad checksum {=u32}, GOOD {=u32}, line low at the start of the gap {=u32}, no window before {=u32}; silent frames {=u32}; first good reply after frame {=u32}, first 0xFFF after frame {=u32} (frame n is sent 330 + 1000 n us after the release)",
            rep,
            NAMES[i],
            l.gaps,
            l.kinds[0],
            l.kinds[1],
            l.kinds[2],
            l.kinds[3],
            l.kinds[4],
            l.kinds[5],
            l.kinds[6],
            FRAMES.saturating_sub(l.gaps),
            l.first_ok,
            l.first_fff
        );
        report_lows("a", rep, i, rec, run, true);
        defmt::info!(
            "[STAT] a rep {=u32} {=str}: non-frame activity (stored gap edges, bad replies included, good replies not) ends {=i32} ns after the release; anomalous windows stored {=u32}",
            rep,
            NAMES[i],
            last_foreign(rec, i, run.zero),
            l.anom_stored
        );
    }
    check!(
        *fails,
        run.got_break && run.stall == 0,
        "[{=str}] a rep {=u32}: break seen on G36 and no observer FIFO overflow (stall mask 0x{=u8:x})",
        rep,
        run.stall
    );
    check!(
        *fails,
        rec.ev_drop == 0,
        "[{=str}] a rep {=u32}: event list not full ({=u32} dropped)",
        rep,
        rec.ev_drop
    );
    check!(
        *fails,
        run.pushed == FRAMES,
        "[{=str}] a rep {=u32}: all 800 frames pushed ({=u32})",
        rep,
        run.pushed
    );
    check!(
        *fails,
        rec.q_over == 0,
        "[{=str}] a rep {=u32}: staging queues never full ({=u32} words lost)",
        rep,
        rec.q_over
    );
    let c = rec.l[0].win_edges;
    check!(
        *fails,
        c + 8 >= FRAMES * 32,
        "[{=str}] a rep {=u32}: own frame edges on G36 in 800 ms: at least 800 x 32 - 8 seen ({=u32}, a line held low adds 2 or 3 per frame)",
        rep,
        c
    );
    defmt::info!(
        "[STAT] a rep {=u32}: break {=u32} us by the CPU timer, observed run {=u32} us, events stored {=usize}, latest frame push {=u32} us late, longest process step {=u32} us, longest loop pass {=u32} us",
        rep,
        run.d_us,
        run.span_us,
        rec.ev_n,
        run.late_max,
        run.proc_max,
        run.iter_max
    );
    dump_window("a", rep, rec, run, -50_000, 5_050_000);
    dump_window("a", rep, rec, run, 300_000_000, 5_000_000);
    dump_window("a", rep, rec, run, 600_000_000, 5_000_000);
}

// ---------------------------------------------------------------------------------------------

fn set_jmp_pin(cfg: &mut Config<'_, PIO0>, gpio: u8) {
    let mut exec = cfg.get_exec();
    exec.jmp_pin = gpio - 16;
    // SAFETY: only the JMP PIN source changes; the wrap settings are the ones `use_program` wrote.
    unsafe { cfg.set_exec(exec) };
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("[INFO] dshot_m8_startup: start");
    let p = embassy_rp::init(Default::default());
    let hz = embassy_rp::clocks::clk_sys_freq();
    let mut fails = 0u32;
    check!(fails, hz == CLK_HZ, "[{=str}] clk_sys is 150 MHz");
    self_test(&mut fails);
    if fails != 0 {
        hil_common::exit(fails);
    }

    // The pads belong to the PIO block that made them last: PIO0 (observers) first, PIO2 (driver) last.
    let Pio {
        common: mut c0,
        sm0: o0,
        sm1: o1,
        sm2: o2,
        sm3: o3,
        ..
    } = Pio::new(p.PIO0, Irqs);
    // SAFETY: the four pins are shared between PIO0 (inputs only) and PIO2 (the driver) on purpose.
    let pins0: [Pin<'static, PIO0>; LINES] = unsafe {
        [
            c0.make_pio_pin(p.PIN_36.clone_unchecked()),
            c0.make_pio_pin(p.PIN_37.clone_unchecked()),
            c0.make_pio_pin(p.PIN_38.clone_unchecked()),
            c0.make_pio_pin(p.PIN_39.clone_unchecked()),
        ]
    };
    c0.set_input_sync_bypass(OE_MASK, OE_MASK);
    let Pio {
        common: mut c2,
        sm0,
        ..
    } = Pio::new(p.PIO2, Irqs);
    let mut pins2 = [
        c2.make_pio_pin(p.PIN_36),
        c2.make_pio_pin(p.PIN_37),
        c2.make_pio_pin(p.PIN_38),
        c2.make_pio_pin(p.PIN_39),
    ];
    for pin in &mut pins2 {
        pin.set_pull(Pull::Up);
    }
    let [a, b, c, d] = &pins2;
    let line_refs = [a, b, c, d];

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
    let obs_cfg: [Config<'static, PIO0>; LINES] = core::array::from_fn(|l| {
        let mut cfg = Config::default();
        cfg.use_program(&obs_loaded, &[]);
        cfg.fifo_join = FifoJoin::RxOnly;
        cfg.shift_in = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Right,
            auto_fill: false,
        };
        // The in pins are what puts PIO0 in the GPIO16-47 window; the program never uses `in`.
        cfg.set_in_pins(&[&pins0[l]]);
        set_jmp_pin(&mut cfg, 36 + l as u8);
        cfg
    });
    let mut o0 = o0;
    let mut o1 = o1;
    let mut o2 = o2;
    let mut o3 = o3;
    o0.set_enable(false);
    o1.set_enable(false);
    o2.set_enable(false);
    o3.set_enable(false);
    o0.set_config(&obs_cfg[0]);
    o1.set_config(&obs_cfg[1]);
    o2.set_config(&obs_cfg[2]);
    o3.set_config(&obs_cfg[3]);

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
        "    out pins, 4   [14]",
        "    set pins, 15  [8]",
        "    jmp x-- bit",
        "    set pindirs, 0",
        ".wrap",
    );
    let main_loaded = c2.load_program(&main_prog.program);
    let mut cfg0 = Config::default();
    cfg0.use_program(&main_loaded, &[]);
    cfg0.clock_divider = U24F8::from_bits(DIV_Q8);
    cfg0.shift_out = ShiftConfig {
        threshold: 32,
        direction: ShiftDirection::Right,
        auto_fill: false,
    };
    cfg0.status_sel = StatusSource::TxFifoLevel;
    cfg0.status_n = StatusN::This(2);
    cfg0.set_set_pins(&line_refs);
    cfg0.set_out_pins(&line_refs);
    let mut sm0 = sm0;
    sm0.set_enable(false);
    sm0.set_config(&cfg0);
    sm0.set_pins(Level::High, &line_refs);
    sm0.set_pin_dirs(pio::Direction::In, &line_refs);

    let mut ctx = Ctx {
        sm0,
        o0,
        o1,
        o2,
        o3,
        main_origin: main_loaded.origin,
        obs_origin: obs_loaded.origin,
        push_failed: 0,
    };
    let mut rec = Rec::new();
    defmt::info!(
        "[INFO] PIO0: observer program {=u8} words at {=u8}, four state machines, divider 1, 2 cycles per count (13.3 ns); PIO2: frame program {=u8} words at {=u8}, DShot300",
        obs_prog.program.code.len() as u8,
        obs_loaded.origin,
        main_prog.program.code.len() as u8,
        main_loaded.origin
    );

    // Informational: the lines as they are at the start (the controllers may be in their application).
    let run = record(&mut ctx, &mut rec, false, false, 100_000);
    defmt::info!(
        "[STAT] start, lines released, 100 ms, nothing sent: edges {=u32} {=u32} {=u32} {=u32}, stall mask 0x{=u8:x}",
        rec.l[0].edges,
        rec.l[1].edges,
        rec.l[2].edges,
        rec.l[3].edges,
        run.stall
    );

    // Positive control: parked lines, nothing driven, 100 ms: no edge.
    Timer::after_millis(PARK_MS).await;
    let run = record(&mut ctx, &mut rec, false, false, 100_000);
    let edges: u32 = rec.l.iter().map(|l| l.edges).sum();
    defmt::info!(
        "[STAT] control, lines released and silent for 3500 ms, then 100 ms nothing sent: edges {=u32} {=u32} {=u32} {=u32}, stall mask 0x{=u8:x}",
        rec.l[0].edges,
        rec.l[1].edges,
        rec.l[2].edges,
        rec.l[3].edges,
        run.stall
    );
    check!(
        fails,
        edges == 0 && run.stall == 0,
        "[{=str}] control: no edge on any line in 100 ms with nothing sent and the lines parked ({=u32})",
        edges
    );

    for rep in 1..=3u32 {
        for frames in [false, true] {
            defmt::info!(
                "[INFO] rep {=u32}, case {=str}: park {=u64} ms",
                rep,
                if frames {
                    "a (frames)"
                } else {
                    "b (no frames)"
                },
                PARK_MS
            );
            Timer::after_millis(PARK_MS).await;
            let run = record(&mut ctx, &mut rec, frames, true, 0);
            if frames {
                report_a(rep, &rec, &run, &mut fails);
            } else {
                report_b(rep, &rec, &run, &mut fails);
            }
        }
    }

    ctx.release_lines();
    let oe = pac::PIO2.dbg_padoe().read() & OE_MASK;
    check!(
        fails,
        oe == 0,
        "[{=str}] end: no line driven, all inputs with pull-ups"
    );
    check!(
        fails,
        ctx.push_failed == 0,
        "[{=str}] no TX push was refused ({=u32})",
        ctx.push_failed
    );
    hil_common::exit(fails)
}
