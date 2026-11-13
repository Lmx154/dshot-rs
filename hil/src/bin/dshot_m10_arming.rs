//! M10 (measurement, controller behaviour only): the arming burst on all four lines at every
//! phase. After a fresh `start()`, 3 s of STOP frames at 1 kHz from `dshot::PioDshot` (PIO2,
//! GPIO36-39), with the four run-length observers (PIO0, 13.3 ns) recording CONTINUOUSLY: every
//! edge of the 3 s is classified, one frame period at a time (from the first edge of our frame to
//! the first edge of the next), as our frame, the controller's normal reply (a group starting 20
//! to 30 us after our last rising edge), or anything else. Groups of the third kind are reported
//! once the line has given its first normal reply. Five repetitions per rate.
//!
//! SAFETY OF THE BENCH: every frame is `LineFrame::STOP` on all four lines.
//!
//! Passes when every repetition completed and our own frame was whole in every period; the
//! controllers' behaviour is reported, not judged.
#![no_std]
#![no_main]
#![allow(clippy::needless_range_loop)]

use defmt_rtt as _;
use dshot::{LineCapture, LineFrame, Parts, PioDshot, Rate, Reply, program};
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

const CLK_HZ: u32 = 150_000_000;
const LINES: usize = 4;
const OE_MASK: u32 = 0xF << 20;
const NAMES: [&str; LINES] = ["M4/G36", "M3/G37", "M2/G38", "M1/G39"];
const STOP4: [LineFrame; LINES] = [LineFrame::STOP; LINES];
const FRAMES: u32 = 3000;
const REPS: u32 = 5;

// ---------------------------------------------------------------------------------------------
// The observers (PIO0 state machines 0-3, one per line), as in dshot_smoke.

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
    t: u64,
    high: bool,
    skip: bool,
    e: [u64; EDGE_CAP],
    n: usize,
    over: u32,
}

impl Line {
    const NEW: Line = Line {
        t: 0,
        high: true,
        skip: true,
        e: [0; EDGE_CAP],
        n: 0,
        over: 0,
    };
}

struct Obs {
    t0: Instant,
    l: [Line; LINES],
    qh: [u32; LINES],
    qt: [u32; LINES],
    ring_over: u32,
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
        if ln.n < EDGE_CAP {
            ln.e[ln.n] = (ln.t << 1) | u64::from(!was_high);
            ln.n += 1;
        } else {
            ln.over += 1;
        }
    }

    fn mark(&mut self) {
        self.sync();
        for ln in &mut self.l {
            ln.n = 0;
        }
    }

    fn now(&self) -> u64 {
        self.t0.elapsed().as_micros() * 150
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

fn bit4(rate: Rate) -> i64 {
    (4 * CLK_HZ / rate.bits_per_second()) as i64
}

fn ns(sys: u64) -> u32 {
    (sys * 20 / 3) as u32
}

/// Our STOP frame in one line's window.
#[derive(Clone, Copy)]
struct Fr {
    /// Index of its first edge and one past its last.
    j: usize,
    end: usize,
    /// Its first falling edge (implied on a line held low before it) and last rising edge.
    first: u64,
    last_rise: u64,
}

/// Finds our STOP frame (12 short lows, 4 long ones at the bit time of `rate`) anywhere in `e`,
/// whatever precedes it; on a line held low before it, bit 0 has no falling edge.
fn find_frame(e: &[u64], rate: Rate) -> Option<Fr> {
    let b = bit4(rate);
    let (t0h, t1h) = (b * 15 / 40, b * 30 / 40);
    let tol = 4 * 8;
    let t = |k: usize| ((e[k] >> 1) * 4) as i64;
    let rising = |k: usize| e[k] & 1 == 1;
    'outer: for j in 0..e.len() {
        let held = rising(j);
        let need = if held { 31 } else { 32 };
        if j + need > e.len() {
            break;
        }
        let t0 = if held { t(j) - t0h } else { t(j) };
        for k in 0..16 {
            let want = if k < 12 { t0h } else { t1h };
            let (tf, r) = if held {
                if k == 0 {
                    (t0, j)
                } else {
                    let f = j + 2 * k - 1;
                    if rising(f) {
                        continue 'outer;
                    }
                    (t(f), j + 2 * k)
                }
            } else {
                let f = j + 2 * k;
                if rising(f) {
                    continue 'outer;
                }
                (t(f), j + 2 * k + 1)
            };
            if (tf - t0 - k as i64 * b).abs() > tol || !rising(r) || (t(r) - tf - want).abs() > tol
            {
                continue 'outer;
            }
        }
        return Some(Fr {
            j,
            end: j + need,
            first: (t0 / 4) as u64,
            last_rise: e[j + need - 1] >> 1,
        });
    }
    None
}

fn stopped(c: &LineCapture) -> bool {
    matches!(c, LineCapture::Reply(Reply::Erpm(e)) if e.is_stopped())
}

fn dump_line(ln: &Line, l: usize, what: &str, k: u32) {
    let e = &ln.e[..ln.n];
    let e0 = e.first().map_or(0, |&x| x >> 1);
    let mut rel = [0u32; 48];
    let m = e.len().min(48);
    for j in 0..m {
        rel[j] = (ns((e[j] >> 1) - e0) << 1) | (e[j] & 1) as u32;
    }
    defmt::info!(
        "[INFO] {=str} {=str} at {=u32}: {=usize} edges; (ns from the first) << 1 | rising: {=[?]}",
        what,
        NAMES[l],
        k,
        ln.n,
        &rel[..m]
    );
}

#[derive(Clone, Copy)]
struct LineSt {
    have_prev: bool,
    prev: Fr,
    first_reply: Option<u64>,
    groups: u32,
    shown: u32,
    in_window: u32,
    longest: u64,
    min_to_next: u64,
    seen_stopped: bool,
    run_start: u32,
    run_len: u32,
    sil_first: u32,
    sil_len: u32,
    g_idx: [u32; 8],
    g_phase: [u32; 8],
    g_n: usize,
}

impl LineSt {
    const NEW: LineSt = LineSt {
        have_prev: false,
        prev: Fr {
            j: 0,
            end: 0,
            first: 0,
            last_rise: 0,
        },
        first_reply: None,
        groups: 0,
        shown: 0,
        in_window: 0,
        longest: 0,
        min_to_next: u64::MAX,
        seen_stopped: false,
        run_start: 0,
        run_len: 0,
        sil_first: 0,
        sil_len: 0,
        g_idx: [0; 8],
        g_phase: [0; 8],
        g_n: 0,
    };
}

struct Tally {
    bad: u32,
    over: u32,
}

/// After exchange `k` and a sync: classifies the period from frame k-1 to frame k on line `l`.
#[allow(clippy::too_many_arguments)]
fn process_line(
    ln: &mut Line,
    s: &mut LineSt,
    t: &mut Tally,
    hist: &[[LineCapture; LINES]; 4],
    l: usize,
    k: u32,
    rate: Rate,
    t_start: u64,
    label: &str,
    rep: u32,
) {
    if ln.over > 0 {
        t.over += ln.over;
        ln.over = 0;
    }
    let n = ln.n;
    if !s.have_prev {
        match find_frame(&ln.e[..n], rate) {
            Some(f) => {
                ln.e.copy_within(f.j..n, 0);
                ln.n = n - f.j;
                s.prev = Fr {
                    j: 0,
                    end: f.end - f.j,
                    first: f.first,
                    last_rise: f.last_rise,
                };
                s.have_prev = true;
            }
            None => {
                t.bad += 1;
                dump_line(ln, l, "no frame", k);
                ln.n = 0;
            }
        }
        return;
    }
    let base = s.prev.end;
    let Some(f) = find_frame(&ln.e[base..n], rate) else {
        t.bad += 1;
        dump_line(ln, l, "frame not whole", k);
        ln.n = 0;
        s.have_prev = false;
        return;
    };
    let cut = base + f.j;
    let b4 = bit4(rate);
    let gap_th = (b4 * 32 / 40) as u64; // 3.2 command bits (4 reply bits), system cycles
    let rel = (s.prev.last_rise * 4) as i64 + 10 * b4 / 40;
    let end = rel + program::release_to_idle(rate) as i64 * b4 / 40;
    let q = &ln.e[base..cut];
    let mut i = 0;
    let mut reply_found = false;
    while i < q.len() {
        let mut j = i + 1;
        while j < q.len() && (q[j] >> 1) - (q[j - 1] >> 1) <= gap_th {
            j += 1;
        }
        let g0 = q[i] >> 1;
        let g1 = q[j - 1] >> 1;
        let after = ns(g0.saturating_sub(s.prev.last_rise));
        if !reply_found && q[i] & 1 == 0 && (20_000..=30_000).contains(&after) {
            reply_found = true;
            if s.first_reply.is_none() {
                s.first_reply = Some(g0);
            }
        } else if let Some(fr0) = s.first_reply {
            s.groups += 1;
            s.longest = s.longest.max(g1 - g0);
            let to_next = f.first.saturating_sub(g1);
            s.min_to_next = s.min_to_next.min(to_next);
            let inwin = q[i..j].iter().any(|&x| {
                let tq = ((x >> 1) * 4) as i64;
                tq >= rel && tq <= end
            });
            s.in_window += u32::from(inwin);
            let phase = ns(g0 - s.prev.first);
            if s.g_n < 8 {
                s.g_idx[s.g_n] = k - 1;
                s.g_phase[s.g_n] = phase;
                s.g_n += 1;
            }
            if s.shown < 6 {
                s.shown += 1;
                let mut runs = [0u32; 24];
                let m = (j - i - 1).min(24);
                for r in 0..m {
                    runs[r] = (ns((q[i + r + 1] >> 1) - (q[i + r] >> 1)) << 1)
                        | u32::from(q[i + r] & 1 == 0);
                }
                defmt::info!(
                    "[STAT] {=str} rep {=u32} {=str}: other group after frame {=u32}: {=u64} ms after start() returned, {=u64} ms after the first normal reply, phase {=u32} ns after that frame's first falling edge, {=usize} edges over {=u32} ns, ends {=u32} ns before the next frame, in the capture window {=bool}; driver captures of exchanges {=u32} and {=u32}: {} / {}; runs (ns << 1 | low): {=[?]}",
                    label,
                    rep,
                    NAMES[l],
                    k - 1,
                    g0.saturating_sub(t_start) / 150_000,
                    g0.saturating_sub(fr0) / 150_000,
                    phase,
                    j - i,
                    ns(g1 - g0),
                    ns(to_next),
                    inwin,
                    k - 1,
                    k,
                    hist[((k - 1) % 4) as usize][l],
                    hist[(k % 4) as usize][l],
                    &runs[..m]
                );
            }
        }
        i = j;
    }
    ln.e.copy_within(cut..n, 0);
    ln.n = n - cut;
    s.prev = Fr {
        j: 0,
        end: f.end - f.j,
        first: f.first,
        last_rise: f.last_rise,
    };
}

struct Ctx {
    drv: Option<PioDshot<'static, PIO2>>,
    obs: Obs,
    tally: Tally,
    incomplete: u32,
}

impl Ctx {
    fn drv(&mut self) -> &mut PioDshot<'static, PIO2> {
        match self.drv.as_mut() {
            Some(d) => d,
            None => {
                defmt::error!("[FAIL] no driver");
                hil_common::exit(1)
            }
        }
    }
}

async fn rep_run(ctx: &mut Ctx, label: &str, rep: u32) {
    let rate = ctx.drv().rate();
    let fl0 = ctx.drv().monitor().foreign_while_live();
    ctx.obs.mark();
    let mut d = Delay;
    ctx.drv().start(&mut d).await;
    let t_start = ctx.obs.now();
    let mut st = [LineSt::NEW; LINES];
    let mut hist = [[LineCapture::Silent; LINES]; 4];
    let mut next = Instant::now();
    for k in 0..FRAMES {
        Timer::at(next).await;
        next += Duration::from_micros(1000);
        let r = ctx.drv().exchange(STOP4).await;
        ctx.obs.sync();
        let x = r;
        hist[(k % 4) as usize] = x.captures;
        for l in 0..LINES {
            let s = &mut st[l];
            if stopped(&x.captures[l]) {
                if s.run_len > s.sil_len {
                    s.sil_first = s.run_start;
                    s.sil_len = s.run_len;
                }
                s.run_len = 0;
                s.seen_stopped = true;
            } else if s.seen_stopped {
                if s.run_len == 0 {
                    s.run_start = k;
                }
                s.run_len += 1;
            }
            process_line(
                &mut ctx.obs.l[l],
                s,
                &mut ctx.tally,
                &hist,
                l,
                k,
                rate,
                t_start,
                label,
                rep,
            );
        }
    }
    let fl = ctx.drv().monitor().foreign_while_live();
    for l in 0..LINES {
        let s = &st[l];
        let open = s.run_len > s.sil_len;
        let (sf, sl) = if open {
            (s.run_start, s.run_len)
        } else {
            (s.sil_first, s.sil_len)
        };
        let sil_last = sf + sl.max(1) - 1;
        let mut rel = [0i32; 8];
        for g in 0..s.g_n {
            rel[g] = s.g_idx[g] as i32 - sil_last as i32;
        }
        defmt::info!(
            "[STAT] {=str} rep {=u32} {=str}: first normal reply {=u64} ms after start(); arming silence exchanges {=u32}..{=u32} ({=u32} frames{=str}); other groups {=u32}, in the capture window {=u32}, longest {=u32} ns, closest to the next frame {=u32} ns; their periods {=[?]}, minus the last silent exchange {=[?]}, phases {=[?]} ns; Foreign while Live {=u32}",
            label,
            rep,
            NAMES[l],
            s.first_reply
                .map_or(0, |t| t.saturating_sub(t_start) / 150_000),
            sf,
            sil_last,
            sl,
            if open { ", still open at the end" } else { "" },
            s.groups,
            s.in_window,
            ns(s.longest),
            if s.min_to_next == u64::MAX {
                u32::MAX
            } else {
                ns(s.min_to_next)
            },
            &s.g_idx[..s.g_n],
            &rel[..s.g_n],
            &s.g_phase[..s.g_n],
            fl[l] - fl0[l]
        );
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("[INFO] dshot_m10_arming: start");
    let p = embassy_rp::init(Default::default());

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
            hil_common::exit(1)
        }
    };
    let levels = pac::SIO.gpio_in(1).read() >> 4;
    let mut obs = Obs {
        t0: Instant::now(),
        l: [Line::NEW; LINES],
        qh: [0; LINES],
        qt: [0; LINES],
        ring_over: 0,
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
        tally: Tally { bad: 0, over: 0 },
        incomplete: 0,
    };

    for rate in [Rate::Dshot300, Rate::Dshot600] {
        if ctx.drv().rate() != rate {
            let drv = ctx.drv.take();
            match drv.map(PioDshot::release) {
                Some(Ok(parts)) => match PioDshot::new(parts, rate, CLK_HZ) {
                    Ok(d) => ctx.drv = Some(d),
                    Err(e) => {
                        defmt::error!("[FAIL] rebuild at {}: {}", rate, e.reason);
                        hil_common::exit(1)
                    }
                },
                _ => {
                    defmt::error!("[FAIL] release before {} refused", rate);
                    hil_common::exit(1)
                }
            }
        }
        let label: &str = match rate {
            Rate::Dshot300 => "DShot300",
            Rate::Dshot600 => "DShot600",
        };
        for rep in 1..=REPS {
            rep_run(&mut ctx, label, rep).await;
        }
    }
    drop(ctx.drv.take());
    let fails = u32::from(ctx.tally.bad != 0)
        + u32::from(ctx.incomplete != 0)
        + u32::from(ctx.obs.ring_over != 0)
        + u32::from(ctx.tally.over != 0);
    if fails == 0 {
        defmt::info!(
            "[PASS] every repetition completed; our frame whole in every period; no ring or edge-buffer overflow"
        );
    } else {
        defmt::error!(
            "[FAIL] periods without our whole frame {=u32}; exchanges not sent {=u32}; ring overflows {=u32}; edge-buffer overflows {=u32}",
            ctx.tally.bad,
            ctx.incomplete,
            ctx.obs.ring_over,
            ctx.tally.over
        );
    }
    hil_common::exit(fails)
}
