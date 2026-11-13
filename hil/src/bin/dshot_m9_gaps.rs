//! M9 (measurement, controller behaviour only): what the four AM32 controllers (GPIO36-39,
//! driven by `dshot::PioDshot` on PIO2) do after one gap in the frame stream, and when they put
//! edges on their lines between our frames. Witness: four run-length observers on PIO0
//! (13.3 ns, DMA rings on channels 4-7), as in `dshot_smoke`.
//!
//! SAFETY OF THE BENCH: every frame is `LineFrame::STOP` on all four lines.
//!
//! A. Armed cut-off: `start()`, 3 s of STOP at 1 kHz, one gap G, then 1 kHz again (up to 2 s):
//!    frames to the first stopped reply per line and the edges that are not ours before it.
//!    G through the API at every length.
//! B. Edges before our frame: `start()`, 10 s at 1 kHz, three repetitions.
//! C. G = 800 ms with no `start()` after it, 5 s of frames, every window with foreign edges shown.
//!
//! Passes when every trial completed and our own frame was whole in every window (the frame-shape
//! test is restricted to our frame); the controllers' behaviour is reported, not judged.
#![no_std]
#![no_main]
#![allow(clippy::needless_range_loop)]

use defmt_rtt as _;
use dshot::{Exchange, LineCapture, LineFrame, Parts, PioDshot, Rate, Reply};
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
const GAPS_MS: [u32; 21] = [
    300, 350, 380, 390, 395, 399, 400, 405, 410, 420, 450, 480, 490, 495, 500, 505, 510, 520, 550,
    600, 800,
];

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
        "[INFO] window {=str} {=str} at {=u32}: {=usize} edges (over {=u32}), first at observer cycle {=u64}, {=u64} us before now; (ns from the first) << 1 | rising: {=[?]}",
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

fn stopped(c: &LineCapture) -> bool {
    matches!(c, LineCapture::Reply(Reply::Erpm(e)) if e.is_stopped())
}

struct Ctx {
    drv: Option<PioDshot<'static, PIO2>>,
    obs: Obs,
    /// Windows in which our frame was not found whole, by phase: streams, after a gap, part C.
    bad: [u32; 3],
    phase: usize,
    incomplete: u32,
    dumps: u32,
}

struct Shot {
    r: Exchange,
    fr: [Option<Fr>; LINES],
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

    /// One exchange of four STOP frames; `mark` false keeps what the window already holds.
    async fn shot(&mut self, mark: bool, what: &str, k: u32) -> Shot {
        if mark {
            self.obs.mark();
        }
        let rate = self.drv().rate();
        let r = self.drv().exchange(STOP4).await;
        self.obs.sync();
        let fr: [Option<Fr>; LINES] =
            core::array::from_fn(|l| find_frame(&self.obs.l[l].e[..self.obs.l[l].n], rate));
        for l in 0..LINES {
            if fr[l].is_none() {
                self.bad[self.phase] += 1;
                if self.dumps < 16 {
                    self.dumps += 1;
                    dump_window(&self.obs, l, what, k);
                }
            }
        }
        Shot { r, fr }
    }

    async fn start(&mut self) -> Instant {
        self.obs.mark();
        let mut d = Delay;
        self.drv().start(&mut d).await;
        Instant::now()
    }
}

/// Part A (and C): returns the frames to the first stopped reply per line (1 = the first frame
/// after the gap; 0 = none).
async fn gap_trial(
    ctx: &mut Ctx,
    label: &str,
    g_ms: u32,
    post_ms: u32,
    detail: bool,
) -> [u32; LINES] {
    ctx.phase = 0;
    ctx.start().await;
    let mut next = Instant::now();
    let mut ok_tail = [0u32; LINES];
    let mut last_rise = [0u64; LINES];
    for i in 0..3000u32 {
        Timer::at(next).await;
        next += Duration::from_micros(1000);
        let s = ctx.shot(true, "before the gap", i).await;
        let x = s.r;
        if i >= 2800 {
            for l in 0..LINES {
                ok_tail[l] += u32::from(stopped(&x.captures[l]));
            }
        }
        for l in 0..LINES {
            last_rise[l] = s.fr[l].map_or(0, |f| f.last_rise);
        }
    }
    let ret = Instant::now();
    ctx.obs.mark();
    let at = ret + Duration::from_millis(u64::from(g_ms));
    while Instant::now() + Duration::from_millis(3) < at {
        Timer::after_millis(1).await;
        ctx.obs.sync();
    }
    Timer::at(at).await;
    ctx.phase = if detail { 2 } else { 1 };
    let mut first = [0u32; LINES];
    let mut foreign = [0u32; LINES];
    let mut low_min = [u32::MAX; LINES];
    let mut low_max = [0u32; LINES];
    let mut gap_us = 0u64;
    let mut shown = 0u32;
    let mut next = Instant::now();
    for i in 0..post_ms {
        Timer::at(next).await;
        next += Duration::from_micros(1000);
        let s = ctx.shot(i != 0, "after the gap", i + 1).await;
        let x = s.r;
        if i == 0 {
            for l in 0..LINES {
                if let Some(f) = s.fr[l]
                    && last_rise[l] != 0
                {
                    gap_us = f.first.saturating_sub(last_rise[l]) / 150;
                    break;
                }
            }
        }
        for l in 0..LINES {
            if first[l] != 0 {
                continue;
            }
            if stopped(&x.captures[l]) {
                first[l] = i + 1;
                continue;
            }
            let ln = &ctx.obs.l[l];
            let e = &ln.e[..ln.n];
            let (a, b) = s.fr[l].map_or((0, 0), |f| (f.j, f.end));
            let ours = |k: usize| k >= a && k < b;
            let mut cnt = 0u32;
            for k in 0..e.len() {
                if ours(k) {
                    continue;
                }
                cnt += 1;
                if e[k] & 1 == 0 && k + 1 < e.len() && !ours(k + 1) && e[k + 1] & 1 == 1 {
                    let w = ns((e[k + 1] >> 1) - (e[k] >> 1));
                    low_min[l] = low_min[l].min(w);
                    low_max[l] = low_max[l].max(w);
                }
            }
            foreign[l] += cnt;
            if detail && cnt > 0 && shown < 30 {
                shown += 1;
                defmt::info!(
                    "[INFO] {=str} part C: frame {=u32}, capture {}",
                    label,
                    i + 1,
                    x.captures[l]
                );
                dump_window(&ctx.obs, l, "part C", i + 1);
            }
        }
        if i >= 99 && first.iter().all(|&f| f != 0) {
            break;
        }
    }
    defmt::info!(
        "[STAT] {=str} gap {=u32} ms requested, {=u64} us measured: frames to the first stopped reply M4..M1 {=[?]} (0 = none in {=u32}); stopped replies in the last 200 before the gap {=[?]}; edges not ours before it {=[?]}, their low runs min {=[?]} max {=[?]} ns",
        label,
        g_ms,
        gap_us,
        &first[..],
        post_ms,
        &ok_tail[..],
        &foreign[..],
        &low_min[..],
        &low_max[..]
    );
    first
}

/// Part B: every window in which edges precede our frame, 10 s after a fresh start().
async fn burst_watch(ctx: &mut Ctx, label: &str, rep: u32) {
    ctx.phase = 0;
    let t_start = ctx.start().await;
    let mut first_good: [Option<Instant>; LINES] = [None; LINES];
    let mut events = [0u32; LINES];
    let mut logged = 0u32;
    let mut next = Instant::now();
    for i in 0..10_000u32 {
        Timer::at(next).await;
        next += Duration::from_micros(1000);
        let s = ctx.shot(true, "part B", i).await;
        let now = Instant::now();
        let x = s.r;
        for l in 0..LINES {
            if first_good[l].is_none() && stopped(&x.captures[l]) {
                first_good[l] = Some(now);
            }
            let Some(f) = s.fr[l] else { continue };
            if f.j == 0 {
                continue;
            }
            events[l] += 1;
            if logged < 40 {
                logged += 1;
                let e = &ctx.obs.l[l].e[..f.j];
                let e0 = e[0] >> 1;
                let mut rel = [0u32; 24];
                let m = e.len().min(24);
                for k in 0..m {
                    rel[k] = (ns((e[k] >> 1) - e0) << 1) | (e[k] & 1) as u32;
                }
                let to_frame = ns(f.first.saturating_sub(e[f.j - 1] >> 1));
                let since_good: i64 = first_good[l].map_or(-1, |t| (now - t).as_millis() as i64);
                defmt::info!(
                    "[STAT] {=str} B rep {=u32} {=str}: edges before our frame at exchange {=u32}, {=u64} ms after start() returned, {=i64} ms after this line's first good reply: {=usize} edges, the last {=u32} ns before our first falling edge; (ns from the first) << 1 | rising: {=[?]}",
                    label,
                    rep,
                    NAMES[l],
                    i,
                    (now - t_start).as_millis(),
                    since_good,
                    f.j,
                    to_frame,
                    &rel[..m]
                );
            }
        }
    }
    defmt::info!(
        "[STAT] {=str} B rep {=u32}: windows with edges before our frame per line M4..M1 {=[?]} ({=u32} shown)",
        label,
        rep,
        &events[..],
        logged
    );
}

async fn run_rate(ctx: &mut Ctx, rate: Rate) {
    let label: &str = match rate {
        Rate::Dshot300 => "DShot300",
        Rate::Dshot600 => "DShot600",
    };
    defmt::info!("[INFO] ===== {=str} =====", label);
    let mut survived = [0u32; LINES];
    let mut restarted = [u32::MAX; LINES];
    for &g in &GAPS_MS {
        let first = gap_trial(ctx, label, g, 2000, false).await;
        for l in 0..LINES {
            if first[l] == 1 || first[l] == 2 {
                survived[l] = survived[l].max(g);
            } else {
                restarted[l] = restarted[l].min(g);
            }
        }
    }
    for l in 0..LINES {
        defmt::info!(
            "[STAT] {=str} {=str}: largest gap survived {=u32} ms, smallest gap that restarted it {=u32} ms (4294967295 = none)",
            label,
            NAMES[l],
            survived[l],
            restarted[l]
        );
    }
    for rep in 1..=3 {
        burst_watch(ctx, label, rep).await;
    }
    let first = gap_trial(ctx, label, 800, 5000, true).await;
    defmt::info!(
        "[STAT] {=str} part C: 800 ms gap, no start(): frames (1 kHz) to the first stopped reply M4..M1 {=[?]}",
        label,
        &first[..]
    );
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("[INFO] dshot_m9_gaps: start");
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
        bad: [0; 3],
        phase: 0,
        incomplete: 0,
        dumps: 0,
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
        run_rate(&mut ctx, rate).await;
    }
    drop(ctx.drv.take());
    let fails = u32::from(ctx.bad.iter().any(|&b| b != 0))
        + u32::from(ctx.incomplete != 0)
        + u32::from(ctx.obs.ring_over != 0);
    if fails == 0 {
        defmt::info!(
            "[PASS] every trial completed; our frame whole in every window; ring overflows 0"
        );
    } else {
        defmt::error!(
            "[FAIL] windows without our whole frame (streams, after a gap, part C) {=[?]}; exchanges not sent {=u32}; ring overflows {=u32}",
            &ctx.bad[..],
            ctx.incomplete,
            ctx.obs.ring_over
        );
    }
    hil_common::exit(fails)
}
