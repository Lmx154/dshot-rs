//! `PioDshot`: bidirectional DShot on four consecutive pins of an RP2350 PIO block, with the
//! reply capture and the start sequence (feature `embassy-rp-0-10`).
//!
//! State machine 0 of the block runs the program of [`crate::program`]: it sends the frames and
//! captures the replies by DMA. The capture blocks when nobody reads it (a cancelled exchange):
//! it neither drops words (the next exchange reads exactly what is left, with no wait on the
//! CPU) nor ends early (no frame may start while a controller may still be replying).
#![allow(unsafe_code)]

use core::mem::ManuallyDrop;

use embassy_rp::dma;
use embassy_rp::gpio::Pull;
use embassy_rp::pio::program::{ArrayVec, PioVersion, Program, SideSet, Wrap};
use embassy_rp::pio::{
    Common, Config, Instance, LoadedProgram, Pin, ShiftConfig, ShiftDirection, StateMachine,
    StatusN, StatusSource,
};
use embedded_hal_async::delay::DelayNs;

use crate::capture::{LINES, LineCapture, decode_capture};
use crate::frame::FrameImage;
use crate::link::{BREAK_MS, LineMonitor, LineState, PARK_MS, release_check};
use crate::program::{self, exec};
use crate::rate::{DividerError, Rate};
use crate::value::{LineFrame, Value};

/// The most RX words of one capture (DShot600).
const MAX_CAPTURE_WORDS: usize = Rate::Dshot600.capture_words() as usize;

/// The parts a [`PioDshot`] is built from and gives back.
pub struct Parts<'d, PIO: Instance> {
    /// The PIO block's instruction memory and pin registration.
    pub common: Common<'d, PIO>,
    /// State machine 0: frames and capture.
    pub sm0: StateMachine<'d, PIO, 0>,
    /// The four motor lines, consecutive GPIO numbers in ascending order (line 0 first).
    pub pins: [Pin<'d, PIO>; LINES],
    /// The DMA channel for the capture.
    pub dma: dma::Channel<'d>,
}

/// Why a [`PioDshot`] could not be built.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum BuildError {
    /// The pins are not consecutive GPIO numbers in ascending order.
    PinsNotConsecutive,
    /// The system clock cannot give the rate (see [`Rate::clock_divider`]).
    Clock(DividerError),
    /// The block has no room for the 15 instructions of the program.
    NoSpace,
}

/// A failed [`PioDshot::new`]: the reason, and the parts, unchanged.
pub struct NewError<'d, PIO: Instance> {
    /// Why.
    pub reason: BuildError,
    /// The parts that were passed in.
    pub parts: Parts<'d, PIO>,
}

impl<PIO: Instance> core::fmt::Debug for NewError<'_, PIO> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("NewError")
            .field("reason", &self.reason)
            .finish_non_exhaustive()
    }
}

/// One frame and its replies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Exchange {
    /// What each line showed in the reply window, line 0 first.
    pub captures: [LineCapture; LINES],
    /// Each line's state after this capture.
    pub states: [LineState; LINES],
}

struct Hw<'d, PIO: Instance> {
    common: Common<'d, PIO>,
    sm0: StateMachine<'d, PIO, 0>,
    pins: [Pin<'d, PIO>; LINES],
    dma: dma::Channel<'d>,
    prog0: LoadedProgram<'d, PIO>,
    cfg0: Config<'d, PIO>,
}

/// Bidirectional DShot to four ESCs on four consecutive pins. See the crate README.
///
/// Call [`PioDshot::start`] before the first [`PioDshot::exchange`]: after power-up or a
/// passthrough session the controllers sit in their bootloaders, and the start sequence brings
/// them into their application. Nothing enforces it.
pub struct PioDshot<'d, PIO: Instance> {
    hw: ManuallyDrop<Hw<'d, PIO>>,
    rate: Rate,
    words: u32,
    /// RX words of a cancelled capture not read yet.
    pending: u32,
    last_all_stop: bool,
    monitor: LineMonitor,
}

/// Sets the pending words when the exchange that owns the capture is dropped mid-transfer.
struct CaptureGuard<'g, 'd> {
    dma: &'g mut dma::Channel<'d>,
    pending: &'g mut u32,
    buf_addr: u32,
    words: u32,
    armed: bool,
}

impl Drop for CaptureGuard<'_, '_> {
    fn drop(&mut self) {
        if self.armed {
            // The transfer was dropped before this guard (declared after it), which aborted the
            // channel and waited for its bus transfers to finish: the write address is final.
            let written = self.dma.write_addr().wrapping_sub(self.buf_addr) / 4;
            *self.pending = self.words.saturating_sub(written.min(self.words));
        }
    }
}

/// Releases the lines if `start()` is dropped during the break.
struct BreakGuard<'g, 'd, PIO: Instance> {
    drv: &'g mut PioDshot<'d, PIO>,
    armed: bool,
}

impl<PIO: Instance> Drop for BreakGuard<'_, '_, PIO> {
    fn drop(&mut self) {
        if self.armed {
            self.drv.exec0(&[exec::RELEASE_ALL]);
        }
    }
}

fn relative(addr: u8, origin: u8) -> u8 {
    addr.wrapping_sub(origin) & 31
}

fn program_of<const N: usize>(code: [u16; N], wrap: (u8, u8)) -> Program<N> {
    Program {
        code: ArrayVec::from(code),
        origin: None,
        wrap: Wrap {
            source: wrap.0,
            target: wrap.1,
        },
        side_set: SideSet::new(false, 0, false),
        version: PioVersion::V0,
    }
}

impl<'d, PIO: Instance + 'd> PioDshot<'d, PIO> {
    /// Takes the parts, loads the program (15 instructions), sets the pad pull-ups and leaves
    /// every line an input.
    ///
    /// The pins must be consecutive GPIO numbers in ascending order, made with this `Common`
    /// after any `make_pio_pin` of another block on the same GPIOs. Building sets the block's
    /// GPIOBASE window to the pins' (GPIO36-39: 16). `clk_sys_hz` is the system clock
    /// (`embassy_rp::clocks::clk_sys_freq()`).
    ///
    /// # Errors
    ///
    /// See [`BuildError`]; the parts come back in the error.
    pub fn new(
        parts: Parts<'d, PIO>,
        rate: Rate,
        clk_sys_hz: u32,
    ) -> Result<Self, NewError<'d, PIO>> {
        let Parts {
            mut common,
            mut sm0,
            mut pins,
            dma,
        } = parts;
        macro_rules! fail {
            ($reason:expr) => {
                return Err(NewError {
                    reason: $reason,
                    parts: Parts {
                        common,
                        sm0,
                        pins,
                        dma,
                    },
                })
            };
        }
        let consecutive = pins
            .iter()
            .zip(pins.iter().skip(1))
            .all(|(a, b)| a.pin().checked_add(1) == Some(b.pin()));
        if !consecutive {
            fail!(BuildError::PinsNotConsecutive);
        }
        let divider = match rate.clock_divider(clk_sys_hz) {
            Ok(d) => d,
            Err(e) => fail!(BuildError::Clock(e)),
        };
        let prog0 =
            match common.try_load_program(&program_of(program::DRIVER, program::DRIVER_WRAP)) {
                Ok(p) => p,
                Err(_) => fail!(BuildError::NoSpace),
            };
        for pin in &mut pins {
            pin.set_pull(Pull::Up);
        }
        let [a, b, c, d] = &pins;
        let refs = [a, b, c, d];

        let mut cfg0 = Config::default();
        cfg0.use_program(&prog0, &[]);
        cfg0.clock_divider = 1u8.into();
        cfg0.clock_divider = cfg0
            .clock_divider
            .wrapping_mul_int(divider.as_q8())
            .wrapping_shr(8);
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
        cfg0.set_set_pins(&refs);
        cfg0.set_out_pins(&refs);
        cfg0.set_in_pins(&refs);

        sm0.set_enable(false);
        sm0.set_config(&cfg0);
        sm0.clear_fifos();
        sm0.restart();
        let samples = program::samples(rate);
        let mut this = Self {
            hw: ManuallyDrop::new(Hw {
                common,
                sm0,
                pins,
                dma,
                prog0,
                cfg0,
            }),
            rate,
            words: rate.capture_words(),
            pending: 0,
            last_all_stop: true,
            monitor: LineMonitor::new(),
        };
        // State machine 0: lines released, latch low, Y = samples - 1 (which also empties the
        // OSR, so the first frame's pull takes word 0), at idle.
        this.hw.sm0.tx().push(samples - 1);
        let idle = this.addr0(program::DRIVER_IDLE);
        this.exec0(&[
            exec::RELEASE_ALL,
            exec::LATCH_LOW,
            exec::PULL,
            exec::OUT_Y_32,
            exec::jmp_to(idle),
        ]);
        this.hw.sm0.clkdiv_restart();
        this.hw.sm0.set_enable(true);
        Ok(this)
    }

    /// Hands the parts back: the state machine stopped and restarted, its FIFOs emptied, the
    /// program freed, every line an input with its pad pull-up on.
    ///
    /// # Errors
    ///
    /// `Err(self)`, unchanged, unless nothing is in flight (a cancelled exchange can leave a
    /// capture behind; the next `exchange` or `start` finishes it) and the last frame pushed was
    /// Stop on all four lines (invariant 3.5).
    pub fn release(mut self) -> Result<Parts<'d, PIO>, Self> {
        if release_check(self.in_flight(), self.last_all_stop).is_err() {
            return Err(self);
        }
        self.stop_all();
        // SAFETY: `hw` is taken exactly once, here, and `self` is forgotten right after, so
        // `Drop` never sees the taken value.
        let hw = unsafe { ManuallyDrop::take(&mut self.hw) };
        core::mem::forget(self);
        let Hw {
            mut common,
            sm0,
            pins,
            dma,
            prog0,
            ..
        } = hw;
        // SAFETY: the state machine is stopped; only this driver's instructions are freed.
        unsafe {
            common.free_instr(prog0.used_memory);
        }
        Ok(Parts {
            common,
            sm0,
            pins,
            dma,
        })
    }

    /// The rate, fixed at build.
    #[must_use]
    pub const fn rate(&self) -> Rate {
        self.rate
    }

    /// The line states and the Foreign counters (Starting and Live apart).
    #[must_use]
    pub const fn monitor(&self) -> &LineMonitor {
        &self.monitor
    }

    /// Starts or restarts the stream (decision, amendment after M1): any capture a cancelled
    /// exchange left is finished; the lines stay released for [`PARK_MS`]; all four are driven
    /// low together for [`BREAK_MS`]; released. Dropped at any await, it leaves the lines
    /// released. Every line becomes Starting.
    pub async fn start(&mut self, delay: &mut impl DelayNs) {
        self.monitor.restart();
        self.finish_cancelled().await;
        delay.delay_ms(PARK_MS).await;
        let mut guard = BreakGuard {
            drv: self,
            armed: true,
        };
        guard.drv.exec0(&[exec::LATCH_LOW, exec::DRIVE_ALL]);
        delay.delay_ms(BREAK_MS).await;
        guard.drv.exec0(&[exec::RELEASE_ALL]);
        guard.armed = false;
        drop(guard);
    }

    /// Sends one frame (one value per line) and returns what each line answered.
    ///
    /// The two image words go into the TX FIFO with no await between them and the program sends
    /// the frame whole, captures the reply window by DMA, and the window is decoded at 8 samples
    /// per bit.
    ///
    /// Dropped before the push, nothing was sent. Dropped after it, the frame still goes out
    /// whole and its capture is discarded: the next `exchange` (or `start`) first reads what is
    /// left of it, then sends its own frame and returns only its own replies. One call is one
    /// frame; the driver never sends a frame of its own and never repeats one.
    pub async fn exchange(&mut self, frames: [LineFrame; LINES]) -> Exchange {
        self.finish_cancelled().await;
        let image = FrameImage::new(frames).words();
        let words = self.words;
        let mut buf = [0u32; MAX_CAPTURE_WORDS];
        let Self {
            hw,
            pending,
            last_all_stop,
            ..
        } = self;
        let Hw { sm0, dma, .. } = &mut **hw;
        let mut guard = CaptureGuard {
            dma,
            pending,
            buf_addr: buf.as_ptr() as u32,
            words,
            armed: false,
        };
        // The DMA is ready before the push.
        let mut transfer = {
            let len = (words as usize).min(MAX_CAPTURE_WORDS);
            let dst = core::ptr::slice_from_raw_parts_mut(buf.as_mut_ptr(), len);
            let src = sm0.rx_fifo_ptr() as *const u32;
            let treq = sm0.rx_treq();
            // SAFETY: `buf` lives in this future, which is pinned while polled, and outlives
            // `transfer`, which is declared after it and aborts the channel when dropped; the
            // source is the RX FIFO of state machine 0 with its DREQ.
            unsafe { guard.dma.read(src, dst, treq, false) }
        };
        // Both words with nothing between them.
        let [w0, w1] = image;
        let tx = sm0.tx();
        tx.push(w0);
        tx.push(w1);
        *last_all_stop = frames.iter().all(|f| f.value == Value::Stop);
        guard.armed = true;
        *guard.pending = words;
        (&mut transfer).await;
        drop(transfer);
        guard.armed = false;
        *guard.pending = 0;
        drop(guard);
        let len = (words as usize).min(MAX_CAPTURE_WORDS);
        let captures = decode_capture(
            buf.get(..len).unwrap_or(&buf),
            self.rate.reply_samples_per_bit(),
        );
        let states = self.monitor.record(&captures);
        Exchange { captures, states }
    }

    // -----------------------------------------------------------------------------------------

    fn addr0(&self, rel: u8) -> u8 {
        self.hw.prog0.origin.wrapping_add(rel) & 31
    }

    fn pc0(&self) -> u8 {
        relative(self.hw.sm0.get_addr(), self.hw.prog0.origin)
    }

    /// A frame or capture is unfinished: a cancelled exchange left words, or the driver is not
    /// at its idle loop or the end of a capture already read.
    fn in_flight(&mut self) -> bool {
        if self.pending > 0 {
            return true;
        }
        let pc = self.pc0();
        let settled = matches!(
            pc,
            program::DRIVER_IDLE
                | program::DRIVER_COMMIT
                | program::DRIVER_SAMPLE
                | program::DRIVER_SAMPLE_END
        );
        let rx_empty = self.hw.sm0.rx().empty();
        let tx_empty = self.hw.sm0.tx().empty();
        !(settled && rx_empty && tx_empty)
    }

    /// Runs instructions on state machine 0, stopped, at divider 1, then restores its divider
    /// and its enable.
    fn exec0(&mut self, instrs: &[u16]) {
        let divider = self.hw.cfg0.clock_divider;
        let sm = &mut self.hw.sm0;
        let enabled = sm.is_enabled();
        sm.set_enable(false);
        sm.set_clock_divider(1u8.into());
        for &i in instrs {
            // SAFETY: the instructions are the driver's own set-up and line instructions
            // (`program::exec`). The only blocking one is `exec::PULL`, used in `new()`, and it
            // returns there because `samples - 1` was pushed to the TX FIFO just before it.
            unsafe { sm.exec_instr(i) };
        }
        sm.set_clock_divider(divider);
        sm.clkdiv_restart();
        sm.set_enable(enabled);
    }

    /// Reads the rest of a capture a cancelled exchange left, by DMA (no wait on the CPU).
    async fn finish_cancelled(&mut self) {
        let left = self.pending;
        if left == 0 {
            return;
        }
        let mut sink = [0u32; MAX_CAPTURE_WORDS];
        let len = (left as usize).min(MAX_CAPTURE_WORDS);
        let Self { hw, pending, .. } = self;
        let Hw { sm0, dma, .. } = &mut **hw;
        let mut guard = CaptureGuard {
            dma,
            pending,
            buf_addr: sink.as_ptr() as u32,
            words: left,
            armed: true,
        };
        let mut transfer = match sink.get_mut(..len) {
            Some(dst) => sm0.rx().dma_pull(guard.dma, dst, false),
            None => return,
        };
        (&mut transfer).await;
        drop(transfer);
        guard.armed = false;
        *guard.pending = 0;
    }

    /// The state machine stopped and restarted, FIFOs empty, lines released.
    fn stop_all(&mut self) {
        self.hw.sm0.set_enable(false);
        self.exec0(&[exec::RELEASE_ALL]);
        self.hw.sm0.clear_fifos();
        self.hw.sm0.restart();
    }
}

impl<PIO: Instance> Drop for PioDshot<'_, PIO> {
    fn drop(&mut self) {
        self.hw.sm0.set_enable(false);
        self.hw.sm0.set_clock_divider(1u8.into());
        // SAFETY: the state machine is stopped; this only releases the four lines.
        unsafe { self.hw.sm0.exec_instr(exec::RELEASE_ALL) };
        self.hw.sm0.clear_fifos();
        // SAFETY: `hw` is dropped exactly once, here; `release` forgets `self` after taking it.
        unsafe { ManuallyDrop::drop(&mut self.hw) };
    }
}

/// Compile-time checks that the driver and its futures are `Send` (contract A5), with a delay
/// whose future is `Send` (the trait's `async fn` does not promise it).
#[allow(dead_code)]
fn assert_send<'d, PIO: Instance + Send>(drv: &mut PioDshot<'d, PIO>) {
    struct SendDelay;
    impl DelayNs for SendDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }
    fn is_send<T: Send>(_: &T) {}
    is_send(&*drv);
    is_send(&drv.exchange([LineFrame::STOP; LINES]));
    is_send(&drv.start(&mut SendDelay));
}
