//! The PIO program of the back-end, as the instruction words it loads, and its timing.
//!
//! It lives here, in the pure core, so that the host tests can check the assembled words (the
//! encoding, invariant 2.1, the cycle counts) without a HAL. The board test assembles the same
//! source with `pio_asm!` and compares.
//!
//! # The driver (state machine 0), 15 instructions
//!
//! `X` counts bits, then samples. `Y` holds the number of samples minus one, loaded once by the
//! CPU. Clock: 40 cycles per DShot bit ([`Rate::clock_divider`](crate::Rate::clock_divider)).
//!
//! ```text
//!  0 idle:  mov x, status            ; all ones while the TX FIFO holds fewer than 2 words
//!  1        jmp x-- idle             ; both words in (x = 0): fall through
//!  2        set pins, 0              ; output latch low; the lines are still inputs
//!  3        set x, 15
//!  4        set pindirs, 15 [1]      ; the four lines driven, low: bit 0 begins
//!  5 bit:   pull ifempty block [12]  ; 15 cycles low (word 0 at bit 0, word 1 at bit 8)
//!  6        out pins, 4 [14]         ; 15 cycles of data
//!  7        set pins, 15 [8]         ; 10 cycles high with the jmp
//!  8        jmp x-- low
//!  9        set pindirs, 0           ; released: the pad pull-ups hold the lines (frame end)
//! 10        mov x, y [1]             ; the first sample 3 cycles after the release
//! 11 smp:   in pins, 4 [2]           ; one sample every 4 cycles, autopush every 8 (blocking)
//! 12        jmp x-- smp              ; (wrap to idle)
//! 13 low:   set pins, 0              ; bits 1 to 15: low phase begins
//! 14        jmp bit
//! ```
//!
//! The lines are switched to output in exactly one place, instruction 4, with the latch already
//! low, so the first edge of a frame is a falling one whatever the line level (no spike), and
//! back to input in exactly one place, instruction 9, the exit of the bit loop. Autopull is off
//! and every pull blocks; the frame starts only when both words are in the FIFO. The capture
//! blocks on a full RX FIFO, so it delivers exactly `8 * words` samples, never fewer, and the
//! program reaches idle only after the whole window: no frame can start while a controller may
//! still be replying.

use crate::rate::Rate;

// ---------------------------------------------------------------------------------------------
// Encoding (RP2350 datasheet section 11.4; the subset this program uses, no side-set).

const fn jmp(cond: u16, addr: u16) -> u16 {
    (cond << 5) | addr
}
const fn in_pins(count: u16, delay: u16) -> u16 {
    (0b010 << 13) | (delay << 8) | count
}
const fn out(dest: u16, count: u16, delay: u16) -> u16 {
    (0b011 << 13) | (delay << 8) | (dest << 5) | (count & 31)
}
const fn pull_ifempty_block(delay: u16) -> u16 {
    (0b100 << 13) | (delay << 8) | (1 << 7) | (1 << 6) | (1 << 5)
}
const fn pull_block() -> u16 {
    (0b100 << 13) | (1 << 7) | (1 << 5)
}
const fn mov(dest: u16, op: u16, src: u16, delay: u16) -> u16 {
    (0b101 << 13) | (delay << 8) | (dest << 5) | (op << 3) | src
}
const fn set(dest: u16, data: u16, delay: u16) -> u16 {
    (0b111 << 13) | (delay << 8) | (dest << 5) | data
}

const COND_ALWAYS: u16 = 0b000;
const COND_X_DEC: u16 = 0b010;
const SET_PINS: u16 = 0b000;
const SET_X: u16 = 0b001;
const SET_PINDIRS: u16 = 0b100;
const OUT_PINS: u16 = 0b000;
const OUT_Y: u16 = 0b010;
const MOV_X: u16 = 0b001;
const MOV_SRC_Y: u16 = 0b010;
const MOV_SRC_STATUS: u16 = 0b101;

/// The driver program (state machine 0), addresses from 0. See the module documentation.
pub const DRIVER: [u16; 15] = [
    mov(MOV_X, 0, MOV_SRC_STATUS, 0),      // 0 idle
    jmp(COND_X_DEC, DRIVER_IDLE as u16),   // 1
    set(SET_PINS, 0, 0),                   // 2
    set(SET_X, 15, 0),                     // 3
    set(SET_PINDIRS, 15, 1),               // 4
    pull_ifempty_block(12),                // 5 bit
    out(OUT_PINS, 4, 14),                  // 6
    set(SET_PINS, 15, 8),                  // 7
    jmp(COND_X_DEC, DRIVER_LOW as u16),    // 8
    set(SET_PINDIRS, 0, 0),                // 9
    mov(MOV_X, 0, MOV_SRC_Y, 1),           // 10
    in_pins(4, 2),                         // 11 smp
    jmp(COND_X_DEC, DRIVER_SAMPLE as u16), // 12
    set(SET_PINS, 0, 0),                   // 13 low
    jmp(COND_ALWAYS, DRIVER_BIT as u16),   // 14
];

/// The driver's `idle`: the status read (`mov x, status`).
pub const DRIVER_IDLE: u8 = 0;
/// The driver's commit point (`jmp x-- idle`): falling through starts the frame.
pub const DRIVER_COMMIT: u8 = 1;
/// The first instruction of the frame (latch low).
pub const DRIVER_FRAME: u8 = 2;
/// The one instruction that switches the lines to output (`set pindirs, 15`).
pub const DRIVER_DRIVE: u8 = 4;
/// The top of the bit loop.
pub const DRIVER_BIT: u8 = 5;
/// The bit loop's exit test (`jmp x-- low`).
pub const DRIVER_BIT_END: u8 = 8;
/// The one instruction that switches the lines back to input (`set pindirs, 0`).
pub const DRIVER_RELEASE: u8 = 9;
/// The sample instruction (`in pins, 4 [2]`).
pub const DRIVER_SAMPLE: u8 = 11;
/// The sample loop's test (`jmp x-- smp`), the wrap source.
pub const DRIVER_SAMPLE_END: u8 = 12;
/// The low phase of bits 1 to 15.
pub const DRIVER_LOW: u8 = 13;
/// Wrap source and target of the driver.
pub const DRIVER_WRAP: (u8, u8) = (DRIVER_SAMPLE_END, DRIVER_IDLE);

/// Instructions the CPU executes on the state machine (`SMx_INSTR`).
pub mod exec {
    use super::*;

    /// `set pins, 0`: output latch low (no effect on a line that is an input).
    pub const LATCH_LOW: u16 = set(SET_PINS, 0, 0);
    /// `set pindirs, 15`: the four lines driven at the latch level. The break uses it, with the
    /// latch low.
    pub const DRIVE_ALL: u16 = set(SET_PINDIRS, 15, 0);
    /// `set pindirs, 0`: the four lines released.
    pub const RELEASE_ALL: u16 = set(SET_PINDIRS, 0, 0);
    /// `pull block`: a word from the TX FIFO into the OSR.
    pub const PULL: u16 = pull_block();
    /// `out y, 32`: the OSR into Y, and the OSR empty.
    pub const OUT_Y_32: u16 = out(OUT_Y, 32, 0);

    /// `jmp addr` (an absolute instruction address, 0 to 31).
    #[must_use]
    pub const fn jmp_to(addr: u8) -> u16 {
        jmp(COND_ALWAYS, (addr & 31) as u16)
    }
}

// ---------------------------------------------------------------------------------------------
// Timing, in cycles of the driver's PIO clock unless said otherwise.

/// PIO cycles from the first falling edge of a frame to the release of the lines: 16 bits of 40.
pub const FRAME_CYCLES: u32 = 16 * Rate::CYCLES_PER_BIT;
/// From the status read that saw both words to the first falling edge: 4 cycles.
pub const STATUS_TO_FIRST_EDGE: u32 = 4;
/// From the last rising edge of a frame to the release: the high phase of the last bit, 10.
pub const LAST_RISE_TO_RELEASE: u32 = 10;
/// From the release to the first reply sample: 3 cycles.
pub const RELEASE_TO_FIRST_SAMPLE: u32 = 3;

/// Samples in one capture: 8 per RX word, 272 at DShot300 and 368 at DShot600.
#[must_use]
pub const fn samples(rate: Rate) -> u32 {
    Rate::SAMPLES_PER_WORD * rate.capture_words()
}

/// From the release to the driver's next `idle`: the samples and the loop overhead, `4 * S + 3`.
#[must_use]
pub const fn release_to_idle(rate: Rate) -> u32 {
    RELEASE_TO_FIRST_SAMPLE + Rate::REPLY_SAMPLE_CYCLES * samples(rate)
}

/// The shortest start-to-start spacing of two frames the program allows (invariant 2.4): the
/// frame, the window and the way back to the first edge, `647 + 4 * S` cycles: 1735 (144.6 us)
/// at DShot300 and 2119 (88.3 us) at DShot600.
#[must_use]
pub const fn min_frame_period(rate: Rate) -> u32 {
    // idle is the status read, then 4 cycles to the edge.
    FRAME_CYCLES + release_to_idle(rate) + STATUS_TO_FIRST_EDGE
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FrameImage;
    use crate::value::{LineFrame, Throttle, Value};
    use std::collections::VecDeque;
    use std::vec::Vec;

    // ---------------------------------------------------------------------------------------
    // An independent decoder of the words, written from the datasheet's encoding table.

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum I {
        Jmp { cond: u16, addr: u16 },
        InPins { count: u16 },
        Out { dest: u16, count: u16 },
        Pull { ifempty: bool, block: bool },
        Push,
        Mov { dest: u16, op: u16, src: u16 },
        Set { dest: u16, data: u16 },
        Other(u16),
    }

    fn decode(w: u16) -> (I, u16) {
        let delay = (w >> 8) & 31;
        let low = w & 0xFF;
        let i = match w >> 13 {
            0 => I::Jmp {
                cond: (low >> 5) & 7,
                addr: low & 31,
            },
            2 if (low >> 5) & 7 == 0 => I::InPins { count: low & 31 },
            3 => I::Out {
                dest: (low >> 5) & 7,
                count: if low & 31 == 0 { 32 } else { low & 31 },
            },
            4 if low & 0x80 != 0 && low & 0x1F == 0 => I::Pull {
                ifempty: low & 0x40 != 0,
                block: low & 0x20 != 0,
            },
            4 if low & 0x80 == 0 => I::Push,
            5 => I::Mov {
                dest: (low >> 5) & 7,
                op: (low >> 3) & 3,
                src: low & 7,
            },
            7 => I::Set {
                dest: (low >> 5) & 7,
                data: low & 31,
            },
            _ => I::Other(w),
        };
        (i, delay)
    }

    #[test]
    fn driver_words_decode_to_the_documented_program() {
        let want = [
            (
                I::Mov {
                    dest: 1,
                    op: 0,
                    src: 5,
                },
                0,
            ),
            (I::Jmp { cond: 2, addr: 0 }, 0),
            (I::Set { dest: 0, data: 0 }, 0),
            (I::Set { dest: 1, data: 15 }, 0),
            (I::Set { dest: 4, data: 15 }, 1),
            (
                I::Pull {
                    ifempty: true,
                    block: true,
                },
                12,
            ),
            (I::Out { dest: 0, count: 4 }, 14),
            (I::Set { dest: 0, data: 15 }, 8),
            (I::Jmp { cond: 2, addr: 13 }, 0),
            (I::Set { dest: 4, data: 0 }, 0),
            (
                I::Mov {
                    dest: 1,
                    op: 0,
                    src: 2,
                },
                1,
            ),
            (I::InPins { count: 4 }, 2),
            (I::Jmp { cond: 2, addr: 11 }, 0),
            (I::Set { dest: 0, data: 0 }, 0),
            (I::Jmp { cond: 0, addr: 5 }, 0),
        ];
        assert_eq!(DRIVER.len(), want.len());
        for (a, (&w, want)) in DRIVER.iter().zip(want.iter()).enumerate() {
            assert_eq!(decode(w), *want, "address {a}: {w:#06x}");
        }
        // Hand-checked words of three instructions against the datasheet table.
        assert_eq!(DRIVER[0], 0xA025); // mov x, status
        assert_eq!(DRIVER[4], 0xE18F); // set pindirs, 15 [1]
        assert_eq!(DRIVER[5], 0x8CE0); // pull ifempty block [12]
    }

    #[test]
    fn exec_words_are_the_instructions_named() {
        assert_eq!(decode(exec::LATCH_LOW), (I::Set { dest: 0, data: 0 }, 0));
        assert_eq!(decode(exec::DRIVE_ALL), (I::Set { dest: 4, data: 15 }, 0));
        assert_eq!(decode(exec::RELEASE_ALL), (I::Set { dest: 4, data: 0 }, 0));
        assert_eq!(
            decode(exec::PULL),
            (
                I::Pull {
                    ifempty: false,
                    block: true
                },
                0
            )
        );
        assert_eq!(decode(exec::OUT_Y_32), (I::Out { dest: 2, count: 32 }, 0));
        assert_eq!(decode(exec::jmp_to(29)), (I::Jmp { cond: 0, addr: 29 }, 0));
        assert_eq!(exec::jmp_to(33), exec::jmp_to(1));
    }

    /// Invariant 2.1 and 2.3 on the assembled words: the lines switch to output at one place,
    /// before the bit loop, with the latch low, and back at one place, the loop's exit; no
    /// non-blocking pull, no autopull dependence, no side-set, no EXEC from data.
    #[test]
    fn driver_switches_the_lines_to_output_only_before_the_bit_loop_and_back_after_it() {
        let mut to_out = Vec::new();
        let mut to_in = Vec::new();
        for (a, &w) in DRIVER.iter().enumerate() {
            match decode(w).0 {
                I::Set { dest: 4, data: 15 } => to_out.push(a),
                I::Set { dest: 4, data: 0 } => to_in.push(a),
                I::Set { dest: 4, .. } => panic!("partial pindirs at {a}"),
                I::Out { dest, .. } => assert!(dest != 4 && dest != 7 && dest != 5, "{a}"),
                I::Mov { dest, .. } => assert!(dest != 3 && dest != 4 && dest != 5, "{a}"),
                I::Pull { block, .. } => assert!(block, "pull noblock at {a}"),
                I::Other(_) | I::Push => panic!("unexpected at {a}"),
                _ => {}
            }
        }
        assert_eq!(to_out, [DRIVER_DRIVE as usize]);
        assert_eq!(to_in, [DRIVER_RELEASE as usize]);
        // Before the drive: the latch is set low and nothing else touches the pins.
        assert_eq!(
            decode(DRIVER[DRIVER_FRAME as usize]).0,
            I::Set { dest: 0, data: 0 }
        );
        // The drive falls straight into the bit loop.
        assert_eq!(DRIVER_DRIVE + 1, DRIVER_BIT);
        // The bit loop is 5..=8 and 13..=14; its only exit is the fall-through of 8 into 9.
        let bit_loop = [5usize, 6, 7, 8, 13, 14];
        for &a in &bit_loop {
            if let I::Jmp { cond, addr } = decode(DRIVER[a]).0 {
                assert!(
                    bit_loop.contains(&(addr as usize)),
                    "jump out of the loop at {a}"
                );
                if a == 8 {
                    assert_eq!(cond, COND_X_DEC);
                } else {
                    assert_eq!(cond, COND_ALWAYS);
                }
            }
        }
        assert_eq!(DRIVER_BIT_END + 1, DRIVER_RELEASE);
        // No other instruction jumps into the frame part except the commit's fall-through.
        for (a, &w) in DRIVER.iter().enumerate() {
            if let I::Jmp { addr, .. } = decode(w).0 {
                let into_frame = (2..=9).contains(&addr) && !bit_loop.contains(&a);
                assert!(!into_frame, "jump into the frame at {a}");
            }
        }
        assert_eq!(DRIVER_WRAP, (12, 0));
        // The break: the CPU drives with the same instruction, latch low first.
        assert_eq!(exec::DRIVE_ALL & !0x1F00, DRIVER[4] & !0x1F00);
        assert_eq!(exec::RELEASE_ALL, DRIVER[9]);
    }

    // ---------------------------------------------------------------------------------------
    // A cycle-accurate model of the state machine for this program (RP2350 datasheet 11.4 and
    // 11.5: delays, wrap, blocking pull, autopush that stalls on a full RX FIFO).

    #[derive(Clone, Copy)]
    enum Status {
        TxBelow(usize),
    }

    struct Sm {
        prog: &'static [u16],
        wrap: (u8, u8),
        pc: u8,
        x: u32,
        y: u32,
        isr: u32,
        isr_count: u32,
        osr: u32,
        osr_count: u32,
        delay: u16,
        status: Status,
        tx: VecDeque<u32>,
        rx: VecDeque<u32>,
        rx_cap: usize,
        latch: u8,
        dirs: u8,
        ticks: u64,
    }

    impl Sm {
        fn new(prog: &'static [u16], wrap: (u8, u8), status: Status) -> Self {
            Sm {
                prog,
                wrap,
                pc: 0,
                x: 0,
                y: 0,
                isr: 0,
                isr_count: 0,
                osr: 0,
                osr_count: 32,
                delay: 0,
                status,
                tx: VecDeque::new(),
                rx: VecDeque::new(),
                rx_cap: 4,
                latch: 0,
                dirs: 0,
                ticks: 0,
            }
        }

        fn tick(&mut self, pins: u8) {
            self.ticks += 1;
            if self.delay > 0 {
                self.delay -= 1;
                return;
            }
            let (i, delay) = decode(self.prog[self.pc as usize]);
            let mut next = None;
            match i {
                I::Jmp { cond, addr } => {
                    let take = match cond {
                        0 => true,
                        1 => self.x == 0,
                        2 => {
                            let t = self.x != 0;
                            self.x = self.x.wrapping_sub(1);
                            t
                        }
                        4 => {
                            let t = self.y != 0;
                            self.y = self.y.wrapping_sub(1);
                            t
                        }
                        _ => panic!("jmp condition {cond}"),
                    };
                    if take {
                        next = Some(addr as u8);
                    }
                }
                I::InPins { count } => {
                    if self.isr_count + u32::from(count) >= 32 && self.rx.len() >= self.rx_cap {
                        return; // autopush stall on a full RX FIFO
                    }
                    let v = u32::from(pins) & ((1 << count) - 1);
                    self.isr = (self.isr << count) | v;
                    self.isr_count += u32::from(count);
                    if self.isr_count >= 32 {
                        self.rx.push_back(self.isr);
                        self.isr = 0;
                        self.isr_count = 0;
                    }
                }
                I::Out { dest, count } => {
                    let v = if count == 32 {
                        self.osr
                    } else {
                        self.osr & ((1 << count) - 1)
                    };
                    self.osr = if count == 32 { 0 } else { self.osr >> count };
                    self.osr_count = (self.osr_count + u32::from(count)).min(32);
                    match dest {
                        0 => self.latch = (v & 0xF) as u8,
                        2 => self.y = v,
                        _ => panic!("out dest {dest}"),
                    }
                }
                I::Pull { ifempty, block } => {
                    assert!(block);
                    if !(ifempty && self.osr_count < 32) {
                        match self.tx.pop_front() {
                            Some(w) => {
                                self.osr = w;
                                self.osr_count = 0;
                            }
                            None => return, // stalled
                        }
                    }
                }
                I::Mov { dest, op, src } => {
                    let v = match src {
                        2 => self.y,
                        5 => {
                            let on = match self.status {
                                Status::TxBelow(n) => self.tx.len() < n,
                            };
                            if on { u32::MAX } else { 0 }
                        }
                        7 => self.osr,
                        _ => panic!("mov src {src}"),
                    };
                    let v = if op == 1 { !v } else { v };
                    match dest {
                        1 => self.x = v,
                        2 => self.y = v,
                        _ => panic!("mov dest {dest}"),
                    }
                }
                I::Set { dest, data } => match dest {
                    0 => self.latch = data as u8,
                    1 => self.x = u32::from(data),
                    4 => self.dirs = data as u8,
                    _ => panic!("set dest {dest}"),
                },
                I::Push | I::Other(_) => panic!("unsupported"),
            }
            self.delay = delay;
            self.pc = match next {
                Some(a) => a,
                None if self.pc == self.wrap.0 => self.wrap.1,
                None => self.pc + 1,
            };
        }
    }

    /// The line level each line shows, given our drive: driven lines follow the latch; released
    /// lines read `external` (the pull-up, or a controller).
    fn line_levels(sm: &Sm, external: u8) -> u8 {
        (sm.latch & sm.dirs) | (external & !sm.dirs)
    }

    /// Driver alone at divider 1 (one tick per cycle), Y loaded with `samples - 1`.
    fn driver(samples: u32) -> Sm {
        let mut sm = Sm::new(&DRIVER, DRIVER_WRAP, Status::TxBelow(2));
        sm.y = samples - 1;
        sm
    }

    /// One cycle of the driver alone: (dirs, latch) as the cycle leaves them.
    fn step(sm: &mut Sm, pins: u8) -> (u8, u8) {
        sm.tick(pins);
        (sm.dirs, sm.latch)
    }

    /// Pushes both words and runs the driver (cycle 0 is the first after the push) until it has
    /// sent one frame and come back to idle. Returns per cycle the (dirs, latch) that cycle left;
    /// the RX words go to `CAPTURED` as a DMA would take them.
    fn run_frame(sm: &mut Sm, words: [u32; 2], pins: impl Fn(u64) -> u8) -> Vec<(u8, u8)> {
        sm.tx.push_back(words[0]);
        sm.tx.push_back(words[1]);
        let mut trace = Vec::new();
        let mut t = 0u64;
        let mut left_idle = false;
        loop {
            trace.push(step(sm, pins(t)));
            t += 1;
            while let Some(w) = sm.rx.pop_front() {
                CAPTURED.with(|c| c.borrow_mut().push(w));
            }
            if sm.pc > DRIVER_COMMIT {
                left_idle = true;
            }
            if left_idle && sm.pc == DRIVER_IDLE {
                return trace;
            }
            assert!(t < 100_000, "no return to idle");
        }
    }

    std::thread_local! {
        static CAPTURED: core::cell::RefCell<Vec<u32>> = const { core::cell::RefCell::new(Vec::new()) };
    }

    fn take_captured() -> Vec<u32> {
        CAPTURED.with(|c| core::mem::take(&mut *c.borrow_mut()))
    }

    fn image(lines: [LineFrame; 4]) -> [u32; 2] {
        FrameImage::new(lines).words()
    }

    fn mixed_image() -> [u32; 2] {
        image([
            LineFrame::STOP,
            LineFrame::new(Value::Command(crate::Command::Beep1), true),
            LineFrame::new(Value::Throttle(Throttle::new(1234).unwrap()), false),
            LineFrame::new(Value::Command(crate::Command::ExtendedTelemetryOn), false),
        ])
    }

    #[test]
    fn driver_frame_is_16_bits_of_15_low_15_data_10_high_with_the_image_levels() {
        for words in [FrameImage::STOP.words(), mixed_image()] {
            let mut sm = driver(samples(Rate::Dshot300));
            take_captured();
            let trace = run_frame(&mut sm, words, |_| 0xF);
            let first = trace.iter().position(|&(d, _)| d != 0).unwrap();
            // Driven for exactly 640 cycles, all four lines, then never again.
            for (t, &(d, _)) in trace.iter().enumerate() {
                let inside = t >= first && t < first + FRAME_CYCLES as usize;
                assert_eq!(d, if inside { 0xF } else { 0 }, "cycle {t}");
            }
            // Cycle 0 is the status read that sees both words.
            assert_eq!(first as u32, STATUS_TO_FIRST_EDGE);
            // At the cycle the lines are switched to output the latch is already low.
            assert_eq!(trace[first].1, 0);
            assert_eq!(trace[first - 1], (0, 0));
            let nibble = |j: usize| ((words[j / 8] >> (4 * (j % 8))) & 0xF) as u8;
            for j in 0..16 {
                let b = first + 40 * j;
                for (k, &(_, latch)) in trace[b..b + 40].iter().enumerate() {
                    let want = match k {
                        0..15 => 0,
                        15..30 => nibble(j),
                        _ => 0xF,
                    };
                    assert_eq!(latch, want, "bit {j}, cycle {k} of the bit");
                }
            }
            let caps = take_captured();
            assert_eq!(caps.len() as u32, Rate::Dshot300.capture_words());
        }
    }

    #[test]
    fn driver_frame_starts_with_a_falling_edge_on_a_line_held_low() {
        // A controller holds every line low (M8): the first change of any line is a falling
        // edge driven by us onto a line that is already low, so the level never goes high
        // before the first data or high phase. No spike.
        let mut sm = driver(samples(Rate::Dshot600));
        sm.tx.push_back(FrameImage::STOP.words()[0]);
        sm.tx.push_back(FrameImage::STOP.words()[1]);
        let mut levels = Vec::new();
        for _ in 0..60 {
            sm.tick(0);
            levels.push(line_levels(&sm, 0));
        }
        let first_high = levels.iter().position(|&l| l != 0).unwrap();
        // The first high is the data phase of bit 0 (a 0 bit is high from cycle 15 of the bit),
        // the drive having begun at cycle 4.
        assert_eq!(first_high as u32, STATUS_TO_FIRST_EDGE + 15);
        take_captured();
    }

    #[test]
    fn driver_samples_every_4_cycles_from_3_cycles_after_the_release() {
        for rate in [Rate::Dshot300, Rate::Dshot600] {
            let mut sm = driver(samples(rate));
            take_captured();
            let input = |t: u64| ((t * 7 + 3) & 0xF) as u8;
            let trace = run_frame(&mut sm, FrameImage::STOP.words(), input);
            let release = trace.iter().rposition(|&(d, _)| d != 0).unwrap() + 1;
            let caps = take_captured();
            assert_eq!(caps.len() as u32, rate.capture_words());
            for (k, nib) in caps
                .iter()
                .flat_map(|w| (0..8).map(move |j| (w >> (28 - 4 * j)) & 0xF))
                .enumerate()
            {
                let t = release as u64 + u64::from(RELEASE_TO_FIRST_SAMPLE) + 4 * k as u64;
                assert_eq!(nib, u32::from(input(t)), "{rate:?} sample {k}");
            }
            // And back to idle `4 S + 3` cycles after the release.
            assert_eq!((trace.len() - release) as u32, release_to_idle(rate));
            // 8 samples per nominal reply bit: the reply bit is 4/5 of 40 cycles.
            assert_eq!(32 / Rate::REPLY_SAMPLE_CYCLES, rate.reply_samples_per_bit());
        }
    }

    #[test]
    fn driver_back_to_back_frames_are_spaced_by_the_minimum_period() {
        for rate in [Rate::Dshot300, Rate::Dshot600] {
            let mut sm = driver(samples(rate));
            for w in FrameImage::STOP.words().into_iter().chain(mixed_image()) {
                sm.tx.push_back(w);
            }
            let mut edges = Vec::new();
            let mut prev = 0;
            for t in 0..6000u64 {
                sm.tick(0xF);
                sm.rx.clear();
                if prev == 0 && sm.dirs != 0 {
                    edges.push(t);
                }
                prev = sm.dirs;
            }
            assert_eq!(edges.len(), 2);
            assert_eq!((edges[1] - edges[0]) as u32, min_frame_period(rate));
        }
        assert_eq!(min_frame_period(Rate::Dshot300), 1735);
        assert_eq!(min_frame_period(Rate::Dshot600), 2119);
    }

    #[test]
    fn driver_waits_for_both_words_and_sends_one_frame_per_push() {
        let mut sm = driver(samples(Rate::Dshot300));
        sm.tx.push_back(FrameImage::STOP.words()[0]);
        for _ in 0..10_000 {
            sm.tick(0xF);
            assert_eq!(sm.dirs, 0, "a frame from one word");
        }
        sm.tx.push_back(FrameImage::STOP.words()[1]);
        let mut frames = 0;
        let mut prev = 0;
        for _ in 0..100_000 {
            sm.tick(0xF);
            sm.rx.clear();
            if prev == 0 && sm.dirs != 0 {
                frames += 1;
            }
            prev = sm.dirs;
        }
        assert_eq!(frames, 1);
    }

    #[test]
    fn driver_capture_stalls_released_and_loses_no_word_when_nobody_reads() {
        // A cancelled exchange stops reading: the capture stalls with the lines released, and
        // whatever is read later is exactly the rest of the words, after which the program
        // reaches idle.
        let rate = Rate::Dshot600;
        let mut sm = driver(samples(rate));
        sm.tx.push_back(FrameImage::STOP.words()[0]);
        sm.tx.push_back(FrameImage::STOP.words()[1]);
        let mut read = Vec::new();
        for t in 0..200_000u64 {
            sm.tick(0xF);
            // The DMA reads the first 10 words, then is cancelled.
            if read.len() < 10
                && let Some(w) = sm.rx.pop_front()
            {
                read.push(w);
            }
            if t > 5000 {
                assert_eq!(sm.dirs, 0);
            }
        }
        assert_eq!(sm.pc, DRIVER_SAMPLE);
        assert_eq!(sm.rx.len(), 4);
        // The next exchange reads the rest: exactly words - 10.
        let mut rest = 0;
        for _ in 0..100_000 {
            sm.tick(0xF);
            if sm.rx.pop_front().is_some() {
                rest += 1;
            }
        }
        assert_eq!(rest + 10, rate.capture_words());
        assert!(sm.pc <= DRIVER_COMMIT, "not in the idle loop");
    }

    fn cancellation_capture_b_is_fresh(rate: Rate, consumed: usize, short_drain: bool) -> bool {
        let words = rate.capture_words() as usize;
        let pending = words - consumed;
        assert!(!short_drain || pending > 0);
        let mut sm = driver(samples(rate));
        let mut frames = 0;
        let tick = |sm: &mut Sm, pins: u8, frames: &mut usize| {
            let before = sm.dirs;
            sm.tick(pins);
            if before == 0 && sm.dirs != 0 {
                *frames += 1;
            }
        };

        // Capture A has a distinct sample on every line: emulate DMA reading exactly the
        // cancellation prefix, including cancellation before any word and after all words.
        sm.tx.extend(FrameImage::STOP.words());
        let mut prefix = Vec::new();
        for _ in 0..10_000 {
            if prefix.len() == consumed {
                break;
            }
            tick(&mut sm, 0x5, &mut frames);
            if let Some(word) = sm.rx.pop_front() {
                prefix.push(word);
            }
        }
        assert_eq!(prefix.len(), consumed);
        assert!(prefix.iter().all(|&word| word == 0x5555_5555));

        // No DMA and no automatic FIFO clearing. Once the frame ends, even a full RX FIFO
        // must leave OE released. With <=4 words pending the capture can already reach idle.
        for elapsed in 0..5_000 {
            tick(&mut sm, 0x5, &mut frames);
            if elapsed > FRAME_CYCLES + STATUS_TO_FIRST_EDGE + 10 {
                assert_eq!(sm.dirs, 0);
            }
        }
        assert_eq!(frames, 1);
        assert_eq!(sm.rx.len(), pending.min(4));
        if pending > 4 {
            assert_eq!(sm.pc, DRIVER_SAMPLE);
        } else {
            assert!(
                sm.pc <= DRIVER_COMMIT,
                "capture A did not reach the idle loop"
            );
        }

        let to_drain = pending - usize::from(short_drain);
        let mut drained = Vec::new();
        for _ in 0..10_000 {
            if drained.len() == to_drain {
                break;
            }
            tick(&mut sm, 0x5, &mut frames);
            assert_eq!(sm.dirs, 0);
            if let Some(word) = sm.rx.pop_front() {
                drained.push(word);
            }
        }
        assert_eq!(drained.len(), to_drain);
        assert!(drained.iter().all(|&word| word == 0x5555_5555));
        // Finish A before changing the external input, so a deliberately omitted drain word
        // is an unambiguous stale A word, not a mixture caused by switching pins mid-capture.
        for _ in 0..5_000 {
            tick(&mut sm, 0x5, &mut frames);
            assert_eq!(sm.dirs, 0);
        }
        assert!(
            sm.pc <= DRIVER_COMMIT,
            "capture did not reach the idle loop"
        );
        assert_eq!(sm.rx.len(), usize::from(short_drain));
        assert_eq!(prefix.len() + drained.len() + sm.rx.len(), words);
        assert_eq!(frames, 1);

        // Push B with the opposite external nibble. Read exactly one capture, just as the
        // next exchange does; a stale A word must be detected rather than silently flushed.
        sm.tx.extend(FrameImage::STOP.words());
        let mut capture_b = Vec::new();
        for _ in 0..10_000 {
            if capture_b.len() == words {
                break;
            }
            tick(&mut sm, 0xA, &mut frames);
            if let Some(word) = sm.rx.pop_front() {
                capture_b.push(word);
            }
        }
        assert_eq!(capture_b.len(), words);
        for _ in 0..5_000 {
            tick(&mut sm, 0xA, &mut frames);
            assert_eq!(sm.dirs, 0);
        }
        assert!(
            sm.pc <= DRIVER_COMMIT,
            "capture did not reach the idle loop"
        );
        assert_eq!(frames, 2);
        assert_eq!(sm.rx.len(), usize::from(short_drain));
        if short_drain {
            assert_eq!(capture_b.first(), Some(&0x5555_5555));
            assert!(capture_b.iter().skip(1).all(|&word| word == 0xAAAA_AAAA));
            assert_eq!(sm.rx.front(), Some(&0xAAAA_AAAA));
        }
        capture_b.iter().all(|&word| word == 0xAAAA_AAAA)
    }

    #[test]
    fn cancellation_drains_every_remaining_a_word_before_fresh_capture_b() {
        for rate in [Rate::Dshot300, Rate::Dshot600] {
            for consumed in 0..=rate.capture_words() as usize {
                assert!(
                    cancellation_capture_b_is_fresh(rate, consumed, false),
                    "{rate:?}, cancellation after {consumed} words"
                );
            }
        }
    }

    #[test]
    fn cancellation_freshness_oracle_rejects_draining_one_word_too_few() {
        for rate in [Rate::Dshot300, Rate::Dshot600] {
            for consumed in 0..rate.capture_words() as usize {
                assert!(
                    !cancellation_capture_b_is_fresh(rate, consumed, true),
                    "{rate:?}, short drain after {consumed} words"
                );
            }
        }
    }

    #[test]
    fn timing_constants_match_the_rate() {
        assert_eq!(FRAME_CYCLES, 640);
        assert_eq!(samples(Rate::Dshot300), 272);
        assert_eq!(samples(Rate::Dshot600), 368);
        assert_eq!(release_to_idle(Rate::Dshot300), 1091);
        assert_eq!(release_to_idle(Rate::Dshot600), 1475);
        // The sampled window covers the measured one: 90.67 us >= 89.5 us, 61.33 >= 60.4.
        for rate in [Rate::Dshot300, Rate::Dshot600] {
            let window_ns = u64::from(samples(rate) * Rate::REPLY_SAMPLE_CYCLES) * 1_000_000_000
                / u64::from(rate.pio_clock_hz());
            assert!(window_ns >= u64::from(rate.capture_window_ns()), "{rate:?}");
        }
        // The frame lasts 53.33 us at DShot300 and 26.67 us at DShot600.
        assert_eq!(
            FRAME_CYCLES * 1_000_000 / (Rate::Dshot300.pio_clock_hz() / 1000),
            53_333
        );
        assert_eq!(
            FRAME_CYCLES * 1_000_000 / (Rate::Dshot600.pio_clock_hz() / 1000),
            26_666
        );
        assert_eq!(LAST_RISE_TO_RELEASE, Rate::Dshot300.phase_cycles()[2]);
    }

    #[test]
    fn samples_is_8_per_word() {
        assert_eq!(samples(Rate::Dshot300), 8 * 34);
    }

    #[test]
    fn min_frame_period_is_647_plus_4_samples() {
        assert_eq!(min_frame_period(Rate::Dshot300), 647 + 4 * 272);
    }

    #[test]
    fn release_to_idle_is_4_samples_plus_3() {
        assert_eq!(release_to_idle(Rate::Dshot600), 4 * 368 + 3);
    }
}
