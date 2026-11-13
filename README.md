# dshot

Bidirectional DShot for AM32 ESCs on four motor lines, `no_std`. Two parts. The core is pure integer code with no dependency and does no I/O; it works on any chip: the values a caller may send, the frame encoder with the inverted checksum, the two-word frame image for a PIO program that drives four consecutive pins in step, the timing constants of that program, the decoding of an ESC's reply word into eRPM or extended telemetry, the turning of the pin samples of a reply window into one result per motor line, and the decoding of the serial telemetry frames of an AM32 ESC. The optional back-end `PioDshot` (feature `embassy-rp-0-10`, embassy-rp 0.10, RP2350) runs that program on one PIO state machine, captures the replies by DMA and carries out the start sequence. The chip feature (`rp235xa` or `rp235xb`) comes from the application. `defmt` 1.x is optional. It is written for the MARV V2 Rev 1 flight controller (RP2350, four AM32 ESCs on GPIO36-39) and takes the clock and the rate as parameters.

Status: version 0.1.0, `publish = false`, and the API may still change. `PioDshot` has been tested on a MARV V2 Rev 1 board with four AM32 2.20 ESCs, at DShot300 and DShot600, with no motors and with zero throttle (Stop frames) only (2026-11-12, see "Measured on the MARV V2 Rev 1 board"). No motor has been spun with it.

## Works with

The core is pure and works on any target. `PioDshot` needs the RP2350 PIO and is written for `embassy-rp` 0.10 and `embedded-hal-async` 1.0 (its `DelayNs` is how `start()` waits), behind the feature `embassy-rp-0-10`; it needs one DMA channel and no embassy executor. It has been run on the RP2350 only, with AM32 2.20 only. It cannot be used on other chips, and no back-end for another HAL exists; the core and the module `program` (the instruction words and the timing of the PIO program) are the place to add one.

## Quick start

The core (compiled by `cargo test`):

```rust
use dshot::{
    decode_reply, Command, ExtendedTelemetry, FrameImage, LineFrame, Rate, Reply, Throttle, Value,
};

// One frame per line: line 0 is GPIO36, line 3 is GPIO39. Here three lines stop and line 1 is
// told to switch extended telemetry on. A command is also zero throttle for that motor.
let lines = [
    LineFrame::STOP,
    LineFrame::new(Value::Command(Command::ExtendedTelemetryOn), false),
    LineFrame::STOP,
    LineFrame::STOP,
];
// The ESC acts on a command on the sixth identical frame in a row.
assert_eq!(Command::ExtendedTelemetryOn.repeats(), 6);
let image = FrameImage::new(lines);
let [word0, word1] = image.words(); // push both to the PIO TX FIFO, no await between them
assert_eq!(FrameImage::STOP.words(), [0xFFFF_FFFF, 0x0000_FFFF]);
assert_ne!([word0, word1], FrameImage::STOP.words());

// The 21 bits of one line's reply, recovered by the caller (first symbol in bit 20).
// A stopped motor replies eRPM 0xFFF:
let Ok(Reply::Erpm(erpm)) = decode_reply(0x52951) else { panic!() };
assert!(erpm.is_stopped());
assert_eq!(erpm.erpm(), 0);
// The acknowledge of command 13 is the extended telemetry frame 0xE00:
assert_eq!(decode_reply(0x5BA2D), Ok(Reply::Extended(ExtendedTelemetry::EnabledAck)));
// A 12-bit eRPM value 0x1F4 with exponent 0 is a period of 500 us per electrical revolution:
// 60 000 000 / 500 = 120 000 eRPM, 17 142 rpm on a 7 pole pair motor.
let Ok(Reply::Erpm(erpm)) = decode_reply(0x92AD9) else { panic!() };
assert_eq!(erpm.period_us(), Some(500));
assert_eq!(erpm.erpm(), 120_000);
assert_eq!(erpm.rpm(7), 17_142);

// Timing for the back-end: PIO clock divider for 150 MHz, and the capture length.
let divider = Rate::Dshot300.clock_divider(150_000_000).unwrap();
assert_eq!((divider.integer, divider.fraction), (12, 128)); // 12.5
assert_eq!(Rate::Dshot300.capture_words(), 34);
```

The samples of a reply window become one result per line. The words are the RX FIFO words of the PIO program in the order they arrive: eight samples a word, the oldest sample in the top nibble, bit `l` of a nibble is line `l`. Here line 2 carries the stopped-motor reply, 8 samples to the symbol, starting at sample 50, and line 3 is low from the first sample, as a bootloader that is transmitting looks:

```rust
use dshot::{decode_capture, CaptureStats, LineCapture, Rate, Reply};

let mut words = [u32::MAX; 34]; // all lines high: 34 words is the DShot300 window
let levels: u32 = 0x52951; // the 21 symbols of a reply for a stopped motor, first symbol in bit 20
for sample in 50..50 + 21 * 8 {
    let symbol = (sample - 50) / 8;
    if (levels >> (20 - symbol)) & 1 == 0 {
        // Clear bit 2 of the nibble of this sample: line 2 is low.
        words[sample / 8] &= !(1u32 << (28 - 4 * (sample % 8) + 2));
    }
}
let samples_per_bit = Rate::Dshot300.reply_samples_per_bit(); // 8
let result = decode_capture(&words, samples_per_bit);
assert_eq!(result[0], LineCapture::Silent);
let LineCapture::Reply(Reply::Erpm(erpm)) = result[2] else { panic!() };
assert!(erpm.is_stopped());

words[0] &= !(1 << (28 + 3)); // the first sample of line 3 is low
let result = decode_capture(&words, samples_per_bit);
assert_eq!(result[3], LineCapture::Foreign);

// Counters a caller may keep, one per line.
let mut stats = [CaptureStats::new(); 4];
for (stat, line) in stats.iter_mut().zip(&result) {
    stat.record(line);
}
assert_eq!((stats[0].silent(), stats[2].replies(), stats[3].foreign()), (1, 1, 1));
```

The serial telemetry of an ESC (10 bytes, no start marker) is found by sliding over the bytes; here two junk bytes come first and the frame is 35 degrees, 16.50 V, 12.34 A, 321 mAh, 100 000 eRPM:

```rust
use dshot::Framer;

let frame = [0x23, 0x06, 0x72, 0x04, 0xD2, 0x01, 0x41, 0x03, 0xE8, 0x3D];
let mut framer = Framer::new();
let mut telemetry = None;
framer.feed(&[0xAA, 0xBB], |_| {});
framer.feed(&frame, |t| telemetry = Some(t)); // slices may be split anywhere
let t = telemetry.unwrap();
assert_eq!((t.celsius(), t.millivolts(), t.milliamps()), (35, 16_500, 12_340));
assert_eq!((t.consumed_mah(), t.erpm(), t.rpm(7)), (321, 100_000, 14_285));
assert_eq!((framer.frames(), framer.rejected_alignments()), (1, 2));
```

A throttle is the only value made from a number, and only 1 to 2000 (wire 48 to 2047) is accepted:

```rust
use dshot::Throttle;
assert!(Throttle::new(0).is_none());
assert!(Throttle::new(2000).is_some());
assert!(Throttle::new(2001).is_none());
```

`PioDshot` (feature `embassy-rp-0-10`; the chip feature, for example `embassy-rp/rp235xb`, comes from the application). Not compiled here, because the README is built on a host. It is the way the board test `hil/src/bin/dshot_smoke.rs` builds and uses it; every frame there is `LineFrame::STOP`:

```rust,ignore
use dshot::{LineFrame, LineState, Parts, PioDshot, Rate};
use embassy_time::{Delay, Duration, Ticker};

bind_interrupts!(struct Irqs {
    PIO2_IRQ_0 => pio::InterruptHandler<PIO2>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>;
});

let Pio { common: mut c2, sm0, .. } = Pio::new(p.PIO2, Irqs);
let pins = [
    c2.make_pio_pin(p.PIN_36),
    c2.make_pio_pin(p.PIN_37),
    c2.make_pio_pin(p.PIN_38),
    c2.make_pio_pin(p.PIN_39),
];
let dma = dma::Channel::new(p.DMA_CH1, Irqs);
let parts = Parts { common: c2, sm0, pins, dma };
let mut drv = PioDshot::new(parts, Rate::Dshot300, clocks::clk_sys_freq()).unwrap();

let mut delay = Delay;
drv.start(&mut delay).await;                 // lines parked 3 s, 25 ms break, released

let mut tick = Ticker::every(Duration::from_millis(1));
loop {
    tick.next().await;
    let x = drv.exchange([LineFrame::STOP; 4]).await;   // one frame, then the four replies
    let all_live = x.states.iter().all(|s| *s == LineState::Live);
    // x.captures: a LineCapture per line; drv.monitor(): the Foreign counters
}

match drv.release() {                        // Ok(Parts), or Err(drv) unchanged
    Ok(parts) => { /* build again, or use the pins */ }
    Err(drv) => { /* a capture is in flight, or the last frame was not all Stop */ }
}
```

## The split: the driver reports, the application decides

`PioDshot` sends a frame when the application calls `exchange()` and reports what each line answered. It has no arming gate, no alarm on bad replies and no hardware cut-off, it does not stop the motors, and it never sends a frame of its own or repeats one. Whether to arm, how to count errors, when to stop and what to do about a silent line are the application's. It should follow established flight-firmware practice: arm when every line has answered (`LineState::Live` on all four), count telemetry errors as a rate, and never stop on single errors.

## What the application provides

- The parts, by value, in `Parts`: the PIO block's `Common`, its `StateMachine<_, 0>`, four `Pin`s made with that `Common::make_pio_pin` (consecutive GPIO numbers in ascending order, line 0 first), and one DMA channel for the capture. `PioDshot::new` returns the parts unchanged inside the error if it cannot take them (`BuildError::PinsNotConsecutive`, `Clock`, `NoSpace`).
- The rate (`Rate`) and `clk_sys` in Hz (`embassy_rp::clocks::clk_sys_freq()`).
- The interrupt bindings: those of the PIO block and of the DMA channel, as for any embassy-rp use.
- A `DelayNs` for `start()`.
- The cadence: when frames are sent. The driver has no timer and no task.
- The decisions: arming, error counting, stopping (see above).
- The number of pole pairs of the motor, for `Erpm::rpm`.
- For the serial telemetry of an ESC: the UART that receives it; the crate gives the framer for its bytes.

## Rules for correct use

1. A command frame is also a frame of zero throttle for that motor. While a command is sent the motor is told to stop.
2. An ESC takes a command only when it is armed and its motor is not running. The two telemetry commands need six identical frames in a row (`Command::repeats`); a different value or a Stop in between restarts the count. Extended telemetry is only taken by an ESC that is armed and stopped, over a bidirectional link; the ESC then confirms with the frame 0xE00 (on) or 0xEFF (off).
3. The caller owns the cadence (see "Cadence"). When frames stop, the ESC stops the motor by itself, after about 510 ms if it is armed and 2 s if not, and then it resets. Nothing in the caller should defeat that.
4. The link is always bidirectional: inverted checksum, lines idle high, and the ESC replies to every frame it accepts, whether the telemetry-request bit is set or not. The request bit asks for a frame on the ESC's separate serial telemetry wire, which this crate does not decode.
5. The rate is fixed while the ESC runs: an ESC learns it once after power-up, and a frame of another rate is ignored. DShot150 is not accepted by AM32 and is not offered. A `PioDshot` has one rate, fixed at build.
6. Push both image words with nothing between them: the PIO program starts a frame only when both are in the FIFO. `exchange()` does this.
7. `decode_reply` accepts the 21 bits with the first symbol (always low) in bit 20; anything else is `BadStart`. A reply that fails is not a value. The check is a 4-bit XOR: a flipped bit of the 20-bit code is always caught, but one edge placed one symbol off (one flipped level) can pass as another valid value across a boundary between two 5-bit groups (384 of the 86 016 single-level errors of valid words, 0.45 %). Check plausibility if that matters.
8. `Erpm::erpm` is 0 for a stopped motor (0xFFF) and for a period of 0. A reply of 0xFFF means the ESC is stopped, not that it is dead; an ESC is also silent for about 300 exchanges around arming (see "What the AM32 2.20 controllers do").

`PioDshot`:

B1. Call `start()` before the first `exchange()`. After power-up or a passthrough session the controllers sit in their bootloaders, and the start sequence brings them into their application. Nothing enforces it. Call it again to restart the stream after frames stopped for more than the ESCs' cut-off; it makes every line `Starting` again.
B2. `start()` finishes any capture a cancelled exchange left, keeps the lines released (inputs, held high by the pad pull-ups) for `PARK_MS` = 3000 ms, drives all four low together for `BREAK_MS` = 25 ms, and releases them. Dropped at any await, it leaves the lines released.
B3. `exchange(frames)` sends one frame, one `LineFrame` per line, and returns an `Exchange`: `captures` (a `LineCapture` per line, decoded at 8 samples per reply bit) and `states` (a `LineState` per line after this capture). One call is one frame. It returns after the reply window: 34 RX words at DShot300, 46 at DShot600 (`Rate::capture_words`).
B4. A line is `Starting` after `start()` and becomes `Live` at its first good reply (`LineCapture::Reply`, extended telemetry included). `Live` stays latched, whatever comes after, until the next `start()`. `Foreign` while `Starting` is normal: a controller whose application is starting holds its line low for about 0.54 s.
B5. `monitor()` gives the `LineMonitor`: `states()`, and `foreign_while_starting()` and `foreign_while_live()`, the `Foreign` captures of each line counted apart by the state the line was in before the capture. The counters saturate and are never cleared (not even by `start()`). They are reports: what a `Foreign` capture on a `Live` line means is the application's decision. The other kinds of capture (`Silent`, `Invalid`) are not counted by the driver; `CaptureStats` is a counter the application may keep for all four.
B6. Dropping the future of `exchange()` is safe. Dropped before the push, nothing was sent. Dropped after it, the frame still goes out whole and its capture is discarded: the next `exchange()` (or `start()`) first reads what is left of it by DMA, then sends its own frame and returns only its own replies. The capture does not drop words and does not end early, so no frame can start while a controller may still be replying.
B7. `release()` hands the parts back (the state machine stopped and restarted, its FIFOs emptied, the program freed, every line an input with its pad pull-up on). It returns `Err(self)`, unchanged, while a frame or capture is in flight (a cancelled exchange can leave one; the next `exchange` or `start` finishes it), and after a frame that was not Stop on all four lines. A later frame that is Stop on all four lines makes it acceptable again.
B8. `Drop` of a `PioDshot` stops its state machine and leaves the four lines released.
B9. Build the pins with `make_pio_pin` after any `make_pio_pin` of another PIO block on the same GPIOs (a pad is driven by the block that called it last). Building sets the block's GPIOBASE window from the pins (GPIO36-39: 16), loads 15 instructions, sets the pad pull-ups and leaves every line an input. At rest the lines are released and high; they are driven only during a frame and the break.
B10. The shortest start-to-start spacing the program allows is `program::min_frame_period`: 144.6 us at DShot300, 88.3 us at DShot600 (the frame and the window, back to back).

## Cadence

Tested at a steady 1 kHz, at DShot300 and DShot600. Keep 1 kHz or more.

Sparse frames make AM32 drop out. Two bench records. In an earlier bench test at DShot300 (since removed from `dshot_smoke`), at one frame per about 0.4 s, two controllers (M4 and M1) lost 330 and 283 replies of about 800 and the other two lost two each; that test never left a gap over 411 ms, so the controller's own cut-off (below) does not explain it, and it is not explained. In `dshot_m11_cadence-20261112-213609.log` (DShot600, nothing sent but one lone frame every 400 ms, 1200 frames in 480 s after `start()` and 3 s at 1 kHz), all four ESCs stopped answering, at 128.6 s, 164.2 s, 213.0 s and 341.4 s after `start()`, and none answered again to the end of the log; every window after that showed lows of the kind a bootloader makes (30 to 52 us). That the ESCs were in their bootloaders is inferred, not shown, and the cause is not established.

A single pause is survived up to the controller's cut-off: with the ESCs armed, gaps of 505 ms and 510 ms were survived (two controllers each) and gaps of 510 ms and 520 ms restarted them, same at both rates; they came back into their application by themselves, with the 0.54 s start-up hold (below). At 550 ms and more they did not come back within 2000 frames. A pause with the lines idling high that is longer than the unarmed cut-off (2 s) leaves a controller in its bootloader (the M2 probe found all four there, parked and silent), where only the break of `start()` brings it back.

With the measurement program `dshot_m1_probe` the shortest frame periods that worked were 250 us at DShot300 and 125 us at DShot600 (1000 of 1000 stopped replies per line); 8 kHz at DShot300 did not fit.

## What the AM32 2.20 controllers do

What an application will see on the four ESCs of the bench board (AM32 2.20, bootloader version 1):

- After a reset (power-up, the cut-off, a passthrough session) the bootloader waits on an idle-high line. It answers junk with serial bytes (lows of 30 to 52 us, which the decoder reports as `Foreign`) and never with a reply. Only the 25 ms break starts the application (the bootloader starts it after 20 ms of low line); `start()` does this, after 3 s parked so that every controller has cut off first.
- After the break each controller holds its line low for about 0.54 s (537 to 541 ms measured) and lets go; it does not listen during that time. About 538 to 541 captures per line are `Foreign`, then replies come: the first good reply after 640 to 646 frames at 1 kHz.
- The armed signal timeout is a fixed count of about 510 ms; not armed it is 2 s. Then the ESC resets.
- Arming. About 0.8 to 1.1 s after its first good reply, each ESC arms and plays a tune with its interrupts off, and does not reply for about 300 exchanges at 1 kHz (299 to 301 measured). Then it sends one late reply (a stopped-motor reply, 0x52951) at an instant that is not tied to our frame, and the next exchange or the next two can also be odd: `Foreign`, `Invalid` or `Silent`, because the capture of the ESC is re-armed with no frame-start trigger after the tune and a reply goes out for every capture it completes, even a rejected one. Then a reply on every exchange. The same follows any other tune the ESC plays with interrupts off. A line that is `Live` can therefore show non-replies at the end of its arming; that is not by itself a fault.
- Overlap. Once per interrupt-off tune a controller's late reply may start under our frame: a push-pull fight of at most one frame (53 us at DShot300) on that line. Our frame stays whole (on the bench: 3 of 3 times it was seen); the controller misses that frame and answers the next; a cut frame cannot be read as a value by the ESC (AM32's frame-time window). The RP2350 datasheet (section 14.9.1) sets no pin current limit. The level during the fight was not measured, and it was not tried with motors fitted.

## What the crate does

- `Throttle`, `Command`, `Value`, `LineFrame`: the closed set of what reaches a motor, 0, 1 to 5, 13, 14 and 48 to 2047. There is no `From<u16>`, no constructor from a wire number and no public field that holds one. Not expressible: 6, 7 to 12 (12 writes flash), 15 to 47 (including 20, 21, and 36, which opens an unbounded write into the ESC's settings), and every value AM32 ignores. Adding a `Command` variant is a major release.
- `LineFrame::word`: the 16-bit frame, 11-bit value, request bit, then the inverted checksum `!(v ^ v >> 4 ^ v >> 8) & 0xF` over those 12 bits.
- `FrameImage`: the two TX FIFO words for four lines. One wire bit is one nibble; the first bit is in the lowest nibble of word 0; bit `i` of a nibble is line `i` (GPIO36 + `i`); the nibble bit is the pin level of the data phase, 1 for a 0 bit and 0 for a 1 bit. Four Stop frames give `[0xFFFF_FFFF, 0x0000_FFFF]`, the `STOP_IMAGE` of the measurement program `hil/src/bin/dshot_m1_probe.rs`. The full layout is in the documentation of `FrameImage`.
- `Rate`: bits per second (300 000, 600 000), 40 PIO cycles per bit in the phases 15 low, 15 data, 10 high (a 0 bit is low for 37.5 %, a 1 bit for 75 %), the PIO clock (bits per second times 40), the divider for a given clk_sys as an integer and an 8-bit fraction (12.5 and 6.25 at 150 MHz) or an error when the rate cannot be had within 0.5 %, 4 PIO cycles per reply sample, 8 samples per nominal reply bit (the reply bit is 4/5 of a command bit, 32 cycles) and the number of capture words.
- Capture window. Measured on 2026-11-12 with the probe on the four ESCs: 89.5 us at DShot300 and 60.4 us at DShot600 after the lines are released (the latest reply end seen in 1000 captures per line, plus 10 us). One RX word is 8 samples of 4 PIO cycles, so the window is 34 words at DShot300 and 46 at DShot600 (rounded up, at most one word more).
- `PARK_MS` (3000), `BREAK_MS` (25), `LineState` (`Starting`, `Live`) and `LineMonitor`: the rules of the start sequence and of the line states as pure code, which `PioDshot` uses.
- `program`: the 15 instruction words of the PIO program (`program::DRIVER`), its addresses, the `exec` words the back-end runs on the state machine, and its timing (`samples`, `release_to_idle`, `min_frame_period`).
- `PioDshot`, `Parts`, `NewError`, `BuildError` (`PinsNotConsecutive`, `Clock(DividerError)`, `NoSpace`; `#[non_exhaustive]`) and `Exchange`: the back-end (feature `embassy-rp-0-10`). `PioDshot::new`, `start`, `exchange`, `release`, `rate`, `monitor`. The driver and its futures are `Send` when the PIO instance is, and the delay's future is (checked at compile time).
- `decode_reply`: undoes the NRZI step and the GCR coding (the table of AM32 `Src/dshot.c:20-23`), checks the checksum, and returns `Reply::Erpm(Erpm)` or `Reply::Extended(ExtendedTelemetry)`, or `ReplyError::{BadStart, BadGcr, BadChecksum}`. A value is eRPM if its top four bits are 0 or odd (the exponent is 0, or the top mantissa bit is set, which a normalised eRPM always has), otherwise extended telemetry, as in the Extended DShot Telemetry specification. `Erpm` has the raw 12-bit value, `is_stopped` (0xFFF), `period_us` (mantissa shifted by the 3-bit exponent), `erpm` (`60 000 000 / period`) and `rpm(pole_pairs)`. `ExtendedTelemetry` has temperature in degrees Celsius, voltage in millivolts (0.25 V steps), current in milliamps (1 A steps), the two acknowledges and `Other { kind, value }` for the types AM32 does not send.
- `decode_capture`: the RX words of one reply window (at most 1024) and the samples per nominal reply bit (`Rate::reply_samples_per_bit`, 8 at both rates; the decoder takes 4 to 64) to `[LineCapture; 4]`, one per line. `Silent`: the line stayed high. `Reply(Reply)`: a reply `decode_reply` accepted. `Invalid(CaptureError)`: edges that look like a reply and do not decode (`Rejected(ReplyError)`, `Glitch`, `LongRun`, `TooManySymbols`, `CutOff`, `BitTime`). `Foreign`: the line is low at the first sample or shows a low longer than 3.5 nominal bits (a controller in its bootloader: lows of 30 to 52 us; or an application's late reply after a tune); never a value and never `Invalid`. The reply starts at the first falling edge; each run between two edges is rounded to whole symbols against the nominal bit time on its own (nothing accumulates), so a bit time anywhere within +-8 % of nominal (the ESCs were measured 3.5 % fast) and edges within one sample of their true place (at 8 samples per bit) decode correctly, wherever the reply starts in the window. A further check refuses a reply whose first-to-last-edge time is more than about 9 % from that many nominal bits. Edges for all four lines are found a word at a time with bit operations; there is no division, no float, no allocation, no 64-bit arithmetic and no state between captures. The layout of the words, the exact guarantee and the cost are in the module documentation. `CaptureStats` is a saturating counter of the four kinds that a caller may keep per line.
- `Framer`, `Telemetry`: the serial telemetry frames, 10 bytes high byte first: temperature (signed degrees), voltage (0.01 V), current (0.01 A), consumption (mAh), eRPM / 100, CRC-8 (polynomial 0x07, start 0) of the first nine. The framer takes bytes singly or in slices, split anywhere, slides until the last 10 bytes have a good CRC, reports the frame and starts over with the next byte, counts the alignments it rejected, and never panics. A window of ten zero bytes passes the CRC and is never reported as telemetry; it is counted apart. About 1 alignment in 256 of random bytes passes the CRC, so after junk a wrong frame is possible, and a plausibility check on the values is the caller's. `Telemetry` has `celsius`, `millivolts`, `milliamps`, `consumed_mah`, `erpm` and `rpm(pole_pairs)`, all integers.
- Integer arithmetic only (`deny(clippy::float_arithmetic)` outside tests). The frame image and `decode_reply` use no division and no 64-bit arithmetic; `Erpm::erpm` does one 32-bit division. The divider function uses 64-bit arithmetic once at construction.

## Not covered

- No motor has been spun with this crate: no throttle above zero was ever sent on the bench and no RPM value other than "stopped" was received. Behaviour with a motor connected or turning is untested.
- The serial telemetry of the ESC: the telemetry UART, the request bit's effect on the telemetry wire, how the four ESCs share that wire, and the delay from a request to the first byte. `Framer` is tested on host only.
- Extended DShot telemetry and the beep and settings commands (`Command`) are untested on hardware.
- Only AM32 2.20 (bootloader version 1, on the GOKU G55M 4-in-1 of the MARV V2 Rev 1 bench) and only the RP2350 with embassy-rp 0.10 were tested. `PioDshot` cannot be used on other chips; the core can.
- Frame rates other than 1 kHz in the board test of the back-end, and sparse frames beyond what the records in "Cadence" show.
- The level fight during an overlap of a late reply and our frame (needs an analogue measurement).
- The cost of `decode_capture` in CPU cycles on the board: the budget in its module documentation is not a measurement.
- Arming, throttle limits, failsafes: the driver holds none. Settings, passthrough and the bootloader are not part of this crate. Any command beyond the seven, DShot150 and a non-bidirectional mode are not offered.

## Tests

Host: `cargo test` (141 unit tests, the proptests, the compile-fail doctests for the integer conversions, and the examples in this file, which are doctests); `cargo test --features _test-hooks` adds the tests of the test hook (144 unit tests). They check the encoded value against the allowed set for every value the API can build, the checksum against a bit-by-bit reference, the image against the probe's `STOP_IMAGE`, a hand-worked image and a reader written from the documented layout, the GCR table against AM32's, the decoding of each of the 4096 reply values and of all 2^20 words (exactly 4096 are accepted), eRPM against an `f64` reference, and the divider against an `f64` reference. The sample decoder is tested with a generator that turns a reply value into samples of four independent lines (a chosen bit time, start and edge displacement), packed as the program packs them: all 4096 values at 8 % fast, nominal and 8 % slow with every edge one sample off in the worst direction, at 16 and 8 samples per bit; proptests of the +-8 % rule, of Foreign (a 52 us low, a low at the first sample, a low inside a reply) and of a plain sample-by-sample reference decoder against `decode_capture` on arbitrary words; the bit time from 30 % fast to 30 % slow. The telemetry framer is tested with frames built as `kiss_telemetry.c` builds them, a CRC-8 from a bit-by-bit reference and the published check value 0xF4, every split position, junk between frames and a plain search against the framer on arbitrary bytes. The start sequence's constants, the `Starting`/`Live` latch with the Foreign counters, and the release rule are tested as pure code. The PIO program is checked on the host by a decoder of the instruction words written from the datasheet's encoding table and by a cycle model of the state machine that executes the same words (`program::DRIVER`) the driver loads: frame shape, one frame per push, the capture that stalls released and loses no word, the spacing of back-to-back frames, and a cancelled capture drained before the next.

On the board: `tools/run.sh dshot_smoke <timeout_seconds>` (it takes the machine-wide probe lock and keeps its log under `hil/logs/`, which git ignores). It builds `PioDshot` on PIO2 and GPIO36-39 and watches the lines with four run-length observers on PIO0 (13.3 ns resolution, DMA rings), independent of the driver, at DShot300 and DShot600. Every frame it sends is `LineFrame::STOP`; it builds no `Throttle` and no `Command`. It prints `[PASS]`/`[FAIL]`/`[STAT]`/`[INFO]` lines and `RESULT: PASS|FAIL`.

## Measured on the MARV V2 Rev 1 board

2026-11-12, `clk_sys` 150 MHz, RP2350, four AM32 2.20 ESCs on GPIO36-39, battery connected, NO motors, zero throttle (Stop frames) only. Six full runs of `hil/src/bin/dshot_smoke.rs`, each `RESULT: PASS` with no failed check, at DShot300 and DShot600. The logs are bench records, kept locally (`hil/logs/` is not in git): `dshot_smoke-20261112-224418.log`, `-224650`, `-224809`, `-224940` (the last three consecutive, with no source change between them), `-230019` (the reviewer's run) and `-230510` (the source as committed: the same program after `cargo fmt` and with one more assertion, that no short pulse or off-grid edge of the driver was recorded anywhere in the run; it counted 0). The figures below are from the first five.

- Per run and rate, 3000 exchanges per line at 1 kHz: 2705 to 2709 good replies; 290 to 293 without a reply, in one block (the arming tune); 1 or 2 odd exchanges per line at the end of that block; none anywhere else. The driver's decode agreed with an independent edge capture (13.3 ns resolution) on every clean exchange. No edge of the driver's frames was more than 53 ns off its grid, and the driver produced no pulse under 300 ns.
- Cancellation, per run and rate: 1078 single cancels (an exchange dropped at each 1 us step from before its push to after its capture window) and, during the drain of a cancelled exchange, 388 (DShot300) and 552 (DShot600) cancels, each followed by a `release()` attempt: every failure counter 0, frames on the line equal to frames pushed.
- Release and rebuild: 100 build/release cycles with no edge on any line; the next `start()` brought all four ESCs back to 100 consecutive good replies.
- Host tests: 141 unit, 5 doctests that run and 6 compile-fail doctests (144 unit with `_test-hooks`).
- Other bench records used in this README: `dshot_m1_probe-20261112-134808.log` (reply timing, capture window, shortest frame period), `dshot_m8_startup-20261112-154050.log` and `-154156` (the 0.54 s hold), `dshot_m10_arming-20261112-175413.log` (the late reply at the end of the arming silence), `dshot_m9_gaps-20261112-174203.log` (the cut-off gaps), `dshot_m11_cadence-20261112-213609.log` (the lone frame every 0.4 s).

## Features, layout, licence

Features, all off by default and additive: `defmt` (`defmt::Format` for the public types), `embassy-rp-0-10` (`PioDshot`, with `embassy-rp` 0.10 and `embedded-hal-async` 1.0; the chip feature comes from the application), `_test-hooks` (outside the API promise: `corrupt_checksum(image, line)` flips the four checksum bits of one line of a built image, for a negative control; it cannot change the value bits).

```text
Cargo.toml   the crate
src/         value.rs (Throttle, Command, Value, LineFrame), frame.rs (FrameImage), rate.rs (Rate),
             reply.rs (decode_reply, Erpm, ExtendedTelemetry), capture.rs (decode_capture,
             LineCapture, CaptureStats), telemetry.rs (Framer, Telemetry), program.rs (the PIO
             program), link.rs (PARK_MS, BREAK_MS, LineState, LineMonitor), pio_dshot.rs (PioDshot)
hil/         on-board programs, among them dshot_smoke and dshot_m1_probe (own workspace; src/bin/,
             logs/ is git-ignored)
tools/       run.sh: builds a hil/ program, flashes and runs it under the machine-wide probe lock
LICENSE      MIT
```
