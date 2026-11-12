# dshot

Bidirectional DShot for AM32 ESCs, `no_std`, integer arithmetic only: the values a caller may send to four motor lines, the frame encoder with the inverted checksum, the two-word frame image for a PIO program that drives GPIO36-39 in step, the timing constants of that program, the decoding of an ESC's reply word into eRPM or extended telemetry, the turning of the pin samples of a reply window into one result per motor line, and the decoding of the serial telemetry frames of an AM32 ESC. The crate does no I/O, owns no pin, no timer and no task. It has no dependencies; `defmt` 1.0 is optional. It is written for the MARV V2 Rev 1 flight controller (RP2350) and the four AM32 ESCs of its 4-in-1 board.

Status: version 0.1.0, `publish = false`, and the API may still change. This is the pure code only, and everything in it is tested on a host only: no code of this crate has run on a board. The frame image is tied to a program that did run on the ESCs (the golden vector below), the sample decoder to the timing that program measured, and nothing more is claimed.

## Quick start

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
assert_eq!(Rate::Dshot300.capture_words(), 68);
```

The samples of a reply window become one result per line. The words are the RX FIFO words of the PIO program in the order they arrive: eight samples a word, the oldest sample in the top nibble, bit `l` of a nibble is line `l`. Here line 2 carries the stopped-motor reply, 16 samples to the symbol, starting at sample 100, and line 3 is low from the first sample, as a bootloader that is transmitting looks:

```rust
use dshot::{decode_capture, CaptureStats, LineCapture, Rate, Reply};

let mut words = [u32::MAX; 68]; // all lines high: 68 words is the DShot300 window
let levels: u32 = 0x52951; // the 21 symbols of a reply for a stopped motor, first symbol in bit 20
for sample in 100..100 + 21 * 16 {
    let symbol = (sample - 100) / 16;
    if (levels >> (20 - symbol)) & 1 == 0 {
        // Clear bit 2 of the nibble of this sample: line 2 is low.
        words[sample / 8] &= !(1u32 << (28 - 4 * (sample % 8) + 2));
    }
}
let samples_per_bit = Rate::Dshot300.reply_samples_per_bit(); // 16
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

## What the application provides

- The PIO back-end: the program, the two FIFO pushes, the capture by DMA. This crate gives it the image and the constants, and takes the words it captured.
- The UART that receives the telemetry wire; this crate gives the framer for its bytes.
- The cadence: when frames are sent, and the pause for the reply.
- The number of pole pairs of the motor, for `Erpm::rpm`.

## Rules for correct use

1. A command frame is also a frame of zero throttle for that motor. While a command is sent the motor is told to stop.
2. An ESC takes a command only when it is armed and its motor is not running. The two telemetry commands need six identical frames in a row (`Command::repeats`); a different value or a Stop in between restarts the count. Extended telemetry is only taken by an ESC that is armed and stopped, over a bidirectional link; the ESC then confirms with the frame 0xE00 (on) or 0xEFF (off).
3. The caller owns the cadence. The crate sends nothing and repeats nothing. When frames stop, the ESC stops the motor by itself: after 0.5 s if it is armed, 2 s if not, and then it resets. Nothing in the caller should defeat that.
4. The link is always bidirectional: inverted checksum, lines idle high, and the ESC replies to every frame it accepts, whether the telemetry-request bit is set or not. The request bit asks for a frame on the ESC's separate serial telemetry wire, which this crate does not decode.
5. The rate is fixed while the ESC runs: an ESC learns it once after power-up, and a frame of another rate is ignored. DShot150 is not accepted by AM32 and is not offered.
6. Push both image words with nothing between them: the PIO program starts a frame only when both are in the FIFO.
7. `decode_reply` accepts the 21 bits with the first symbol (always low) in bit 20; anything else is `BadStart`. A reply that fails is not a value. The check is a 4-bit XOR: a flipped bit of the 20-bit code is always caught, but one edge placed one symbol off (one flipped level) can pass as another valid value across a boundary between two 5-bit groups (384 of the 86 016 single-level errors of valid words, 0.45 %). Check plausibility if that matters.
8. `Erpm::erpm` is 0 for a stopped motor (0xFFF) and for a period of 0. A reply of 0xFFF means the ESC is stopped, not that it is dead; an ESC may also stay silent for a while around arming.

## What the crate does

- `Throttle`, `Command`, `Value`, `LineFrame`: the closed set of what reaches a motor, 0, 1 to 5, 13, 14 and 48 to 2047. There is no `From<u16>`, no constructor from a wire number and no public field that holds one. Not expressible: 6, 7 to 12 (12 writes flash), 15 to 47 (including 20, 21, and 36, which opens an unbounded write into the ESC's settings), and every value AM32 ignores. Adding a `Command` variant is a major release.
- `LineFrame::word`: the 16-bit frame, 11-bit value, request bit, then the inverted checksum `!(v ^ v >> 4 ^ v >> 8) & 0xF` over those 12 bits.
- `FrameImage`: the two TX FIFO words for four lines. One wire bit is one nibble; the first bit is in the lowest nibble of word 0; bit `i` of a nibble is line `i` (GPIO36 + `i`); the nibble bit is the pin level of the data phase, 1 for a 0 bit and 0 for a 1 bit. Four Stop frames give `[0xFFFF_FFFF, 0x0000_FFFF]`, the `STOP_IMAGE` of the measurement program `dshot/hil/src/bin/dshot_m1_probe.rs`. The full layout is in the documentation of `FrameImage`.
- `Rate`: bits per second (300 000, 600 000), 40 PIO cycles per bit in the phases 15 low, 15 data, 10 high (a 0 bit is low for 37.5 %, a 1 bit for 75 %), the PIO clock (bits per second times 40), the divider for a given clk_sys as an integer and an 8-bit fraction (12.5 and 6.25 at 150 MHz) or an error when the rate cannot be had within 0.5 %, 2 PIO cycles per reply sample, 16 samples per nominal reply bit (the reply bit is 4/5 of a command bit, 32 cycles) and the number of capture words.
- Capture window. Measured on 2026-11-12 with the probe on the four ESCs: 89.5 us at DShot300 and 60.4 us at DShot600 after the lines are released (the latest reply end seen in 1000 captures per line, plus 10 us). One RX word is 8 samples of 2 PIO cycles, so the window is 68 words at DShot300 and 91 at DShot600 (rounded up, at most one word more). Nothing else from the bench is repeated here.
- `decode_reply`: undoes the NRZI step and the GCR coding (the table of AM32 `Src/dshot.c:20-23`), checks the checksum, and returns `Reply::Erpm(Erpm)` or `Reply::Extended(ExtendedTelemetry)`, or `ReplyError::{BadStart, BadGcr, BadChecksum}`. A value is eRPM if its top four bits are 0 or odd (the exponent is 0, or the top mantissa bit is set, which a normalised eRPM always has), otherwise extended telemetry, as in the Extended DShot Telemetry specification (<https://github.com/bird-sanctuary/extended-dshot-telemetry>). `Erpm` has the raw 12-bit value, `is_stopped` (0xFFF), `period_us` (mantissa shifted by the 3-bit exponent), `erpm` (`60 000 000 / period`) and `rpm(pole_pairs)`. `ExtendedTelemetry` has temperature in degrees Celsius, voltage in millivolts (0.25 V steps), current in milliamps (1 A steps), the two acknowledges and `Other { kind, value }` for the types AM32 does not send.
- `decode_capture`: the RX words of one reply window (at most 1024) and the samples per nominal reply bit (`Rate::reply_samples_per_bit`, 16 at both rates; it works at 8) to `[LineCapture; 4]`, one per line. `Silent`: the line stayed high. `Reply(Reply)`: a reply `decode_reply` accepted. `Invalid(CaptureError)`: edges that look like a reply and do not decode (`Rejected(ReplyError)`, `Glitch`, `LongRun`, `TooManySymbols`, `CutOff`, `BitTime`). `Foreign`: the line is low at the first sample or shows a low longer than 3.5 nominal bits (a controller in its bootloader: lows of 30 to 52 us); never a value and never `Invalid`. The reply starts at the first falling edge; each run between two edges is rounded to whole symbols against the nominal bit time on its own (nothing accumulates), so a bit time anywhere within +-8 % of nominal (the ESCs were measured 3.5 % fast) and edges within one sample of their true place decode correctly, wherever the reply starts in the window. A further check refuses a reply whose first-to-last-edge time is more than about 9 % from that many nominal bits. Edges for all four lines are found a word at a time with bit operations; there is no division, no float, no allocation, no 64-bit arithmetic and no state between captures. The layout of the words, the exact guarantee and the cost are in the module documentation. `CaptureStats` is a saturating counter of the four kinds that a caller may keep per line.
- `Framer`, `Telemetry`: the serial telemetry frames, 10 bytes high byte first: temperature (signed degrees), voltage (0.01 V), current (0.01 A), consumption (mAh), eRPM / 100, CRC-8 (polynomial 0x07, start 0) of the first nine. The framer takes bytes singly or in slices, split anywhere, slides until the last 10 bytes have a good CRC, reports the frame and starts over with the next byte, counts the alignments it rejected, and never panics. A window of ten zero bytes passes the CRC and is never reported as telemetry; it is counted apart. About 1 alignment in 256 of random bytes passes the CRC, so after junk a wrong frame is possible, and a plausibility check on the values is the caller's. `Telemetry` has `celsius`, `millivolts`, `milliamps`, `consumed_mah`, `erpm` and `rpm(pole_pairs)`, all integers.
- Integer arithmetic only (`deny(clippy::float_arithmetic)` outside tests). The frame image and `decode_reply` use no division and no 64-bit arithmetic; `Erpm::erpm` does one 32-bit division. The divider function uses 64-bit arithmetic once at construction.

## Not covered

- The PIO back-end: the program, FIFO pushes, DMA, pin set-up and pulls, the idle state of the lines between frames.
- Everything on a board. `decode_capture` has not run on a capture from a board: the measurement program's logs hold no raw reply capture (their dumps are of our own frame, and the sampler edge lists in them are empty), so there is no golden vector from a real reply; the stopped-motor case in the tests is built from the measured timing. Its cost in cycles is an estimate from the machine code, not a measurement.
- The telemetry UART, the request bit's effect on the telemetry wire, how the four ESCs share that wire, and the delay from a request to the first byte.
- Arming, cadence, throttle limits, failsafes, and any state: the crate holds none.
- Settings, passthrough and the bootloader.
- Any command beyond the seven, and DShot150 or a non-bidirectional mode.

## Tests

Host only: `cargo test` (unit tests, proptests, the compile-fail tests for the integer conversions, and the examples in this file, which are doctests); `cargo test --features _test-hooks` adds the tests of the test hook. They check the encoded value against the allowed set for every value the API can build, the checksum against a bit-by-bit reference, the image against the probe's `STOP_IMAGE`, a hand-worked image and a reader written from the documented layout, the GCR table against AM32's, the decoding of each of the 4096 reply values and of all 2^20 words (exactly 4096 are accepted), eRPM against an `f64` reference, and the divider against an `f64` reference. The sample decoder is tested with a generator that turns a reply value into samples of four independent lines (a chosen bit time, start and edge displacement), packed as the probe packs them: all 4096 values at 8 % fast, nominal and 8 % slow with every edge one sample off in the worst direction, at 16 and 8 samples per bit; proptests of the +-8 % rule, of Foreign (a 52 us low, a low at the first sample, a low inside a reply) and of a plain sample-by-sample reference decoder against `decode_capture` on arbitrary words; the bit time from 30 % fast to 30 % slow. The telemetry framer is tested with frames built as `kiss_telemetry.c` builds them, a CRC-8 from a bit-by-bit reference and the published check value 0xF4, every split position, junk between frames and a plain search against the framer on arbitrary bytes. No test has run on a board.

Planning notes on how the ESCs time their replies are kept with the project (`_plan/m1-dshot-reply-timing.md`) and are not repeated here.

## Features, layout, licence

Feature `defmt` (off by default): `defmt::Format` for the public types. Feature `_test-hooks` (off by default, outside the API promise): `corrupt_checksum(image, line)` flips the four checksum bits of one line of a built image, for a negative control; it cannot change the value bits.

```text
Cargo.toml   the crate
src/         value.rs (Throttle, Command, Value, LineFrame), frame.rs (FrameImage), rate.rs (Rate),
             reply.rs (decode_reply, Erpm, ExtendedTelemetry), capture.rs (decode_capture,
             LineCapture, CaptureStats), telemetry.rs (Framer, Telemetry)
hil/         on-board measurement program dshot_m1_probe (own workspace)
tools/       run.sh for hil/
```

MIT, see LICENSE.
