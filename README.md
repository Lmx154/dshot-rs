# dshot

Bidirectional DShot for AM32 ESCs, `no_std`, integer arithmetic only: the values a caller may send to four motor lines, the frame encoder with the inverted checksum, the two-word frame image for a PIO program that drives GPIO36-39 in step, the timing constants of that program, and the decoding of an ESC's reply word into eRPM or extended telemetry. The crate does no I/O, owns no pin, no timer and no task. It has no dependencies; `defmt` 1.0 is optional. It is written for the MARV V2 Rev 1 flight controller (RP2350) and the four AM32 ESCs of its 4-in-1 board.

Status: version 0.1.0, `publish = false`, and the API may still change. This is the pure core only. It has been tested on a host only; no code of this crate has run on a board. The frame image is tied to a program that did run on the ESCs (the golden vector below), and nothing more is claimed.

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

A throttle is the only value made from a number, and only 1 to 2000 (wire 48 to 2047) is accepted:

```rust
use dshot::Throttle;
assert!(Throttle::new(0).is_none());
assert!(Throttle::new(2000).is_some());
assert!(Throttle::new(2001).is_none());
```

## What the application provides

- The PIO back-end: the program, the two FIFO pushes, the capture by DMA. This crate gives it the image and the constants.
- The 21 bits of each reply, recovered from the pin samples.
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
- Integer arithmetic only (`deny(clippy::float_arithmetic)` outside tests). The frame image and `decode_reply` use no division and no 64-bit arithmetic; `Erpm::erpm` does one 32-bit division. The divider function uses 64-bit arithmetic once at construction.

## Not covered

- The PIO back-end: the program, FIFO pushes, DMA, pin set-up and pulls, the idle state of the lines between frames.
- The sample-level reply decoder: turning pin samples into the 21 bits, edge timing and the reply's 3.5 % faster bit time.
- Serial telemetry (the 10-byte frames on the ESC's telemetry wire) and the request bit's effect on it.
- Arming, cadence, throttle limits, failsafes, and any state: the crate holds none.
- Settings, passthrough and the bootloader.
- Any command beyond the seven, and DShot150 or a non-bidirectional mode.

## Tests

Host only: `cargo test` (unit tests, proptests, the compile-fail tests for the integer conversions, and the examples in this file, which are doctests); `cargo test --features _test-hooks` adds the tests of the test hook. They check the encoded value against the allowed set for every value the API can build, the checksum against a bit-by-bit reference, the image against the probe's `STOP_IMAGE`, a hand-worked image and a reader written from the documented layout, the GCR table against AM32's, the decoding of each of the 4096 reply values and of all 2^20 words (exactly 4096 are accepted), eRPM against an `f64` reference, and the divider against an `f64` reference. No test has run on a board.

Planning notes on how the ESCs time their replies are kept with the project (`_plan/m1-dshot-reply-timing.md`) and are not repeated here.

## Features, layout, licence

Feature `defmt` (off by default): `defmt::Format` for the public types. Feature `_test-hooks` (off by default, outside the API promise): `corrupt_checksum(image, line)` flips the four checksum bits of one line of a built image, for a negative control; it cannot change the value bits.

```text
Cargo.toml   the crate
src/         value.rs (Throttle, Command, Value, LineFrame), frame.rs (FrameImage), rate.rs (Rate),
             reply.rs (decode_reply, Erpm, ExtendedTelemetry)
hil/         on-board measurement program dshot_m1_probe (own workspace)
tools/       run.sh for hil/
```

MIT, see LICENSE.
