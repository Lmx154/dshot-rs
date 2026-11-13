//! Bidirectional DShot for AM32 ESCs on four motor lines: the values a caller may send, the
//! frame encoder, the two-word frame image for the PIO program, the timing constants of that
//! program, and the decoding of a reply word.
//!
//! The core is pure integer code: it does no I/O, owns no pin and has no timer. The optional
//! back-end `PioDshot` (feature `embassy-rp-0-10`, RP2350) runs the program on one PIO state
//! machine and captures the replies. Without it, the caller builds a
//! [`FrameImage`] from four [`LineFrame`]s, pushes its two words to the PIO back-end, and hands
//! the words the program captured after the frame to [`decode_capture`], which gives one
//! [`LineCapture`] per line (a reply decoded by [`decode_reply`], silence, a refused one, or a
//! line that is not an ESC's application). [`Framer`] finds the 10-byte serial telemetry frames
//! of an ESC in the bytes of its telemetry wire.
//!
//! What can reach a motor is closed: [`Value::Stop`], the seven [`Command`]s and a [`Throttle`]
//! of 1 to 2000 (wire values 0, 1 to 5, 13, 14 and 48 to 2047). No integer converts to any of
//! them except through [`Throttle::new`].
//!
//! The link is always the bidirectional one (inverted checksum, lines idle high); there is no
//! normal mode. Only DShot300 and DShot600 exist ([`Rate`]).
//!
//! Examples: see README.md, whose examples are compiled by `cargo test`.
#![no_std]
#![cfg_attr(not(feature = "embassy-rp-0-10"), forbid(unsafe_code))]
#![cfg_attr(feature = "embassy-rp-0-10", deny(unsafe_code))]
#![warn(missing_docs)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]
#![cfg_attr(not(test), deny(clippy::float_arithmetic))]

#[cfg(test)]
extern crate std;

mod capture;
mod frame;
mod link;
#[cfg(feature = "embassy-rp-0-10")]
mod pio_dshot;
pub mod program;
mod rate;
mod reply;
mod telemetry;
mod value;

pub use capture::{
    CaptureError, CaptureStats, LINES, LineCapture, MAX_SAMPLES_PER_BIT, MAX_WORDS,
    MIN_SAMPLES_PER_BIT, decode_capture,
};
pub use frame::FrameImage;
#[cfg(feature = "_test-hooks")]
pub use frame::corrupt_checksum;
pub use link::{BREAK_MS, LineMonitor, LineState, PARK_MS};
#[cfg(feature = "embassy-rp-0-10")]
pub use pio_dshot::{BuildError, Exchange, NewError, Parts, PioDshot};
pub use rate::{ClockDivider, DividerError, Rate};
pub use reply::{Erpm, ExtendedTelemetry, Reply, ReplyError, decode_reply};
pub use telemetry::{Framer, Telemetry};
pub use value::{Command, LineFrame, Throttle, Value};

#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeDoctests;
