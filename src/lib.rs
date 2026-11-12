//! Bidirectional DShot for AM32 ESCs on four motor lines: the values a caller may send, the
//! frame encoder, the two-word frame image for the PIO program, the timing constants of that
//! program, and the decoding of a reply word.
//!
//! Pure integer code: the crate does no I/O, owns no pin and has no timer. The caller builds a
//! [`FrameImage`] from four [`LineFrame`]s, pushes its two words to the PIO back-end, and hands
//! the 21 bits it recovers from each line's capture to [`decode_reply`].
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
#![forbid(unsafe_code)]
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

mod frame;
mod rate;
mod reply;
mod value;

pub use frame::FrameImage;
#[cfg(feature = "_test-hooks")]
pub use frame::corrupt_checksum;
pub use rate::{ClockDivider, DividerError, Rate};
pub use reply::{Erpm, ExtendedTelemetry, Reply, ReplyError, decode_reply};
pub use value::{Command, LineFrame, Throttle, Value};

#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeDoctests;
