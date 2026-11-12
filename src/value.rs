//! The values a caller may put on a motor line, and nothing else.
//!
//! There is no `From<u16>`, no constructor from a raw wire number and no public field that holds
//! one. A throttle is made with [`Throttle::new`]; a command is one of seven variants. Every
//! other DShot value (6, 7 to 12, 15 to 47, and in particular 12, which writes flash, and 36,
//! which opens an unbounded write into the ESC's settings) cannot be expressed.

/// A throttle level from 1 to 2000, sent as the wire values 48 to 2047.
///
/// Level 0 is not a throttle: it is [`Value::Stop`]. The wire number is private; the only way in
/// is [`Throttle::new`].
///
/// ```
/// use dshot::Throttle;
/// assert!(Throttle::new(0).is_none());
/// assert_eq!(Throttle::new(1000).map(Throttle::level), Some(1000));
/// assert!(Throttle::new(2001).is_none());
/// ```
///
/// There is no conversion from an integer:
///
/// ```compile_fail,E0308
/// let _ = dshot::Throttle::from(5u16);
/// ```
///
/// and the field is private:
///
/// ```compile_fail,E0423
/// let _ = dshot::Throttle(5);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Throttle(u16);

impl Throttle {
    /// The lowest level, wire value 48.
    pub const MIN_LEVEL: u16 = 1;
    /// The highest level, wire value 2047.
    pub const MAX_LEVEL: u16 = 2000;
    /// Wire value minus level.
    const OFFSET: u16 = 47;

    /// The throttle for `level` (1 to 2000), or `None` for any other number.
    #[must_use]
    pub const fn new(level: u16) -> Option<Throttle> {
        if matches!(level, Self::MIN_LEVEL..=Self::MAX_LEVEL) {
            Some(Throttle(level + Self::OFFSET))
        } else {
            None
        }
    }

    /// The level this throttle was made from, 1 to 2000.
    #[must_use]
    pub const fn level(self) -> u16 {
        self.0 - Self::OFFSET
    }
}

/// The seven special commands an AM32 ESC is allowed to receive from this crate.
///
/// The set is closed: adding a variant is a major release and needs the owner's approval. The
/// crate cannot send 6, 7 to 12 (12 writes the settings to flash), 20, 21, 36 (an unbounded
/// write into the ESC's settings copy), nor any other value the ESC ignores.
///
/// Two facts about every command:
///
/// - A command frame is also a frame of ZERO throttle for that motor. The motor stops while it
///   is sent.
/// - The ESC acts on a command only when it is armed and its motor is not running. Beeps act at
///   once; the two telemetry commands act on the sixth identical frame in a row (see
///   [`Command::repeats`]); a different value in between restarts the count.
///
/// ```compile_fail,E0277
/// let _: dshot::Command = 1u8.into();
/// ```
///
/// ```compile_fail,E0277
/// let _ = dshot::Command::try_from(13u8);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    /// Beep, tone 1 (wire 1).
    Beep1,
    /// Beep, tone 2 (wire 2).
    Beep2,
    /// Beep, tone 3 (wire 3).
    Beep3,
    /// Beep, tone 4 (wire 4).
    Beep4,
    /// Beep, tone 5 (wire 5).
    Beep5,
    /// Switch extended telemetry on (wire 13). Taken only by an ESC that is armed and stopped,
    /// on the sixth identical frame in a row, over a bidirectional link. The ESC answers with
    /// [`ExtendedTelemetry::Enabled`](crate::ExtendedTelemetry::Enabled).
    ExtendedTelemetryOn,
    /// Switch extended telemetry off (wire 14). Same conditions as
    /// [`Command::ExtendedTelemetryOn`]; the answer is
    /// [`ExtendedTelemetry::Disabled`](crate::ExtendedTelemetry::Disabled).
    ExtendedTelemetryOff,
}

impl Command {
    /// Every command, once.
    pub const ALL: [Command; 7] = [
        Command::Beep1,
        Command::Beep2,
        Command::Beep3,
        Command::Beep4,
        Command::Beep5,
        Command::ExtendedTelemetryOn,
        Command::ExtendedTelemetryOff,
    ];

    /// How many identical frames in a row the ESC needs before it acts: 6 for the two telemetry
    /// commands, 1 for the beeps (AM32 `Src/dshot.c:166`). The caller sends that many.
    #[must_use]
    pub const fn repeats(self) -> u8 {
        match self {
            Command::ExtendedTelemetryOn | Command::ExtendedTelemetryOff => 6,
            _ => 1,
        }
    }

    /// The 11-bit wire value.
    pub(crate) const fn wire(self) -> u16 {
        match self {
            Command::Beep1 => 1,
            Command::Beep2 => 2,
            Command::Beep3 => 3,
            Command::Beep4 => 4,
            Command::Beep5 => 5,
            Command::ExtendedTelemetryOn => 13,
            Command::ExtendedTelemetryOff => 14,
        }
    }
}

/// What one motor line is told to do.
///
/// ```compile_fail,E0277
/// let _ = dshot::Value::from(36u16);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Value {
    /// Zero throttle (wire 0).
    Stop,
    /// A special command. The motor gets zero throttle while it is sent.
    Command(Command),
    /// A throttle level.
    Throttle(Throttle),
}

impl Value {
    /// The 11-bit wire value: 0, 1 to 5, 13, 14 or 48 to 2047.
    pub(crate) const fn wire(self) -> u16 {
        match self {
            Value::Stop => 0,
            Value::Command(c) => c.wire(),
            Value::Throttle(t) => t.0,
        }
    }
}

impl From<Command> for Value {
    fn from(c: Command) -> Self {
        Value::Command(c)
    }
}

impl From<Throttle> for Value {
    fn from(t: Throttle) -> Self {
        Value::Throttle(t)
    }
}

/// What one line gets in one frame: a value and the telemetry-request bit.
///
/// The request bit asks the ESC for one serial telemetry frame on its separate wire. It does not
/// change what the ESC does with the value, and on a bidirectional link the ESC replies to every
/// frame whether the bit is set or not.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LineFrame {
    /// What the motor is told to do.
    pub value: Value,
    /// The telemetry-request bit of the frame.
    pub telemetry_request: bool,
}

impl LineFrame {
    /// Zero throttle, no telemetry request.
    pub const STOP: LineFrame = LineFrame {
        value: Value::Stop,
        telemetry_request: false,
    };

    /// A frame with `value` and the request bit.
    #[must_use]
    pub const fn new(value: Value, telemetry_request: bool) -> Self {
        LineFrame {
            value,
            telemetry_request,
        }
    }

    /// The 16-bit frame word of the bidirectional link: the 11-bit value, the request bit, then
    /// the inverted checksum `!(v ^ v >> 4 ^ v >> 8) & 0xF` over those 12 bits. There is no
    /// non-inverted mode.
    #[must_use]
    pub const fn word(self) -> u16 {
        let v = (self.value.wire() << 1) | self.telemetry_request as u16;
        (v << 4) | checksum(v)
    }
}

/// The inverted DShot checksum of the 12 bits `v` (value and request bit): a nibble.
pub(crate) const fn checksum(v: u16) -> u16 {
    !(v ^ (v >> 4) ^ (v >> 8)) & 0xF
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use std::vec::Vec;

    /// The checksum written out bit by bit, from the DShot definition: XOR of the three nibbles
    /// of the 12 bits, inverted for the bidirectional link.
    fn reference_checksum(v: u16) -> u16 {
        let mut c = 0u16;
        for bit in 0..4 {
            let mut x = 0;
            for nibble in 0..3 {
                x ^= (v >> (4 * nibble + bit)) & 1;
            }
            c |= (x ^ 1) << bit;
        }
        c
    }

    fn throttle(level: u16) -> Throttle {
        Throttle::new(level).unwrap()
    }

    /// Every value a caller can build: Stop, all commands, all 2000 throttle levels.
    fn all_values() -> Vec<Value> {
        let mut all = Vec::new();
        all.push(Value::Stop);
        all.extend(Command::ALL.map(Value::Command));
        all.extend((1..=2000).map(|l| Value::Throttle(throttle(l))));
        all
    }

    #[test]
    fn new_throttle_accepts_1_to_2000_and_nothing_else() {
        for level in 0..=u16::MAX {
            let t = Throttle::new(level);
            assert_eq!(t.is_some(), (1..=2000).contains(&level), "level {level}");
            if let Some(t) = t {
                assert_eq!(t.level(), level);
            }
        }
    }

    #[test]
    fn level_returns_what_new_was_given_and_wire_is_level_plus_47() {
        assert_eq!(throttle(1).level(), 1);
        assert_eq!(throttle(2000).level(), 2000);
        assert_eq!(Value::Throttle(throttle(1)).wire(), 48);
        assert_eq!(Value::Throttle(throttle(2000)).wire(), 2047);
        assert_eq!(Value::Throttle(throttle(999)).wire(), 1046);
    }

    #[test]
    fn repeats_is_6_for_the_telemetry_commands_and_1_for_the_beeps() {
        for c in Command::ALL {
            let want = match c {
                Command::ExtendedTelemetryOn | Command::ExtendedTelemetryOff => 6,
                Command::Beep1
                | Command::Beep2
                | Command::Beep3
                | Command::Beep4
                | Command::Beep5 => 1,
            };
            assert_eq!(c.repeats(), want, "{c:?}");
        }
    }

    #[test]
    fn command_all_lists_each_variant_once_with_its_wire_value() {
        // The match has no wildcard: a new variant stops this test from compiling.
        let slot = |c: Command| match c {
            Command::Beep1 => (0, 1),
            Command::Beep2 => (1, 2),
            Command::Beep3 => (2, 3),
            Command::Beep4 => (3, 4),
            Command::Beep5 => (4, 5),
            Command::ExtendedTelemetryOn => (5, 13),
            Command::ExtendedTelemetryOff => (6, 14),
        };
        for (i, c) in Command::ALL.into_iter().enumerate() {
            assert_eq!(slot(c), (i, c.wire()));
        }
    }

    #[test]
    fn word_stop_is_000f_and_known_vectors_hold() {
        let w = |value, req| LineFrame::new(value, req).word();
        assert_eq!(w(Value::Stop, false), 0x000F);
        assert_eq!(w(Value::Stop, true), 0x001E);
        assert_eq!(LineFrame::STOP.word(), 0x000F);
        // Beep1: value 1, request 0 -> v = 0b000000000010 = 0x002; 0x002 ^ 0x000 ^ 0x000 = 2 in the
        // low nibble; inverted: 0xD. Word 0x002D.
        assert_eq!(w(Value::Command(Command::Beep1), false), 0x002D);
        // Throttle wire 1046 = 0x416 (level 999). Without the request bit v = 0x416 << 1 = 0x82C.
        // Nibbles 0x8, 0x2, 0xC: C ^ 2 ^ 8 = 6; inverted 9. Word 0x82C9.
        assert_eq!(w(Value::Throttle(throttle(999)), false), 0x82C9);
        // With the request bit v = 0x82D. Nibbles 0x8, 0x2, 0xD: D ^ 2 ^ 8 = 7; inverted 8. Word 0x82D8.
        assert_eq!(w(Value::Throttle(throttle(999)), true), 0x82D8);
        // Command 13: v = 26 = 0x01A: nibbles 0, 1, A: A ^ 1 ^ 0 = B; inverted 4. Word 0x01A4.
        assert_eq!(
            w(Value::Command(Command::ExtendedTelemetryOn), false),
            0x01A4
        );
        assert_eq!(
            w(Value::Command(Command::ExtendedTelemetryOff), false),
            0x01C2
        );
        assert_eq!(w(Value::Command(Command::Beep5), false), 0x00A5);
        assert_eq!(w(Value::Throttle(throttle(1)), false), 0x0609);
        assert_eq!(w(Value::Throttle(throttle(2000)), false), 0xFFE1);
    }

    #[test]
    fn word_checksum_matches_the_bit_by_bit_reference_for_all_12_bit_values() {
        for v in 0..0x1000u16 {
            assert_eq!(checksum(v), reference_checksum(v), "{v:#x}");
        }
        for value in all_values() {
            for req in [false, true] {
                let word = LineFrame::new(value, req).word();
                assert_eq!(word & 0xF, reference_checksum(word >> 4));
            }
        }
    }

    #[test]
    fn word_keeps_the_encoded_value_inside_the_allowed_set() {
        // Invariant 3.1: stop, all commands and all 2000 throttle levels, with and without the
        // request bit, give only 0, 1..=5, 13, 14 and 48..=2047; and together they give ALL of
        // them (nothing outside the set is reachable, nothing inside is missing).
        let mut seen = std::collections::BTreeSet::new();
        for value in all_values() {
            for req in [false, true] {
                let word = LineFrame::new(value, req).word();
                let wire = word >> 5;
                assert_eq!(((word >> 4) & 1 == 1), req);
                let allowed = matches!(wire, 0 | 1..=5 | 13 | 14 | 48..=2047);
                assert!(allowed, "{value:?} gave {wire}");
                seen.insert(wire);
            }
        }
        let expected: std::collections::BTreeSet<u16> = [0, 1, 2, 3, 4, 5, 13, 14]
            .into_iter()
            .chain(48..=2047)
            .collect();
        assert_eq!(seen, expected);
    }

    #[test]
    fn line_frame_new_stores_both_fields_and_stop_is_the_zero_frame() {
        let f = LineFrame::new(Value::Command(Command::Beep3), true);
        assert_eq!(f.value, Value::Command(Command::Beep3));
        assert!(f.telemetry_request);
        assert_eq!(LineFrame::STOP, LineFrame::new(Value::Stop, false));
    }

    #[test]
    fn value_from_command_and_throttle_wrap_them() {
        assert_eq!(Value::from(Command::Beep2), Value::Command(Command::Beep2));
        assert_eq!(Value::from(throttle(7)), Value::Throttle(throttle(7)));
    }

    proptest! {
        #[test]
        fn word_checksum_proptest(level in 1u16..=2000, req in any::<bool>()) {
            let word = LineFrame::new(Value::Throttle(throttle(level)), req).word();
            prop_assert_eq!(word >> 5, level + 47);
            prop_assert_eq!(word & 0xF, reference_checksum(word >> 4));
            // The receiver's check: XOR of all four nibbles is 0xF with an inverted checksum.
            let x = (word ^ (word >> 4) ^ (word >> 8) ^ (word >> 12)) & 0xF;
            prop_assert_eq!(x, 0xF);
        }
    }
}
