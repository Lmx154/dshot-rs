//! Decoding of one line's reply WORD: the 21 bits an ESC sends after a frame, already recovered
//! from the pin samples, into an eRPM value or an extended telemetry value.
//!
//! # The reply
//!
//! Idle high; a falling edge starts the reply. It has 21 symbols: a leading symbol that is
//! LOW, then 20 symbols that are the line level after each step of an NRZI code (a 1 toggles the
//! line, a 0 keeps it). The 20 bits are four 5-bit GCR codes, most significant first, for the
//! 16 bits `value (12) | checksum (4)`; the table is `gcr_encode_table` in AM32 `Src/dshot.c:20-23`.
//! The checksum is `!(v ^ v >> 4 ^ v >> 8) & 0xF` over the 12 value bits, the same inverted
//! checksum as the command frame.
//!
//! # Telling eRPM from extended telemetry
//!
//! The 12-bit value of an eRPM frame is `eee mmmmmmmmm`: a 3-bit exponent and a 9-bit mantissa,
//! normalised so that bit 8 (the top mantissa bit) is set whenever the exponent is not 0. The
//! codes that normalisation leaves free carry extended telemetry. With `t = value >> 8`, the top
//! four bits: the frame is eRPM if `t == 0` or `t` is odd, and extended telemetry otherwise
//! (`t` = 2, 4, 6, 8, 10, 12, 14). Source: the Extended DShot Telemetry specification,
//! <https://github.com/bird-sanctuary/extended-dshot-telemetry> ("If the last four bits are 0 OR
//! the 8th bit is 1, it is a eRPM frame, the other ranges are EDT frames"; the types are
//! 0x2 temperature in degC, 0x4 voltage in 0.25 V, 0x6 current in 1 A, 0x8 and 0xA debug, 0xC
//! stress, 0xE status), and what AM32 sends: `Src/dshot.c:242-346`. AM32 sends only 0x2, 0x4
//! and 0x6, and for the acknowledge of command 13 and 14 the status frames 0xE00 and 0xEFF.

use crate::value::checksum;

/// Value the GCR inverse table holds for a 5-bit pattern that is not a code.
const NOT_A_CODE: u8 = 0xFF;

/// The 5-bit code of each nibble 0 to F: AM32 `Src/dshot.c:20-23`.
const GCR: [u8; 16] = [
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111, 0b11010, 0b01001,
    0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
];

/// The nibble of each 5-bit code, [`NOT_A_CODE`] for the 16 patterns that are not codes.
const GCR_INV: [u8; 32] = {
    let mut table = [NOT_A_CODE; 32];
    let mut nibble = 0;
    while nibble < 16 {
        // bound: nibble < 16 = GCR.len(), and a code is masked to 0..32 = table.len().
        #[allow(clippy::indexing_slicing)]
        {
            table[GCR[nibble] as usize & 31] = nibble as u8;
        }
        nibble += 1;
    }
    table
};

/// Why a reply word was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ReplyError {
    /// The word is not 21 bits with a low leading symbol: a bit above bit 20 is set, or bit 20
    /// (the leading symbol, which an ESC always drives low) is set.
    BadStart,
    /// One of the four 5-bit groups is not a GCR code.
    BadGcr,
    /// The checksum nibble does not match the 12 value bits.
    BadChecksum,
}

/// The 12-bit value of an eRPM reply: `eee mmmmmmmmm`, see [`Erpm::period_us`].
///
/// Only [`decode_reply`] makes one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Erpm(u16);

impl Erpm {
    /// The raw 12-bit value as the ESC sent it.
    #[must_use]
    pub const fn raw(self) -> u16 {
        self.0
    }

    /// True for 0xFFF, which an ESC sends when its motor is stopped.
    #[must_use]
    pub const fn is_stopped(self) -> bool {
        self.0 == 0xFFF
    }

    /// The period of one electrical revolution in microseconds: `mantissa << exponent`, 1 to
    /// 65 408. `None` when the motor is stopped (0xFFF) or the period is 0 (a value of 0, which
    /// no speed corresponds to).
    #[must_use]
    pub const fn period_us(self) -> Option<u32> {
        if self.is_stopped() {
            return None;
        }
        let period = ((self.0 & 0x1FF) as u32) << (self.0 >> 9);
        if period == 0 { None } else { Some(period) }
    }

    /// Electrical revolutions per minute, `60 000 000 / period`: 0 when [`Erpm::period_us`] is
    /// `None`. One 32-bit division.
    #[must_use]
    pub const fn erpm(self) -> u32 {
        match self.period_us() {
            Some(period) => 60_000_000 / period,
            None => 0,
        }
    }

    /// Mechanical revolutions per minute for a motor with `pole_pairs` pole pairs (the number of
    /// magnets over two): [`Erpm::erpm`] over `pole_pairs`, rounded down; 0 for 0 pole pairs.
    #[must_use]
    pub const fn rpm(self, pole_pairs: u8) -> u32 {
        if pole_pairs == 0 {
            return 0;
        }
        self.erpm() / pole_pairs as u32
    }
}

/// An extended telemetry value, sent in place of an eRPM value on some replies after command 13.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ExtendedTelemetry {
    /// ESC temperature, 0 to 255 degrees Celsius (type 0x2).
    Temperature {
        /// Degrees Celsius.
        celsius: u8,
    },
    /// Supply voltage, in 0.25 V steps on the wire (type 0x4).
    Voltage {
        /// Millivolts, a multiple of 250 up to 63 750.
        millivolts: u32,
    },
    /// Current, in 1 A steps on the wire (type 0x6).
    Current {
        /// Milliamps, a multiple of 1000 up to 255 000.
        milliamps: u32,
    },
    /// The acknowledge of command 13, the status frame 0xE00.
    EnabledAck,
    /// The acknowledge of command 14, the status frame 0xEFF.
    DisabledAck,
    /// A type AM32 does not send: 0x8, 0xA, 0xC, or 0xE with a payload other than 0x00 and 0xFF.
    Other {
        /// The type: the top four bits of the 12-bit value, an even number from 2 to 14.
        kind: u8,
        /// The low eight bits of the value.
        value: u8,
    },
}

/// What a reply word holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Reply {
    /// The speed of the motor.
    Erpm(Erpm),
    /// An extended telemetry value.
    Extended(ExtendedTelemetry),
}

/// Decodes the 21 bits of one line's reply.
///
/// `levels` holds the 21 symbols as line levels, 1 = high, the first symbol in bit 20 and the
/// last in bit 0. The first symbol is the low one the ESC drives when it starts, so bit 20 must
/// be 0; no bit above it may be set. The recovery of these bits from pin samples is not done
/// here. A reply for a stopped motor is `0x52951` (an eRPM value of 0xFFF).
///
/// # Errors
///
/// [`ReplyError::BadStart`] for a word that is not 21 bits with a low first symbol,
/// [`ReplyError::BadGcr`] when a 5-bit group is no code of the table, and
/// [`ReplyError::BadChecksum`] when the 4-bit checksum does not fit the value.
///
/// A flipped bit of the 20-bit code is always caught. A flipped LEVEL (an edge placed one symbol
/// off) is caught except across the three boundaries between 5-bit groups, where it changes two
/// neighbouring nibbles by the same XOR and the XOR checksum cannot see it: 384 of the 86 016
/// single-level flips of valid words (0.45 %) decode to another valid value. That is a property of
/// the DShot checksum; a caller that cannot accept it adds a plausibility check on the value.
pub fn decode_reply(levels: u32) -> Result<Reply, ReplyError> {
    if levels >> 20 != 0 {
        return Err(ReplyError::BadStart);
    }
    // Undo the NRZI step: code bit k is level k+1 xor level k, with the leading low symbol.
    let code = (levels ^ (levels >> 1)) & 0xF_FFFF;
    let mut word: u16 = 0;
    let mut group = 0;
    while group < 4 {
        let five = (code >> (15 - 5 * group)) & 0x1F;
        let nibble = match GCR_INV.get(five as usize) {
            Some(&n) if n != NOT_A_CODE => n,
            _ => return Err(ReplyError::BadGcr),
        };
        word = (word << 4) | u16::from(nibble);
        group += 1;
    }
    let value = word >> 4;
    if word & 0xF != checksum(value) {
        return Err(ReplyError::BadChecksum);
    }
    Ok(classify(value))
}

/// Tells eRPM from extended telemetry in a 12-bit value, by the rule in the module documentation.
const fn classify(value: u16) -> Reply {
    let kind = value >> 8;
    if kind == 0 || kind & 1 == 1 {
        return Reply::Erpm(Erpm(value));
    }
    let payload = (value & 0xFF) as u8;
    Reply::Extended(match kind {
        0x2 => ExtendedTelemetry::Temperature { celsius: payload },
        0x4 => ExtendedTelemetry::Voltage {
            millivolts: payload as u32 * 250,
        },
        0x6 => ExtendedTelemetry::Current {
            milliamps: payload as u32 * 1000,
        },
        0xE if payload == 0x00 => ExtendedTelemetry::EnabledAck,
        0xE if payload == 0xFF => ExtendedTelemetry::DisabledAck,
        _ => ExtendedTelemetry::Other {
            kind: kind as u8,
            value: payload,
        },
    })
}

/// The 21 reply bits for a 12-bit `value` (test helper, the inverse of [`decode_reply`]).
#[cfg(test)]
pub(crate) fn encode_reply(value: u16) -> u32 {
    encode_word((value << 4) | checksum(value))
}

/// The 21 reply bits for a 16-bit word, whatever its checksum nibble (test helper).
#[cfg(test)]
pub(crate) fn encode_word(word: u16) -> u32 {
    let mut code = 0u32;
    for shift in [12, 8, 4, 0] {
        code = (code << 5) | u32::from(GCR[usize::from((word >> shift) & 0xF)]);
    }
    code_to_levels(code)
}

/// The 21 levels for a 20-bit code, with the leading low symbol (test helper).
#[cfg(test)]
pub(crate) fn code_to_levels(code: u32) -> u32 {
    let mut level = 0u32;
    let mut levels = 0u32;
    for k in (0..20).rev() {
        level ^= (code >> k) & 1;
        levels = (levels << 1) | level;
    }
    // The leading symbol is 0 and sits in bit 20: the loop produced the 20 following ones.
    levels
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use std::vec::Vec;

    /// The table as written in AM32 `Src/dshot.c:20-23`, copied literally.
    const AM32_TABLE: [u8; 16] = [
        0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111, 0b11010, 0b01001,
        0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
    ];

    /// The 12-bit value a decoded reply came from.
    fn value_of(reply: Reply) -> u16 {
        match reply {
            Reply::Erpm(e) => e.raw(),
            Reply::Extended(x) => match x {
                ExtendedTelemetry::Temperature { celsius } => 0x200 | u16::from(celsius),
                ExtendedTelemetry::Voltage { millivolts } => 0x400 | (millivolts / 250) as u16,
                ExtendedTelemetry::Current { milliamps } => 0x600 | (milliamps / 1000) as u16,
                ExtendedTelemetry::EnabledAck => 0xE00,
                ExtendedTelemetry::DisabledAck => 0xEFF,
                ExtendedTelemetry::Other { kind, value } => {
                    (u16::from(kind) << 8) | u16::from(value)
                }
            },
        }
    }

    /// eRPM or extended by the exponent / mantissa form of the specification: an eRPM value is
    /// normalised, so with a non-zero exponent its mantissa top bit (bit 8) is set.
    fn is_erpm_by_spec(v: u16) -> bool {
        let exponent = v >> 9;
        let mantissa_top = (v >> 8) & 1;
        exponent == 0 || mantissa_top == 1
    }

    #[test]
    fn gcr_table_equals_am32s_and_has_16_distinct_codes() {
        assert_eq!(GCR, AM32_TABLE);
        let mut codes = AM32_TABLE.to_vec();
        codes.sort_unstable();
        codes.dedup();
        assert_eq!(codes.len(), 16);
        // The probe's hexadecimal list of facts-am32-link.md section 5.
        let hex = [
            0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D,
            0x0E, 0x0F,
        ];
        assert_eq!(AM32_TABLE, hex);
    }

    #[test]
    fn decode_reply_all_16_gcr_codes_round_trip_in_every_group() {
        // Each nibble in each of the three value groups, with the checksum made to fit by the
        // other nibbles.
        for nibble in 0..16u16 {
            for group in 0..3 {
                let value = nibble << (4 * group);
                let r = decode_reply(encode_reply(value)).unwrap();
                assert_eq!(value_of(r), value);
            }
        }
        // And in the checksum group: value 0 has checksum 0xF, so find values for every checksum.
        let mut seen = [false; 16];
        for value in 0..0x1000u16 {
            seen[usize::from(checksum(value))] = true;
        }
        assert!(seen.iter().all(|&s| s));
    }

    #[test]
    fn decode_reply_every_value_round_trips() {
        for value in 0..0x1000u16 {
            let r = decode_reply(encode_reply(value)).unwrap();
            assert_eq!(value_of(r), value);
        }
    }

    #[test]
    fn decode_reply_accepts_exactly_4096_of_the_2_pow_20_words() {
        let mut ok = 0u32;
        for levels in 0..(1u32 << 20) {
            if let Ok(r) = decode_reply(levels) {
                ok += 1;
                assert_eq!(encode_reply(value_of(r)), levels);
            }
        }
        assert_eq!(ok, 4096);
    }

    #[test]
    fn decode_reply_a_bit_outside_21_bits_or_a_high_leading_symbol_is_bad_start() {
        let good = encode_reply(0xFFF);
        assert_eq!(decode_reply(good | 1 << 20), Err(ReplyError::BadStart));
        assert_eq!(decode_reply(good | 1 << 21), Err(ReplyError::BadStart));
        assert_eq!(decode_reply(good | 1 << 31), Err(ReplyError::BadStart));
        assert_eq!(decode_reply(u32::MAX), Err(ReplyError::BadStart));
        assert!(decode_reply(good).is_ok());
    }

    #[test]
    fn decode_reply_every_5_bit_value_outside_the_table_is_bad_gcr_in_every_group() {
        let valid = encode_code_of(0x1234);
        let mut tried = 0;
        for five in 0..32u32 {
            if AM32_TABLE.contains(&(five as u8)) {
                continue;
            }
            for group in 0..4 {
                let shift = 15 - 5 * group;
                let code = (valid & !(0x1F << shift)) | (five << shift);
                assert_eq!(
                    decode_reply(code_to_levels(code)),
                    Err(ReplyError::BadGcr),
                    "five {five:#07b} group {group}"
                );
                tried += 1;
            }
        }
        assert_eq!(tried, 16 * 4);
    }

    /// The 20-bit code of a 12-bit value.
    fn encode_code_of(value: u16) -> u32 {
        let word = (value << 4) | checksum(value);
        let mut code = 0u32;
        for shift in [12, 8, 4, 0] {
            code = (code << 5) | u32::from(AM32_TABLE[usize::from((word >> shift) & 0xF)]);
        }
        code
    }

    #[test]
    fn decode_reply_a_wrong_checksum_is_bad_checksum() {
        for value in [0x000u16, 0xFFF, 0x22D, 0x1B3] {
            for wrong in 0..16u16 {
                let word = (value << 4) | wrong;
                let levels = encode_word(word);
                let want = if wrong == checksum(value) {
                    Ok(())
                } else {
                    Err(ReplyError::BadChecksum)
                };
                assert_eq!(
                    decode_reply(levels).map(|_| ()),
                    want,
                    "{value:#x} {wrong:#x}"
                );
            }
        }
    }

    #[test]
    fn decode_reply_a_flipped_code_bit_is_always_an_error() {
        // One bit of the 20-bit code changes one nibble, which the XOR checksum always sees.
        for value in 0..0x1000u16 {
            let code = encode_code_of(value);
            for bit in 0..20 {
                let r = decode_reply(code_to_levels(code ^ (1 << bit)));
                assert!(r.is_err(), "value {value:#x} code bit {bit}: {r:?}");
            }
        }
    }

    #[test]
    fn decode_reply_a_flipped_level_is_an_error_except_across_a_group_boundary() {
        // A level is two code bits (NRZI), so one level flipped (an edge one symbol off) can
        // change two nibbles. When they are the two sides of a group boundary and both change by
        // XOR 9 (GCR 0x1A <-> 0x1B with 0x09 <-> 0x19, or 0x0D <-> 0x1D), the XOR checksum does
        // not see it. Exactly 128 values at each of the three boundaries; every other flip is an
        // error. The 12-bit checksum is the protocol's, so this is a property of DShot.
        let mut accepted = std::collections::BTreeMap::<u32, u32>::new();
        for value in 0..0x1000u16 {
            let levels = encode_reply(value);
            for bit in 0..21 {
                match decode_reply(levels ^ (1 << bit)) {
                    Err(_) => {}
                    Ok(r) => {
                        *accepted.entry(bit).or_default() += 1;
                        let changed = value_of(r) ^ value;
                        let want = match bit {
                            15 => 0x990,
                            10 => 0x099,
                            5 => 0x009,
                            _ => panic!("bit {bit} accepted for {value:#x}"),
                        };
                        assert_eq!(changed, want, "value {value:#x} bit {bit}");
                    }
                }
            }
        }
        assert_eq!(
            accepted,
            [(5, 128), (10, 128), (15, 128)].into_iter().collect()
        );
    }

    #[test]
    fn decode_reply_stopped_motor_is_0x52951_hand_derived_from_the_probes_toggles() {
        // The probe's self-test lists the symbols where the line toggles for 0xFFF.
        const TOGGLES: [u32; 16] = [0, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 16, 17, 20];
        let mut levels = 0u32;
        let mut high = true;
        for symbol in 0..21 {
            if TOGGLES.contains(&symbol) {
                high = !high;
            }
            levels = (levels << 1) | u32::from(high);
        }
        assert_eq!(levels, 0x52951);
        // By hand: 0xFFF then checksum !(F^F^F) = 0 -> nibbles F F F 0 -> codes 01111 01111 01111
        // 11001; a 1 toggles: after the low leading symbol the levels are
        // 0 1 0 1 0 | 0 1 0 1 0 | 0 1 0 1 0 | 1 0 0 0 1 (symbols 1..=20), hence 0x52951.
        let Ok(Reply::Erpm(e)) = decode_reply(0x52951) else {
            panic!("not eRPM")
        };
        assert_eq!(e.raw(), 0xFFF);
        assert!(e.is_stopped());
        assert_eq!(e.erpm(), 0);
        assert_eq!(e.period_us(), None);
    }

    #[test]
    fn decode_reply_hand_worked_vectors_of_each_kind() {
        let d = |levels: u32| decode_reply(levels).unwrap();
        // Computed with a separate script from the same rules.
        // 0xE00: enable acknowledge.
        assert_eq!(d(0x5BA2D), Reply::Extended(ExtendedTelemetry::EnabledAck));
        // 0xEFF: disable acknowledge.
        assert_eq!(d(0x5D6AD), Reply::Extended(ExtendedTelemetry::DisabledAck));
        // 0x22D: type 2, 0x2D = 45 degC.
        assert_eq!(
            d(0xE7123),
            Reply::Extended(ExtendedTelemetry::Temperature { celsius: 45 })
        );
        // 0x443: type 4, 0x43 = 67 steps of 0.25 V = 16.75 V.
        assert_eq!(
            d(0xB5BAB),
            Reply::Extended(ExtendedTelemetry::Voltage { millivolts: 16_750 })
        );
        // 0x60C: type 6, 12 A.
        assert_eq!(
            d(0xDBA99),
            Reply::Extended(ExtendedTelemetry::Current { milliamps: 12_000 })
        );
        // 0x000 and 0x1FF: eRPM, exponent 0.
        assert_eq!(value_of(d(0x8BA35)), 0x000);
        assert_eq!(d(0x9294B), Reply::Erpm(Erpm(0x1FF)));
    }

    #[test]
    fn decode_reply_types_am32_does_not_send_are_other() {
        for (value, kind, payload) in [
            (0x855u16, 0x8u8, 0x55u8),
            (0xA00, 0xA, 0x00),
            (0xC7F, 0xC, 0x7F),
            (0xE01, 0xE, 0x01),
            (0xEFE, 0xE, 0xFE),
            (0xE80, 0xE, 0x80),
        ] {
            assert_eq!(
                decode_reply(encode_reply(value)),
                Ok(Reply::Extended(ExtendedTelemetry::Other {
                    kind,
                    value: payload
                }))
            );
        }
    }

    #[test]
    fn decode_reply_follows_the_exponent_mantissa_rule_for_all_values() {
        for value in 0..0x1000u16 {
            let r = decode_reply(encode_reply(value)).unwrap();
            assert_eq!(
                matches!(r, Reply::Erpm(_)),
                is_erpm_by_spec(value),
                "{value:#x}"
            );
        }
        // The 7 x 256 extended codes: exponent 1..=7 with bit 8 clear.
        let extended = (0..0x1000u16).filter(|&v| !is_erpm_by_spec(v)).count();
        assert_eq!(extended, 7 * 256);
    }

    #[test]
    fn raw_is_the_12_bit_value_the_esc_sent() {
        for value in [0u16, 1, 0x1FF, 0x200, 0x3FF, 0xFFE, 0xFFF] {
            if let Ok(Reply::Erpm(e)) = decode_reply(encode_reply(value)) {
                assert_eq!(e.raw(), value);
            }
        }
        assert_eq!(Erpm(0x5A5).raw(), 0x5A5);
    }

    #[test]
    fn is_stopped_only_for_0xfff() {
        for value in 0..0x1000u16 {
            assert_eq!(Erpm(value).is_stopped(), value == 0xFFF);
        }
    }

    #[test]
    fn period_us_is_mantissa_shifted_by_the_exponent() {
        assert_eq!(Erpm(0x000).period_us(), None);
        assert_eq!(Erpm(0x001).period_us(), Some(1));
        assert_eq!(Erpm(0x1FF).period_us(), Some(511));
        // exponent 1, mantissa 0x100: 512 us.
        assert_eq!(Erpm(0x300).period_us(), Some(512));
        // exponent 7, mantissa 0x1FF would be 0xFFF: stopped.
        assert_eq!(Erpm(0xFFF).period_us(), None);
        // exponent 7, mantissa 0x1FE: 65 280 us.
        assert_eq!(Erpm(0xFFE).period_us(), Some(0x1FE << 7));
    }

    #[test]
    fn erpm_matches_an_f64_reference_for_all_4096_values() {
        for value in 0..0x1000u16 {
            let e = Erpm(value);
            let exponent = i32::from(value >> 9);
            let mantissa = f64::from(value & 0x1FF);
            let period = mantissa * 2f64.powi(exponent);
            let want = if value == 0xFFF || period == 0.0 {
                0.0
            } else {
                (60.0e6 / period).floor()
            };
            assert_eq!(f64::from(e.erpm()), want, "{value:#x}");
        }
        // 1000 us per electrical revolution is 60 000 eRPM: exponent 1, mantissa 500 = 0x1F4.
        assert_eq!(Erpm(0x200 | 0x1F4).erpm(), 60_000);
    }

    #[test]
    fn rpm_divides_erpm_by_the_pole_pairs() {
        for value in 0..0x1000u16 {
            let e = Erpm(value);
            assert_eq!(e.rpm(0), 0);
            assert_eq!(e.rpm(1), e.erpm());
            for pairs in [2u8, 7, 14, 255] {
                let want = (f64::from(e.erpm()) / f64::from(pairs)).floor();
                assert_eq!(f64::from(e.rpm(pairs)), want, "{value:#x} {pairs}");
            }
        }
    }

    #[test]
    fn extended_values_are_scaled_to_millivolts_and_milliamps() {
        let ext = |v: u16| match decode_reply(encode_reply(v)).unwrap() {
            Reply::Extended(x) => x,
            Reply::Erpm(_) => panic!("eRPM"),
        };
        assert_eq!(ext(0x2FF), ExtendedTelemetry::Temperature { celsius: 255 });
        assert_eq!(ext(0x400), ExtendedTelemetry::Voltage { millivolts: 0 });
        assert_eq!(
            ext(0x4FF),
            ExtendedTelemetry::Voltage { millivolts: 63_750 }
        );
        assert_eq!(
            ext(0x6FF),
            ExtendedTelemetry::Current { milliamps: 255_000 }
        );
    }

    #[test]
    fn every_error_variant_is_produced() {
        let errors: Vec<ReplyError> = std::vec![
            decode_reply(1 << 20).unwrap_err(),
            decode_reply(0).unwrap_err(),
            decode_reply(encode_word(0x0000)).unwrap_err(),
        ];
        assert_eq!(
            errors,
            [
                ReplyError::BadStart,
                ReplyError::BadGcr,
                ReplyError::BadChecksum
            ]
        );
    }

    proptest! {
        #[test]
        fn decode_reply_round_trips_through_the_inverse(value in 0u16..0x1000) {
            let levels = encode_reply(value);
            prop_assert_eq!(levels >> 20, 0);
            let r = decode_reply(levels).unwrap();
            prop_assert_eq!(value_of(r), value);
            prop_assert_eq!(matches!(r, Reply::Erpm(_)), is_erpm_by_spec(value));
        }

        #[test]
        fn decode_reply_never_panics_and_is_exact_on_any_word(levels in any::<u32>()) {
            if let Ok(r) = decode_reply(levels) {
                prop_assert_eq!(encode_reply(value_of(r)), levels);
            }
        }

        #[test]
        fn erpm_is_monotonic_in_the_period(a in 1u16..0xFFF, b in 1u16..0xFFF) {
            if let (Some(pa), Some(pb)) = (Erpm(a).period_us(), Erpm(b).period_us())
                && pa < pb
            {
                prop_assert!(Erpm(a).erpm() >= Erpm(b).erpm());
            }
        }
    }
}
