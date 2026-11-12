//! The four-line frame image for the PIO program.

use crate::value::LineFrame;

/// The two TX FIFO words that put one frame on all four motor lines.
///
/// This is exactly what the measurement program `dshot_m1_probe` pushes (its `STOP_IMAGE`), and
/// four [`LineFrame::STOP`] give its value `[0xFFFF_FFFF, 0x0000_FFFF]`. The image can only be
/// built from [`LineFrame`]s: there is no constructor from words, so a back-end that takes a
/// `FrameImage` cannot be handed a value this crate does not allow.
///
/// # Layout
///
/// The frame has 16 bits per line, sent most significant bit first: bit 15 of
/// [`LineFrame::word`] is the first bit on the wire, bit 0 the last. Number the bits on the wire
/// `j = 0..16` (`j = 0` first). Each wire bit takes one NIBBLE of the image:
///
/// - `j` in 0..8 is in word 0, `j` in 8..16 in word 1 (the order of [`FrameImage::words`]);
/// - within the word the nibble is `j % 8`, the lowest nibble first: bits `4 * (j % 8)` to
///   `4 * (j % 8) + 3`, so the first bit is in bits 0 to 3 of word 0;
/// - bit `i` of the nibble is for line `i`, the pin `i` places above the first pin of the group
///   (line 0 is GPIO36, line 3 is GPIO39);
/// - the nibble bit is the PIN LEVEL OF THE DATA PHASE, not the data bit: it is 1 (line high)
///   for a 0 bit and 0 (line stays low) for a 1 bit.
///
/// The program shifts the FIFO out to the pins four bits at a time, least significant bits
/// first (`out pins, 4`, shift right), in the second of its three phases of each bit: all
/// lines low, the nibble, all lines high.
///
/// The checksum bits are wire bits 12 to 15, so they are nibbles 4 to 7 of word 1 (bits 16 to 31).
///
/// ```compile_fail,E0423
/// let _ = dshot::FrameImage([0xFFFF_FFFF, 0x0000_FFFF]);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FrameImage([u32; 2]);

/// Bit `4 * k` for the eight bits `k = 0..8` of `x` (a byte), the other bits clear.
const fn spread_nibbles(x: u32) -> u32 {
    let x = (x | (x << 12)) & 0x000F_000F;
    let x = (x | (x << 6)) & 0x0303_0303;
    (x | (x << 3)) & 0x1111_1111
}

/// The levels of one line for the 8 wire bits in `bits` (wire bit 0 in bit 0), as bit `line` of
/// each nibble.
const fn line_levels(bits: u32, line: u32) -> u32 {
    (spread_nibbles(bits) ^ 0x1111_1111) << line
}

/// The two words for one line's frame word, to be OR-ed into the image.
const fn line_words(frame: LineFrame, line: u32) -> [u32; 2] {
    // Wire order: reversed, so that bit j of `wire` is the j-th bit sent.
    let wire = (frame.word().reverse_bits()) as u32;
    [line_levels(wire & 0xFF, line), line_levels(wire >> 8, line)]
}

impl FrameImage {
    /// Four Stop frames without a telemetry request: the probe's `STOP_IMAGE`.
    pub const STOP: FrameImage = FrameImage::new([LineFrame::STOP; 4]);

    /// The image of one frame per line. `lines[i]` goes to line `i` (GPIO36 + `i`).
    #[must_use]
    pub const fn new(lines: [LineFrame; 4]) -> FrameImage {
        let [a, b, c, d] = lines;
        let a = line_words(a, 0);
        let b = line_words(b, 1);
        let c = line_words(c, 2);
        let d = line_words(d, 3);
        FrameImage([a[0] | b[0] | c[0] | d[0], a[1] | b[1] | c[1] | d[1]])
    }

    /// The words for the TX FIFO, in the order to push them. Push both with no await between
    /// them: the PIO program starts a frame only when both are in the FIFO.
    #[must_use]
    pub const fn words(self) -> [u32; 2] {
        self.0
    }
}

/// Test hook, outside the crate's API promise (feature `_test-hooks`).
///
/// Flips the four checksum bits of `line` in an image already built, so the ESC must reject the
/// frame. It touches only wire bits 12 to 15 of that line: the value, the request bit and the
/// other lines are unchanged, so it cannot change what the frame asks for. Returns `None` when
/// `line` is not 0 to 3.
#[cfg(feature = "_test-hooks")]
#[must_use]
pub fn corrupt_checksum(image: FrameImage, line: usize) -> Option<FrameImage> {
    if line >= 4 {
        return None;
    }
    let [w0, w1] = image.0;
    Some(FrameImage([w0, w1 ^ (0x1111_0000 << line)]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::value::{Command, Throttle, Value};
    use proptest::prelude::*;

    /// The documented layout, read back the slow way: wire bit `j` of line `i` is the inverse of
    /// bit `i` of nibble `j % 8` of word `j / 8`.
    fn read_back(image: FrameImage, line: usize) -> u16 {
        let words = image.words();
        let mut frame = 0u16;
        for j in 0..16 {
            let nibble = (words[j / 8] >> (4 * (j % 8))) & 0xF;
            let level = (nibble >> line) & 1;
            let bit = 1 - level;
            frame |= (bit as u16) << (15 - j);
        }
        frame
    }

    fn value_strategy() -> impl Strategy<Value = Value> {
        prop_oneof![
            Just(Value::Stop),
            proptest::sample::select(Command::ALL.to_vec()).prop_map(Value::Command),
            (1u16..=2000).prop_map(|l| Value::Throttle(Throttle::new(l).unwrap())),
        ]
    }

    fn frame_strategy() -> impl Strategy<Value = LineFrame> {
        (value_strategy(), any::<bool>()).prop_map(|(v, r)| LineFrame::new(v, r))
    }

    fn lines_strategy() -> impl Strategy<Value = [LineFrame; 4]> {
        [
            frame_strategy(),
            frame_strategy(),
            frame_strategy(),
            frame_strategy(),
        ]
    }

    #[test]
    fn new_frame_image_of_four_stops_is_the_probes_stop_image() {
        // `STOP_IMAGE` of hil/src/bin/dshot_m1_probe.rs, which ran on the ESCs.
        let stop_image = [0xFFFF_FFFFu32, 0x0000_FFFF];
        assert_eq!(FrameImage::new([LineFrame::STOP; 4]).words(), stop_image);
        assert_eq!(FrameImage::STOP.words(), stop_image);
        assert_eq!(FrameImage::STOP, FrameImage::new([LineFrame::STOP; 4]));
    }

    #[test]
    fn words_gives_the_hand_worked_layout() {
        // Line 1 alone sends 0x82D8 = 1000 0010 1101 1000 (throttle wire 1046, request bit); the
        // other lines send Stop, 0x000F = twelve 0 bits then four 1 bits. A nibble bit is the pin
        // level, the inverse of the data bit.
        // Line 1 levels, wire bits 0..16: 0111 1101 0010 0111.
        // Lines 0, 2, 3 levels: 1 for wire bits 0..12, 0 for 12..16.
        // Word 0 (wire bits 0..8): every nibble is 0xF except where line 1 is low, wire bits 0
        // and 6: nibble 0 and nibble 6 are 0xD. Nibble 7 first: 0xFDFF_FFFD.
        // Word 1 (wire bits 8..16), nibble 0 first: 0xD, 0xD, 0xF, 0xD (line 1 low at wire bits
        // 8, 9, 11; the others high), then wire bits 12..16 where the others are low: line 1
        // alone can be high: 0x0, 0x2, 0x2, 0x2. Nibble 7 first: 0x2220_DFDD.
        let lines = [
            LineFrame::STOP,
            LineFrame::new(Value::Throttle(Throttle::new(999).unwrap()), true),
            LineFrame::STOP,
            LineFrame::STOP,
        ];
        assert_eq!(lines[1].word(), 0x82D8);
        assert_eq!(FrameImage::new(lines).words(), [0xFDFF_FFFD, 0x2220_DFDD]);
        let mixed = [
            LineFrame::new(Value::Throttle(Throttle::new(999).unwrap()), false),
            LineFrame::STOP,
            LineFrame::new(Value::Command(Command::ExtendedTelemetryOn), false),
            LineFrame::new(Value::Command(Command::Beep1), false),
        ];
        assert_eq!(
            [mixed[0].word(), mixed[2].word(), mixed[3].word()],
            [0x82C9, 0x01A4, 0x002D]
        );
        // Computed by a separate script from the layout in the documentation.
        assert_eq!(FrameImage::new(mixed).words(), [0xBEFF_FFFE, 0x4D14_F3EA]);
    }

    #[test]
    fn new_frame_image_changes_only_the_nibble_bits_of_the_changed_line() {
        let base = [
            LineFrame::new(Value::Throttle(Throttle::new(500).unwrap()), false),
            LineFrame::new(Value::Command(Command::Beep4), true),
            LineFrame::STOP,
            LineFrame::new(Value::Throttle(Throttle::new(2000).unwrap()), true),
        ];
        let other = LineFrame::new(Value::Throttle(Throttle::new(1234).unwrap()), true);
        for line in 0..4 {
            let mut changed = base;
            changed[line] = other;
            let a = FrameImage::new(base).words();
            let b = FrameImage::new(changed).words();
            let line_mask = 0x1111_1111u32 << line;
            for w in 0..2 {
                assert_eq!((a[w] ^ b[w]) & !line_mask, 0, "line {line} word {w}");
            }
            assert_ne!(a, b);
        }
    }

    proptest! {
        #[test]
        fn new_frame_image_reads_back_as_four_frame_words(lines in lines_strategy()) {
            let image = FrameImage::new(lines);
            for (i, line) in lines.iter().enumerate() {
                prop_assert_eq!(read_back(image, i), line.word());
            }
        }

        #[test]
        fn new_frame_image_one_line_changes_only_that_line(
            lines in lines_strategy(), replacement in frame_strategy(), line in 0usize..4
        ) {
            let mut changed = lines;
            changed[line] = replacement;
            let (a, b) = (FrameImage::new(lines), FrameImage::new(changed));
            for i in 0..4 {
                if i != line {
                    prop_assert_eq!(read_back(a, i), read_back(b, i));
                }
            }
            prop_assert_eq!(read_back(b, line), replacement.word());
        }
    }

    #[cfg(feature = "_test-hooks")]
    mod hooks {
        use super::*;

        #[test]
        fn corrupt_checksum_flips_only_the_checksum_bits_of_one_line() {
            let lines = [
                LineFrame::new(Value::Throttle(Throttle::new(999).unwrap()), true),
                LineFrame::new(Value::Command(Command::ExtendedTelemetryOn), false),
                LineFrame::STOP,
                LineFrame::new(Value::Command(Command::Beep5), true),
            ];
            let image = FrameImage::new(lines);
            for line in 0..4 {
                let bad = corrupt_checksum(image, line).unwrap();
                for (i, l) in lines.iter().enumerate() {
                    let (good, got) = (l.word(), read_back(bad, i));
                    if i == line {
                        assert_eq!(got >> 4, good >> 4, "value and request bits of line {i}");
                        assert_ne!(got & 0xF, good & 0xF);
                        assert_eq!(got & 0xF, !good & 0xF);
                    } else {
                        assert_eq!(got, good);
                    }
                }
            }
            assert_eq!(corrupt_checksum(image, 4), None);
            assert_eq!(corrupt_checksum(image, usize::MAX), None);
        }

        #[test]
        fn corrupt_checksum_of_stop_gives_a_frame_the_esc_rejects() {
            let bad = corrupt_checksum(FrameImage::STOP, 2).unwrap();
            assert_eq!(read_back(bad, 2), 0x0000);
            assert_eq!(read_back(bad, 0), 0x000F);
        }

        proptest! {
            #[test]
            fn corrupt_checksum_never_changes_the_value_bits(
                lines in lines_strategy(), line in 0usize..4
            ) {
                let bad = corrupt_checksum(FrameImage::new(lines), line).unwrap();
                for (i, l) in lines.iter().enumerate() {
                    let got = read_back(bad, i);
                    prop_assert_eq!(got >> 4, l.word() >> 4);
                    prop_assert_eq!(got & 0xF != l.word() & 0xF, i == line);
                }
            }
        }
    }
}
