//! The serial telemetry of an AM32 ESC: the 10-byte frames on its telemetry wire.
//!
//! The ESC sends them at 115200 8N1 on a single wire, after a DShot frame with the telemetry
//! request bit set or at an interval it is configured to (`Src/kiss_telemetry.c`,
//! `f421/Src/serial_telemetry.c`). The bytes of a frame, first byte first:
//!
//! | byte | content |
//! |---|---|
//! | 0 | temperature in degrees Celsius, signed |
//! | 1, 2 | voltage in units of 0.01 V, high byte first |
//! | 3, 4 | current in units of 0.01 A |
//! | 5, 6 | consumption in mAh |
//! | 7, 8 | eRPM divided by 100 |
//! | 9 | CRC-8, polynomial 0x07, start value 0, no reflection, no final XOR, over bytes 0 to 8 |
//!
//! There is no start marker and no gap that this module can see, so the [`Framer`] looks for a
//! frame at every alignment.
//!
//! # How sure a frame is
//!
//! A CRC of 8 bits lets 1 in 256 random 10-byte windows through. On a wire that carries only
//! frames this never matters, because the framer is in step after the first one. After junk, or
//! at the start, about one alignment in 256 passes by chance and gives a wrong frame, and the
//! plausibility of the values (a voltage of 0 on a powered ESC, a temperature of -128) is the
//! caller's check. A frame made only of zero bytes also passes the CRC (the CRC of zeros is
//! zero); it is what a wire held low looks like, so it is never reported as telemetry and is
//! counted apart by [`Framer::all_zero_windows`]. A genuine frame of all zeros (0 degrees, 0 V,
//! 0 A, 0 mAh, 0 eRPM) is lost with it.

/// Bytes in one frame.
const FRAME_LEN: usize = 10;

/// `crc8_step[i]`: the CRC of the single byte `i`, so that a byte moves the CRC by one lookup.
const CRC_TABLE: [u8; 256] = {
    let mut table = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = i as u8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
            bit += 1;
        }
        // bound: i < 256 = table.len().
        #[allow(clippy::indexing_slicing)]
        {
            table[i] = crc;
        }
        i += 1;
    }
    table
};

/// CRC-8, polynomial 0x07, start 0, no reflection, no final XOR (AM32 `get_crc8`).
fn crc8(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &byte in bytes {
        // bound: the index is a u8 and the table has 256 entries.
        #[allow(clippy::indexing_slicing)]
        {
            crc = CRC_TABLE[usize::from(crc ^ byte)];
        }
    }
    crc
}

/// One telemetry frame, with its CRC checked.
///
/// Only a [`Framer`] makes one. The values are what the ESC measured and scaled; this crate
/// does not judge them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Telemetry {
    temperature: i8,
    centivolts: u16,
    centiamps: u16,
    milliamp_hours: u16,
    erpm_hundreds: u16,
}

impl Telemetry {
    /// ESC temperature in degrees Celsius, -128 to 127.
    #[must_use]
    pub const fn celsius(self) -> i8 {
        self.temperature
    }

    /// Supply voltage in millivolts: the wire has 0.01 V steps, so a multiple of 10, up to
    /// 655 350.
    #[must_use]
    pub const fn millivolts(self) -> u32 {
        self.centivolts as u32 * 10
    }

    /// Current in milliamps: the wire has 0.01 A steps, so a multiple of 10, up to 655 350.
    #[must_use]
    pub const fn milliamps(self) -> u32 {
        self.centiamps as u32 * 10
    }

    /// Charge drawn since the ESC started, in mAh, as it counted it.
    #[must_use]
    pub const fn consumed_mah(self) -> u16 {
        self.milliamp_hours
    }

    /// Electrical revolutions per minute: the wire has hundreds, so a multiple of 100, up to
    /// 6 553 500.
    #[must_use]
    pub const fn erpm(self) -> u32 {
        self.erpm_hundreds as u32 * 100
    }

    /// Mechanical revolutions per minute for a motor with `pole_pairs` pole pairs:
    /// [`Telemetry::erpm`] over `pole_pairs`, rounded down; 0 for 0 pole pairs.
    #[must_use]
    pub const fn rpm(self, pole_pairs: u8) -> u32 {
        if pole_pairs == 0 {
            return 0;
        }
        self.erpm() / pole_pairs as u32
    }

    /// The values of a 10-byte window whose CRC has been checked.
    const fn from_bytes(window: [u8; FRAME_LEN]) -> Self {
        let [t, vh, vl, ih, il, ch, cl, eh, el, _crc] = window;
        Self {
            temperature: t as i8,
            centivolts: u16::from_be_bytes([vh, vl]),
            centiamps: u16::from_be_bytes([ih, il]),
            milliamp_hours: u16::from_be_bytes([ch, cl]),
            erpm_hundreds: u16::from_be_bytes([eh, el]),
        }
    }
}

/// Finds telemetry frames in a stream of bytes, with no marker to go by.
///
/// Feed it the bytes as they come, one at a time with [`Framer::push`] or in slices with
/// [`Framer::feed`], split anywhere. It holds the last up to 10 bytes. When it has 10 and they
/// are a frame (the CRC over the first nine is the tenth) it reports the frame and starts over
/// with the next byte, so back-to-back frames are found at their own boundaries and a window
/// that straddles two frames is never looked at. When the 10 bytes are not a frame, the oldest
/// byte is dropped and the next byte makes a new window: that is how it finds the start of a
/// frame after junk, and each such step is one rejected alignment.
///
/// A window of ten zero bytes passes the CRC and is never reported: it is counted in
/// [`Framer::all_zero_windows`] and the framer slides on. About 1 alignment in 256 of random
/// bytes passes the CRC, see the module documentation.
///
/// The framer never panics, whatever the bytes. All counters saturate.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Framer {
    window: [u8; FRAME_LEN],
    filled: usize,
    frames: u32,
    rejected: u32,
    all_zero: u32,
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

impl Framer {
    /// A framer that has seen no byte.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            window: [0; FRAME_LEN],
            filled: 0,
            frames: 0,
            rejected: 0,
            all_zero: 0,
        }
    }

    /// Takes one byte; gives the frame if this byte completes one.
    pub fn push(&mut self, byte: u8) -> Option<Telemetry> {
        if self.filled == FRAME_LEN {
            self.window.copy_within(1.., 0);
            self.window[FRAME_LEN - 1] = byte;
        } else {
            // bound: filled < FRAME_LEN = window.len() in this branch.
            #[allow(clippy::indexing_slicing)]
            {
                self.window[self.filled] = byte;
            }
            self.filled += 1;
            if self.filled < FRAME_LEN {
                return None;
            }
        }
        let window = self.window;
        let [body @ .., crc] = window;
        if crc8(&body) != crc {
            self.rejected = self.rejected.saturating_add(1);
            return None;
        }
        if window == [0; FRAME_LEN] {
            self.all_zero = self.all_zero.saturating_add(1);
            return None;
        }
        self.frames = self.frames.saturating_add(1);
        self.filled = 0;
        Some(Telemetry::from_bytes(window))
    }

    /// Takes a slice of bytes, in order, and calls `on_frame` for each frame they complete.
    /// The same as [`Framer::push`] on each byte; where the slices are cut does not matter.
    pub fn feed(&mut self, bytes: &[u8], mut on_frame: impl FnMut(Telemetry)) {
        for &byte in bytes {
            if let Some(frame) = self.push(byte) {
                on_frame(frame);
            }
        }
    }

    /// Frames reported so far.
    #[must_use]
    pub const fn frames(&self) -> u32 {
        self.frames
    }

    /// Alignments of 10 bytes that failed the CRC so far: junk, or the start of a stream.
    #[must_use]
    pub const fn rejected_alignments(&self) -> u32 {
        self.rejected
    }

    /// Windows of ten zero bytes so far, which pass the CRC and are not telemetry.
    #[must_use]
    pub const fn all_zero_windows(&self) -> u32 {
        self.all_zero
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use std::vec;
    use std::vec::Vec;

    /// AM32 `update_crc8` and `get_crc8` (`Src/functions.c:109-127`) written out as in C, bit
    /// by bit: the independent reference.
    fn reference_crc8(buf: &[u8]) -> u8 {
        let mut crc = 0u8;
        for &b in buf {
            let mut crc_u = b ^ crc;
            for _ in 0..8 {
                crc_u = if crc_u & 0x80 != 0 {
                    0x07 ^ (crc_u << 1)
                } else {
                    crc_u << 1
                };
            }
            crc = crc_u;
        }
        crc
    }

    /// `makeTelemPackage` of `Src/kiss_telemetry.c`: the 10 bytes the ESC sends.
    fn make_telem_package(
        temp: i8,
        voltage: u16,
        current: u16,
        consumption: u16,
        e_rpm: u16,
    ) -> [u8; FRAME_LEN] {
        let mut p = [0u8; FRAME_LEN];
        p[0] = temp as u8;
        p[1] = (voltage >> 8) as u8;
        p[2] = (voltage & 0xFF) as u8;
        p[3] = (current >> 8) as u8;
        p[4] = (current & 0xFF) as u8;
        p[5] = (consumption >> 8) as u8;
        p[6] = (consumption & 0xFF) as u8;
        p[7] = (e_rpm >> 8) as u8;
        p[8] = (e_rpm & 0xFF) as u8;
        p[9] = reference_crc8(&p[..9]);
        p
    }

    fn telemetry(temp: i8, cv: u16, ca: u16, mah: u16, erpm100: u16) -> Telemetry {
        Telemetry {
            temperature: temp,
            centivolts: cv,
            centiamps: ca,
            milliamp_hours: mah,
            erpm_hundreds: erpm100,
        }
    }

    /// 35 C, 16.50 V, 12.34 A, 321 mAh, 100 000 eRPM; the CRC 0x3D from a separate script.
    const FRAME: [u8; 10] = [0x23, 0x06, 0x72, 0x04, 0xD2, 0x01, 0x41, 0x03, 0xE8, 0x3D];

    fn all_frames(bytes: &[u8]) -> (Vec<Telemetry>, Framer) {
        let mut f = Framer::new();
        let mut got = Vec::new();
        f.feed(bytes, |t| got.push(t));
        (got, f)
    }

    #[test]
    fn crc8_has_the_published_check_value_f4_for_123456789() {
        assert_eq!(crc8(b"123456789"), 0xF4);
        assert_eq!(reference_crc8(b"123456789"), 0xF4);
    }

    #[test]
    fn crc8_equals_the_bit_by_bit_reference() {
        assert_eq!(crc8(&[]), 0);
        for b in 0..=255u8 {
            assert_eq!(crc8(&[b]), reference_crc8(&[b]), "{b}");
            assert_eq!(
                crc8(&[b, 0xA5, b ^ 0x5A]),
                reference_crc8(&[b, 0xA5, b ^ 0x5A])
            );
        }
        assert_eq!(crc8(&[0; 9]), 0);
    }

    #[test]
    fn the_hand_built_frame_matches_kiss_telemetry_c() {
        assert_eq!(make_telem_package(35, 1650, 1234, 321, 1000), FRAME);
        assert_eq!(reference_crc8(&FRAME[..9]), 0x3D);
        let (got, f) = all_frames(&FRAME);
        assert_eq!(got, [telemetry(35, 1650, 1234, 321, 1000)]);
        assert_eq!(f.frames(), 1);
    }

    #[test]
    fn celsius_is_the_signed_first_byte() {
        for t in [i8::MIN, -40, -1, 0, 1, 35, i8::MAX] {
            assert_eq!(telemetry(t, 0, 0, 0, 0).celsius(), t);
            let frame = make_telem_package(t, 1, 0, 0, 0);
            let (got, _) = all_frames(&frame);
            assert_eq!(got[0].celsius(), t);
        }
        // Negative: 0xF6 is -10, and the frame is accepted.
        let frame = make_telem_package(-10, 1650, 0, 0, 0);
        assert_eq!(frame[0], 0xF6);
        assert_eq!(frame[9], 0x53);
        assert_eq!(all_frames(&frame).0[0].celsius(), -10);
    }

    #[test]
    fn millivolts_is_ten_times_the_centivolts() {
        assert_eq!(telemetry(0, 1650, 0, 0, 0).millivolts(), 16_500);
        assert_eq!(telemetry(0, 0, 0, 0, 0).millivolts(), 0);
        assert_eq!(telemetry(0, u16::MAX, 0, 0, 0).millivolts(), 655_350);
    }

    #[test]
    fn milliamps_is_ten_times_the_centiamps() {
        assert_eq!(telemetry(0, 0, 1234, 0, 0).milliamps(), 12_340);
        assert_eq!(telemetry(0, 0, u16::MAX, 0, 0).milliamps(), 655_350);
    }

    #[test]
    fn consumed_mah_is_the_consumption_field() {
        assert_eq!(telemetry(0, 0, 0, 321, 0).consumed_mah(), 321);
        assert_eq!(telemetry(0, 0, 0, u16::MAX, 0).consumed_mah(), u16::MAX);
    }

    #[test]
    fn erpm_is_a_hundred_times_the_field() {
        assert_eq!(telemetry(0, 0, 0, 0, 1000).erpm(), 100_000);
        assert_eq!(telemetry(0, 0, 0, 0, u16::MAX).erpm(), 6_553_500);
        assert_eq!(telemetry(0, 0, 0, 0, 0).erpm(), 0);
    }

    #[test]
    fn rpm_divides_the_erpm_by_the_pole_pairs() {
        let t = telemetry(0, 0, 0, 0, 1000);
        assert_eq!(t.rpm(0), 0);
        assert_eq!(t.rpm(1), 100_000);
        assert_eq!(t.rpm(7), 14_285);
        assert_eq!(t.rpm(255), 392);
    }

    #[test]
    fn new_framer_has_seen_nothing_and_default_is_new() {
        let f = Framer::new();
        assert_eq!(
            (f.frames(), f.rejected_alignments(), f.all_zero_windows()),
            (0, 0, 0)
        );
        assert_eq!(f, Framer::default());
    }

    #[test]
    fn push_gives_the_frame_on_the_tenth_byte_only() {
        let mut f = Framer::new();
        for &b in &FRAME[..9] {
            assert_eq!(f.push(b), None);
        }
        assert_eq!(f.push(FRAME[9]), Some(telemetry(35, 1650, 1234, 321, 1000)));
        assert_eq!(f.rejected_alignments(), 0);
        // And the next ten bytes make the next one, back to back, with nothing rejected.
        for &b in &FRAME[..9] {
            assert_eq!(f.push(b), None);
        }
        assert!(f.push(FRAME[9]).is_some());
        assert_eq!((f.frames(), f.rejected_alignments()), (2, 0));
    }

    #[test]
    fn push_finds_a_frame_after_junk_by_sliding() {
        let mut stream = vec![0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77];
        stream.extend_from_slice(&FRAME);
        let mut f = Framer::new();
        let found: Vec<_> = stream.iter().filter_map(|&b| f.push(b)).collect();
        assert_eq!(found, [telemetry(35, 1650, 1234, 321, 1000)]);
        // 17 bytes, the first window at the tenth: 8 windows, 7 rejected before the frame.
        assert_eq!(f.rejected_alignments(), 7);
    }

    #[test]
    fn feed_takes_slices_split_anywhere_and_calls_back_for_each_frame() {
        let a = make_telem_package(30, 1600, 100, 5, 700);
        let b = make_telem_package(-5, 1590, 2000, 6, 0);
        let c = make_telem_package(31, 1580, 3000, 7, 65535);
        let mut stream = Vec::new();
        stream.extend_from_slice(&a);
        stream.extend_from_slice(&[0xDE, 0xAD, 0xBE]);
        stream.extend_from_slice(&b);
        stream.extend_from_slice(&c);
        let want = [
            telemetry(30, 1600, 100, 5, 700),
            telemetry(-5, 1590, 2000, 6, 0),
            telemetry(31, 1580, 3000, 7, 65535),
        ];
        let (whole, whole_f) = all_frames(&stream);
        assert_eq!(whole, want);
        // Every split position, and every pair of split positions.
        for cut in 0..=stream.len() {
            let mut f = Framer::new();
            let mut got = Vec::new();
            f.feed(&stream[..cut], |t| got.push(t));
            f.feed(&stream[cut..], |t| got.push(t));
            assert_eq!(got, want, "cut {cut}");
            assert_eq!(f, whole_f, "cut {cut}");
        }
        for cut in (0..=stream.len()).step_by(3) {
            for cut2 in cut..=stream.len() {
                let mut f = Framer::new();
                let mut got = Vec::new();
                f.feed(&stream[..cut], |t| got.push(t));
                f.feed(&stream[cut..cut2], |t| got.push(t));
                f.feed(&stream[cut2..], |t| got.push(t));
                assert_eq!(got, want, "cuts {cut} {cut2}");
            }
        }
        // Byte by byte through push gives the same.
        let mut f = Framer::new();
        let one: Vec<_> = stream.iter().filter_map(|&b| f.push(b)).collect();
        assert_eq!(one, want);
        assert_eq!(f, whole_f);
    }

    #[test]
    fn frames_rejected_alignments_and_all_zero_windows_count_what_they_say() {
        let (got, f) = all_frames(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]);
        assert!(got.is_empty());
        assert_eq!(
            (f.frames(), f.rejected_alignments(), f.all_zero_windows()),
            (0, 2, 0)
        );
        let (got, f) = all_frames(&FRAME);
        assert_eq!(got.len(), 1);
        assert_eq!(
            (f.frames(), f.rejected_alignments(), f.all_zero_windows()),
            (1, 0, 0)
        );
    }

    #[test]
    fn rejected_alignments_counts_a_frame_with_a_flipped_bit_and_the_slides() {
        for bit in 0..80 {
            let mut bad = FRAME;
            bad[bit / 8] ^= 1 << (bit % 8);
            let (got, f) = all_frames(&bad);
            assert!(got.is_empty(), "bit {bit}");
            assert_eq!(f.rejected_alignments(), 1, "bit {bit}");
        }
    }

    #[test]
    fn all_zero_windows_pass_the_crc_and_are_not_telemetry() {
        assert_eq!(reference_crc8(&[0; 9]), 0);
        let (got, f) = all_frames(&[0; 10]);
        assert!(got.is_empty());
        assert_eq!(
            (f.frames(), f.rejected_alignments(), f.all_zero_windows()),
            (0, 0, 1)
        );
        // A wire held low: one zero window for each byte after the ninth.
        let (got, f) = all_frames(&[0; 100]);
        assert!(got.is_empty());
        assert_eq!(f.all_zero_windows(), 91);
        assert_eq!(f.rejected_alignments(), 0);
        // And a real frame after it is still found.
        let mut stream = vec![0u8; 25];
        stream.extend_from_slice(&FRAME);
        let (got, f) = all_frames(&stream);
        assert_eq!(got, [telemetry(35, 1650, 1234, 321, 1000)]);
        assert_eq!(f.all_zero_windows(), 16);
        // A frame that is zero except for a field is not an all-zero window.
        let almost = make_telem_package(0, 0, 0, 0, 1);
        assert_eq!(all_frames(&almost).0, [telemetry(0, 0, 0, 0, 1)]);
    }

    #[test]
    fn about_1_in_256_windows_of_random_bytes_pass_the_crc() {
        // A fixed pseudo-random stream; the fraction of windows that pass is near 1 / 256.
        let mut x = 0x1234_5678u32;
        let bytes: Vec<u8> = (0..400_000)
            .map(|_| {
                x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (x >> 24) as u8
            })
            .collect();
        // Every alignment, with no consumption: the CRC of each window by hand.
        let passes = bytes
            .windows(10)
            .filter(|w| reference_crc8(&w[..9]) == w[9])
            .count();
        let rate = passes as f64 / (bytes.len() - 9) as f64;
        assert!((rate - 1.0 / 256.0).abs() < 0.0008, "{rate}");
    }

    #[test]
    fn a_stream_with_junk_between_frames_loses_none() {
        let frames: Vec<[u8; 10]> = (0..20)
            .map(|i| make_telem_package(20 + i as i8, 1600 + i, 50 * i, i * 3, 500 + 11 * i))
            .collect();
        let mut stream = Vec::new();
        let mut junk = 0x9Eu8;
        for (i, fr) in frames.iter().enumerate() {
            // Junk of 0 to 7 bytes that cannot start a frame by chance: checked below.
            for _ in 0..(i % 8) {
                junk = junk.wrapping_mul(37).wrapping_add(11);
                stream.push(junk);
            }
            stream.extend_from_slice(fr);
        }
        let (got, f) = all_frames(&stream);
        // A false frame inside the junk is possible (1 in 256 per alignment); with this fixed
        // stream there is none, and every frame is found, in order.
        let want: Vec<Telemetry> = frames.iter().map(|b| Telemetry::from_bytes(*b)).collect();
        assert_eq!(got, want);
        assert_eq!(f.frames(), 20);
    }

    #[test]
    fn from_bytes_reads_the_fields_high_byte_first() {
        let t = Telemetry::from_bytes([0xFE, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x00]);
        assert_eq!(t.celsius(), -2);
        assert_eq!(t.millivolts(), 0x1234 * 10);
        assert_eq!(t.milliamps(), 0x5678 * 10);
        assert_eq!(t.consumed_mah(), 0x9ABC);
        assert_eq!(t.erpm(), 0xDEF0 * 100);
    }

    proptest! {
        #[test]
        fn framer_round_trips_every_field_split_anywhere(
            temp in any::<i8>(),
            volt in any::<u16>(),
            amp in any::<u16>(),
            mah in any::<u16>(),
            erpm in any::<u16>(),
            cut in 0usize..=10,
        ) {
            let frame = make_telem_package(temp, volt, amp, mah, erpm);
            let mut f = Framer::new();
            let mut got = Vec::new();
            f.feed(&frame[..cut], |t| got.push(t));
            f.feed(&frame[cut..], |t| got.push(t));
            if frame == [0; 10] {
                prop_assert!(got.is_empty());
                prop_assert_eq!(f.all_zero_windows(), 1);
            } else {
                let t = telemetry(temp, volt, amp, mah, erpm);
                prop_assert_eq!(got, vec![t]);
                prop_assert_eq!(t.celsius(), temp);
                prop_assert_eq!(t.millivolts(), u32::from(volt) * 10);
                prop_assert_eq!(t.milliamps(), u32::from(amp) * 10);
                prop_assert_eq!(t.consumed_mah(), mah);
                prop_assert_eq!(t.erpm(), u32::from(erpm) * 100);
            }
        }

        #[test]
        fn framer_on_arbitrary_bytes_never_panics_and_matches_a_plain_search(
            bytes in prop::collection::vec(any::<u8>(), 0..300),
            cut in 0usize..300,
        ) {
            // The plain search: at each start, a window of ten; a frame with a good CRC and
            // not all zero is taken and the search goes on after it, anything else moves the
            // start by one.
            let mut start = 0;
            let (mut want, mut rejected, mut zero) = (Vec::new(), 0u32, 0u32);
            while start + 10 <= bytes.len() {
                let w = &bytes[start..start + 10];
                if reference_crc8(&w[..9]) != w[9] {
                    rejected += 1;
                    start += 1;
                } else if w == [0; 10] {
                    zero += 1;
                    start += 1;
                } else {
                    want.push(Telemetry::from_bytes(w.try_into().unwrap()));
                    start += 10;
                }
            }
            let cut = cut.min(bytes.len());
            let mut f = Framer::new();
            let mut got = Vec::new();
            f.feed(&bytes[..cut], |t| got.push(t));
            f.feed(&bytes[cut..], |t| got.push(t));
            prop_assert_eq!(&got, &want);
            prop_assert_eq!(f.frames() as usize, want.len());
            prop_assert_eq!(f.rejected_alignments(), rejected);
            prop_assert_eq!(f.all_zero_windows(), zero);
        }
    }

    #[test]
    fn counters_saturate() {
        let mut f = Framer::new();
        f.frames = u32::MAX;
        f.rejected = u32::MAX;
        f.all_zero = u32::MAX;
        f.feed(&FRAME, |_| {});
        f.feed(&[1; 20], |_| {});
        f.feed(&[0; 20], |_| {});
        assert_eq!(f.frames(), u32::MAX);
        assert_eq!(f.rejected_alignments(), u32::MAX);
        assert_eq!(f.all_zero_windows(), u32::MAX);
    }
}
