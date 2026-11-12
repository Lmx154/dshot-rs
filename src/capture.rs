//! From the pin samples of one reply window to a result per motor line.
//!
//! [`decode_capture`] takes the RX FIFO words of one capture, exactly as the PIO program of the
//! measurement program `dshot_m1_probe` produces them, finds the edges of all four lines a word
//! at a time, turns each run between two edges into a whole number of symbols, and gives the 21
//! line levels of each line to [`decode_reply`]. It holds no state between captures. The
//! counters a caller may keep are [`CaptureStats`].
//!
//! # The samples
//!
//! The PIO program does `in pins, 4` every 2 PIO cycles with autopush at 32 bits and the input
//! shift register shifting LEFT (`probe: ShiftDirection::Left`), so:
//!
//! - word `i` holds the samples `8 * i` to `8 * i + 7`;
//! - sample `8 * i + j` is the nibble at bits `28 - 4 * j` to `31 - 4 * j`: the OLDEST sample is
//!   in the top nibble, the newest in bits 0 to 3;
//! - bit `l` of a nibble is line `l` (GPIO36 + `l`): 1 is high;
//! - the level before the first sample is high (the lines are released with their pull-ups).
//!
//! The first sample is taken 12 PIO cycles after the last rising edge of the frame the
//! program sent, one more every 2 cycles ([`Rate::reply_samples_per_bit`](crate::Rate) of them
//! in one nominal reply bit, 16 at both rates).
//!
//! # The method
//!
//! A reply starts at the first falling edge of the line; its first symbol is low; it has 21
//! symbols and after its last edge the line stays high. Every run between two edges is made a
//! whole number of symbols by rounding it against the NOMINAL bit time (half up), on its own:
//! no time is carried from one run to the next, so a bit time that is wrong by a few percent
//! cannot add up over the 21 symbols. The symbol positions are then summed as integers.
//!
//! Per word, `w ^ (w >> 4 | previous << 28)` has a 1 for every line that changed between two
//! consecutive samples. A word with no change costs a handful of instructions; every edge costs
//! one count-leading-zeros, one multiplication by a reciprocal taken from a table (no division),
//! and the update of two words of state of its line. The levels are built as a mask of the
//! symbols at which the line toggles and turned into levels by a prefix XOR at the end.
//!
//! # Cost
//!
//! An estimate from the machine code for thumbv8m (not measured on a board): about 12
//! instructions per word whether or not a line changed in it, and about 29 per edge, so some
//! 3000 to 3800 cycles for a DShot300 window (68 words) in which all four lines carry a
//! 16-edge reply, about 1100 when all four are silent. That is more than the 1500 cycles of the
//! design budget; the code is correct first and has not been tuned further.
//!
//! # What is guaranteed
//!
//! With `n` samples per nominal bit, a reply is decoded correctly when its real bit time is
//! within +-8 % of nominal and every edge is seen within `d` samples of where it really was
//! (this includes the quantisation of the sampling itself), as long as
//! `d < 0.13 * n` for the worst run, which is the one of three symbols: for `n = 16` up to
//! about 2 samples, for `n = 8` 1 sample. Outside that, a reply may be refused, and a run
//! wrongly counted gives a wrong set of levels, which `decode_reply` refuses except in the one
//! case the protocol itself allows: a single edge one symbol off across a boundary between two
//! 5-bit groups can pass the 4-bit checksum (README, rule 7; 0.45 % of single-level errors).
//!
//! One more check guards the bit time as a whole: the time from the first to the last edge must
//! be about that many nominal bits (within 9.4 % and 2 samples), else the result is
//! [`CaptureError::BitTime`]. It does not carry time from run to run, it only refuses. Measured
//! on synthetic captures of all 4096 values with edges one sample off: with the bit time
//! between 8 % fast and 8 % slow every reply is right; between 10 % and 30 % off, in both
//! directions, none was wrong (a bit time 20 % off is refused or foreign), see the tests.
//!
//! # What is not a reply
//!
//! - [`LineCapture::Silent`]: the line stayed high for the whole window.
//! - [`LineCapture::Foreign`]: the line is low at the first sample, or shows a low longer than
//!   3.5 nominal bits (the longest low in a reply is 3 symbols). A controller sitting in its
//!   bootloader looks like this: lows of 30 to 52 us. It is never a reply and never
//!   [`LineCapture::Invalid`]. A reply that begins in the very first sample cannot be told from
//!   a line that was already low; it is Foreign too. The real replies start 23 us or more after
//!   the end of our frame.
//! - [`LineCapture::Invalid`]: edges that look like a reply and do not decode.
//! - [`LineCapture::Reply`]: a reply that [`decode_reply`] accepted.

use crate::reply::{Reply, ReplyError, decode_reply};

/// The number of motor lines in a capture.
pub const LINES: usize = 4;

/// The most words [`decode_capture`] looks at: 1024, which is 8192 samples and 15 times the
/// window of DShot300. Words beyond that are not read.
pub const MAX_WORDS: usize = 1024;

/// Fewest samples per nominal bit the decoder works with; a smaller number is taken as this.
pub const MIN_SAMPLES_PER_BIT: u32 = 4;
/// Most samples per nominal bit; a larger number is taken as this.
pub const MAX_SAMPLES_PER_BIT: u32 = 64;

/// Symbols in a reply, the leading low one included.
const SYMBOLS: u32 = 21;
/// A reply is cut off by the window when it has fewer than this many nominal bits after its
/// first edge: 21 symbols at +8 % speed are 19.3 bits, so a whole reply always has more.
const WHOLE_REPLY_BITS: u32 = 19;
/// The toggle mask has bit `21 - s` for a toggle at symbol `s`.
const FIRST_TOGGLE: u32 = 1 << SYMBOLS;

/// What one line showed in a capture.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LineCapture {
    /// The line stayed high for the whole window: the ESC did not answer.
    Silent,
    /// A reply that [`decode_reply`] accepted.
    Reply(Reply),
    /// Edges that look like a reply and do not decode, and why.
    Invalid(CaptureError),
    /// Not a reply and not an ESC in its application: the line was low at the first sample or
    /// had a low longer than 3.5 nominal bits. A bootloader that is transmitting looks like this.
    Foreign,
}

/// Why edges that began like a reply were refused.
///
/// If several things are wrong, the first one in time is reported (the end of the window is the
/// last place).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum CaptureError {
    /// The 21 levels were made and [`decode_reply`] refused them: bad GCR code or checksum
    /// (`BadStart` cannot happen: the first symbol is the first low).
    Rejected(ReplyError),
    /// Two edges closer than half a nominal bit.
    Glitch,
    /// A run of four or more symbols inside the reply. (A low one that long is `Foreign`.)
    LongRun,
    /// More than 21 symbols: an edge after the end of the reply.
    TooManySymbols,
    /// The window ended before the reply could be complete: the line was low at its end, or
    /// fewer than 19 nominal bits after the first edge were captured, or there was no sample.
    CutOff,
    /// The edges are in order but the bit time is not: the time from the first to the last edge
    /// is more than 9.4 % (and 2 samples) away from that many nominal bits. A bit time within
    /// 8 % of nominal never gives this.
    BitTime,
}

/// Saturating counters of what one line showed, for a caller that keeps statistics.
///
/// The decoder keeps nothing between captures; this is only a place to count. Make one per line
/// and [`CaptureStats::record`] each result.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CaptureStats {
    replies: u32,
    silent: u32,
    invalid: u32,
    foreign: u32,
}

impl CaptureStats {
    /// Counters at zero.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            replies: 0,
            silent: 0,
            invalid: 0,
            foreign: 0,
        }
    }

    /// Counts one result; a counter that reaches `u32::MAX` stays there.
    pub fn record(&mut self, result: &LineCapture) {
        let counter = match result {
            LineCapture::Reply(_) => &mut self.replies,
            LineCapture::Silent => &mut self.silent,
            LineCapture::Invalid(_) => &mut self.invalid,
            LineCapture::Foreign => &mut self.foreign,
        };
        *counter = counter.saturating_add(1);
    }

    /// Captures in which the line had a reply.
    #[must_use]
    pub const fn replies(&self) -> u32 {
        self.replies
    }

    /// Captures in which the line stayed high.
    #[must_use]
    pub const fn silent(&self) -> u32 {
        self.silent
    }

    /// Captures in which the line had edges that did not decode.
    #[must_use]
    pub const fn invalid(&self) -> u32 {
        self.invalid
    }

    /// Captures in which the line was foreign.
    #[must_use]
    pub const fn foreign(&self) -> u32 {
        self.foreign
    }

    /// All captures counted, saturating.
    #[must_use]
    pub const fn total(&self) -> u32 {
        self.replies
            .saturating_add(self.silent)
            .saturating_add(self.invalid)
            .saturating_add(self.foreign)
    }
}

/// `ceil(65536 / n)` for `n` from [`MIN_SAMPLES_PER_BIT`] to [`MAX_SAMPLES_PER_BIT`]: the
/// reciprocal that replaces the division of a run by the bit time. Exact for every run the
/// decoder rounds (below 4 bits), see the test.
const RECIPROCAL: [u32; (MAX_SAMPLES_PER_BIT - MIN_SAMPLES_PER_BIT + 1) as usize] = {
    let mut table = [0; (MAX_SAMPLES_PER_BIT - MIN_SAMPLES_PER_BIT + 1) as usize];
    let mut n = MIN_SAMPLES_PER_BIT;
    while n <= MAX_SAMPLES_PER_BIT {
        // bound: n - MIN is below the table length by the loop condition.
        #[allow(clippy::indexing_slicing)]
        {
            table[(n - MIN_SAMPLES_PER_BIT) as usize] = 65_536_u32.div_ceil(n);
        }
        n += 1;
    }
    table
};

/// The numbers derived from the samples per bit, made once per capture.
#[derive(Clone, Copy)]
struct Bit {
    /// Samples per nominal bit, clamped.
    n: u32,
    /// Added to a run before the reciprocal: half a bit, which rounds half up.
    half: u32,
    /// `ceil(65536 / n)`.
    recip: u32,
}

impl Bit {
    fn new(samples_per_bit: u32) -> Self {
        let n = samples_per_bit.clamp(MIN_SAMPLES_PER_BIT, MAX_SAMPLES_PER_BIT);
        // bound: n is clamped into the table range.
        #[allow(clippy::indexing_slicing)]
        let recip = RECIPROCAL[(n - MIN_SAMPLES_PER_BIT) as usize];
        Self {
            n,
            half: n >> 1,
            recip,
        }
    }

    /// A run in samples as a number of symbols, rounded half up. Exact below 4 bits and never
    /// smaller than the exact value above (the run is below 2^13 samples, so no overflow).
    #[inline(always)]
    fn symbols(self, run: u32) -> u32 {
        ((run + self.half) * self.recip) >> 16
    }

    /// True for a low of more than 3.5 nominal bits.
    #[inline(always)]
    fn is_foreign_low(self, run: u32) -> bool {
        2 * run > 7 * self.n
    }
}

/// The hot state of a line: the sample of its last edge, and the toggle mask (0 before its first
/// edge). Two words, so that they load and store together.
#[derive(Clone, Copy)]
struct Hot {
    prev: u32,
    toggles: u32,
}

/// Everything `scan` fills in.
struct State {
    hot: [Hot; LINES],
    cold: Cold,
}

/// What only the cold paths touch.
struct Cold {
    /// Sample of the first edge, per slot.
    first: [u32; LINES],
    /// The first fault in time, per slot.
    fault: [Option<CaptureError>; LINES],
    /// One bit per slot: the line is foreign.
    foreign: u32,
}

/// Reads all edges of a capture into `state`. The slot of line `l` is `3 - l`: the leading-zero
/// count of the change mask is `4 * j + (3 - l)` for line `l` in sample `j` of the word.
#[inline(never)]
fn scan(words: &[u32], bit: Bit, state: &mut State) {
    // All lines high before the first sample.
    let mut before = u32::MAX;
    let mut base = 0u32;
    for &word in words {
        let mut changed = word ^ ((word >> 4) | (before << 28));
        before = word;
        while changed != 0 {
            let q = changed.leading_zeros();
            changed ^= 0x8000_0000 >> q;
            let slot = (q & 3) as usize;
            let sample = base + (q >> 2);
            // bound: slot is masked to 0..4 = hot.len().
            #[allow(clippy::indexing_slicing)]
            let line = &mut state.hot[slot];
            let symbols = bit.symbols(sample - line.prev);
            // The toggle at the previous edge is the lowest bit of the mask; this edge is
            // `symbols` places below it. It is gone below bit 0 after the 21st symbol. With
            // no previous edge the mask is 0, and so is this.
            let next = (line.toggles & line.toggles.wrapping_neg()).wrapping_shr(symbols);
            if symbols.wrapping_sub(1) < 3 && next != 0 {
                line.toggles |= next;
                line.prev = sample;
                continue;
            }
            edge_cold(state, slot, sample, (word << q) >> 31 == 1, bit.n);
        }
        base += 8;
    }
}

/// The edges that are not a run of 1 to 3 symbols inside a reply: the first edge of a line, a
/// glitch, a long run, a low that makes the line foreign, an edge after the 21st symbol.
#[cold]
#[inline(never)]
// bound: slot is masked to 0..4 = LINES by the caller.
#[allow(clippy::indexing_slicing)]
fn edge_cold(state: &mut State, slot: usize, sample: u32, rising: bool, n: u32) {
    let bit = Bit::new(n);
    let line = &mut state.hot[slot];
    let cold = &mut state.cold;
    if line.toggles == 0 {
        // The first edge: the start of a reply, or of a line that was low from the start.
        line.toggles = FIRST_TOGGLE;
        cold.first[slot] = sample;
        if sample == 0 {
            cold.foreign |= 1 << slot;
        }
    } else {
        let run = sample - line.prev;
        if rising && bit.is_foreign_low(run) {
            cold.foreign |= 1 << slot;
        } else if cold.fault[slot].is_none() {
            cold.fault[slot] = Some(match bit.symbols(run) {
                0 => CaptureError::Glitch,
                1..=3 => CaptureError::TooManySymbols,
                _ => CaptureError::LongRun,
            });
        }
    }
    line.prev = sample;
}

/// The 21 levels from the toggle mask: a prefix XOR from the top says whether the line is low.
const fn levels_of(toggles: u32) -> u32 {
    let mut x = toggles;
    x ^= x >> 1;
    x ^= x >> 2;
    x ^= x >> 4;
    x ^= x >> 8;
    x ^= x >> 16;
    (!x >> 1) & 0x1F_FFFF
}

/// Decodes the capture of one reply window into a result for each of the four lines (index `l`
/// is line `l`, GPIO36 + `l`).
///
/// `words` are the RX FIFO words in the order they were received, see the module documentation
/// for the layout; at most [`MAX_WORDS`] are read. `samples_per_bit` is the number of samples in
/// one nominal reply bit, [`Rate::reply_samples_per_bit`](crate::Rate::reply_samples_per_bit)
/// (16 at both rates); a number outside [`MIN_SAMPLES_PER_BIT`] to [`MAX_SAMPLES_PER_BIT`] is
/// taken as the nearest of them. An empty `words` gives [`CaptureError::CutOff`] on every line.
///
/// Integer arithmetic only, no division, no allocation, no loop over single samples, no panic.
#[must_use]
pub fn decode_capture(words: &[u32], samples_per_bit: u32) -> [LineCapture; LINES] {
    let words = words.get(..MAX_WORDS).unwrap_or(words);
    let Some(&last) = words.last() else {
        return [LineCapture::Invalid(CaptureError::CutOff); LINES];
    };
    let bit = Bit::new(samples_per_bit);
    let mut state = State {
        hot: [Hot {
            prev: 0,
            toggles: 0,
        }; LINES],
        cold: Cold {
            first: [0; LINES],
            fault: [None; LINES],
            foreign: 0,
        },
    };
    scan(words, bit, &mut state);
    let State { hot, cold } = state;
    let samples = words.len() as u32 * 8;
    core::array::from_fn(|line| {
        let slot = LINES - 1 - line;
        // bound: slot is below LINES for every line below LINES.
        #[allow(clippy::indexing_slicing)]
        let (hot, first, fault) = (hot[slot], cold.first[slot], cold.fault[slot]);
        classify(
            hot,
            first,
            fault,
            cold.foreign >> slot & 1 != 0,
            (last >> line) & 1 == 0,
            samples,
            bit,
        )
    })
}

/// The result of one line once all edges are in.
#[inline(never)]
fn classify(
    hot: Hot,
    first: u32,
    fault: Option<CaptureError>,
    foreign: bool,
    low_at_end: bool,
    samples: u32,
    bit: Bit,
) -> LineCapture {
    if foreign {
        return LineCapture::Foreign;
    }
    if hot.toggles == 0 {
        return LineCapture::Silent;
    }
    if low_at_end && bit.is_foreign_low(samples - hot.prev) {
        return LineCapture::Foreign;
    }
    if let Some(fault) = fault {
        return LineCapture::Invalid(fault);
    }
    if low_at_end || samples < first + WHOLE_REPLY_BITS * bit.n {
        return LineCapture::Invalid(CaptureError::CutOff);
    }
    // The last toggle is the lowest bit of the mask, at symbol 21 minus its place. The time
    // from the first edge to there must be about that many nominal bits: this refuses a reply
    // whose bit time is far off even where the runs round to levels that pass the checksum.
    let symbols = SYMBOLS - hot.toggles.trailing_zeros();
    let ideal = symbols * bit.n;
    let slack = (ideal >> 4) + (ideal >> 5) + 2;
    let span = hot.prev - first;
    if span + slack < ideal || span > ideal + slack {
        return LineCapture::Invalid(CaptureError::BitTime);
    }
    match decode_reply(levels_of(hot.toggles)) {
        Ok(reply) => LineCapture::Reply(reply),
        Err(error) => LineCapture::Invalid(CaptureError::Rejected(error)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reply::{encode_reply, encode_word};
    use crate::{Erpm, Rate};
    use proptest::prelude::*;
    use std::format;
    use std::vec;
    use std::vec::Vec;

    // ---------------------------------------------------------------------------------------
    // The synthetic capture: a reply, as true edge times, as the samples of one line, and the
    // four lines packed like the probe packs them.

    /// The 21 levels of a reply given the value it carries.
    fn levels_for(value: u16) -> u32 {
        encode_reply(value)
    }

    /// The `Reply` that `decode_reply` gives for `value`.
    fn reply_for(value: u16) -> LineCapture {
        LineCapture::Reply(crate::decode_reply(encode_reply(value)).unwrap())
    }

    /// True times of the edges, in samples, of a reply with the given 21 levels that starts at
    /// `start` and has `bit` samples per symbol: the line is high before, low in symbol 0, and
    /// high again after symbol 20.
    fn reply_edges(levels: u32, start: f64, bit: f64) -> Vec<f64> {
        let mut edges = Vec::new();
        let mut before = true;
        for symbol in 0..=21u32 {
            let level = symbol == 21 || (levels >> (20 - symbol)) & 1 == 1;
            if level != before {
                edges.push(start + f64::from(symbol) * bit);
            }
            before = level;
        }
        edges
    }

    /// How the sampler sees an edge that really is at time `t` (in samples).
    #[derive(Clone, Copy, Debug)]
    enum Seen {
        /// The first sample at or after `t`: what an ideal sampler sees.
        Ideal,
        /// Some whole sample within one sample of `t`, chosen by the number.
        Within1(u8),
        /// The least (-1) or the most (+1) sample within one sample of `t`.
        Extreme(i8),
        /// The ideal sample plus -1, 0 or +1.
        IdealPlus(i8),
    }

    fn seen(t: f64, how: Seen) -> usize {
        let at = |v: f64| v as usize;
        match how {
            Seen::Ideal => at(t.ceil()),
            Seen::Within1(r) => {
                let lo = (t - 1.0).ceil() as i64;
                let hi = (t + 1.0).floor() as i64;
                (lo + i64::from(r) % (hi - lo + 1)) as usize
            }
            Seen::Extreme(sign) => {
                if sign < 0 {
                    at((t - 1.0).ceil())
                } else {
                    at((t + 1.0).floor())
                }
            }
            Seen::IdealPlus(j) => (t.ceil() as i64 + i64::from(j)) as usize,
        }
    }

    /// The level of each of `samples` samples of a line that is high until the first edge and
    /// toggles at each of the sample indices `edges`.
    fn render(edges: &[usize], samples: usize) -> Vec<bool> {
        let mut level = true;
        let mut next = 0;
        (0..samples)
            .map(|s| {
                while next < edges.len() && edges[next] == s {
                    level = !level;
                    next += 1;
                }
                level
            })
            .collect()
    }

    /// Four lines of samples as RX words, as the PIO program makes them: eight samples a word,
    /// the oldest in the top nibble, line `l` in bit `l` of the nibble.
    fn pack(lines: &[Vec<bool>; LINES]) -> Vec<u32> {
        let samples = lines[0].len();
        assert_eq!(samples % 8, 0);
        let mut words = vec![0u32; samples / 8];
        for (l, line) in lines.iter().enumerate() {
            assert_eq!(line.len(), samples);
            for (s, &high) in line.iter().enumerate() {
                words[s / 8] |= u32::from(high) << (28 - 4 * (s % 8) + l);
            }
        }
        words
    }

    /// The edge samples of each line the way the probe's `extract` reads the words: the sample
    /// index is `word * 8 + (31 - p) / 4` and the line is `p & 3`, for every bit `p` set in
    /// `w ^ (w >> 4 | previous << 28)`, taken from the top.
    fn probe_extract(words: &[u32]) -> [Vec<u32>; LINES] {
        let mut out: [Vec<u32>; LINES] = Default::default();
        let mut prev: u32 = 0xF;
        for (wi, &w) in words.iter().enumerate() {
            let before = (w >> 4) | (prev << 28);
            let mut d = w ^ before;
            prev = w & 0xF;
            while d != 0 {
                let p = 31 - d.leading_zeros();
                d &= !(1 << p);
                let sample = wi as u32 * 8 + (31 - p) / 4;
                out[(p & 3) as usize].push(sample);
            }
        }
        out
    }

    /// One line of a capture: a reply or nothing.
    #[derive(Clone, Debug)]
    struct Spec {
        levels: Option<u32>,
        start: f64,
        bit: f64,
        seen: Seen,
    }

    /// The words of a capture of `words` words with one `Spec` per line, edges seen as `Spec`
    /// says.
    fn capture(specs: &[Spec; LINES], words: usize) -> Vec<u32> {
        let samples = words * 8;
        let lines: [Vec<bool>; LINES] = core::array::from_fn(|l| {
            let spec = &specs[l];
            let edges: Vec<usize> = spec
                .levels
                .map(|levels| {
                    reply_edges(levels, spec.start, spec.bit)
                        .into_iter()
                        .map(|t| seen(t, spec.seen))
                        .collect()
                })
                .unwrap_or_default();
            render(&edges, samples)
        });
        pack(&lines)
    }

    /// A line made of the given edge sample indices.
    fn line_of(edges: &[usize], samples: usize) -> Vec<bool> {
        render(edges, samples)
    }

    fn quiet(samples: usize) -> Vec<bool> {
        vec![true; samples]
    }

    fn spec(value: u16, start: f64, bit: f64) -> Spec {
        Spec {
            levels: Some(levels_for(value)),
            start,
            bit,
            seen: Seen::Ideal,
        }
    }

    const NONE: Spec = Spec {
        levels: None,
        start: 0.0,
        bit: 0.0,
        seen: Seen::Ideal,
    };

    // ---------------------------------------------------------------------------------------
    // A plain reference decoder, sample by sample, for the differential test.

    fn reference(words: &[u32], spb: u32) -> [LineCapture; LINES] {
        let n = spb.clamp(MIN_SAMPLES_PER_BIT, MAX_SAMPLES_PER_BIT) as usize;
        let words = &words[..words.len().min(MAX_WORDS)];
        if words.is_empty() {
            return [LineCapture::Invalid(CaptureError::CutOff); LINES];
        }
        let samples = words.len() * 8;
        core::array::from_fn(|line| {
            let level: Vec<bool> = (0..samples)
                .map(|s| (words[s / 8] >> (28 - 4 * (s % 8) + line)) & 1 == 1)
                .collect();
            let mut edges = Vec::new();
            let mut before = true;
            for (s, &l) in level.iter().enumerate() {
                if l != before {
                    edges.push(s);
                }
                before = l;
            }
            let mut foreign = edges.first() == Some(&0);
            for (i, w) in edges.windows(2).enumerate() {
                // Edge i + 1 is a rising one when i + 1 is odd: the run before it was low.
                if (i + 1) % 2 == 1 && 2 * (w[1] - w[0]) > 7 * n {
                    foreign = true;
                }
            }
            let low_at_end = !level[samples - 1];
            if low_at_end && 2 * (samples - edges[edges.len() - 1]) > 7 * n {
                foreign = true;
            }
            if foreign {
                return LineCapture::Foreign;
            }
            if edges.is_empty() {
                return LineCapture::Silent;
            }
            let mut fault = None;
            let mut positions = vec![0usize];
            for w in edges.windows(2) {
                let symbols = (w[1] - w[0] + n / 2) / n;
                let last = *positions.last().unwrap();
                let error = match symbols {
                    0 => Some(CaptureError::Glitch),
                    1..=3 if last + symbols > 21 => Some(CaptureError::TooManySymbols),
                    1..=3 => None,
                    _ => Some(CaptureError::LongRun),
                };
                if let Some(e) = error {
                    fault.get_or_insert(e);
                    positions.push(last);
                } else {
                    positions.push(last + symbols);
                }
            }
            if let Some(e) = fault {
                return LineCapture::Invalid(e);
            }
            if low_at_end || samples < edges[0] + 19 * n {
                return LineCapture::Invalid(CaptureError::CutOff);
            }
            let last = *positions.last().unwrap();
            let ideal = last * n;
            let slack = (ideal >> 4) + (ideal >> 5) + 2;
            let span = edges[edges.len() - 1] - edges[0];
            if span + slack < ideal || span > ideal + slack {
                return LineCapture::Invalid(CaptureError::BitTime);
            }
            let mut levels = 0u32;
            for symbol in 0..21 {
                let toggles = positions.iter().filter(|&&p| p <= symbol).count();
                // Odd number of toggles: low.
                levels = (levels << 1) | u32::from(toggles % 2 == 0);
            }
            match crate::decode_reply(levels) {
                Ok(reply) => LineCapture::Reply(reply),
                Err(e) => LineCapture::Invalid(CaptureError::Rejected(e)),
            }
        })
    }

    // ---------------------------------------------------------------------------------------
    // Fixed cases

    #[test]
    fn decode_capture_all_lines_silent_is_silent_on_every_line() {
        for words in [1, 8, 68, 91, MAX_WORDS] {
            let capture = vec![u32::MAX; words];
            assert_eq!(decode_capture(&capture, 16), [LineCapture::Silent; LINES]);
            assert_eq!(decode_capture(&capture, 8), [LineCapture::Silent; LINES]);
        }
    }

    #[test]
    fn decode_capture_four_different_values_on_four_lines_in_one_capture() {
        // eRPM 500 us, stopped, temperature 45 C, the enable acknowledge: different timing each.
        let values = [0x1F4u16, 0xFFF, 0x22D, 0xE00];
        let starts = [100.0, 103.4, 98.2, 101.9];
        let bits = [15.45, 15.40, 15.9, 16.4];
        let specs: [Spec; LINES] = core::array::from_fn(|l| spec(values[l], starts[l], bits[l]));
        let words = capture(&specs, 68);
        let got = decode_capture(&words, 16);
        for (l, &v) in values.iter().enumerate() {
            assert_eq!(got[l], reply_for(v), "line {l}");
        }
        assert_ne!(got[0], got[1]);
    }

    #[test]
    fn decode_capture_lines_are_independent_a_silent_one_next_to_replies() {
        let specs = [
            spec(0xFFF, 120.0, 15.4),
            NONE,
            spec(0x123, 130.5, 15.4),
            NONE,
        ];
        let got = decode_capture(&capture(&specs, 68), 16);
        assert_eq!(got[0], reply_for(0xFFF));
        assert_eq!(got[1], LineCapture::Silent);
        assert_eq!(got[2], reply_for(0x123));
        assert_eq!(got[3], LineCapture::Silent);
    }

    #[test]
    fn decode_capture_the_stopped_motor_with_the_timing_measured_on_2026_11_12() {
        // Probe, m1 note: the reply starts 26.0 us (DShot300) or 23.5 us (DShot600) after our
        // last rising edge, the bit takes 2572 ns or 1285 ns. The first sample is 12 PIO cycles
        // after that edge and one sample is 2 PIO cycles. At 12 MHz and 24 MHz, in samples:
        // start (26.0 us * 12 MHz - 12) / 2 = 150.0, and 2572 ns / (2 / 12 MHz) = 15.43;
        // (23.5 us * 24 MHz - 12) / 2 = 276.0, and 1285 ns / (2 / 24 MHz) = 15.42.
        for (rate, start_us, bit_ns, cycles_per_us) in [
            (Rate::Dshot300, 26.0, 2572.0, 12.0),
            (Rate::Dshot600, 23.5, 1285.0, 24.0),
        ] {
            let sample_ns = 2000.0 / cycles_per_us;
            let start = (start_us * cycles_per_us - 12.0) / 2.0;
            let bit = bit_ns / sample_ns;
            assert!((bit - 15.43_f64).abs() < 0.02, "{bit}");
            // The four lines start up to 0.67 us apart.
            let spread = [0.0, 0.15, 0.4, 0.67];
            for seen in [
                Seen::Ideal,
                Seen::Within1(0),
                Seen::Within1(1),
                Seen::Extreme(1),
            ] {
                let specs: [Spec; LINES] = core::array::from_fn(|l| Spec {
                    seen,
                    ..spec(0xFFF, start + spread[l] * 1000.0 / sample_ns, bit)
                });
                let words = capture(&specs, rate.capture_words() as usize);
                let got = decode_capture(&words, rate.reply_samples_per_bit());
                assert_eq!(got, [reply_for(0xFFF); LINES], "{rate:?} {seen:?}");
                let LineCapture::Reply(crate::Reply::Erpm(e)) = got[0] else {
                    panic!("not eRPM")
                };
                assert_eq!(e, crate::decode_reply(0x52951).map(erpm_of).unwrap());
                assert!(e.is_stopped());
            }
        }
    }

    fn erpm_of(r: crate::Reply) -> Erpm {
        match r {
            crate::Reply::Erpm(e) => e,
            crate::Reply::Extended(_) => panic!("extended"),
        }
    }

    #[test]
    fn decode_capture_8_samples_per_bit_decodes_the_same_values() {
        for (bit, seen) in [
            (7.4, Seen::Within1(0)),
            (8.0, Seen::Ideal),
            (8.6, Seen::Extreme(1)),
        ] {
            let values = [0x1F4u16, 0xFFF, 0x22D, 0xE00];
            let specs: [Spec; LINES] = core::array::from_fn(|l| Spec {
                seen,
                ..spec(values[l], 40.0 + l as f64 * 1.7, bit)
            });
            let got = decode_capture(&capture(&specs, 40), 8);
            for (l, &v) in values.iter().enumerate() {
                assert_eq!(got[l], reply_for(v), "bit {bit} line {l}");
            }
        }
    }

    #[test]
    fn decode_capture_a_reply_cut_off_by_the_window_end_is_cutoff_or_still_right() {
        // The same reply with the window ending after every possible number of words.
        for (bit, spb) in [(15.4, 16u32), (8.2, 8)] {
            for value in [0xFFFu16, 0x1F4, 0x22D, 0x123] {
                let specs = [spec(value, 20.0, bit), NONE, NONE, NONE];
                let full = capture(&specs, 68);
                let first = 20 + 1;
                for keep in 1..=68usize {
                    let got = decode_capture(&full[..keep], spb)[0];
                    let samples = keep * 8;
                    if samples < first + 19 * spb as usize {
                        // Not yet 19 bits after the first edge: always reported as cut off.
                        let want = if keep * 8 <= first {
                            LineCapture::Silent
                        } else {
                            LineCapture::Invalid(CaptureError::CutOff)
                        };
                        assert_eq!(got, want, "value {value:#x} words {keep}");
                    } else {
                        // Later: a cut can only take away what the checksum notices, or nothing.
                        assert!(
                            got == reply_for(value) || matches!(got, LineCapture::Invalid(_)),
                            "value {value:#x} words {keep}: {got:?}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn decode_capture_a_window_that_ends_in_a_low_is_cutoff() {
        // The window ends while the line is low, in a run of two symbols.
        let mut edges = Vec::new();
        for i in 0..6 {
            edges.push(100 + 16 * i);
        }
        let lines = [
            quiet(176),
            line_of(&edges, 176),
            line_of(&edges[..5], 176),
            quiet(176),
        ];
        let words = pack(&lines);
        let got = decode_capture(&words[..22], 16);
        assert_eq!(got[1], LineCapture::Invalid(CaptureError::CutOff));
        assert_eq!(got[2], LineCapture::Invalid(CaptureError::CutOff));
        assert_eq!(got[0], LineCapture::Silent);
    }

    #[test]
    fn decode_capture_an_empty_capture_is_cutoff_on_every_line() {
        assert_eq!(
            decode_capture(&[], 16),
            [LineCapture::Invalid(CaptureError::CutOff); LINES]
        );
    }

    #[test]
    fn decode_capture_reads_at_most_max_words() {
        let mut words = vec![u32::MAX; MAX_WORDS + 50];
        // A reply on line 0 that starts after the last word read.
        let specs = [
            spec(0xFFF, 8.0 * MAX_WORDS as f64 + 20.0, 15.4),
            NONE,
            NONE,
            NONE,
        ];
        let beyond = capture(&specs, MAX_WORDS + 50);
        words.copy_from_slice(&beyond);
        assert_eq!(decode_capture(&words, 16), [LineCapture::Silent; LINES]);
        // And one inside the first MAX_WORDS words is read.
        let specs = [
            spec(0xFFF, 8.0 * (MAX_WORDS - 40) as f64, 15.4),
            NONE,
            NONE,
            NONE,
        ];
        let inside = capture(&specs, MAX_WORDS + 50);
        assert_eq!(decode_capture(&inside, 16)[0], reply_for(0xFFF));
    }

    #[test]
    fn decode_capture_takes_samples_per_bit_outside_the_range_as_the_nearest_end() {
        let specs = [spec(0x1F4, 50.0, 4.3), spec(0xFFF, 50.0, 63.0), NONE, NONE];
        let words = capture(&specs, 200);
        assert_eq!(decode_capture(&words, 0), decode_capture(&words, 4));
        assert_eq!(decode_capture(&words, 3), decode_capture(&words, 4));
        assert_eq!(decode_capture(&words, 65), decode_capture(&words, 64));
        assert_eq!(decode_capture(&words, u32::MAX), decode_capture(&words, 64));
        assert_eq!(decode_capture(&words, 64)[1], reply_for(0xFFF));
    }

    #[test]
    fn decode_capture_foreign_a_line_low_at_the_first_sample() {
        // Any length of low from the first sample, then high, then even a good reply.
        for low in [1usize, 2, 5, 16, 60, 300] {
            let mut edges = vec![0, low];
            edges.extend(
                reply_edges(levels_for(0xFFF), 400.0, 15.4)
                    .into_iter()
                    .map(|t| t.ceil() as usize),
            );
            let lines = [
                line_of(&edges, 800),
                quiet(800),
                line_of(&edges[2..], 800),
                quiet(800),
            ];
            let got = decode_capture(&pack(&lines), 16);
            assert_eq!(got[0], LineCapture::Foreign, "low {low}");
            assert_eq!(got[1], LineCapture::Silent);
            assert_eq!(got[2], reply_for(0xFFF));
        }
    }

    #[test]
    fn decode_capture_foreign_the_idle_capture_of_the_lead_run_with_all_lines_low() {
        // Log dshot_m1_probe-20261112-134953.log line 11: "idle capture 150183 ns (11264
        // samples, no frame sent): edges 1 1 1 1, first sample 0x0". Four lines low at the first
        // sample and one edge each (their place is not in the log: here 30, 90, 700, 5000).
        let rises = [30usize, 90, 700, 5000];
        let lines: [Vec<bool>; LINES] = core::array::from_fn(|l| line_of(&[0, rises[l]], 11264));
        let words = pack(&lines);
        assert_eq!(words.len(), 1408);
        assert_eq!(words[0] >> 28, 0);
        assert_eq!(decode_capture(&words, 16), [LineCapture::Foreign; LINES]);
    }

    #[test]
    fn decode_capture_foreign_a_bootloader_low_of_52_us_at_both_rates() {
        // Lows of 30 to 52 us (m1 note) at 6 and 12 samples per us.
        for (samples_per_us, window) in [(6.0, 68usize), (12.0, 91)] {
            for low_us in [30.0, 40.0, 52.0] {
                let low = (low_us * samples_per_us) as usize;
                let samples = window * 8;
                let lines = [
                    line_of(&[40, 40 + low], samples),
                    quiet(samples),
                    line_of(&[40, 40 + low, 40 + low + 20, 40 + low + 60], samples),
                    quiet(samples),
                ];
                let got = decode_capture(&pack(&lines), 16);
                assert_eq!(got[0], LineCapture::Foreign);
                assert_eq!(got[2], LineCapture::Foreign);
                assert_eq!(got[1], LineCapture::Silent);
            }
        }
    }

    #[test]
    fn decode_capture_foreign_starts_above_3_5_bits_of_low_and_not_at_it() {
        // A low of exactly 3.5 bits is 4 symbols rounded: not a reply, but not foreign.
        for n in [16usize, 8] {
            let run = 7 * n / 2;
            let at = |low: usize| {
                let lines = [
                    line_of(&[50, 50 + low], 400),
                    quiet(400),
                    quiet(400),
                    quiet(400),
                ];
                decode_capture(&pack(&lines), n as u32)[0]
            };
            // Three symbols of low and then nothing: a run that is allowed, a reply that is not.
            assert!(matches!(at(run - 1), LineCapture::Invalid(_)), "n {n}");
            assert_eq!(
                at(run),
                LineCapture::Invalid(CaptureError::LongRun),
                "n {n}"
            );
            assert_eq!(at(run + 1), LineCapture::Foreign, "n {n}");
        }
    }

    #[test]
    fn decode_capture_every_line_capture_variant_is_produced() {
        let silent = decode_capture(&[u32::MAX; 4], 16)[0];
        let reply = decode_capture(
            &capture(&[spec(0xFFF, 30.0, 15.4), NONE, NONE, NONE], 60),
            16,
        )[0];
        let invalid = decode_capture(&[], 16)[0];
        let foreign = decode_capture(&[0, u32::MAX], 16)[0];
        assert!(matches!(silent, LineCapture::Silent));
        assert!(matches!(reply, LineCapture::Reply(_)));
        assert!(matches!(invalid, LineCapture::Invalid(_)));
        assert!(matches!(foreign, LineCapture::Foreign));
    }

    /// 21 levels with a run of at most three symbols in a row that `decode_reply` refuses with
    /// `error`.
    fn refused_levels(error: crate::ReplyError) -> u32 {
        for value in 0..0x1000u16 {
            for word in [
                (value << 4) | crate::value::checksum(value),
                (value << 4) | (crate::value::checksum(value) ^ 0x5),
            ] {
                for flip in 0..20 {
                    let levels = encode_word(word) ^ (1 << flip);
                    if crate::decode_reply(levels) == Err(error) && max_run(levels) <= 3 {
                        return levels;
                    }
                }
            }
        }
        panic!("none")
    }

    /// The longest run of equal levels among the 21 symbols and the idle high that follows.
    fn max_run(levels: u32) -> u32 {
        let mut best = 0;
        let mut run = 0;
        let mut before = None;
        for symbol in 0..=21u32 {
            let level = symbol == 21 || (levels >> (20 - symbol)) & 1 == 1;
            run = if before == Some(level) { run + 1 } else { 1 };
            if symbol < 21 {
                best = best.max(run);
            }
            before = Some(level);
        }
        best
    }

    #[test]
    fn decode_capture_every_capture_error_variant_is_produced() {
        let one = |levels: u32| {
            let specs = [
                Spec {
                    levels: Some(levels),
                    start: 60.0,
                    bit: 15.4,
                    seen: Seen::Ideal,
                },
                NONE,
                NONE,
                NONE,
            ];
            decode_capture(&capture(&specs, 68), 16)[0]
        };
        // Rejected by decode_reply: a bad code, a bad checksum.
        let bad_gcr = refused_levels(crate::ReplyError::BadGcr);
        assert_eq!(
            one(bad_gcr),
            LineCapture::Invalid(CaptureError::Rejected(crate::ReplyError::BadGcr))
        );
        let bad_sum = refused_levels(crate::ReplyError::BadChecksum);
        assert_eq!(
            one(bad_sum),
            LineCapture::Invalid(CaptureError::Rejected(crate::ReplyError::BadChecksum))
        );
        // The rest from edge lists, 16 samples to the symbol.
        let on = |edges: &[usize]| {
            let lines = [line_of(edges, 544), quiet(544), quiet(544), quiet(544)];
            decode_capture(&pack(&lines), 16)[0]
        };
        // Glitch: a pulse of 2 samples.
        assert_eq!(
            on(&[100, 102, 140, 156]),
            LineCapture::Invalid(CaptureError::Glitch)
        );
        // LongRun: five symbols high inside the reply.
        assert_eq!(
            on(&[100, 116, 196, 212]),
            LineCapture::Invalid(CaptureError::LongRun)
        );
        // TooManySymbols: edges every symbol, 24 of them, the 23rd is after the 21st symbol.
        let many: Vec<usize> = (0..24).map(|i| 100 + 16 * i).collect();
        assert_eq!(
            on(&many),
            LineCapture::Invalid(CaptureError::TooManySymbols)
        );
        // BitTime: a good reply 14 % too slow.
        let slow = Spec {
            levels: Some(levels_for(0x1F4)),
            start: 60.0,
            bit: 1.14 * 16.0,
            seen: Seen::Ideal,
        };
        assert_eq!(
            decode_capture(&capture(&[slow, NONE, NONE, NONE], 68), 16)[0],
            LineCapture::Invalid(CaptureError::BitTime)
        );
        // CutOff: a reply whose window ends in the middle of it.
        let specs = [spec(0xFFF, 60.0, 15.4), NONE, NONE, NONE];
        let words = capture(&specs, 68);
        assert_eq!(
            decode_capture(&words[..20], 16)[0],
            LineCapture::Invalid(CaptureError::CutOff)
        );
    }

    #[test]
    fn decode_capture_is_not_fooled_by_levels_of_a_flipped_symbol() {
        // One level wrong inside the reply: refused, except for the boundary case the XOR
        // checksum cannot see, which decodes to the value `decode_reply` itself gives.
        let mut accepted = 0;
        for value in 0..0x1000u16 {
            let levels = levels_for(value);
            for flip in 0..20 {
                let bad = levels ^ (1 << flip);
                let specs = [
                    Spec {
                        levels: Some(bad),
                        start: 60.0,
                        bit: 15.4,
                        seen: Seen::Ideal,
                    },
                    NONE,
                    NONE,
                    NONE,
                ];
                let got = decode_capture(&capture(&specs, 68), 16)[0];
                match crate::decode_reply(bad) {
                    Ok(r) if max_run(bad) <= 3 => {
                        assert_eq!(got, LineCapture::Reply(r));
                        accepted += 1;
                    }
                    // A flip that makes a run of four low symbols is a foreign low (4 > 3.5).
                    _ => assert!(
                        matches!(got, LineCapture::Invalid(_) | LineCapture::Foreign),
                        "{value:#x} {flip}: {got:?}"
                    ),
                }
            }
        }
        // 384 of the 86 016 single flips pass the checksum (README rule 7); the flip of level 20
        // is not here, and a few of the 384 give a run of more than three symbols.
        assert!(accepted > 300 && accepted <= 384, "{accepted}");
    }

    #[test]
    fn decode_capture_a_bit_time_far_from_nominal_is_never_a_reply() {
        // Bit time from 30 % fast to 30 % slow, edges within one sample, a spread of values:
        // inside 8 % every reply is right; beyond 11 % none is wrong (the XOR checksum alone
        // lets about 1 % of replies through at 18 % fast); beyond 15 % none is accepted.
        for spb in [16u32, 8] {
            for percent in (70..=130).step_by(3) {
                let scale = f64::from(percent) / 100.0;
                for value in (0..0x1000u16).step_by(7) {
                    for r in 0..3u8 {
                        let seen = Seen::Within1(r.wrapping_mul(97) ^ value as u8);
                        let specs = [
                            Spec {
                                seen,
                                ..spec(value, 20.0 + f64::from(r) * 0.37, scale * f64::from(spb))
                            },
                            NONE,
                            NONE,
                            NONE,
                        ];
                        let got = decode_capture(&capture(&specs, 100), spb)[0];
                        let right = got == reply_for(value);
                        let reply = matches!(got, LineCapture::Reply(_));
                        let tag = format!("spb {spb} scale {scale} value {value:#x} {got:?}");
                        if (92..=108).contains(&percent) {
                            assert!(right, "{tag}");
                        } else if percent <= 89 || percent >= 111 {
                            assert!(!reply || right, "{tag}");
                        }
                        if percent <= 85 || percent >= 115 {
                            assert!(!reply, "{tag}");
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn decode_capture_bit_time_error_is_produced_by_a_reply_that_is_too_slow_or_too_fast() {
        for (scale, spb) in [(0.86, 16u32), (1.13, 16), (0.86, 8), (1.13, 8)] {
            let specs = [spec(0x1F4, 30.0, scale * f64::from(spb)), NONE, NONE, NONE];
            let got = decode_capture(&capture(&specs, 100), spb)[0];
            assert_eq!(
                got,
                LineCapture::Invalid(CaptureError::BitTime),
                "{scale} {spb}"
            );
        }
    }

    // ---------------------------------------------------------------------------------------
    // CaptureStats

    #[test]
    fn capture_stats_new_is_all_zero() {
        let s = CaptureStats::new();
        assert_eq!(
            (s.replies(), s.silent(), s.invalid(), s.foreign()),
            (0, 0, 0, 0)
        );
        assert_eq!(s, CaptureStats::default());
        assert_eq!(s.total(), 0);
    }

    #[test]
    fn capture_stats_record_counts_each_kind_in_its_own_counter() {
        let mut s = CaptureStats::new();
        let reply = reply_for(0xFFF);
        let invalid = LineCapture::Invalid(CaptureError::Glitch);
        for _ in 0..5 {
            s.record(&reply);
        }
        for _ in 0..3 {
            s.record(&LineCapture::Silent);
        }
        s.record(&invalid);
        s.record(&LineCapture::Foreign);
        s.record(&LineCapture::Foreign);
        assert_eq!(
            (s.replies(), s.silent(), s.invalid(), s.foreign()),
            (5, 3, 1, 2)
        );
        assert_eq!(s.total(), 11);
    }

    #[test]
    fn capture_stats_record_saturates_at_u32_max() {
        let mut s = CaptureStats {
            replies: u32::MAX,
            silent: u32::MAX - 1,
            invalid: u32::MAX,
            foreign: u32::MAX,
        };
        s.record(&reply_for(0));
        s.record(&LineCapture::Silent);
        s.record(&LineCapture::Silent);
        s.record(&LineCapture::Invalid(CaptureError::CutOff));
        s.record(&LineCapture::Foreign);
        assert_eq!(s.replies(), u32::MAX);
        assert_eq!(s.silent(), u32::MAX);
        assert_eq!(s.invalid(), u32::MAX);
        assert_eq!(s.foreign(), u32::MAX);
        assert_eq!(s.total(), u32::MAX);
    }

    #[test]
    fn capture_stats_getters_read_the_counters() {
        let s = CaptureStats {
            replies: 1,
            silent: 2,
            invalid: 3,
            foreign: 4,
        };
        assert_eq!(
            (s.replies(), s.silent(), s.invalid(), s.foreign()),
            (1, 2, 3, 4)
        );
        assert_eq!(s.total(), 10);
    }

    // ---------------------------------------------------------------------------------------
    // The pieces

    #[test]
    fn the_test_generator_packs_samples_the_way_the_probe_reads_them() {
        // Edges put in at known samples on known lines come back from the probe's extract.
        let samples = 96;
        let edges: [Vec<usize>; LINES] = [vec![3, 20], vec![7, 8 + 7, 40], vec![], vec![0, 95]];
        let lines: [Vec<bool>; LINES] = core::array::from_fn(|l| line_of(&edges[l], samples));
        let words = pack(&lines);
        let got = probe_extract(&words);
        for l in 0..LINES {
            let want: Vec<u32> = edges[l].iter().map(|&e| e as u32).collect();
            assert_eq!(got[l], want, "line {l}");
        }
        // Spot checks of the layout: sample 0 of line 3 is bit 28 + 3 of word 0, sample 7 of
        // line 1 is bit 1, and a quiet capture is all ones.
        assert_eq!(words[0] >> 31, 0);
        assert_eq!(words[0] >> 28 & 0xF, 0b0111);
        assert_eq!(words[0] & 0b10, 0);
        assert_eq!(pack(&[quiet(8), quiet(8), quiet(8), quiet(8)]), [u32::MAX]);
    }

    #[test]
    fn levels_of_turns_the_toggle_mask_into_the_21_levels() {
        for value in 0..0x1000u16 {
            let levels = levels_for(value);
            let mut toggles = 0u32;
            let mut before = true;
            for symbol in 0..=21u32 {
                let level = symbol == 21 || (levels >> (20 - symbol)) & 1 == 1;
                if level != before {
                    toggles |= 1 << (21 - symbol);
                }
                before = level;
            }
            assert_eq!(levels_of(toggles), levels, "{value:#x}");
        }
    }

    #[test]
    fn symbols_is_the_run_over_the_bit_rounded_half_up_for_every_run_it_rounds() {
        for n in MIN_SAMPLES_PER_BIT..=MAX_SAMPLES_PER_BIT {
            let bit = Bit::new(n);
            assert_eq!(bit.n, n);
            for run in 0..(MAX_WORDS as u32 * 8) {
                let exact = (run + n / 2) / n;
                let got = bit.symbols(run);
                if exact < 4 {
                    assert_eq!(got, exact, "n {n} run {run}");
                } else {
                    assert!(got >= 4, "n {n} run {run}");
                }
            }
        }
    }

    #[test]
    fn is_foreign_low_is_a_low_of_more_than_3_5_bits() {
        for n in MIN_SAMPLES_PER_BIT..=MAX_SAMPLES_PER_BIT {
            let bit = Bit::new(n);
            for run in 0..(8 * n) {
                assert_eq!(bit.is_foreign_low(run), f64::from(run) > 3.5 * f64::from(n));
            }
        }
    }

    // ---------------------------------------------------------------------------------------
    // Invariants 2.5 and 2.10, and the sweeps behind them

    /// The words for a capture of one reply on each line: values, scales and the way edges are
    /// seen vary per line.
    fn four_replies(
        values: [u16; LINES],
        scales: [f64; LINES],
        starts: [f64; LINES],
        seen: [Seen; LINES],
        spb: u32,
        words: usize,
    ) -> Vec<u32> {
        let specs: [Spec; LINES] = core::array::from_fn(|l| Spec {
            seen: seen[l],
            ..spec(values[l], starts[l], scales[l] * f64::from(spb))
        });
        capture(&specs, words)
    }

    #[test]
    fn the_worst_case_edges_of_every_value_at_the_extreme_bit_times_decode() {
        // Invariant 2.5, by sweep: all 4096 values, bit time 8 % fast, nominal and 8 % slow,
        // two start phases, every edge as far from its true place as the model allows (one
        // sample), alternately early and late, which makes each run as wrong as it can be.
        for (spb, words) in [(16u32, 60usize), (8, 36)] {
            for value in 0..0x1000u16 {
                for scale in [0.92, 1.0, 1.08] {
                    for phase in [0.0, 0.5] {
                        for sign in [-1i8, 1] {
                            let bit = scale * f64::from(spb);
                            let levels = levels_for(value);
                            let edges = reply_edges(levels, 20.0 + phase, bit);
                            let seen_edges: Vec<usize> = edges
                                .iter()
                                .enumerate()
                                .map(|(i, &t)| {
                                    seen(t, Seen::Extreme(if i % 2 == 0 { sign } else { -sign }))
                                })
                                .collect();
                            let lines = [
                                line_of(&seen_edges, words * 8),
                                quiet(words * 8),
                                quiet(words * 8),
                                quiet(words * 8),
                            ];
                            let got = decode_capture(&pack(&lines), spb)[0];
                            assert_eq!(
                                got,
                                reply_for(value),
                                "spb {spb} value {value:#x} scale {scale} phase {phase} sign {sign}"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn an_edge_one_sample_too_far_at_16_samples_per_bit_in_the_ideal_plus_one_model_decodes() {
        // At 16 samples per bit the margin is wider: the ideal sample plus -1, 0 or +1 on
        // every edge, in every alternation, on every value.
        for value in 0..0x1000u16 {
            for scale in [0.92, 1.08] {
                for pattern in 0..3 {
                    let bit = scale * 16.0;
                    let edges = reply_edges(levels_for(value), 21.3, bit);
                    let seen_edges: Vec<usize> = edges
                        .iter()
                        .enumerate()
                        .map(|(i, &t)| {
                            let j = match pattern {
                                0 => 1,
                                1 => {
                                    if i % 2 == 0 {
                                        1
                                    } else {
                                        -1
                                    }
                                }
                                _ => {
                                    if i % 2 == 0 {
                                        -1
                                    } else {
                                        1
                                    }
                                }
                            };
                            seen(t, Seen::IdealPlus(j))
                        })
                        .collect();
                    let lines = [
                        line_of(&seen_edges, 480),
                        quiet(480),
                        quiet(480),
                        quiet(480),
                    ];
                    assert_eq!(
                        decode_capture(&pack(&lines), 16)[0],
                        reply_for(value),
                        "{value:#x} {scale} {pattern}"
                    );
                }
            }
        }
    }

    prop_compose! {
        fn line_case(spb: u32, words: usize)(
            value in 0u16..0x1000,
            present in prop::bool::weighted(0.85),
            scale in 0.92f64..=1.08,
            at in 0.0f64..1.0,
            random in any::<u8>(),
            model in 0u8..3,
        ) -> (Option<u16>, f64, f64, Seen) {
            let bit = scale * f64::from(spb);
            let room = (words * 8) as f64 - 21.0 * bit - 6.0;
            let start = 3.0 + at * (room - 3.0);
            let how = match model {
                0 => Seen::Ideal,
                1 => Seen::Within1(random),
                _ => Seen::Extreme(if random & 1 == 0 { -1 } else { 1 }),
            };
            (present.then_some(value), start, bit, how)
        }
    }

    fn check_lines(cases: &[(Option<u16>, f64, f64, Seen); LINES], spb: u32, words: usize) {
        let specs: [Spec; LINES] = core::array::from_fn(|l| {
            let (value, start, bit, how) = cases[l];
            Spec {
                levels: value.map(levels_for),
                start,
                bit,
                seen: how,
            }
        });
        let got = decode_capture(&capture(&specs, words), spb);
        for (l, (value, ..)) in cases.iter().enumerate() {
            let want = value.map_or(LineCapture::Silent, reply_for);
            assert_eq!(got[l], want, "line {l}: {:?}", cases[l]);
        }
    }

    proptest! {
        #[test]
        fn invariant_2_5_bit_time_within_8_percent_any_start_one_sample_at_16_samples_per_bit(
            cases in [line_case(16, 68), line_case(16, 68), line_case(16, 68), line_case(16, 68)],
        ) {
            check_lines(&cases, 16, 68);
        }

        #[test]
        fn invariant_2_5_bit_time_within_8_percent_any_start_one_sample_at_8_samples_per_bit(
            cases in [line_case(8, 40), line_case(8, 40), line_case(8, 40), line_case(8, 40)],
        ) {
            check_lines(&cases, 8, 40);
        }

        #[test]
        fn invariant_2_5_ideal_plus_minus_one_at_16_samples_per_bit(
            value in 0u16..0x1000,
            scale in 0.92f64..=1.08,
            at in 0.0f64..1.0,
            jitter in prop::collection::vec(-1i8..=1, 22),
        ) {
            let bit = scale * 16.0;
            let start = 3.0 + at * (68.0 * 8.0 - 21.0 * bit - 9.0);
            let edges = reply_edges(levels_for(value), start, bit);
            let seen_edges: Vec<usize> = edges
                .iter()
                .zip(&jitter)
                .map(|(&t, &j)| seen(t, Seen::IdealPlus(j)))
                .collect();
            let lines = [
                quiet(544),
                quiet(544),
                line_of(&seen_edges, 544),
                quiet(544),
            ];
            prop_assert_eq!(decode_capture(&pack(&lines), 16)[2], reply_for(value));
        }

        #[test]
        fn invariant_2_10_a_low_of_a_bootloader_is_foreign(
            spb in prop_oneof![Just(16u32), Just(8u32)],
            at in 1usize..200,
            low_bits in 3.6f64..30.0,
            bursts in prop::collection::vec((4usize..100, 1usize..40), 0..4),
        ) {
            // A low of 3.6 to 30 bits (at 16 samples a bit and 6 samples per us: 9.6 to 80 us)
            // at any place but the first sample, then some more activity.
            let low = (low_bits * f64::from(spb)).ceil() as usize;
            let mut edges = vec![at, at + low];
            let mut t = at + low;
            for (gap, width) in bursts {
                edges.push(t + gap);
                edges.push(t + gap + width);
                t += gap + width;
            }
            let samples = 8 * ((t + 40) / 8 + 1);
            let lines = [
                line_of(&edges, samples),
                quiet(samples),
                quiet(samples),
                quiet(samples),
            ];
            let got = decode_capture(&pack(&lines), spb);
            prop_assert_eq!(got[0], LineCapture::Foreign);
            prop_assert_eq!(got[1], LineCapture::Silent);
        }

        #[test]
        fn invariant_2_10_a_line_low_at_the_first_sample_is_foreign(
            spb in prop_oneof![Just(16u32), Just(8u32)],
            low in 1usize..400,
            value in 0u16..0x1000,
            with_reply in any::<bool>(),
        ) {
            let mut edges = vec![0, low];
            if with_reply {
                let after = low + 5;
                edges.extend(
                    reply_edges(levels_for(value), after as f64, 1.0 * f64::from(spb))
                        .into_iter()
                        .map(|t| t.ceil() as usize),
                );
            }
            let samples = 8 * ((edges[edges.len() - 1] + 20) / 8 + 1);
            let lines = [
                quiet(samples),
                line_of(&edges, samples),
                quiet(samples),
                quiet(samples),
            ];
            let got = decode_capture(&pack(&lines), spb);
            prop_assert_eq!(got[1], LineCapture::Foreign);
            prop_assert_eq!(got[0], LineCapture::Silent);
        }

        #[test]
        fn invariant_2_10_a_long_low_in_the_middle_of_a_reply_is_foreign(
            spb in prop_oneof![Just(16u32), Just(8u32)],
            value in 0u16..0x1000,
            scale in 0.92f64..=1.08,
            into in 0.0f64..1.0,
            low_bits in 3.6f64..20.0,
            how in prop_oneof![Just(Seen::Ideal), any::<u8>().prop_map(Seen::Within1)],
        ) {
            let bit = scale * f64::from(spb);
            let start = 40.0;
            let samples = 8 * ((start + 21.0 * bit + 22.0 * f64::from(spb)) as usize / 8 + 1);
            let edges: Vec<usize> = reply_edges(levels_for(value), start, bit)
                .into_iter()
                .map(|t| seen(t, how))
                .collect();
            let mut line = render(&edges, samples);
            // Force a low of low_bits bits from a place inside the reply (after its start).
            let from = 41 + (into * 21.0 * bit) as usize;
            let len = (low_bits * f64::from(spb)).ceil() as usize;
            for s in line.iter_mut().skip(from).take(len) {
                *s = false;
            }
            let lines = [quiet(samples), quiet(samples), quiet(samples), line];
            let got = decode_capture(&pack(&lines), spb);
            prop_assert_eq!(got[3], LineCapture::Foreign);
            prop_assert_eq!(got[0], LineCapture::Silent);
        }

        #[test]
        fn decode_capture_arbitrary_words_never_panic_and_match_the_plain_reference(
            words in prop::collection::vec(any::<u32>(), 0..90),
            spb in 0u32..80,
        ) {
            prop_assert_eq!(decode_capture(&words, spb), reference(&words, spb));
        }

        #[test]
        fn decode_capture_random_runs_match_the_plain_reference(
            runs in [
                prop::collection::vec(1usize..70, 0..40),
                prop::collection::vec(1usize..70, 0..40),
                prop::collection::vec(1usize..30, 0..60),
                prop::collection::vec(1usize..30, 0..60),
            ],
            spb in prop_oneof![Just(16u32), Just(8u32), 4u32..40],
            words in 20usize..90,
        ) {
            let samples = words * 8;
            let lines: [Vec<bool>; LINES] = core::array::from_fn(|l| {
                let mut edges = Vec::new();
                let mut t = 0usize;
                for &r in &runs[l] {
                    t += r;
                    edges.push(t + l);
                }
                edges.retain(|&e| e < samples);
                line_of(&edges, samples)
            });
            let packed = pack(&lines);
            prop_assert_eq!(decode_capture(&packed, spb), reference(&packed, spb));
        }

        #[test]
        fn decode_capture_damaged_replies_match_the_plain_reference_and_a_reply_is_only_valid_levels(
            value in 0u16..0x1000,
            scale in 0.85f64..1.15,
            at in 10.0f64..120.0,
            flips in prop::collection::vec((0usize..60, 0u32..32), 0..4),
            spb in prop_oneof![Just(16u32), Just(8u32)],
        ) {
            let specs = [
                Spec { seen: Seen::Within1(1), ..spec(value, at, scale * f64::from(spb)) },
                NONE, NONE, NONE,
            ];
            let mut words = capture(&specs, 60);
            for (w, bit) in flips {
                words[w] ^= 1 << bit;
            }
            let got = decode_capture(&words, spb);
            prop_assert_eq!(got, reference(&words, spb));
        }

        #[test]
        fn decode_capture_a_reply_with_the_timing_outside_the_bounds_is_a_reply_only_if_it_is_right(
            value in 0u16..0x1000,
            scale in 0.5f64..1.6,
            at in 10.0f64..120.0,
            random in any::<u8>(),
        ) {
            // Not a promise: outside the tolerance the levels are wrong and decode_reply
            // refuses almost all of them. The test pins that any Reply that does come out is
            // one that decode_reply accepts from the levels the runs gave (the reference).
            let specs = [
                Spec { seen: Seen::Within1(random), ..spec(value, at, scale * 16.0) },
                NONE, NONE, NONE,
            ];
            let words = capture(&specs, 100);
            let got = decode_capture(&words, 16);
            prop_assert_eq!(got, reference(&words, 16));
        }
    }

    #[test]
    fn decode_capture_arbitrary_samples_per_bit_never_panics() {
        let words = capture(&[spec(0xFFF, 30.0, 15.0), NONE, NONE, NONE], 60);
        for spb in (0..300).chain([u32::MAX, u32::MAX - 1]) {
            let _ = decode_capture(&words, spb);
        }
    }

    #[test]
    fn four_replies_helper_builds_what_the_cases_expect() {
        let words = four_replies(
            [0x1F4, 0xFFF, 0x22D, 0xE00],
            [1.0, 0.92, 1.08, 1.0],
            [50.0, 60.0, 70.0, 80.0],
            [Seen::Ideal; LINES],
            16,
            68,
        );
        let got = decode_capture(&words, 16);
        assert_eq!(got[0], reply_for(0x1F4));
        assert_eq!(got[3], reply_for(0xE00));
    }
}
