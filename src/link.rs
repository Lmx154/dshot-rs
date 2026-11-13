//! The rules of the link that the back-end enforces, as pure code: the start sequence's waits,
//! which lines are live, and when the motor-line parts may be handed back.
//!
//! The numbers come from the decision `_plan/decision-esc-link-20261112.md` (amendment after M1
//! and fourth amendment) and from the measurements M1, M2 and M8 on the bench (four AM32 2.20
//! controllers, bootloader version 1, MARV V2 Rev 1).

use crate::capture::{LINES, LineCapture};

/// `start()` holds the lines released and high this long before the break: 3000 ms.
///
/// After this long with no frame on a high line every controller has cut off and is parked,
/// silent, in its bootloader, whatever it did before. Measured in M2 (`_plan/m2-bootloader-
/// probe.md`): from not armed 1500 ms failed and 2000 ms passed with one run per value; 2000 ms
/// is the application's own cut-off, so it sits on the edge, and 3000 ms adds a full second for
/// clock tolerance and boot time (decision, fourth amendment).
pub const PARK_MS: u32 = 3000;

/// `start()` drives all four lines low together this long: 25 ms.
///
/// The AM32 bootloader starts the application after 20 ms of low line (bootloader `main.c`
/// 841-853); 25 ms worked on all four controllers in M2 and M8 (first good reply after 640 to
/// 646 frames at 1 kHz).
pub const BREAK_MS: u32 = 25;

/// What a line is since the last `start()`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LineState {
    /// No good reply yet since the last `start()`. A controller whose application is
    /// starting holds its line low for about 0.54 s after the break (M8), so captures are
    /// Foreign for a while: normal here.
    Starting,
    /// The line has given a good reply since the last `start()`. The state and the counters are
    /// reports: what a Foreign capture on a Live line means is the firmware's decision.
    Live,
}

/// Which lines are live, and the Foreign captures of each line counted apart while it was
/// Starting and while it was Live. Counters saturate and are never cleared.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LineMonitor {
    live: [bool; LINES],
    foreign_starting: [u32; LINES],
    foreign_live: [u32; LINES],
}

impl Default for LineMonitor {
    fn default() -> Self {
        Self::new()
    }
}

impl LineMonitor {
    /// Every line Starting, every counter 0.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            live: [false; LINES],
            foreign_starting: [0; LINES],
            foreign_live: [0; LINES],
        }
    }

    /// Takes the result of one capture: a Foreign capture is counted under the state the line
    /// was in before this capture, and a line becomes Live at its first good reply
    /// ([`LineCapture::Reply`]). Live stays latched whatever comes after, until
    /// [`LineMonitor::restart`]. Returns the state of each line after this capture.
    pub fn record(&mut self, captures: &[LineCapture; LINES]) -> [LineState; LINES] {
        for ((capture, live), (starting_count, live_count)) in
            captures.iter().zip(self.live.iter_mut()).zip(
                self.foreign_starting
                    .iter_mut()
                    .zip(self.foreign_live.iter_mut()),
            )
        {
            match capture {
                LineCapture::Foreign => {
                    let counter = if *live { live_count } else { starting_count };
                    *counter = counter.saturating_add(1);
                }
                LineCapture::Reply(_) => *live = true,
                LineCapture::Silent | LineCapture::Invalid(_) => {}
            }
        }
        self.states()
    }

    /// Every line back to Starting: what `start()` does. The counters are kept.
    pub fn restart(&mut self) {
        self.live = [false; LINES];
    }

    /// The state of each line.
    #[must_use]
    pub fn states(&self) -> [LineState; LINES] {
        self.live.map(|live| {
            if live {
                LineState::Live
            } else {
                LineState::Starting
            }
        })
    }

    /// Foreign captures per line while the line was Starting.
    #[must_use]
    pub const fn foreign_while_starting(&self) -> [u32; LINES] {
        self.foreign_starting
    }

    /// Foreign captures per line while the line was Live.
    #[must_use]
    pub const fn foreign_while_live(&self) -> [u32; LINES] {
        self.foreign_live
    }
}

/// Why the motor-line parts may not be handed back now.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(not(feature = "embassy-rp-0-10"), allow(dead_code))]
pub(crate) enum ReleaseRefusal {
    /// A frame or its capture is not finished (a cancelled exchange left one behind).
    InFlight,
    /// The last frame pushed was not Stop on all four lines.
    NotStopped,
}

/// Invariant 3.5: the parts go back only when nothing is in flight and the last pushed frame
/// was Stop on all four lines.
#[cfg_attr(not(feature = "embassy-rp-0-10"), allow(dead_code))]
pub(crate) const fn release_check(
    in_flight: bool,
    last_all_stop: bool,
) -> Result<(), ReleaseRefusal> {
    if in_flight {
        Err(ReleaseRefusal::InFlight)
    } else if last_all_stop {
        Ok(())
    } else {
        Err(ReleaseRefusal::NotStopped)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CaptureError, Reply, decode_reply};

    fn stopped() -> LineCapture {
        LineCapture::Reply(decode_reply(0x52951).unwrap())
    }

    #[test]
    fn constants_are_the_decided_values() {
        assert_eq!(PARK_MS, 3000);
        assert_eq!(BREAK_MS, 25);
        // The break is above the bootloader's 20 ms.
        const { assert!(BREAK_MS > 20) };
        // T_PARK is above the unarmed cut-off of 2 s.
        const { assert!(PARK_MS > 2000) };
    }

    #[test]
    fn line_monitor_new_is_all_starting_and_zero() {
        let m = LineMonitor::new();
        assert_eq!(m.states(), [LineState::Starting; LINES]);
        assert_eq!(m.foreign_while_starting(), [0; LINES]);
        assert_eq!(m.foreign_while_live(), [0; LINES]);
        assert_eq!(LineMonitor::default(), m);
    }

    #[test]
    fn line_monitor_record_latches_live_at_the_first_good_reply_only() {
        let mut m = LineMonitor::new();
        let silent = LineCapture::Silent;
        let invalid = LineCapture::Invalid(CaptureError::CutOff);
        let s = m.record(&[silent, invalid, LineCapture::Foreign, stopped()]);
        assert_eq!(
            s,
            [
                LineState::Starting,
                LineState::Starting,
                LineState::Starting,
                LineState::Live
            ]
        );
        // Live stays latched through silence, invalid and Foreign captures.
        let s = m.record(&[silent, silent, silent, LineCapture::Foreign]);
        assert_eq!(s[3], LineState::Live);
        let s = m.record(&[silent, silent, silent, invalid]);
        assert_eq!(s[3], LineState::Live);
        // An extended telemetry frame is a good reply too.
        let ack = LineCapture::Reply(Reply::Extended(crate::ExtendedTelemetry::EnabledAck));
        let s = m.record(&[ack, silent, silent, silent]);
        assert_eq!(s[0], LineState::Live);
        assert_eq!(m.states(), s);
    }

    #[test]
    fn line_monitor_record_counts_foreign_under_the_state_before_the_capture() {
        let mut m = LineMonitor::new();
        let f = LineCapture::Foreign;
        let q = LineCapture::Silent;
        m.record(&[f, f, q, q]);
        m.record(&[f, q, q, q]);
        assert_eq!(m.foreign_while_starting(), [2, 1, 0, 0]);
        m.record(&[stopped(), q, q, q]);
        m.record(&[f, f, q, q]);
        assert_eq!(m.foreign_while_starting(), [2, 2, 0, 0]);
        assert_eq!(m.foreign_while_live(), [1, 0, 0, 0]);
    }

    #[test]
    fn line_monitor_restart_clears_live_and_keeps_the_counters() {
        let mut m = LineMonitor::new();
        let f = LineCapture::Foreign;
        m.record(&[stopped(); LINES]);
        m.record(&[f, f, f, f]);
        m.restart();
        assert_eq!(m.states(), [LineState::Starting; LINES]);
        assert_eq!(m.foreign_while_live(), [1; LINES]);
        m.record(&[f, f, f, f]);
        assert_eq!(m.foreign_while_starting(), [1; LINES]);
        assert_eq!(m.foreign_while_live(), [1; LINES]);
    }

    #[test]
    fn line_monitor_counters_saturate() {
        let mut m = LineMonitor::new();
        m.foreign_starting = [u32::MAX - 1; LINES];
        m.foreign_live = [u32::MAX; LINES];
        m.record(&[LineCapture::Foreign; LINES]);
        m.record(&[LineCapture::Foreign; LINES]);
        assert_eq!(m.foreign_while_starting(), [u32::MAX; LINES]);
        m.live = [true; LINES];
        m.record(&[LineCapture::Foreign; LINES]);
        assert_eq!(m.foreign_while_live(), [u32::MAX; LINES]);
    }

    #[test]
    fn line_monitor_foreign_while_starting_and_while_live_read_their_own_counters() {
        let mut m = LineMonitor::new();
        m.foreign_starting = [1, 2, 3, 4];
        m.foreign_live = [5, 6, 7, 8];
        assert_eq!(m.foreign_while_starting(), [1, 2, 3, 4]);
        assert_eq!(m.foreign_while_live(), [5, 6, 7, 8]);
    }

    #[test]
    fn release_check_refuses_while_in_flight_whatever_else() {
        for last_all_stop in [false, true] {
            assert_eq!(
                release_check(true, last_all_stop),
                Err(ReleaseRefusal::InFlight)
            );
        }
    }

    #[test]
    fn release_check_refuses_after_a_frame_that_was_not_stop() {
        assert_eq!(release_check(false, false), Err(ReleaseRefusal::NotStopped));
    }

    #[test]
    fn release_check_accepts_after_stop_on_all_four() {
        assert_eq!(release_check(false, true), Ok(()));
    }
}
