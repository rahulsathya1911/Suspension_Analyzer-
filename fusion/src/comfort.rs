//! ISO 2631-1 comfort computation with Wk biquad cascade.
//!
//! PRD §5.6: Wk weighting via bilinear transform, band RMS,
//! p_phys and y_ref computation.

use std::f64::consts::PI;

/// Second-order biquad filter section.
#[derive(Debug, Clone)]
struct Biquad {
    b0: f64, b1: f64, b2: f64,
    a1: f64, a2: f64,
    // State
    x1: f64, x2: f64,
    y1: f64, y2: f64,
}

impl Biquad {
    fn new(b0: f64, b1: f64, b2: f64, a1: f64, a2: f64) -> Self {
        Self { b0, b1, b2, a1, a2, x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0 }
    }

    fn reset(&mut self) {
        self.x1 = 0.0; self.x2 = 0.0;
        self.y1 = 0.0; self.y2 = 0.0;
    }

    fn process(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1 - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

/// Design a 2nd-order low-pass biquad via bilinear transform.
fn lowpass_biquad(f0: f64, q: f64, fs: f64) -> Biquad {
    let w0 = 2.0 * PI * f0 / fs;
    let alpha = w0.sin() / (2.0 * q);
    let cos_w0 = w0.cos();

    let b0 = (1.0 - cos_w0) / 2.0;
    let b1 = 1.0 - cos_w0;
    let b2 = (1.0 - cos_w0) / 2.0;
    let a0 = 1.0 + alpha;
    let a1 = -2.0 * cos_w0;
    let a2 = 1.0 - alpha;

    Biquad::new(b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)
}

/// Design a 2nd-order high-pass biquad via bilinear transform.
fn highpass_biquad(f0: f64, q: f64, fs: f64) -> Biquad {
    let w0 = 2.0 * PI * f0 / fs;
    let alpha = w0.sin() / (2.0 * q);
    let cos_w0 = w0.cos();

    let b0 = (1.0 + cos_w0) / 2.0;
    let b1 = -(1.0 + cos_w0);
    let b2 = (1.0 + cos_w0) / 2.0;
    let a0 = 1.0 + alpha;
    let a1 = -2.0 * cos_w0;
    let a2 = 1.0 - alpha;

    Biquad::new(b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)
}

/// Design a 2nd-order bandpass biquad.
fn bandpass_biquad(f0: f64, q: f64, fs: f64) -> Biquad {
    let w0 = 2.0 * PI * f0 / fs;
    let alpha = w0.sin() / (2.0 * q);
    let cos_w0 = w0.cos();

    let b0 = alpha;
    let b1 = 0.0;
    let b2 = -alpha;
    let a0 = 1.0 + alpha;
    let a1 = -2.0 * cos_w0;
    let a2 = 1.0 - alpha;

    Biquad::new(b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)
}

/// ISO 2631-1 Wk weighting filter as a biquad cascade.
///
/// Approximated as a cascade of:
/// 1. High-pass at 0.4 Hz (below Wk band)
/// 2. Low-pass at 100 Hz (anti-alias)
/// 3. Band-pass centred at ~6 Hz (peak Wk sensitivity)
/// 4. Low-pass at 12.5 Hz (Wk rolloff above peak)
///
/// This gives gain ≈1 in 4–8 Hz, rolling off above and below.
pub struct WkFilter {
    hp: Biquad,
    lp_anti: Biquad,
    bp_peak: Biquad,
    lp_roll: Biquad,
    gain: f64,
}

impl WkFilter {
    /// Create Wk filter for a given sample rate.
    pub fn new(fs: f64) -> Self {
        Self {
            hp: highpass_biquad(0.4, 0.707, fs),
            lp_anti: lowpass_biquad(100.0, 0.707, fs),
            bp_peak: bandpass_biquad(6.3, 0.64, fs),    // centred on Wk peak
            lp_roll: lowpass_biquad(12.5, 0.707, fs),    // Wk rolloff
            gain: 1.0,
        }
    }

    pub fn reset(&mut self) {
        self.hp.reset();
        self.lp_anti.reset();
        self.bp_peak.reset();
        self.lp_roll.reset();
    }

    /// Calibrate gain so that a 5 Hz sine has unity Wk gain.
    pub fn calibrate(&mut self, fs: f64) {
        // Run a 5 Hz sine through the filter and measure the gain
        let f_test = 5.0;
        let n_cycles = 20;
        let n_samples = (n_cycles as f64 * fs / f_test) as usize;

        self.reset();

        let mut max_in = 0.0_f64;
        let mut max_out = 0.0_f64;

        for i in 0..n_samples {
            let t = i as f64 / fs;
            let x = (2.0 * PI * f_test * t).sin();
            let y = self.process_raw(x);
            if i > n_samples / 2 {  // skip transient
                max_in = max_in.max(x.abs());
                max_out = max_out.max(y.abs());
            }
        }

        if max_out > 1e-12 {
            self.gain = max_in / max_out;
        }

        self.reset();
    }

    fn process_raw(&mut self, x: f64) -> f64 {
        let y = self.hp.process(x);
        let y = self.lp_anti.process(y);
        let y = self.bp_peak.process(y);
        let y = self.lp_roll.process(y);
        y
    }

    /// Process one sample through the Wk filter.
    pub fn process(&mut self, x: f64) -> f64 {
        self.process_raw(x) * self.gain
    }
}

/// Compute ISO 2631-1 weighted RMS acceleration.
///
/// Applies the Wk filter to the body acceleration time series
/// and returns the weighted RMS.
pub fn weighted_rms(acc: &[f64], fs: f64) -> f64 {
    let mut wk = WkFilter::new(fs);
    wk.calibrate(fs);

    let mut sum_sq = 0.0;
    let mut count = 0;

    for (i, &a) in acc.iter().enumerate() {
        let w = wk.process(a);
        // Skip initial transient (first 0.5 s)
        if i as f64 / fs > 0.5 {
            sum_sq += w * w;
            count += 1;
        }
    }

    if count > 0 {
        (sum_sq / count as f64).sqrt()
    } else {
        0.0
    }
}

/// Compute band RMS of a signal in a frequency band [f_lo, f_hi].
///
/// Uses bandpass filtering.
pub fn band_rms(signal: &[f64], fs: f64, f_lo: f64, f_hi: f64) -> f64 {
    let f_center = (f_lo * f_hi).sqrt();
    let bw = f_hi - f_lo;
    let q = f_center / bw;

    let mut bp = bandpass_biquad(f_center, q.max(0.3), fs);

    let mut sum_sq = 0.0;
    let mut count = 0;
    let skip = (0.5 * fs) as usize; // skip transient

    for (i, &x) in signal.iter().enumerate() {
        let y = bp.process(x);
        if i > skip {
            sum_sq += y * y;
            count += 1;
        }
    }

    if count > 0 {
        (sum_sq / count as f64).sqrt()
    } else {
        0.0
    }
}

/// Compute p_phys: weighted RMS of body acceleration from the nominal twin
/// driven by the fused road estimate zr_hat(t).
///
/// PRD §5.6: Run the nominal plant with the Kalman-estimated road,
/// then apply Wk weighting to the resulting body acceleration.
pub fn compute_p_phys(
    zr_hat: &[f64],
    ms: f64, mu: f64, k: f64, c: f64, kt: f64,
    travel_limit_m: f64,
    plant_dt: f64,
    comfort_fs: f64,
) -> f64 {
    let plant_out = crate::plant::run_nominal(ms, mu, k, c, kt, travel_limit_m, zr_hat, plant_dt);
    // Decimate body_acc to comfort_fs
    let decimate = (1.0 / plant_dt / comfort_fs).round() as usize;
    let acc: Vec<f64> = plant_out.body_acc.iter()
        .step_by(decimate.max(1))
        .copied()
        .collect();
    weighted_rms(&acc, comfort_fs)
}

/// Compute y_ref: weighted RMS of body acceleration from the perturbed plant
/// on the true road.
pub fn compute_y_ref(body_acc_perturbed: &[f64], plant_dt: f64, comfort_fs: f64) -> f64 {
    let decimate = (1.0 / plant_dt / comfort_fs).round() as usize;
    let acc: Vec<f64> = body_acc_perturbed.iter()
        .step_by(decimate.max(1))
        .copied()
        .collect();
    weighted_rms(&acc, comfort_fs)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wk_gain_near_unity_at_5hz() {
        let fs = 200.0;
        let mut wk = WkFilter::new(fs);
        wk.calibrate(fs);

        // Generate 5 Hz sine
        let duration = 5.0;
        let n = (duration * fs) as usize;
        let mut max_out = 0.0_f64;

        for i in 0..n {
            let t = i as f64 / fs;
            let x = (2.0 * PI * 5.0 * t).sin();
            let y = wk.process(x);
            if i > n / 2 {
                max_out = max_out.max(y.abs());
            }
        }

        // After calibration, gain at 5 Hz should be near 1
        assert!(
            max_out > 0.7 && max_out < 1.5,
            "Wk gain at 5 Hz = {max_out:.4}, expected ~1.0"
        );
    }

    #[test]
    fn wk_rolls_off_above_8hz() {
        let fs = 200.0;
        let mut wk = WkFilter::new(fs);
        wk.calibrate(fs);

        // Measure gain at 5 Hz
        let gain_5 = measure_gain(&mut wk, 5.0, fs);
        wk.reset();

        // Measure gain at 20 Hz
        let gain_20 = measure_gain(&mut wk, 20.0, fs);

        assert!(
            gain_20 < gain_5 * 0.8,
            "20 Hz gain ({gain_20:.4}) should be below 5 Hz gain ({gain_5:.4})"
        );
    }

    fn measure_gain(wk: &mut WkFilter, freq: f64, fs: f64) -> f64 {
        let duration = 5.0;
        let n = (duration * fs) as usize;
        let mut max_out = 0.0_f64;

        for i in 0..n {
            let t = i as f64 / fs;
            let x = (2.0 * PI * freq * t).sin();
            let y = wk.process(x);
            if i > n / 2 {
                max_out = max_out.max(y.abs());
            }
        }
        max_out
    }

    #[test]
    fn band_rms_picks_correct_band() {
        let fs = 200.0;
        let duration = 5.0;
        let n = (duration * fs) as usize;

        // Signal with energy at 3 Hz only
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                (2.0 * PI * 3.0 * t).sin()
            })
            .collect();

        let lo = band_rms(&signal, fs, 0.5, 2.0);
        let mid = band_rms(&signal, fs, 2.0, 8.0);
        let hi = band_rms(&signal, fs, 8.0, 20.0);

        // Most energy should be in the mid band (2-8 Hz)
        assert!(mid > lo, "Mid band should capture 3 Hz signal");
        assert!(mid > hi, "Mid band should capture 3 Hz signal");
    }

    #[test]
    fn weighted_rms_nonzero() {
        let fs = 200.0;
        let duration = 5.0;
        let n = (duration * fs) as usize;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                0.5 * (2.0 * PI * 5.0 * t).sin()
            })
            .collect();

        let wrms = weighted_rms(&signal, fs);
        assert!(wrms > 0.1, "Weighted RMS should be nonzero for 5 Hz signal: {wrms}");
    }
}

