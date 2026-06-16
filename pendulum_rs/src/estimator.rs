//! Phase 2 — identify the arm's dynamics from its motion, so RuVector can
//! *recall* the gain instead of being told the parameters by an oracle.
//!
//! The LQR is built from the linearization at upright `ẋ = A x + B u`. The only
//! rows of that system that depend on the arm's parameters (link lengths,
//! masses, friction) are the two **acceleration** rows — rows 2 and 3 — plus the
//! input column entries `b[2], b[3]`. Those ten numbers are the "dynamics
//! signature": a fingerprint that maps one-to-one onto the balance gain. Seed
//! them for known arms (offline), then recognize them online.
//!
//! This module is pure Rust — no RuVector — so the base crate still builds
//! without the vector DB. [`crate::memory`] (behind the `vectordb` feature) is
//! what stores and recalls these signatures.

use crate::control::{linearize_upright, LinModel};
use crate::simulator::Pendulum;
use std::f64::consts::PI;

/// Signature width: rows 2 and 3 of the linearization (4 state coefficients
/// each) interleaved with the two input-column entries `b[2]`, `b[3]`.
pub const SIG_DIM: usize = 10;

/// The ten-number dynamics fingerprint, laid out as
/// `[a20 a21 a22 a23 b2 | a30 a31 a32 a33 b3]`.
pub type Signature = [f64; SIG_DIM];

/// Read the **open-loop** signature straight from a model's linearization: rows
/// 2,3 of `A` plus `b[2], b[3]`. This is the conceptual fingerprint; the demo
/// actually matches in *closed-loop* space (see [`closed_loop_signature`]).
pub fn signature_from_model(sim: &Pendulum) -> Signature {
    sig_from_rows(&linearize_upright(sim))
}

fn sig_from_rows(m: &LinModel) -> Signature {
    [
        m.a[2][0], m.a[2][1], m.a[2][2], m.a[2][3], m.b[2],
        m.a[3][0], m.a[3][1], m.a[3][2], m.a[3][3], m.b[3],
    ]
}

/// The **closed-loop** signature seeded into RuVector: the acceleration rows of
/// `A − b·K` (how the arm accelerates per unit state error *while running the
/// probe gain `k`*) interleaved with the input entries `b`. Layout matches
/// [`OnlineEstimator::estimate`]'s output exactly: `[p20 p21 p22 p23 b2 | p30
/// p31 p32 p33 b3]` where `p_ij = a[2+i][j] − b[2+i]·k[j]`.
///
/// We match here, rather than in open-loop space, because the online regression
/// *directly* measures these closed-loop coefficients; recovering open-loop `A`
/// would mean adding `b·K` back, which multiplies the `b` estimation error by
/// the (large) gain and swamps the fingerprint. Both seed and query must use the
/// same `k` ([`crate::control::nominal_probe_gain`]).
pub fn closed_loop_signature(sim: &Pendulum, k: &crate::control::Vec4) -> Signature {
    let m = linearize_upright(sim);
    let row = |i: usize| -> [f64; 5] {
        let b = m.b[i];
        [
            m.a[i][0] - b * k[0],
            m.a[i][1] - b * k[1],
            m.a[i][2] - b * k[2],
            m.a[i][3] - b * k[3],
            b,
        ]
    };
    let r0 = row(2);
    let r1 = row(3);
    [
        r0[0], r0[1], r0[2], r0[3], r0[4],
        r1[0], r1[1], r1[2], r1[3], r1[4],
    ]
}

/// Euclidean distance between two signatures (unwhitened — callers that whiten
/// do it before comparing). Handy for diagnostics / unit tests.
pub fn sig_distance(a: &Signature, b: &Signature) -> f64 {
    a.iter().zip(b).map(|(x, y)| (x - y) * (x - y)).sum::<f64>().sqrt()
}

/// A short, dithered probe that identifies the live dynamics signature from the
/// arm's *actual* accelerations.
///
/// While probing, the arm applies its (stale) stabilizing torque `u = −K·s` plus
/// a small, state-independent **dither** `d(t)`. So each step the true dynamics
/// read `acc = A·s + b·(−K·s + d) = (A − bK)·s + b·d`. We record the regressors
/// `[e0, e1, ω0, ω1, d]` — the state error at upright and the *known exogenous*
/// dither — alongside the measured accelerations `(ω_next − ω)/dt`, and solve
/// two ridge regressions (one per acceleration row) over a sliding window.
///
/// The dither is the key. The stabilizing torque alone is a linear function of
/// the state, so a regression on the *total* torque can't tell `B` apart from
/// `A`. The dither is exogenous (an external signal, not a function of state),
/// so its regression coefficient *is* `b`, and the state coefficient is `A − bK`
/// — from which `A` is recovered by adding back the known stale gain `K`. This
/// is the instrumental-variable view of "persistent excitation".
pub struct OnlineEstimator {
    window: usize,
    /// Regressor rows `[e0, e1, ω0, ω1, d]` — note the 5th column is the *dither*
    /// (the exogenous probe), not the total applied torque.
    rows: Vec<[f64; 5]>,
    /// Measured joint-0 / joint-1 angular accelerations, aligned with `rows`.
    acc0: Vec<f64>,
    acc1: Vec<f64>,
    /// Ridge regularization added to the normal-equation diagonal.
    lambda: f64,
}

impl OnlineEstimator {
    /// `window` = max samples kept (older ones slide out); `lambda` = ridge term
    /// (a touch of regularization keeps the solve stable before excitation has
    /// fully spanned the regressor space).
    pub fn new(window: usize, lambda: f64) -> Self {
        Self {
            window,
            rows: Vec::with_capacity(window),
            acc0: Vec::with_capacity(window),
            acc1: Vec::with_capacity(window),
            lambda,
        }
    }

    /// How many samples are currently in the window.
    pub fn len(&self) -> usize {
        self.rows.len()
    }

    pub fn is_empty(&self) -> bool {
        self.rows.is_empty()
    }

    pub fn clear(&mut self) {
        self.rows.clear();
        self.acc0.clear();
        self.acc1.clear();
    }

    /// Feed one control step. `theta`/`omega` are the state **before** the step,
    /// `dither` the exogenous probe torque added to joint 0 this step (the part
    /// of the torque that is *not* a function of the state), `omega_next` the
    /// velocities **after** the step.
    pub fn observe(&mut self, theta: &[f64], omega: &[f64], dither: f64, omega_next: &[f64], dt: f64) {
        let wrap = |a: f64| (a + PI).rem_euclid(2.0 * PI) - PI;
        self.rows.push([wrap(theta[0] - PI), wrap(theta[1] - PI), omega[0], omega[1], dither]);
        self.acc0.push((omega_next[0] - omega[0]) / dt);
        self.acc1.push((omega_next[1] - omega[1]) / dt);
        if self.rows.len() > self.window {
            self.rows.remove(0);
            self.acc0.remove(0);
            self.acc1.remove(0);
        }
    }

    /// Solve the two ridge regressions and return the measured **closed-loop**
    /// signature directly: `[c0[0..4], b0, c1[0..4], b1]`, where `c_i[0..4]` are
    /// the state coefficients `A_row_i − b_i·K` (under whatever gain `K` the arm
    /// was running) and `c_i[4]` is the dither (input) coefficient `b_i`. No
    /// open-loop reconstruction — this layout matches [`closed_loop_signature`]
    /// computed with the same gain. Returns `None` until enough samples exist.
    pub fn estimate(&self) -> Option<Signature> {
        if self.rows.len() < 6 {
            return None;
        }
        let c0 = ridge_solve5(&self.rows, &self.acc0, self.lambda)?;
        let c1 = ridge_solve5(&self.rows, &self.acc1, self.lambda)?;
        Some([
            c0[0], c0[1], c0[2], c0[3], c0[4],
            c1[0], c1[1], c1[2], c1[3], c1[4],
        ])
    }
}

/// Ridge least squares for `y ≈ X·c` with 5 unknowns. The regressor columns
/// (torque ~tens, angle errors ~0.01) span very different scales, so we
/// standardize each column to unit variance before forming the normal equations
/// `(X̃ᵀX̃ + λI) c̃ = X̃ᵀy`, then unscale the solution back. This keeps the ridge
/// term comparable across columns and the system well-conditioned. Returns
/// `None` if the system is singular even after regularization.
fn ridge_solve5(rows: &[[f64; 5]], y: &[f64], lambda: f64) -> Option<[f64; 5]> {
    const N: usize = 5;
    // Per-column standard deviation (floored so a dead column doesn't divide by 0).
    let n = rows.len() as f64;
    let mut mean = [0.0f64; N];
    for r in rows {
        for i in 0..N {
            mean[i] += r[i];
        }
    }
    for m in &mut mean {
        *m /= n;
    }
    let mut scale = [0.0f64; N];
    for r in rows {
        for i in 0..N {
            let d = r[i] - mean[i];
            scale[i] += d * d;
        }
    }
    for s in &mut scale {
        *s = (*s / n).sqrt().max(1e-9);
    }

    // Normal equations in the standardized space (columns divided by `scale`).
    let mut ata = [[0.0f64; N]; N];
    let mut aty = [0.0f64; N];
    for (r, &yi) in rows.iter().zip(y) {
        let xs: [f64; N] = std::array::from_fn(|i| r[i] / scale[i]);
        for i in 0..N {
            aty[i] += xs[i] * yi;
            for j in 0..N {
                ata[i][j] += xs[i] * xs[j];
            }
        }
    }
    for i in 0..N {
        ata[i][i] += lambda;
    }
    let cs = gauss_solve5(ata, aty)?;
    // Unscale: a unit change in the raw column is `1/scale` in the standardized
    // one, so the raw coefficient is `c̃ / scale`.
    Some(std::array::from_fn(|i| cs[i] / scale[i]))
}

/// Gaussian elimination with partial pivoting for a 5×5 system.
fn gauss_solve5(mut a: [[f64; 5]; 5], mut b: [f64; 5]) -> Option<[f64; 5]> {
    const N: usize = 5;
    for col in 0..N {
        let mut pivot = col;
        for r in (col + 1)..N {
            if a[r][col].abs() > a[pivot][col].abs() {
                pivot = r;
            }
        }
        if a[pivot][col].abs() < 1e-12 {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        let diag = a[col][col];
        for r in (col + 1)..N {
            let factor = a[r][col] / diag;
            for c in col..N {
                a[r][c] -= factor * a[col][c];
            }
            b[r] -= factor * b[col];
        }
    }
    let mut x = [0.0; N];
    for col in (0..N).rev() {
        let mut s = b[col];
        for c in (col + 1)..N {
            s -= a[col][c] * x[c];
        }
        x[col] = s / a[col][col];
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn arm(l1: f64, m1: f64, b1: f64) -> Pendulum {
        Pendulum::new(vec![1.0, m1], vec![1.0, l1], vec![0.05, b1], 9.81, 0.005)
    }

    #[test]
    fn signature_separates_configs() {
        // Different link-2 lengths must produce distinguishable signatures.
        let a = signature_from_model(&arm(1.0, 1.0, 0.05));
        let b = signature_from_model(&arm(2.0, 1.0, 0.05));
        assert!(sig_distance(&a, &b) > 1.0, "configs should be far apart");
    }

    #[test]
    fn online_recovers_signature() {
        // Simulate a dithered probe near upright and check the regression
        // recovers the true signature to within a few percent.
        let dt = 0.005;
        let mut sim = arm(1.0, 1.0, 0.05);
        sim.reset(vec![PI - 0.02, PI + 0.02], vec![0.0, 0.0]);
        let k = crate::control::balance_gain(&sim, dt);
        let mut est = OnlineEstimator::new(400, 1e-4);
        for step in 0..400 {
            let theta = sim.theta.clone();
            let omega = sim.omega.clone();
            let u_fb = -(k[0] * (theta[0] - PI) + k[1] * (theta[1] - PI) + k[2] * omega[0] + k[3] * omega[1]);
            // Multi-sine dither: the exogenous probe that makes B identifiable.
            let t = step as f64 * dt;
            let dither = 6.0 * (2.0 * PI * 1.7 * t).sin() + 4.0 * (2.0 * PI * 3.3 * t).sin();
            sim.step(&[(u_fb + dither).clamp(-150.0, 150.0), 0.0]);
            est.observe(&theta, &omega, dither, &sim.omega, dt);
        }
        let measured = est.estimate().expect("enough samples");
        // Compare in the same closed-loop space the demo seeds/queries in.
        let truth = closed_loop_signature(&sim, &k);
        let rel = sig_distance(&measured, &truth) / sig_distance(&truth, &[0.0; SIG_DIM]);
        assert!(rel < 0.1, "recovered signature within 10% (got {rel:.3})");
    }
}
