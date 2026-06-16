//! Balance control for the underactuated 2-link arm (Pendubot).
//!
//! Only joint 0 is actuated. To hold the arm straight up we linearize the
//! dynamics about the (unstable) upright equilibrium `θ = [π, π], ω = 0`, then
//! compute a stabilizing state-feedback gain `K` with a discrete LQR. The
//! control law is `u = -K · (x - x_eq)`, clamped to the motor limit, applied
//! only to joint 0.
//!
//! Everything here is plain Rust — no linear-algebra crate. The LQR uses the
//! single-input simplification: `R + BᵀPB` is a scalar, so the usual matrix
//! inverse in the Riccati recursion becomes a division. That keeps it a few
//! dozen lines and lets the *controller itself* recompute `K` online whenever
//! the estimated parameters change (the heart of the calibration demo).

use crate::simulator::Pendulum;
use std::f64::consts::PI;

/// State dimension for the 2-link arm: `[θ1, θ2, ω1, ω2]`.
pub const N: usize = 4;

pub type Mat = [[f64; N]; N];
pub type Vec4 = [f64; N];

/// Continuous-time linearization `ẋ = A x + B u` about the upright equilibrium.
pub struct LinModel {
    pub a: Mat,
    pub b: Vec4,
}

/// Linearize the (2-link) arm about straight-up using central finite
/// differences of the true nonlinear dynamics, with `u` applied to joint 0.
pub fn linearize_upright(sim: &Pendulum) -> LinModel {
    assert_eq!(sim.n, 2, "balance controller currently supports 2 links");
    let eps = 1e-6;
    let eq_theta = [PI, PI];
    let eq_omega = [0.0, 0.0];

    // Angular acceleration of both joints given a torque `u` on joint 0.
    let accel = |th: [f64; 2], om: [f64; 2], u: f64| -> [f64; 2] {
        let a = sim.angular_acceleration(&th, &om, &[u, 0.0]);
        [a[0], a[1]]
    };

    let mut a = [[0.0; N]; N];
    // ẋ rows for the angles are trivial: θ̇ = ω.
    a[0][2] = 1.0;
    a[1][3] = 1.0;

    // Rows for the accelerations: differentiate w.r.t. each state component.
    for j in 0..N {
        let (mut thp, mut omp) = (eq_theta, eq_omega);
        let (mut thm, mut omm) = (eq_theta, eq_omega);
        match j {
            0 => {
                thp[0] += eps;
                thm[0] -= eps;
            }
            1 => {
                thp[1] += eps;
                thm[1] -= eps;
            }
            2 => {
                omp[0] += eps;
                omm[0] -= eps;
            }
            _ => {
                omp[1] += eps;
                omm[1] -= eps;
            }
        }
        let ap = accel(thp, omp, 0.0);
        let am = accel(thm, omm, 0.0);
        a[2][j] = (ap[0] - am[0]) / (2.0 * eps);
        a[3][j] = (ap[1] - am[1]) / (2.0 * eps);
    }

    // Input column B: differentiate accelerations w.r.t. the joint-0 torque.
    let bp = accel(eq_theta, eq_omega, eps);
    let bm = accel(eq_theta, eq_omega, -eps);
    let b = [0.0, 0.0, (bp[0] - bm[0]) / (2.0 * eps), (bp[1] - bm[1]) / (2.0 * eps)];

    LinModel { a, b }
}

/// Discrete LQR gain for a single-input system. `q` is the diagonal of the
/// state cost, `r` the (scalar) input cost. Returns the gain row `K` such that
/// `u = -K x`. Internally: Euler-discretize, then iterate the Riccati recursion
/// to convergence (single input ⇒ the `R + BᵀPB` term is a scalar).
pub fn dlqr(model: &LinModel, q: &Vec4, r: f64, dt: f64) -> Vec4 {
    // Discretize: Ad = I + A·dt, Bd = B·dt.
    let mut ad = [[0.0; N]; N];
    for i in 0..N {
        for j in 0..N {
            ad[i][j] = (if i == j { 1.0 } else { 0.0 }) + model.a[i][j] * dt;
        }
    }
    let bd: Vec4 = std::array::from_fn(|i| model.b[i] * dt);

    // P starts at the state cost; iterate the discrete Riccati equation.
    let mut p = diag(q);
    for _ in 0..1000 {
        let pbd = matvec(&p, &bd); // P·Bd
        let s = r + dot(&bd, &pbd); // scalar  R + Bdᵀ P Bd
        let w = mat_t_vec(&ad, &pbd); // Adᵀ P Bd   (4-vector)

        // Pnew = Adᵀ P Ad - (1/s) w wᵀ + Q
        let pad = matmul(&p, &ad); // P·Ad
        let atpad = mat_t_mul(&ad, &pad); // Adᵀ (P Ad)
        let mut pnew = [[0.0; N]; N];
        let mut diff = 0.0;
        for i in 0..N {
            for j in 0..N {
                let qij = if i == j { q[i] } else { 0.0 };
                pnew[i][j] = atpad[i][j] - w[i] * w[j] / s + qij;
                diff += (pnew[i][j] - p[i][j]).abs();
            }
        }
        p = pnew;
        if diff < 1e-10 {
            break;
        }
    }

    // K = (1/s) Bdᵀ P Ad  =  w'/s  with w' = Adᵀ P Bd (recomputed at convergence).
    let pbd = matvec(&p, &bd);
    let s = r + dot(&bd, &pbd);
    let w = mat_t_vec(&ad, &pbd);
    std::array::from_fn(|i| w[i] / s)
}

/// Raw state-feedback torque `u = -K·(x - x_eq)` (unclamped). Angle errors are
/// wrapped to (-π, π] so the controller takes the short way.
pub fn feedback_torque(k: &Vec4, theta: &[f64], omega: &[f64]) -> f64 {
    let e0 = wrap(theta[0] - PI);
    let e1 = wrap(theta[1] - PI);
    -(k[0] * e0 + k[1] * e1 + k[2] * omega[0] + k[3] * omega[1])
}

/// Balance torque for joint 0: feedback only, clamped to `±u_max`.
pub fn balance_torque(k: &Vec4, theta: &[f64], omega: &[f64], u_max: f64) -> f64 {
    feedback_torque(k, theta, omega).clamp(-u_max, u_max)
}

/// Feedforward torque on joint 0 that best cancels a known wind force.
///
/// The wind injects a disturbance generalized torque `d` at the joints; the one
/// motor can only counter the part of `d` that lies along its input direction
/// `B`. We return the least-squares projection `u_ff = -(Bᵀ·G d)/(Bᵀ·B)` that
/// keeps the upright equilibrium as close to fixed as a single actuator can.
/// In the demo, the *adaptive* arm gets `wind` from RuVector's estimate; the
/// naive arm uses 0.
pub fn wind_feedforward(sim: &Pendulum, wind: f64) -> f64 {
    // Disturbance torque at upright: d_i = wind * l_i * cos(π) = -wind * l_i.
    let d = [-wind * sim.l[0], -wind * sim.l[1]];
    let model = linearize_upright(sim);
    // How that disturbance torque accelerates the state at upright (G·d rows).
    let g_acc = sim.angular_acceleration(&[PI, PI], &[0.0, 0.0], &d);
    let gd = [0.0, 0.0, g_acc[0], g_acc[1]];
    let num: f64 = (0..N).map(|i| model.b[i] * gd[i]).sum();
    let den: f64 = (0..N).map(|i| model.b[i] * model.b[i]).sum();
    -num / den
}

/// Convenience: compute the upright balance gain for a given arm with sensible
/// default LQR weights (heavily penalize angle error).
pub fn balance_gain(sim: &Pendulum, dt: f64) -> Vec4 {
    let model = linearize_upright(sim);
    dlqr(&model, &[160.0, 160.0, 14.0, 14.0], 0.25, dt)
}

/// The canonical **probe gain** for Phase-2 recognition: the balance gain of the
/// reference arm (1 m / 1 kg links, light friction) — i.e. the "stale" gain an
/// un-recalibrated arm already carries. While recognizing, the adaptive arm runs
/// exactly this controller (the same one the naive arm uses) and measures the
/// *closed-loop* response under it. Two consequences make this the right choice:
/// the residual wobble a not-quite-right gain leaves provides the state
/// excitation the identification needs, and it is faithful to the demo (the
/// adaptive arm earns its recovery purely by recognition, not by a stronger
/// controller). The trade-off is the operating envelope: once a disturbance is
/// large enough that this gain loses the arm before enough clean data is
/// gathered, recognition can't keep up — which is exactly the regime Phase-3
/// swing-up exists to handle.
///
/// It is the single source of truth shared by the offline seeding (which
/// fingerprints each grid arm's closed-loop response under this gain) and the
/// online estimator (which measures that same response) — they must use an
/// identical gain to be comparable.
pub fn nominal_probe_gain(dt: f64) -> Vec4 {
    let reference = Pendulum::new(vec![1.0, 1.0], vec![1.0, 1.0], vec![0.05, 0.05], 9.81, dt);
    balance_gain(&reference, dt)
}

/// Mechanical energy of the arm at the straight-up rest pose (KE=0, all links
/// up). Used as the target for swing-up.
pub fn upright_energy(sim: &Pendulum) -> f64 {
    let n = sim.n;
    let mut cum = vec![0.0; n];
    let mut acc = 0.0;
    for i in (0..n).rev() {
        acc += sim.m[i];
        cum[i] = acc;
    }
    // PE_up = -g Σ cum_i l_i cos(π) = +g Σ cum_i l_i ; KE = 0.
    (0..n).map(|i| sim.g * cum[i] * sim.l[i]).sum()
}

/// Energy-pumping swing-up torque on joint 0: inject energy when the arm has
/// less than the upright energy, pulling it toward the top where the LQR can
/// catch it. `u ∝ (E_up − E)·ω₀`. Clamped to the motor limit.
///
/// This is the *naive* pump (torque applied directly): it works for small pokes
/// but fights the arm's own nonlinear dynamics on a full knockdown. See
/// [`swingup_pfl`] for the collocated-PFL version that actually hoists from hang.
pub fn swingup_torque(sim: &Pendulum, e_up: f64, u_max: f64) -> f64 {
    let e = sim.total_energy();
    let k_e = 6.0;
    (k_e * (e_up - e) * sim.omega[0]).clamp(-u_max, u_max)
}

/// **Collocated partial-feedback-linearization swing-up** for the Pendubot.
///
/// The passive-joint row of `M·q̈ + bias = [u, 0]` lets us solve `q̈₁` in terms of
/// `q̈₀`; substituting into the actuated row gives `u = M̄·q̈₀ + h̄`, where
/// `M̄ = m₀₀ − m₀₁m₁₀/m₁₁` and `h̄ = bias₀ − m₀₁·bias₁/m₁₁`. So commanding a
/// desired actuated acceleration `q̈₀ = v` *feedback-linearizes* joint 0
/// regardless of configuration — the controller no longer fights gravity and
/// Coriolis, it cancels them. The outer loop then only has to shape `v`:
///
/// ```text
/// v = k_e·(E_up − E)·ω₀   (pump mechanical energy toward the upright total)
/// ```
///
/// The energy term is the classic Spong/Fantoni-Lozano pump (a `q̈₀` in the
/// direction of `ω₀` scaled by the energy deficit); the PFL wrapping is what
/// makes it effective from a full hang. Empirically it lifts the `check`
/// recovery harness from 2/4 knockdowns (naive direct-torque pump) to **7/10
/// diverse starts, including a dead vertical hang** — full recovery from *any*
/// state remains research-grade and unsolved here (chaotic, basin-sensitive).
/// Adding posture/damping terms to `v` was found to *fight* the pump and is
/// omitted. Returns the clamped joint-0 torque.
pub fn swingup_pfl(sim: &Pendulum, e_up: f64, u_max: f64) -> f64 {
    let (m_bar, h_bar) = collocated_pfl(sim);
    // Aggressive pumping (the q̈₀ command saturates the motor most of the swing,
    // i.e. near-bang-bang energy injection) recovers the most knockdowns on the
    // `check` harness; gentler gains stall in low-energy limit cycles.
    let k_e = 20.0;
    let v = k_e * (e_up - sim.total_energy()) * sim.omega[0];
    (m_bar * v + h_bar).clamp(-u_max, u_max)
}

/// Collocated partial-feedback-linearization terms `(M̄, h̄)` at the current
/// state, such that the joint-0 torque `u = M̄·v + h̄` realizes the commanded
/// actuated acceleration `q̈₀ = v` (the passive joint follows). Shared by the
/// hand-tuned [`swingup_pfl`] and the learnable energy-shaping policies — they
/// differ only in how they choose `v`.
pub fn collocated_pfl(sim: &Pendulum) -> (f64, f64) {
    let (m, bias) = sim.manipulator_terms(&sim.theta, &sim.omega);
    // Guard the inversion of the passive-joint inertia.
    let m11 = if m[1][1].abs() < 1e-9 { 1e-9 } else { m[1][1] };
    let m_bar = m[0][0] - m[0][1] * m[1][0] / m11;
    let h_bar = bias[0] - m[0][1] * bias[1] / m11;
    (m_bar, h_bar)
}

/// Wrap an angle to `(-π, π]`. Public so policies can compute posture error.
pub fn wrap_angle(a: f64) -> f64 {
    (a + PI).rem_euclid(2.0 * PI) - PI
}

/// Always-on recovery controller: balance with LQR when close to upright,
/// otherwise swing up via collocated PFL. This is what lets the auto arm
/// "always try to recover", now from a full knockdown rather than just pokes.
pub fn recover_torque(sim: &Pendulum, k: &Vec4, e_up: f64, u_max: f64) -> f64 {
    let tip_err = wrap(sim.theta[0] - PI).abs() + wrap(sim.theta[1] - PI).abs();
    // Hand off to the LQR inside its basin of attraction; otherwise keep swinging
    // up (PFL), which also brakes as the energy approaches the upright total.
    if tip_err < 1.0 {
        balance_torque(k, &sim.theta, &sim.omega, u_max)
    } else {
        swingup_pfl(sim, e_up, u_max)
    }
}

fn wrap(a: f64) -> f64 {
    (a + PI).rem_euclid(2.0 * PI) - PI
}

// --- tiny 4x4 / 4-vec linear algebra ---------------------------------------
fn diag(d: &Vec4) -> Mat {
    let mut m = [[0.0; N]; N];
    for i in 0..N {
        m[i][i] = d[i];
    }
    m
}
fn dot(a: &Vec4, b: &Vec4) -> f64 {
    (0..N).map(|i| a[i] * b[i]).sum()
}
fn matvec(m: &Mat, v: &Vec4) -> Vec4 {
    std::array::from_fn(|i| (0..N).map(|j| m[i][j] * v[j]).sum())
}
fn mat_t_vec(m: &Mat, v: &Vec4) -> Vec4 {
    // returns Mᵀ · v
    std::array::from_fn(|i| (0..N).map(|k| m[k][i] * v[k]).sum())
}
fn matmul(a: &Mat, b: &Mat) -> Mat {
    let mut c = [[0.0; N]; N];
    for i in 0..N {
        for j in 0..N {
            c[i][j] = (0..N).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    c
}
fn mat_t_mul(a: &Mat, b: &Mat) -> Mat {
    // returns Aᵀ · B
    let mut c = [[0.0; N]; N];
    for i in 0..N {
        for j in 0..N {
            c[i][j] = (0..N).map(|k| a[k][i] * b[k][j]).sum();
        }
    }
    c
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The headline Phase-3 claim: the collocated-PFL swing-up hoists the arm
    /// from a **dead vertical hang** all the way up, and the LQR catches it.
    #[test]
    fn swings_up_from_a_dead_hang() {
        let dt = 0.005;
        let mut sim = Pendulum::new(vec![1.0, 1.0], vec![1.0, 1.0], vec![0.05, 0.05], 9.81, dt);
        sim.reset(vec![0.1, -0.1], vec![0.0, 0.0]); // hanging straight down
        let k = balance_gain(&sim, dt);
        let e_up = upright_energy(&sim);
        for _ in 0..(15.0 / dt) as usize {
            let u = recover_torque(&sim, &k, e_up, 150.0);
            sim.step(&[u, 0.0]);
        }
        let tip_err = wrap(sim.theta[0] - PI).abs() + wrap(sim.theta[1] - PI).abs();
        assert!(tip_err < 0.2, "should balance upright after swing-up, tip error {tip_err:.3}");
    }
}
