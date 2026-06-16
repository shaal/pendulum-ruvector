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
