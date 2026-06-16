//! Physics correctness guard: a *passive* pendulum (no torque, no damping) must
//! conserve total mechanical energy. This pins two things at once —
//!
//! 1. the equations of motion + RK4 integrator in `simulator` are consistent with
//!    the `total_energy` accounting (a sign error or a missing coupling term would
//!    show up as energy drift), and
//! 2. it is the reference value to compare the native build against the wasm build,
//!    so "the browser runs the same true physics" is a checkable claim, not a hope.

use pendulum_rs::simulator::Pendulum;
use std::f64::consts::PI;

/// Relative energy drift over a passive rollout, returned so the same helper can
/// be reused from a wasm parity check later.
fn passive_energy_drift(theta0: Vec<f64>, dt: f64, secs: f64) -> f64 {
    let n = theta0.len();
    let mut sim = Pendulum::new(vec![1.0; n], vec![1.0; n], vec![0.0; n], 9.81, dt);
    sim.reset(theta0, vec![0.0; n]);
    let e0 = sim.total_energy();
    let steps = (secs / dt).round() as usize;
    for _ in 0..steps {
        sim.step(&vec![0.0; n]);
    }
    let e1 = sim.total_energy();
    (e1 - e0).abs() / e0.abs().max(1e-9)
}

#[test]
fn passive_double_pendulum_conserves_energy() {
    // A real, coupled (non-tiny) swing for 20 s. RK4 is not symplectic, so a slow
    // drift is expected; "true physics" means it stays small, not exactly zero.
    let drift = passive_energy_drift(vec![0.6, 0.4], 0.002, 20.0);
    assert!(drift < 1e-3, "double-pendulum energy drift {drift:.2e} too large");
}

#[test]
fn passive_triple_pendulum_conserves_energy() {
    let drift = passive_energy_drift(vec![0.5, 0.3, 0.2], 0.002, 20.0);
    assert!(drift < 1e-3, "triple-pendulum energy drift {drift:.2e} too large");
}

#[test]
fn at_rest_hanging_straight_down_stays_put() {
    // The stable equilibrium: released hanging straight down (θ=0) with no input,
    // it must not move and energy must be exactly constant.
    let drift = passive_energy_drift(vec![0.0, 0.0], 0.005, 10.0);
    assert!(drift < 1e-12, "equilibrium should be exactly conserved, got {drift:.2e}");
}

#[test]
fn full_swing_amplitude_energy_bounded() {
    // A near-inverted, highly chaotic start (both links up near π). Even here the
    // passive system must not manufacture or lose meaningful energy.
    let drift = passive_energy_drift(vec![PI - 0.3, PI - 0.2], 0.001, 15.0);
    assert!(drift < 5e-3, "chaotic high-energy drift {drift:.2e} too large");
}
