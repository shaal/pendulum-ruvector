//! Phase 2 end-to-end: seed RuVector, identify a disturbed arm from its actual
//! motion, and confirm the recall returns the correct *config class* — i.e. the
//! whole pipeline (control + estimator + memory) composes. This is the robust
//! identification assertion; the brittle "catch it while it's falling" timing is
//! exercised by the demo binary itself, not pinned here.
#![cfg(feature = "vectordb")]

use pendulum_rs::control::nominal_probe_gain;
use pendulum_rs::estimator::OnlineEstimator;
use pendulum_rs::memory::ConfigMemory;
use pendulum_rs::simulator::Pendulum;
use std::f64::consts::PI;

const DT: f64 = 0.005;
const U_MAX: f64 = 150.0;

/// Probe `arm` (held near upright by `k_hold`) with a dither and return the
/// EMA-smoothed signature RuVector recall, after `n_steps`.
fn probe_and_recall(
    mem: &ConfigMemory,
    arm: &mut Pendulum,
    k_hold: &[f64; 4],
    n_steps: usize,
) -> pendulum_rs::memory::RecalledConfig {
    let mut est = OnlineEstimator::new(n_steps + 1, 1e-4);
    for step in 0..n_steps {
        let t = step as f64 * DT;
        let theta = arm.theta.clone();
        let omega = arm.omega.clone();
        let e0 = (theta[0] - PI + PI).rem_euclid(2.0 * PI) - PI;
        let e1 = (theta[1] - PI + PI).rem_euclid(2.0 * PI) - PI;
        let u_fb = -(k_hold[0] * e0 + k_hold[1] * e1 + k_hold[2] * omega[0] + k_hold[3] * omega[1]);
        let dither = 6.0 * (2.0 * PI * 1.7 * t).sin() + 4.0 * (2.0 * PI * 3.3 * t).sin();
        arm.step(&[(u_fb + dither).clamp(-U_MAX, U_MAX), 0.0]);
        est.observe(&theta, &omega, dither, &arm.omega, DT);
    }
    let sig = est.estimate().expect("enough samples to estimate");
    mem.recall(&sig).unwrap().expect("recall returns a neighbour")
}

#[test]
fn pipeline_recalls_the_correct_length_class() {
    let path = std::env::temp_dir()
        .join("pendulum_phase2_e2e.db")
        .to_string_lossy()
        .into_owned();
    let mut mem = ConfigMemory::new(&path).unwrap();
    mem.seed_grid().unwrap();
    // Seeds and probe must share the same gain (the closed-loop signature is
    // defined under it). Disturbances are kept inside this gain's clean-hold
    // range so the probe gathers an accurate, full-length window — the goal here
    // is to pin identification correctness, not the edge-of-envelope catch.
    let k_probe = nominal_probe_gain(DT);

    // A longer-link arm (1.5 m, an exact grid config) must recall a long
    // neighbour, never a short one.
    let mut long_arm = Pendulum::new(vec![1.0, 1.0], vec![1.0, 1.5], vec![0.05, 0.05], 9.81, DT);
    long_arm.reset(vec![PI - 0.03, PI + 0.03], vec![0.0, 0.0]);
    let long_hit = probe_and_recall(&mem, &mut long_arm, &k_probe, 300);
    assert!(
        (long_hit.l1 - 1.5).abs() < 1e-6,
        "1.5 m arm should recall the 1.5 m config, got l1={}",
        long_hit.l1
    );

    // A short-link arm (0.6 m) must recall a short neighbour — proving the
    // signature discriminates length, rather than collapsing everything together.
    let mut short_arm = Pendulum::new(vec![1.0, 1.0], vec![1.0, 0.6], vec![0.05, 0.05], 9.81, DT);
    short_arm.reset(vec![PI - 0.03, PI + 0.03], vec![0.0, 0.0]);
    let short_hit = probe_and_recall(&mem, &mut short_arm, &k_probe, 300);
    assert!(
        short_hit.l1 <= 1.0,
        "0.6 m arm should recall a short neighbour, got l1={}",
        short_hit.l1
    );

    // The two disturbances must land on *different* configs — the pipeline tells
    // them apart purely from motion.
    assert_ne!(long_hit.l1, short_hit.l1, "distinct arms recalled the same config");
}
