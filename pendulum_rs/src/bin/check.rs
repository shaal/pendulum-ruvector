//! Diagnostic: which arm configurations can the LQR actually balance, and how
//! big are the gains? Helps pick a demonstrable adaptive scenario instead of
//! guessing. Not part of the demo — a control-tuning aid.

use pendulum_rs::control::{balance_gain, recover_torque, upright_energy};
use pendulum_rs::simulator::Pendulum;
use std::f64::consts::PI;

const DT: f64 = 0.005;
const U_MAX: f64 = 150.0;

/// Can the always-on recover controller (swing-up + LQR catch) get the arm back
/// to upright from a knocked-down start within `secs`? Returns final tip error.
fn recovery_test(theta0: Vec<f64>, secs: f64) -> f64 {
    let mut sim = Pendulum::new(vec![1.0, 1.0], vec![1.0, 1.0], vec![0.05, 0.05], 9.81, DT);
    sim.reset(theta0, vec![0.0, 0.0]);
    let k = balance_gain(&sim, DT);
    let _e_up = upright_energy(&sim);
    let wrap = |a: f64| (a + PI).rem_euclid(2.0 * PI) - PI;
    for _ in 0..(secs / DT) as usize {
        let u = recover_torque(&sim, &k, _e_up, U_MAX);
        sim.step(&[u, 0.0]);
    }
    wrap(sim.theta[0] - PI).abs() + wrap(sim.theta[1] - PI).abs()
}

fn settle_error(m1: f64, l1: f64, gain_from: Option<(f64, f64)>) -> (f64, f64) {
    // Build the *true* arm.
    let mut sim = Pendulum::new(vec![1.0, m1], vec![1.0, l1], vec![0.05, 0.05], 9.81, DT);
    // Gain computed either for this arm (adaptive) or for a reference arm (naive).
    let gain_sim = match gain_from {
        Some((gm, gl)) => Pendulum::new(vec![1.0, gm], vec![1.0, gl], vec![0.05, 0.05], 9.81, DT),
        None => Pendulum::new(vec![1.0, m1], vec![1.0, l1], vec![0.05, 0.05], 9.81, DT),
    };
    let k = balance_gain(&gain_sim, DT);
    let kmag = (k[0] * k[0] + k[1] * k[1] + k[2] * k[2] + k[3] * k[3]).sqrt();

    sim.reset(vec![PI - 0.2, PI + 0.15], vec![0.0, 0.0]);
    let wrap = |a: f64| (a + PI).rem_euclid(2.0 * PI) - PI;
    let mut last = 0.0f64;
    for step in 0..(5.0 / DT) as usize {
        let e0 = wrap(sim.theta[0] - PI);
        let e1 = wrap(sim.theta[1] - PI);
        let u = -(k[0] * e0 + k[1] * e1 + k[2] * sim.omega[0] + k[3] * sim.omega[1]);
        sim.step(&[u.clamp(-U_MAX, U_MAX), 0.0]);
        if step as f64 * DT > 4.0 {
            last = last.max(wrap(sim.theta[0] - PI).abs() + wrap(sim.theta[1] - PI).abs());
        }
    }
    (last, kmag)
}

fn main() {
    println!("config (m1,l1) | own-gain settle / |K|  | naive(light-gain) settle");
    for &(m1, l1) in &[
        (1.0, 1.0),
        (2.0, 1.0),
        (3.0, 1.0),
        (5.0, 1.0),
        (1.0, 1.5),
        (1.0, 2.0),
        (1.0, 0.6),
        (2.0, 1.5),
    ] {
        let (own, kmag) = settle_error(m1, l1, None);
        // "naive" = gain computed for the reference light arm (1,1).
        let (naive, _) = settle_error(m1, l1, Some((1.0, 1.0)));
        println!(
            "  ({:.1},{:.1})        | {:6.3} rad / {:7.1}   | {:6.3} rad  {}",
            m1,
            l1,
            own,
            kmag,
            naive,
            if naive > 0.5 && own < 0.2 { "<-- CONTRAST" } else { "" }
        );
    }

    println!("\nRecovery (swing-up + catch) from knocked-down starts, 15s:");
    for (label, theta0) in [
        ("small poke   ", vec![PI - 0.5, PI + 0.4]),
        ("big poke     ", vec![PI - 1.2, PI + 0.9]),
        ("sideways      ", vec![PI - 1.8, PI + 1.5]),
        ("hanging down  ", vec![0.1, -0.1]),
    ] {
        let final_err = recovery_test(theta0, 15.0);
        let ok = if final_err < 0.2 { "RECOVERED ✅" } else { "did not catch ❌" };
        println!("  {label} -> final tip error {:.2} rad  {}", final_err, ok);
    }
}
