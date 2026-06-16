//! Stage 1 — evolutionary swing-up policy search (library half).
//!
//! The hand-tuned collocated-PFL swing-up recovers 7/10 of the `check` harness
//! knockdowns. Here we make the swing-up *learnable*: a [`SwingUpPolicy`] chooses
//! the commanded actuated acceleration `v` (the PFL inversion `u = M̄·v + h̄` is
//! shared), a [`rollout`] scores a policy on a knockdown, and a potential-shaped
//! [`fitness`] turns that into a number an evolutionary search maximizes.
//!
//! The architecture stays **hybrid**: the optimal LQR still catches at the top;
//! only the swing-up (the regime the LQR can't reach) is learned. The `evolve`
//! binary drives the search over [`EnergyShapingPolicy`] parameters.

use crate::control::{balance_gain, balance_torque, collocated_pfl, swingup_pfl, upright_energy, wrap_angle};
use crate::simulator::Pendulum;

const DT: f64 = 0.005;
const U_MAX: f64 = 150.0;

/// A swing-up controller: given the arm state (and its upright-energy target),
/// produce the joint-0 torque to apply while knocked out of the LQR basin.
pub trait SwingUpPolicy {
    fn torque(&self, sim: &Pendulum, e_up: f64, u_max: f64) -> f64;
}

/// The hand-tuned Phase-3 controller — the baseline the search must beat.
pub struct PflBaseline;
impl SwingUpPolicy for PflBaseline {
    fn torque(&self, sim: &Pendulum, e_up: f64, u_max: f64) -> f64 {
        swingup_pfl(sim, e_up, u_max)
    }
}

/// Number of evolvable parameters in [`EnergyShapingPolicy`].
pub const NP: usize = 5;

/// A parameterized energy-shaping swing-up: it shapes the commanded actuated
/// acceleration `v` as a linear combination of physically-meaningful features,
/// then lets the shared PFL inversion realize it. It is a strict superset of the
/// baseline — `p = [20, 0, 0, 0, 0]` reproduces `swingup_pfl` exactly.
#[derive(Debug, Clone, Copy)]
pub struct EnergyShapingPolicy {
    pub p: [f64; NP],
}

impl EnergyShapingPolicy {
    /// The parameters that reproduce the hand-tuned baseline.
    pub fn baseline() -> Self {
        let mut p = [0.0; NP];
        p[0] = 20.0;
        Self { p }
    }
}

impl SwingUpPolicy for EnergyShapingPolicy {
    fn torque(&self, sim: &Pendulum, e_up: f64, u_max: f64) -> f64 {
        let (m_bar, h_bar) = collocated_pfl(sim);
        let ed = e_up - sim.total_energy(); // energy deficit
        let th0 = wrap_angle(sim.theta[0] - std::f64::consts::PI);
        // Features: energy-pump on ω₀, energy-pump on the passive joint's swing,
        // energy-pump via posture, velocity damping, posture regulation.
        let v = self.p[0] * ed * sim.omega[0]
            + self.p[1] * ed * sim.omega[1]
            + self.p[2] * ed * sim.theta[0].sin()
            + self.p[3] * sim.omega[0]
            + self.p[4] * th0;
        (m_bar * v + h_bar).clamp(-u_max, u_max)
    }
}

/// Outcome of simulating one knockdown under a policy.
#[derive(Debug, Clone, Copy)]
pub struct Rollout {
    /// Ended balanced upright (tip error < 0.2 rad).
    pub caught: bool,
    /// Tip error at the end of the rollout.
    pub final_tip: f64,
    /// Time (s) at which it first held upright for ≥1 s (else the full duration).
    pub time_to_catch: f64,
    /// ∫ tip-error dt — the potential-shaping term (time spent away from upright).
    pub integral_tip: f64,
}

/// Simulate a knockdown recovery under `policy`: LQR catch inside the basin,
/// the policy's swing-up outside it. Deterministic given inputs.
pub fn rollout<P: SwingUpPolicy>(theta0: &[f64], policy: &P, secs: f64) -> Rollout {
    let mut sim = Pendulum::new(vec![1.0, 1.0], vec![1.0, 1.0], vec![0.05, 0.05], 9.81, DT);
    sim.reset(theta0.to_vec(), vec![0.0, 0.0]);
    let k = balance_gain(&sim, DT);
    let e_up = upright_energy(&sim);

    let tip = |s: &Pendulum| wrap_angle(s.theta[0] - std::f64::consts::PI).abs()
        + wrap_angle(s.theta[1] - std::f64::consts::PI).abs();

    let mut integral_tip = 0.0;
    let mut hold = 0.0;
    let mut time_to_catch = secs;
    let mut caught_once = false;
    let steps = (secs / DT) as usize;
    for step in 0..steps {
        let e = tip(&sim);
        integral_tip += e * DT;
        if e < 0.2 {
            hold += DT;
            if hold >= 1.0 && !caught_once {
                caught_once = true;
                time_to_catch = step as f64 * DT;
            }
        } else {
            hold = 0.0;
        }
        // Hybrid: LQR inside the basin, learned swing-up outside it.
        let u = if e < 1.0 {
            balance_torque(&k, &sim.theta, &sim.omega, U_MAX)
        } else {
            policy.torque(&sim, e_up, U_MAX)
        };
        sim.step(&[u, 0.0]);
    }
    let final_tip = tip(&sim);
    Rollout {
        caught: final_tip < 0.2,
        final_tip,
        time_to_catch,
        integral_tip,
    }
}

/// Scalar fitness (higher is better) from a rollout. A caught arm scores high,
/// rewarded for catching *fast* and for spending little time away from upright
/// (potential-based shaping — it doesn't move the optimum). A miss scores
/// negative, proportional to how far from upright it ended, so the search still
/// gets gradient from failures.
pub fn fitness(r: &Rollout) -> f64 {
    if r.caught {
        200.0 - 10.0 * r.time_to_catch - 0.5 * r.integral_tip
    } else {
        -50.0 - 20.0 * r.final_tip
    }
}

/// The canonical knockdown starts the `check` harness reports on — shared so the
/// baseline and the evolved champion are judged on exactly the same scenarios.
pub fn knockdown_starts() -> Vec<(&'static str, Vec<f64>)> {
    use std::f64::consts::PI;
    vec![
        ("small poke    ", vec![PI - 0.5, PI + 0.4]),
        ("big poke      ", vec![PI - 1.2, PI + 0.9]),
        ("sideways      ", vec![PI - 1.8, PI + 1.5]),
        ("hard sideways ", vec![PI - 2.4, PI + 0.6]),
        ("link-2 folded ", vec![PI - 0.3, PI + 2.2]),
        ("both folded   ", vec![PI - 1.5, PI - 1.5]),
        ("half down     ", vec![PI / 2.0, PI / 2.0]),
        ("hanging down  ", vec![0.1, -0.1]),
        ("hang + twist  ", vec![0.2, PI - 0.3]),
        ("near top fast ", vec![PI - 0.8, PI + 0.8]),
    ]
}

/// How many of the canonical knockdowns a policy recovers within `secs`.
pub fn recovery_count<P: SwingUpPolicy>(policy: &P, secs: f64) -> usize {
    knockdown_starts()
        .iter()
        .filter(|(_, theta0)| rollout(theta0, policy, secs).caught)
        .count()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn baseline_policy_matches_handtuned() {
        // EnergyShapingPolicy::baseline() must reproduce swingup_pfl's recovery.
        let secs = 15.0;
        let n_param = recovery_count(&EnergyShapingPolicy::baseline(), secs);
        let n_fixed = recovery_count(&PflBaseline, secs);
        assert_eq!(n_param, n_fixed, "param baseline should match the hand-tuned one");
        assert!(n_fixed >= 7, "hand-tuned baseline should recover ≥7/10, got {n_fixed}");
    }

    #[test]
    fn evolved_champion_beats_baseline() {
        // The champion the `evolve` search finds at the default seed (=1). Pinned
        // here so the "learning beats hand-tuning" claim is a fast, reproducible
        // library check, not just a slow binary run. It recovers all 10.
        let champion = EnergyShapingPolicy { p: [35.14, 7.42, 4.24, -6.89, 2.12] };
        let base = recovery_count(&PflBaseline, 15.0);
        let champ = recovery_count(&champion, 15.0);
        assert!(champ > base, "evolved champion ({champ}) should beat baseline ({base})");
        assert_eq!(champ, 10, "this champion recovers all 10 knockdowns");
    }

    #[test]
    fn fitness_prefers_catching() {
        let caught = Rollout { caught: true, final_tip: 0.0, time_to_catch: 3.0, integral_tip: 20.0 };
        let missed = Rollout { caught: false, final_tip: 3.0, time_to_catch: 15.0, integral_tip: 200.0 };
        assert!(fitness(&caught) > fitness(&missed));
    }
}
