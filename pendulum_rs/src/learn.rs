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

/// The Stage-1 nominal champion (`evolve` default seed) — strong on the nominal
/// arm, and the warm-start point for domain-randomized search.
pub const NOMINAL_CHAMPION: [f64; NP] = [35.14, 7.42, 4.24, -6.89, 2.12];

/// The Stage-2.5 domain-randomized champion (`evolve RANDOMIZE_ARM=1 SEED=4`,
/// under the closest-approach fitness): 10/10 on the nominal harness and the
/// best held-out transfer found (28/80).
pub const DR_CHAMPION: [f64; NP] =
    [33.52139610363168, 7.611966723879871, -3.6270607062030154, -10.524715621403294, 0.8285028042271357];

/// The Stage-2.6 per-arm champion **library**: one evolved swing-up policy per
/// anchor config on the seed grid (`evolve LIBRARY=1`, seed 100). Stored in
/// RuVector and recalled per arm. Tuple = `(l1, m1, b1, params)`.
pub const POLICY_LIBRARY: [(f64, f64, f64, [f64; NP]); 15] = [
    (0.6, 1.0, 0.05, [41.49674709780919, 9.220746002426706, 17.453385539452004, -3.35354674662523, 1.9981340100796339]),
    (0.6, 2.0, 0.05, [32.213929640227626, 5.528204227253068, 10.106962602614054, -9.617910340366592, -3.402306442737929]),
    (0.6, 3.0, 0.05, [42.21044621373286, 7.195015825440296, 9.899125189970158, -4.561395389424075, -4.558493826843489]),
    (1.0, 1.0, 0.05, [36.07338907782604, 6.713344033070091, 2.9613077625644477, -6.972207398354313, 2.9628812168331935]),
    (1.0, 2.0, 0.05, [44.73612935485408, 8.777718485563181, 2.8435724765696992, -6.6768897138255205, -5.1218902460831535]),
    (1.0, 3.0, 0.05, [43.09465498136796, 7.890820892630525, 11.27214416824532, -8.805131142569195, -0.8527154737045067]),
    (1.5, 1.0, 0.05, [46.954019489170236, 13.176367738439753, 14.223153007338198, -4.337839484456615, -0.9414598572656963]),
    (1.5, 2.0, 0.05, [40.55210648294607, 12.531375304703989, -0.5397098559065614, -10.182968569563927, -6.869404219365489]),
    (1.5, 3.0, 0.05, [44.95642075726414, 8.326448539841383, 6.367099180350552, -12.030068299206699, 3.253018215640121]),
    (2.0, 1.0, 0.05, [41.8678713308941, 19.077536951608426, -1.0093357966066714, -9.570719847793857, 0.5539580715378758]),
    (2.0, 2.0, 0.05, [36.91525425601262, 7.43863783743908, 13.644614071821511, -11.933309814133983, -5.9331665054078755]),
    (2.0, 3.0, 0.05, [37.21959425810403, 12.63982770076396, 13.899369636712832, -6.023993138861657, 3.8567604273255776]),
    (2.5, 1.0, 0.05, [57.3377558368983, 19.574191846343094, 10.217494433269987, -12.076884667976884, -1.149012970830449]),
    (2.5, 2.0, 0.05, [48.63464770761959, 10.746496317529525, -2.880663506246548, -12.571623627346641, -2.1047909088448304]),
    (2.5, 3.0, 0.05, [29.947550047109164, 16.523943743667967, -4.844358825651268, -10.701060383814484, 0.035044166569080604]),
];

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
    /// ∫ tip-error dt — time spent away from upright.
    pub integral_tip: f64,
    /// Closest the arm ever got to upright (min tip error over the rollout). The
    /// key shaping signal: it gives the search a gradient even when the arm is
    /// never caught — a near-miss scores better than a hopeless spin.
    pub min_tip: f64,
}

/// Simulate a knockdown recovery under `policy`: LQR catch inside the basin,
/// the policy's swing-up outside it. Deterministic given inputs. This is the
/// nominal-arm convenience wrapper around [`rollout_config`].
pub fn rollout<P: SwingUpPolicy>(theta0: &[f64], policy: &P, secs: f64) -> Rollout {
    rollout_config(1.0, 1.0, 0.05, theta0, policy, secs)
}

/// As [`rollout`] but for an arbitrary arm `(link-2 length, mass, friction)` —
/// the basis for domain-randomized training and cross-arm evaluation.
pub fn rollout_config<P: SwingUpPolicy>(
    l1: f64,
    m1: f64,
    b1: f64,
    theta0: &[f64],
    policy: &P,
    secs: f64,
) -> Rollout {
    let mut sim = Pendulum::new(vec![1.0, m1], vec![1.0, l1], vec![0.05, b1], 9.81, DT);
    sim.reset(theta0.to_vec(), vec![0.0, 0.0]);
    let k = balance_gain(&sim, DT);
    let e_up = upright_energy(&sim);

    let tip = |s: &Pendulum| wrap_angle(s.theta[0] - std::f64::consts::PI).abs()
        + wrap_angle(s.theta[1] - std::f64::consts::PI).abs();

    let mut integral_tip = 0.0;
    let mut hold = 0.0;
    let mut time_to_catch = secs;
    let mut caught_once = false;
    let mut min_tip = tip(&sim);
    let steps = (secs / DT) as usize;
    for step in 0..steps {
        let e = tip(&sim);
        integral_tip += e * DT;
        min_tip = min_tip.min(e);
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
        min_tip,
    }
}

/// Scalar fitness (higher is better) from a rollout. A caught arm always
/// outscores any miss, and is rewarded for catching *fast*. A miss is scored by
/// its **closest approach** to upright (`min_tip`), not its arbitrary final
/// state — so on a hard arm it can't yet catch, a candidate that swings *nearer*
/// the top scores higher and the search keeps a gradient instead of a flat
/// negative. (Separation: catches ≥ 25, misses ≤ −2, so the order never crosses.)
pub fn fitness(r: &Rollout) -> f64 {
    if r.caught {
        100.0 - 5.0 * r.time_to_catch // ∈ [25, 100]
    } else {
        -10.0 * r.min_tip // ∈ ~[−2, −60]; closer approach ⇒ higher
    }
}

/// A held-out set of arm configs `(l1, m1, b1)` for testing *generalization*:
/// these specific arms are not the ones domain-randomized training samples
/// (training draws continuously), so recovering them shows the policy transfers
/// to arms it never trained on, not memorizes a fixed set.
pub fn held_out_configs() -> Vec<(f64, f64, f64)> {
    let mut v = Vec::new();
    for &l1 in &[0.8, 1.3, 1.8, 2.3] {
        for &m1 in &[1.5, 2.5] {
            v.push((l1, m1, 0.05));
        }
    }
    v
}

/// Recovery rate of a policy across a set of arm configs (each tried from every
/// canonical knockdown start): returns `(caught, total)`.
pub fn recovery_rate_over<P: SwingUpPolicy>(
    policy: &P,
    configs: &[(f64, f64, f64)],
    secs: f64,
) -> (usize, usize) {
    let mask = recovered_mask(policy, configs, secs);
    (mask.iter().filter(|&&b| b).count(), mask.len())
}

/// Per-trial caught flags over `configs × knockdown_starts()`, row-major by
/// config then start. Lets callers compute a *union ceiling* (recovered by at
/// least one policy) and difficulty breakdowns.
pub fn recovered_mask<P: SwingUpPolicy>(
    policy: &P,
    configs: &[(f64, f64, f64)],
    secs: f64,
) -> Vec<bool> {
    let starts = knockdown_starts();
    let mut mask = Vec::with_capacity(configs.len() * starts.len());
    for &(l1, m1, b1) in configs {
        for (_, theta0) in &starts {
            mask.push(rollout_config(l1, m1, b1, theta0, policy, secs).caught);
        }
    }
    mask
}

/// The [`POLICY_LIBRARY`] champion that recovers the most knockdowns on a given
/// arm — the per-arm *best*. Used to build a **performance-keyed** policy store
/// (Stage 2.7): store the champion that *performs* best on each profile arm,
/// rather than the one that happened to *train* on it.
pub fn best_library_champion_for(l1: f64, m1: f64, b1: f64, secs: f64) -> [f64; NP] {
    let starts = knockdown_starts();
    POLICY_LIBRARY
        .iter()
        .max_by_key(|&&(_, _, _, p)| {
            starts
                .iter()
                .filter(|(_, t)| rollout_config(l1, m1, b1, t, &EnergyShapingPolicy { p }, secs).caught)
                .count()
        })
        .map(|&(_, _, _, p)| p)
        .expect("library is non-empty")
}

/// Link-length band for an arm, used to stratify the held-out generalization
/// report. (Counter-intuitively, *short* link-2 is the hard case here: less
/// leverage for the single motor to pump energy, so fewer knockdowns recover —
/// the report's per-band ceiling makes this plain.)
pub fn link_band(l1: f64, _m1: f64) -> &'static str {
    if l1 <= 1.0 {
        "short"
    } else if l1 <= 1.8 {
        "mid"
    } else {
        "long"
    }
}

/// **Live consumer** (Stage 2): recover an arm by *recalling* the nearest stored
/// swing-up policy from RuVector for that arm's config signature and running it
/// (LQR catch + recalled swing-up). Falls back to the hand-tuned baseline if no
/// policy is stored. This closes the loop: evolution discovers → RuVector stores
/// → the controller recalls and uses it at runtime.
#[cfg(feature = "vectordb")]
pub fn rollout_recalling_policy(
    mem: &crate::memory::ConfigMemory,
    l1: f64,
    m1: f64,
    b1: f64,
    theta0: &[f64],
    secs: f64,
) -> Rollout {
    let sig = mem.config_signature(l1, m1, b1);
    if let Ok(Some(rc)) = mem.recall_policy(&sig) {
        if rc.params.len() == NP {
            let mut p = [0.0; NP];
            p.copy_from_slice(&rc.params);
            return rollout_config(l1, m1, b1, theta0, &EnergyShapingPolicy { p }, secs);
        }
    }
    rollout_config(l1, m1, b1, theta0, &PflBaseline, secs)
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
    fn domain_randomized_champion_generalizes() {
        // The warm-started domain-randomized champion (evolve RANDOMIZE_ARM=1
        // SEED=2) judged on the held-out arms it never trained on: it must beat
        // the hand-tuned baseline clearly and at least match the nominal-only
        // champion's (surprisingly strong) cross-arm transfer. Pinned so the
        // "domain randomization generalizes" claim is a fast, reproducible check.
        let configs = held_out_configs();
        let dr = EnergyShapingPolicy { p: DR_CHAMPION };
        let nominal = EnergyShapingPolicy { p: NOMINAL_CHAMPION };
        let (dr_c, _) = recovery_rate_over(&dr, &configs, 15.0);
        let (base_c, _) = recovery_rate_over(&PflBaseline, &configs, 15.0);
        let (nom_c, _) = recovery_rate_over(&nominal, &configs, 15.0);
        assert!(dr_c > base_c, "DR ({dr_c}) should beat the hand-tuned baseline ({base_c})");
        assert!(dr_c >= nom_c, "DR ({dr_c}) should at least match the nominal champion ({nom_c})");
    }

    /// The Stage-2.5 finding: no single policy generalizes decisively — the
    /// *union* of policies (what per-arm recall can deploy) recovers far more
    /// held-out cases than the best single one. This is the structural reason
    /// domain randomization can't win alone, and why per-arm recall is the path.
    #[test]
    fn policy_union_exceeds_any_single() {
        let configs = held_out_configs();
        let mb = recovered_mask(&PflBaseline, &configs, 15.0);
        let mn = recovered_mask(&EnergyShapingPolicy { p: NOMINAL_CHAMPION }, &configs, 15.0);
        let md = recovered_mask(&EnergyShapingPolicy { p: DR_CHAMPION }, &configs, 15.0);
        let count = |m: &[bool]| m.iter().filter(|&&b| b).count();
        let best_single = count(&mb).max(count(&mn)).max(count(&md));
        let union = (0..mb.len()).filter(|&i| mb[i] || mn[i] || md[i]).count();
        assert!(
            union > best_single + 5,
            "union ({union}) should clearly exceed the best single policy ({best_single})"
        );
    }

    #[test]
    fn best_library_champion_is_the_per_arm_max() {
        // The helper must return the library champion that genuinely recovers the
        // most on the given arm (the per-arm best the oracle is built from).
        let (l1, m1, b1) = (1.0, 2.0, 0.05);
        let starts = knockdown_starts();
        let rec = |p: [f64; NP]| {
            starts
                .iter()
                .filter(|(_, t)| rollout_config(l1, m1, b1, t, &EnergyShapingPolicy { p }, 15.0).caught)
                .count()
        };
        let best = best_library_champion_for(l1, m1, b1, 15.0);
        let max_manual = POLICY_LIBRARY.iter().map(|&(_, _, _, p)| rec(p)).max().unwrap();
        assert_eq!(rec(best), max_manual, "helper returns the per-arm best library champion");
    }

    #[test]
    fn fitness_prefers_catching() {
        let caught = Rollout { caught: true, final_tip: 0.0, time_to_catch: 3.0, integral_tip: 20.0, min_tip: 0.0 };
        let missed = Rollout { caught: false, final_tip: 3.0, time_to_catch: 15.0, integral_tip: 200.0, min_tip: 2.0 };
        assert!(fitness(&caught) > fitness(&missed));
    }
}
