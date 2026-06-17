//! Model-Predictive swing-up — the *predictive* counterpart to the reactive
//! energy-shaping policy in [`crate::learn`].
//!
//! The energy-shaping swing-up looks only at the arm's **current** state and
//! fires a torque. This controller instead *predicts where the arm is going*:
//! every few milliseconds it forks the real arm ([`Pendulum`] is `Clone`),
//! rolls a handful of candidate torque plans forward through the **exact same
//! RK4 dynamics** the simulator uses, scores each predicted trajectory, and
//! commits only the first move of the best plan — then re-plans (receding
//! horizon). No learned world model is needed: the simulator *is* a perfect
//! forward model, so the lookahead is exact.
//!
//! The plan search is the **Cross-Entropy Method** (CEM): sample a population
//! of torque sequences from a Gaussian, keep the lowest-cost "elite", refit the
//! Gaussian to them, repeat. The cost is *energy-aware* — pumping mechanical
//! energy toward the upright total is rewarded when far away (this is the
//! approximate cost-to-go that lets a short 0.4 s horizon still solve a
//! multi-second swing-up), while a velocity penalty that switches on only near
//! the top makes the arm *arrive catchable* so the LQR can grab it.
//!
//! Crucially [`MpcSwingUp`] implements [`SwingUpPolicy`], so it is a **drop-in**
//! for the energy-shaping policy: the same LQR catch, the same basin threshold,
//! the same knockdown harness. The only thing that changes is the swing-up
//! brain — reactive vs. predictive — which is exactly the comparison the
//! `mpc_check` binary reports.

use std::cell::{Cell, RefCell};
use std::f64::consts::PI;

use crate::control::{balance_gain, balance_torque, upright_energy};
use crate::learn::SwingUpPolicy;
use crate::simulator::Pendulum;

#[cfg(feature = "vectordb")]
use std::collections::HashMap;
#[cfg(feature = "vectordb")]
use std::rc::Rc;
#[cfg(feature = "vectordb")]
use ruvector_core::types::DbOptions;
#[cfg(feature = "vectordb")]
use ruvector_core::{DistanceMetric, SearchQuery, VectorDB, VectorEntry};

const DT: f64 = 0.005;
const U_MAX: f64 = 150.0;

/// Tunable knobs for the CEM-MPC planner.
#[derive(Debug, Clone)]
pub struct MpcConfig {
    /// Number of piecewise-constant torque "moves" in one plan.
    pub horizon: usize,
    /// Sim steps each move is held (so a move spans `hold_steps · dt` seconds,
    /// and the plan looks `horizon · hold_steps · dt` seconds ahead).
    pub hold_steps: usize,
    /// CEM population sampled per refinement iteration.
    pub pop: usize,
    /// How many lowest-cost candidates define the elite the Gaussian refits to.
    pub elite: usize,
    /// CEM refinement iterations per re-plan.
    pub iters: usize,
    /// Initial torque std-dev as a fraction of the motor limit.
    pub init_std_frac: f64,
    /// Weight on the terminal energy error `(E_end − E_up)²` — the swing-up pump.
    pub w_energy: f64,
    /// Weight on the *closest approach* to upright over the horizon — the catch.
    pub w_tip: f64,
    /// Weight on terminal velocity, gated to switch on only near the top.
    pub w_vel: f64,
    /// Weight on control effort `∫|u|` — keeps plans economical.
    pub w_u: f64,
    /// Seed for the deterministic sampler.
    pub seed: u64,
}

impl Default for MpcConfig {
    fn default() -> Self {
        Self {
            horizon: 12,
            hold_steps: 8, // a move = 0.04 s; plan looks 0.48 s ahead
            pop: 64,
            elite: 8,
            iters: 3,
            init_std_frac: 0.6,
            w_energy: 1.0,
            w_tip: 800.0,
            w_vel: 2.0,
            w_u: 0.0002,
            seed: 0x00C0_FFEE,
        }
    }
}

/// Tip error to the upright equilibrium `[π, π]`, wrapped to the short way.
fn tip(s: &Pendulum) -> f64 {
    let w = |a: f64| (a + PI).rem_euclid(2.0 * PI) - PI;
    w(s.theta[0] - PI).abs() + w(s.theta[1] - PI).abs()
}

/// Receding-horizon CEM swing-up controller.
pub struct MpcSwingUp {
    cfg: MpcConfig,
    state: RefCell<PlanState>,
    /// Total candidate trajectories rolled out while planning — the planner's
    /// compute cost. Warm-starting from [`PlanMemory`] aims to reach the same
    /// outcome at a smaller count.
    rollouts: Cell<u64>,
    /// Optional RuVector plan memory: `state → previously-good plan`. When set,
    /// each re-plan seeds CEM from the nearest remembered plan instead of cold.
    #[cfg(feature = "vectordb")]
    memory: Option<Rc<RefCell<PlanMemory>>>,
}

/// Mutable planner state behind a `RefCell` so the `&self` [`SwingUpPolicy`]
/// interface can carry a warm-started plan and a re-plan countdown between the
/// per-step calls the harness makes.
struct PlanState {
    /// Warm-start mean torque sequence (shifted forward after each commit).
    mean: Vec<f64>,
    /// Torque currently being held until the next re-plan.
    committed: f64,
    /// Sim steps left before the next re-plan.
    countdown: usize,
    rng: Smix,
}

impl MpcSwingUp {
    pub fn new(cfg: MpcConfig) -> Self {
        let mean = vec![0.0; cfg.horizon];
        let seed = cfg.seed;
        Self {
            cfg,
            state: RefCell::new(PlanState { mean, committed: 0.0, countdown: 0, rng: Smix(seed) }),
            rollouts: Cell::new(0),
            #[cfg(feature = "vectordb")]
            memory: None,
        }
    }

    /// As [`MpcSwingUp::new`] but backed by a shared RuVector [`PlanMemory`].
    /// Pass the *same* `Rc` to several controllers so they pool their plans —
    /// the more states the memory has seen, the warmer each cold start.
    #[cfg(feature = "vectordb")]
    pub fn with_memory(cfg: MpcConfig, memory: Rc<RefCell<PlanMemory>>) -> Self {
        let mut s = Self::new(cfg);
        s.memory = Some(memory);
        s
    }

    /// Total candidate trajectories this controller has rolled out while
    /// planning — the measure warm-starting is meant to shrink.
    pub fn planning_rollouts(&self) -> u64 {
        self.rollouts.get()
    }

    /// Predicted cost of executing `seq` (one torque per move) from the forked
    /// arm `sim`. Lower is better.
    fn cost(&self, sim: &Pendulum, seq: &[f64], e_up: f64, u_max: f64) -> f64 {
        let mut s = sim.clone();
        let dt = s.dt;
        let mut min_tip = tip(&s);
        let mut effort = 0.0;
        for &u in seq {
            let uc = u.clamp(-u_max, u_max);
            for _ in 0..self.cfg.hold_steps {
                s.step(&[uc, 0.0]);
                min_tip = min_tip.min(tip(&s));
                effort += uc.abs() * dt;
            }
        }
        let e_end = s.total_energy();
        let tip_end = tip(&s);
        let vel_sq = s.omega[0] * s.omega[0] + s.omega[1] * s.omega[1];
        // The velocity penalty only bites near the top (tip_end → 0): during the
        // pump, high speed is *wanted*, so the gate frees it.
        let gate = (-(tip_end * tip_end) / 0.5).exp();
        self.cfg.w_energy * (e_end - e_up).powi(2)
            + self.cfg.w_tip * min_tip * min_tip
            + self.cfg.w_vel * vel_sq * gate
            + self.cfg.w_u * effort
    }

    /// Run CEM from the current state and return the optimized mean plan.
    fn plan(&self, sim: &Pendulum, e_up: f64, u_max: f64, st: &mut PlanState) -> Vec<f64> {
        let h = self.cfg.horizon;
        let mut mean = st.mean.clone();

        // Warm start: a plan RuVector remembers for a nearby state. We do NOT
        // replace the search with it — we inject it as one extra candidate each
        // iteration, so it can only help: if it scores well it pulls the elite
        // toward it (a free refinement worth an extra CEM pass); if it doesn't
        // transfer, it simply loses and is ignored. That keeps recall strictly
        // safe — unlike overwriting the mean, which can lock a cheap search onto
        // a stale plan it then can't escape. No memory ⇒ recalled is None.
        let recalled: Option<Vec<f64>> = {
            #[cfg(feature = "vectordb")]
            {
                self.memory
                    .as_ref()
                    .and_then(|m| m.borrow().recall(&embedding(sim)))
                    .filter(|p| p.len() == h)
            }
            #[cfg(not(feature = "vectordb"))]
            {
                None
            }
        };

        let mut std = vec![self.cfg.init_std_frac * u_max; h];
        let floor = 0.05 * u_max; // don't let the search collapse to a point

        for _ in 0..self.cfg.iters {
            // pop samples + the current mean + (optionally) the recalled plan.
            let extra = 1 + recalled.is_some() as u64;
            self.rollouts.set(self.rollouts.get() + self.cfg.pop as u64 + extra);
            // Sample the population, plus the current mean as a guaranteed elite
            // candidate (keeps the search monotone — it can never get worse).
            let mut cands: Vec<Vec<f64>> = Vec::with_capacity(self.cfg.pop + 2);
            for _ in 0..self.cfg.pop {
                cands.push((0..h).map(|k| (mean[k] + std[k] * st.rng.gauss()).clamp(-u_max, u_max)).collect());
            }
            cands.push(mean.clone());
            if let Some(r) = &recalled {
                cands.push(r.clone());
            }

            let mut scored: Vec<(f64, usize)> =
                cands.iter().enumerate().map(|(i, seq)| (self.cost(sim, seq, e_up, u_max), i)).collect();
            scored.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

            let elite = self.cfg.elite.min(scored.len());
            for k in 0..h {
                let m = scored[..elite].iter().map(|&(_, i)| cands[i][k]).sum::<f64>() / elite as f64;
                let var =
                    scored[..elite].iter().map(|&(_, i)| (cands[i][k] - m).powi(2)).sum::<f64>() / elite as f64;
                mean[k] = m;
                std[k] = var.sqrt().max(floor);
            }
        }

        // Remember the refined plan for this state so future near-by states can
        // warm-start from it.
        #[cfg(feature = "vectordb")]
        if let Some(mem) = &self.memory {
            mem.borrow_mut().insert(&embedding(sim), &mean);
        }
        mean
    }
}

impl SwingUpPolicy for MpcSwingUp {
    fn torque(&self, sim: &Pendulum, e_up: f64, u_max: f64) -> f64 {
        let mut st = self.state.borrow_mut();
        if st.countdown == 0 {
            let plan = self.plan(sim, e_up, u_max, &mut st);
            st.committed = plan[0].clamp(-u_max, u_max);
            // Warm-start the next re-plan from this one shifted forward by a move.
            let mut shifted = plan[1..].to_vec();
            shifted.push(*plan.last().unwrap());
            st.mean = shifted;
            st.countdown = self.cfg.hold_steps;
        }
        st.countdown -= 1;
        st.committed
    }
}

/// Width of the state embedding RuVector indexes plans by.
#[cfg(feature = "vectordb")]
const EMB_DIM: usize = 6;

/// Embed the arm state for plan recall. Angles go in as `(cos, sin)` so the
/// wrap-around at ±π is continuous; velocities are scaled to sit on the same
/// order as the trig terms, so Euclidean distance weights pose and motion
/// comparably.
#[cfg(feature = "vectordb")]
fn embedding(sim: &Pendulum) -> Vec<f32> {
    let vs = 10.0; // rad/s that maps to a unit of embedding distance
    vec![
        sim.theta[0].cos() as f32,
        sim.theta[0].sin() as f32,
        sim.theta[1].cos() as f32,
        sim.theta[1].sin() as f32,
        (sim.omega[0] / vs) as f32,
        (sim.omega[1] / vs) as f32,
    ]
}

/// A RuVector store of `state → plan`: the controller's *collective memory of
/// what worked*. Each refined plan is inserted keyed by its [`embedding`]; a
/// re-plan recalls the nearest one to warm-start from. This is RuVector doing
/// what it is actually good at — fast nearest-neighbour lookup — to accelerate
/// the predictive planner, rather than pretending to be a learned world model.
#[cfg(feature = "vectordb")]
pub struct PlanMemory {
    db: VectorDB,
    next: usize,
    horizon: usize,
    /// Max embedding distance at which a remembered plan is reused.
    recall_thresh: f32,
}

#[cfg(feature = "vectordb")]
impl PlanMemory {
    /// Open a **fresh** store (any existing file is removed so a "practice" run
    /// starts with no memory). `horizon` must match the controller's plan length.
    pub fn new(storage_path: &str, horizon: usize, recall_thresh: f32) -> Result<Self, Box<dyn std::error::Error>> {
        let _ = std::fs::remove_file(storage_path);
        let opts = DbOptions {
            dimensions: EMB_DIM,
            distance_metric: DistanceMetric::Euclidean,
            storage_path: storage_path.to_string(),
            ..Default::default()
        };
        Ok(Self { db: VectorDB::new(opts)?, next: 0, horizon, recall_thresh })
    }

    /// How many plans are remembered so far.
    pub fn len(&self) -> usize {
        self.next
    }

    pub fn is_empty(&self) -> bool {
        self.next == 0
    }

    /// The nearest remembered plan to `emb`, if one is within `recall_thresh`.
    fn recall(&self, emb: &[f32]) -> Option<Vec<f64>> {
        let res = self.db.search(SearchQuery { vector: emb.to_vec(), k: 1, filter: None, ef_search: None }).ok()?;
        let top = res.into_iter().next()?;
        if top.score > self.recall_thresh {
            return None;
        }
        let plan = top
            .metadata?
            .get("plan")?
            .as_array()?
            .iter()
            .filter_map(|x| x.as_f64())
            .collect::<Vec<f64>>();
        (plan.len() == self.horizon).then_some(plan)
    }

    /// Remember `plan` for state `emb`.
    fn insert(&mut self, emb: &[f32], plan: &[f64]) {
        let mut md = HashMap::new();
        md.insert("plan".to_string(), serde_json::json!(plan));
        let _ = self.db.insert(VectorEntry {
            id: Some(format!("s{}", self.next)),
            vector: emb.to_vec(),
            metadata: Some(md),
        });
        self.next += 1;
    }
}

/// Rich per-rollout metrics — a superset of [`crate::learn::Rollout`] that adds
/// the two "fewer moves" signals: total actuation and torque reversals.
#[derive(Debug, Clone, Copy)]
pub struct Metrics {
    /// Ended balanced upright (tip error < 0.2 rad).
    pub caught: bool,
    /// Time (s) it first held upright for ≥1 s (else the full duration).
    pub time_to_catch: f64,
    /// `∫|u| dt` — total actuation spent (the "how hard did it work" signal).
    pub effort: f64,
    /// Number of torque sign reversals — the back-and-forth "move" count.
    pub reversals: usize,
    /// Closest the arm ever got to upright.
    pub min_tip: f64,
    /// Tip error at the end of the rollout.
    pub final_tip: f64,
}

/// Drive one knockdown recovery under `policy` (LQR catch inside the basin, the
/// policy's swing-up outside it) and record the rich [`Metrics`]. The control
/// path is byte-for-byte the same as [`crate::learn::rollout_config`]; this just
/// also tallies effort and reversals so any two [`SwingUpPolicy`] implementations
/// can be compared on identical ground.
pub fn rollout_metrics<P: SwingUpPolicy>(
    l1: f64,
    m1: f64,
    b1: f64,
    theta0: &[f64],
    policy: &P,
    secs: f64,
) -> Metrics {
    let mut sim = Pendulum::new(vec![1.0, m1], vec![1.0, l1], vec![0.05, b1], 9.81, DT);
    sim.reset(theta0.to_vec(), vec![0.0, 0.0]);
    let k = balance_gain(&sim, DT);
    let e_up = upright_energy(&sim);

    let mut effort = 0.0;
    let mut reversals = 0usize;
    let mut last_sign = 0i32;
    let mut hold = 0.0;
    let mut time_to_catch = secs;
    let mut caught_once = false;
    let mut min_tip = tip(&sim);

    let steps = (secs / DT) as usize;
    for step in 0..steps {
        let e = tip(&sim);
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

        // Hybrid: LQR inside the basin, the policy's swing-up outside it.
        let u = if e < 1.0 {
            balance_torque(&k, &sim.theta, &sim.omega, U_MAX)
        } else {
            policy.torque(&sim, e_up, U_MAX)
        };

        effort += u.abs() * DT;
        let sign = if u > 1e-6 { 1 } else if u < -1e-6 { -1 } else { 0 };
        if sign != 0 {
            if last_sign != 0 && sign != last_sign {
                reversals += 1;
            }
            last_sign = sign;
        }

        sim.step(&[u, 0.0]);
    }

    let final_tip = tip(&sim);
    Metrics { caught: final_tip < 0.2, time_to_catch, effort, reversals, min_tip, final_tip }
}

/// Deterministic splitmix64 — mirrors the RNG in [`crate::learn`] so the planner
/// is reproducible without pulling an external crate.
struct Smix(u64);
impl Smix {
    fn u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn unit(&mut self) -> f64 {
        (self.u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn gauss(&mut self) -> f64 {
        let u1 = self.unit().max(1e-12);
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * self.unit()).cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::learn::knockdown_starts;

    /// The headline claim: predictive MPC swings the nominal arm up and into the
    /// LQR catch from a dead hang — using only the exact simulator as its model.
    #[test]
    fn mpc_swings_up_from_a_dead_hang() {
        let mpc = MpcSwingUp::new(MpcConfig::default());
        let m = rollout_metrics(1.0, 1.0, 0.05, &[0.1, -0.1], &mpc, 15.0);
        assert!(m.caught, "MPC should catch from a dead hang, final tip {:.3}", m.final_tip);
    }

    /// On the canonical knockdown harness, predictive MPC should be a credible
    /// swing-up controller (recovers most starts) — the `mpc_check` binary
    /// reports the full effort/reversal comparison.
    #[test]
    fn mpc_recovers_most_knockdowns() {
        let recovered = knockdown_starts()
            .iter()
            .filter(|(_, t)| {
                let mpc = MpcSwingUp::new(MpcConfig::default());
                rollout_metrics(1.0, 1.0, 0.05, t, &mpc, 15.0).caught
            })
            .count();
        assert!(recovered >= 7, "MPC should recover ≥7/10 knockdowns, got {recovered}");
    }
}
