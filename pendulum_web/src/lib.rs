//! WebAssembly bindings for the pendulum × RuVector exhibit.
//!
//! Everything here runs **in the browser tab** — the physics (same RK4 Lagrangian
//! dynamics as the native crate), and RuVector's vector database (in-memory). The
//! JS/Svelte shell creates these handles, steps them once per animation frame, and
//! reads flat position arrays to draw on a Canvas2D.
//!
//! M0 (the spike) exposes one station — [`FreeSwing`] — plus a [`ruvector_smoke`]
//! call that proves RuVector runs in the tab and keeps it linked into the bundle
//! for an honest size measurement.

use wasm_bindgen::prelude::*;

use pendulum_rs::control::{balance_gain, balance_torque, nominal_probe_gain, Vec4};
use pendulum_rs::estimator::{OnlineEstimator, Signature, SIG_DIM};
use pendulum_rs::memory::ConfigMemory;
use pendulum_rs::simulator::Pendulum;
use ruvector_core::types::DbOptions;
use ruvector_core::{DistanceMetric, SearchQuery, VectorDB, VectorEntry};
use std::f64::consts::PI;

/// Control timestep — matches the native crate so the browser physics is identical.
const DT: f64 = 0.005;

#[wasm_bindgen(start)]
pub fn start() {
    #[cfg(feature = "console_error_panic_hook")]
    console_error_panic_hook::set_once();
}

/// Station 0 — a free-swinging n-link pendulum. Released from a sprawl and left
/// passive (no applied torque), it swings chaotically: the warm-up that motivates
/// why remembering past dynamics (RuVector) is worth anything.
#[wasm_bindgen]
pub struct FreeSwing {
    sim: Pendulum,
}

#[wasm_bindgen]
impl FreeSwing {
    /// `links` ∈ [1, 6]; `damping` is per-joint viscous friction (0 = frictionless).
    #[wasm_bindgen(constructor)]
    pub fn new(links: usize, damping: f64) -> FreeSwing {
        let n = links.clamp(1, 6);
        let mut sim = Pendulum::new(vec![1.0; n], vec![1.0; n], vec![damping; n], 9.81, DT);
        // A near-horizontal sprawl so it starts with energy and swings visibly.
        let theta0: Vec<f64> = (0..n).map(|i| 1.2 - 0.15 * i as f64).collect();
        sim.reset(theta0, vec![0.0; n]);
        FreeSwing { sim }
    }

    /// Advance the physics by `steps` fixed timesteps (passive — zero torque).
    pub fn step(&mut self, steps: usize) {
        let zero = vec![0.0; self.sim.n];
        for _ in 0..steps {
            self.sim.step(&zero);
        }
    }

    /// Live-tune per-joint damping from a slider.
    pub fn set_damping(&mut self, d: f64) {
        for i in 0..self.sim.n {
            self.sim.set_damping(i, d.max(0.0));
        }
    }

    /// A tiny kick to the tip joint — the "chaos" button. Two identical arms given
    /// this nudge diverge within seconds.
    pub fn nudge(&mut self, delta: f64) {
        let last = self.sim.n - 1;
        self.sim.omega[last] += delta;
    }

    /// Flat `[x0, y0, x1, y1, …]` joint positions including the anchor (n+1 points),
    /// in physics units. The Canvas2D renderer scales these to pixels. Returned as
    /// a `Float64Array` to JS.
    pub fn positions(&self) -> Vec<f64> {
        self.sim
            .link_positions()
            .into_iter()
            .flat_map(|(x, y)| [x, y])
            .collect()
    }

    /// Total mechanical energy — used to show that the passive system conserves it
    /// (and to compare native vs wasm: it should match the native reference).
    pub fn energy(&self) -> f64 {
        self.sim.total_energy()
    }

    /// Number of links.
    pub fn links(&self) -> usize {
        self.sim.n
    }
}

/// Proof that RuVector's in-memory vector DB runs in the browser. Creates a tiny
/// in-memory store, inserts two vectors, and returns the id of the nearest match
/// to a query — entirely client-side, no server. Also keeps `ruvector-core` linked
/// into the wasm bundle so M0's size measurement reflects the real page.
#[wasm_bindgen]
pub fn ruvector_smoke() -> String {
    let opts = DbOptions {
        dimensions: 3,
        distance_metric: DistanceMetric::Euclidean,
        // Ignored in memory-only mode (no `storage` feature => MemoryStorage).
        storage_path: "mem".to_string(),
        ..Default::default()
    };
    let db = match VectorDB::new(opts) {
        Ok(db) => db,
        Err(e) => return format!("init error: {e:?}"),
    };
    let _ = db.insert(VectorEntry {
        id: Some("origin".into()),
        vector: vec![0.0, 0.0, 0.0],
        metadata: None,
    });
    let _ = db.insert(VectorEntry {
        id: Some("far".into()),
        vector: vec![1.0, 1.0, 1.0],
        metadata: None,
    });
    match db.search(SearchQuery {
        vector: vec![0.95, 0.95, 0.95],
        k: 1,
        filter: None,
        ef_search: None,
    }) {
        Ok(results) => results
            .into_iter()
            .next()
            .map(|r| format!("nearest={} (score {:.3})", r.id, r.score))
            .unwrap_or_else(|| "no results".into()),
        Err(e) => format!("search error: {e:?}"),
    }
}

// ───────────────────────── Station: Recognize (Phase 2) ─────────────────────
// A faithful, steppable port of `pendulum_rs/src/bin/estimate.rs`: two
// underactuated arms balance upright; at t=1s link-2 grows on both. The naive arm
// keeps its stale gain and topples; the adaptive arm runs a dithered probe,
// identifies its live dynamics signature, recalls the nearest arm from RuVector,
// and adopts that gain. A successful catch is written back, so the *same*
// disturbance is recognized faster next time — the learning.

const U_MAX: f64 = 150.0;
const DISTURB_T: f64 = 1.0;
const MIN_SAMPLES: usize = 25;
const PROBE_WINDOW: usize = 240;
const CHECK_EVERY: usize = 5;
const FREEZE_TIP: f64 = 0.18;
const GRID_TIGHT: f32 = 1.5;
const GRID_LOOSE: f32 = 5.0;
const GRID_CONSENSUS: usize = 3;
const LEARNED_THRESH: f32 = 13.0;
const LEARNED_CONSENSUS: usize = 2;
const EMA_ALPHA: f64 = 0.5;

fn nominal_arm() -> Pendulum {
    Pendulum::new(vec![1.0, 1.0], vec![1.0, 1.0], vec![0.05, 0.05], 9.81, DT)
}

fn tip_error(sim: &Pendulum) -> f64 {
    let w = |a: f64| (a + PI).rem_euclid(2.0 * PI) - PI;
    w(sim.theta[0] - PI).abs() + w(sim.theta[1] - PI).abs()
}

fn probe_torque(k: &Vec4, sim: &Pendulum, t: f64) -> (f64, f64) {
    let e0 = (sim.theta[0] - PI + PI).rem_euclid(2.0 * PI) - PI;
    let e1 = (sim.theta[1] - PI + PI).rem_euclid(2.0 * PI) - PI;
    let u_fb = -(k[0] * e0 + k[1] * e1 + k[2] * sim.omega[0] + k[3] * sim.omega[1]);
    let dither = 6.0 * (2.0 * PI * 1.7 * t).sin() + 4.0 * (2.0 * PI * 3.3 * t).sin();
    ((u_fb + dither).clamp(-U_MAX, U_MAX), dither)
}

fn flat(sim: &Pendulum) -> Vec<f64> {
    sim.link_positions().into_iter().flat_map(|(x, y)| [x, y]).collect()
}

/// Station 2 — RuVector recognizes a changed arm and recalls its gain.
#[wasm_bindgen]
pub struct Recalibrator {
    mem: ConfigMemory,
    naive: Pendulum,
    adaptive: Pendulum,
    k_naive: Vec4,
    k_adaptive: Vec4,
    k_probe: Vec4,
    est: OnlineEstimator,

    new_l1: f64,
    learn_enabled: bool,

    step: usize,
    probing: bool,
    committed: bool,
    disturbed: bool,

    lag: f64, // -1 = not yet recognized this encounter
    last_lag: f64,
    recalled_id: String,
    recalled_l1: f64,
    recalled_learned: bool,
    recall_distance: f64,

    agree_id: String,
    agree_count: usize,
    smoothed: Option<Signature>,
    measured_sig: Option<Signature>,
    encounter: usize,
}

#[wasm_bindgen]
impl Recalibrator {
    #[wasm_bindgen(constructor)]
    pub fn new(new_l1: f64) -> Recalibrator {
        // Fresh in-memory RuVector store seeded with the arm grid.
        let mut mem = ConfigMemory::new("phase2_configs.db").expect("memory init");
        let _ = mem.seed_grid();
        let mut r = Recalibrator {
            mem,
            naive: nominal_arm(),
            adaptive: nominal_arm(),
            k_naive: [0.0; 4],
            k_adaptive: [0.0; 4],
            k_probe: nominal_probe_gain(DT),
            est: OnlineEstimator::new(PROBE_WINDOW, 1e-4),
            new_l1,
            learn_enabled: true,
            step: 0,
            probing: false,
            committed: false,
            disturbed: false,
            lag: -1.0,
            last_lag: -1.0,
            recalled_id: String::new(),
            recalled_l1: 0.0,
            recalled_learned: false,
            recall_distance: 0.0,
            agree_id: String::new(),
            agree_count: 0,
            smoothed: None,
            measured_sig: None,
            encounter: 1,
        };
        r.start_scenario();
        r
    }

    fn start_scenario(&mut self) {
        self.naive = nominal_arm();
        self.adaptive = nominal_arm();
        let theta0 = vec![PI - 0.05, PI + 0.05];
        self.naive.reset(theta0.clone(), vec![0.0; 2]);
        self.adaptive.reset(theta0, vec![0.0; 2]);
        let k0 = balance_gain(&self.naive, DT);
        self.k_naive = k0;
        self.k_adaptive = k0;
        self.k_probe = nominal_probe_gain(DT);
        self.est.clear();
        self.step = 0;
        self.probing = false;
        self.committed = false;
        self.disturbed = false;
        self.lag = -1.0;
        self.recalled_id = String::new();
        self.recalled_l1 = 0.0;
        self.recalled_learned = false;
        self.recall_distance = 0.0;
        self.agree_id = String::new();
        self.agree_count = 0;
        self.smoothed = None;
        self.measured_sig = None;
    }

    /// Set the disturbance length (link-2's new length); applied on the next
    /// encounter. If still pre-disturbance this encounter, it takes effect here.
    pub fn set_new_len(&mut self, l1: f64) {
        self.new_l1 = l1.clamp(0.6, 3.0);
    }

    pub fn set_learning(&mut self, on: bool) {
        self.learn_enabled = on;
    }

    /// Throw the same disturbance again, keeping what RuVector has learned —
    /// the lag should shrink on a repeat.
    pub fn next_encounter(&mut self) {
        if self.lag >= 0.0 {
            self.last_lag = self.lag;
        }
        self.encounter += 1;
        self.start_scenario();
    }

    /// Wipe everything RuVector learned and re-seed the cold grid.
    pub fn forget(&mut self) {
        if let Ok(mut m) = ConfigMemory::new("phase2_configs.db") {
            let _ = m.seed_grid();
            self.mem = m;
        }
        self.encounter = 1;
        self.last_lag = -1.0;
        self.start_scenario();
    }

    /// Advance the scenario by `steps` control timesteps.
    pub fn tick(&mut self, steps: usize) {
        let disturb_step = (DISTURB_T / DT) as usize;
        for _ in 0..steps {
            let step = self.step;
            let t = step as f64 * DT;

            if step == disturb_step {
                for arm in [&mut self.naive, &mut self.adaptive] {
                    arm.set_length(1, self.new_l1);
                }
                self.probing = true;
                self.disturbed = true;
                self.est.clear();
            }

            // naive: stale gain forever
            let un = balance_torque(&self.k_naive, &self.naive.theta, &self.naive.omega, U_MAX);
            self.naive.step(&[un, 0.0]);

            // adaptive
            if self.probing && !self.committed {
                let theta_before = self.adaptive.theta.clone();
                let omega_before = self.adaptive.omega.clone();
                let clean = tip_error(&self.adaptive) < FREEZE_TIP;
                let (u, dither) = probe_torque(&self.k_probe, &self.adaptive, t);
                self.adaptive.step(&[u, 0.0]);
                if clean {
                    self.est.observe(&theta_before, &omega_before, dither, &self.adaptive.omega, DT);
                }

                if self.est.len() >= MIN_SAMPLES && step % CHECK_EVERY == 0 {
                    if let Some(raw) = self.est.estimate() {
                        let sig: Signature = match self.smoothed {
                            Some(prev) => {
                                let mut s = [0.0f64; SIG_DIM];
                                for i in 0..SIG_DIM {
                                    s[i] = EMA_ALPHA * raw[i] + (1.0 - EMA_ALPHA) * prev[i];
                                }
                                self.smoothed = Some(s);
                                s
                            }
                            None => {
                                self.smoothed = Some(raw);
                                raw
                            }
                        };
                        if let Ok(Some(rc)) = self.mem.recall(&sig) {
                            self.recall_distance = rc.score as f64;
                            if rc.id == self.agree_id {
                                self.agree_count += 1;
                            } else {
                                self.agree_id = rc.id.clone();
                                self.agree_count = 1;
                            }
                            let commit = if rc.learned {
                                rc.score < LEARNED_THRESH && self.agree_count >= LEARNED_CONSENSUS
                            } else {
                                rc.score < GRID_TIGHT
                                    || (rc.score < GRID_LOOSE && self.agree_count >= GRID_CONSENSUS)
                            };
                            if commit {
                                self.k_adaptive = rc.k;
                                self.committed = true;
                                self.lag = t - DISTURB_T;
                                self.recalled_id = rc.id.clone();
                                self.recalled_l1 = rc.l1;
                                self.recalled_learned = rc.learned;
                                self.measured_sig = Some(sig);
                                // Self-learning: remember this arm's measured
                                // signature so the next encounter is faster.
                                if self.learn_enabled {
                                    let _ = self.mem.learn_from_arm(sig, self.new_l1, 1.0, 0.05);
                                }
                            }
                        }
                    }
                }
            } else {
                let ua =
                    balance_torque(&self.k_adaptive, &self.adaptive.theta, &self.adaptive.omega, U_MAX);
                self.adaptive.step(&[ua, 0.0]);
            }

            self.step += 1;
        }
    }

    // --- rendering ---
    pub fn naive_positions(&self) -> Vec<f64> {
        flat(&self.naive)
    }
    pub fn adaptive_positions(&self) -> Vec<f64> {
        flat(&self.adaptive)
    }

    // --- HUD ---
    pub fn tip_error_naive(&self) -> f64 {
        tip_error(&self.naive)
    }
    pub fn tip_error_adaptive(&self) -> f64 {
        tip_error(&self.adaptive)
    }
    /// "nominal" | "probing" | "recognized"
    pub fn phase(&self) -> String {
        if !self.disturbed {
            "nominal".into()
        } else if self.committed {
            "recognized".into()
        } else {
            "probing".into()
        }
    }
    pub fn lag(&self) -> f64 {
        self.lag
    }
    pub fn last_lag(&self) -> f64 {
        self.last_lag
    }
    pub fn recalled_id(&self) -> String {
        self.recalled_id.clone()
    }
    pub fn recalled_l1(&self) -> f64 {
        self.recalled_l1
    }
    pub fn recalled_learned(&self) -> bool {
        self.recalled_learned
    }
    pub fn recall_distance(&self) -> f64 {
        self.recall_distance
    }
    pub fn committed(&self) -> bool {
        self.committed
    }
    pub fn disturbed(&self) -> bool {
        self.disturbed
    }
    pub fn time(&self) -> f64 {
        self.step as f64 * DT
    }
    pub fn new_len(&self) -> f64 {
        self.new_l1
    }
    pub fn encounter(&self) -> usize {
        self.encounter
    }
}
