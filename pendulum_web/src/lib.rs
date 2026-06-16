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

use pendulum_rs::simulator::Pendulum;
use ruvector_core::types::DbOptions;
use ruvector_core::{DistanceMetric, SearchQuery, VectorDB, VectorEntry};

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
