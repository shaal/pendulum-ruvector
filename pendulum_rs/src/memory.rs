//! Phase 2 — RuVector *is* the estimator. This module stores arm configurations
//! in RuVector keyed by their dynamics [`Signature`], and recalls the nearest
//! one for a measured signature. That recall is what replaces the Phase-1
//! oracle: instead of being *told* the new parameters, the arm looks up the
//! closest arm it has seen and reuses its known-good gain.
//!
//! Layout: each seed's embedding is the *whitened* signature (z-scored per
//! dimension against the seed grid, so gravity-stiffness terms ~tens and input
//! terms ~1 contribute equally to nearest-neighbor distance). The payload
//! metadata carries the true parameters, the balance gain `K`, and the upright
//! energy `e_up` — everything the controller needs to adopt that arm.

use crate::control::{balance_gain, nominal_probe_gain, upright_energy, Vec4};
use crate::estimator::{closed_loop_signature, Signature, SIG_DIM};
use crate::simulator::Pendulum;
use ruvector_core::types::DbOptions;
use ruvector_core::{DistanceMetric, SearchQuery, VectorDB, VectorEntry};
use std::collections::HashMap;

/// A configuration recalled from RuVector for a measured signature.
#[derive(Debug, Clone)]
pub struct RecalledConfig {
    pub id: String,
    /// Whitened-L2 distance to the query — lower means a more confident match.
    pub score: f32,
    pub l1: f64,
    pub m1: f64,
    pub b1: f64,
    /// The balance gain to adopt.
    pub k: Vec4,
    /// Upright energy target (for the swing-up controller).
    pub e_up: f64,
    /// `true` if this entry was written from a real catch (a config we have
    /// actually stabilized), vs a coarse offline grid seed. A learned match can
    /// be trusted from a rougher/earlier estimate — that is what shrinks the lag.
    pub learned: bool,
}

/// The control timestep the seeded gains are computed at (must match the
/// controller's `dt`, since the discrete LQR gain depends on it).
const SEED_DT: f64 = 0.005;

/// RuVector-backed memory of arm configurations.
pub struct ConfigMemory {
    db: VectorDB,
    /// Per-dimension whitening statistics, learned from the seed grid.
    mean: Signature,
    std: Signature,
    next_id: usize,
}

impl ConfigMemory {
    /// Open a **fresh** RuVector store at `storage_path` (any existing file is
    /// removed so each demo run starts clean and the self-learning curve is
    /// reproducible). The index is L2 over whitened signatures.
    pub fn new(storage_path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        // Start from an empty store: persisted seeds/learned points from a prior
        // run would otherwise make the "first encounter" already-learned.
        let _ = std::fs::remove_file(storage_path);
        let opts = DbOptions {
            dimensions: SIG_DIM,
            distance_metric: DistanceMetric::Euclidean,
            storage_path: storage_path.to_string(),
            ..Default::default()
        };
        Ok(Self {
            db: VectorDB::new(opts)?,
            mean: [0.0; SIG_DIM],
            std: [1.0; SIG_DIM],
            next_id: 0,
        })
    }

    /// Build an arm for a `(l1, m1, b1)` config (link-1 fixed at the reference
    /// 1 m / 1 kg / 0.05 friction; the second link is what we vary).
    fn arm(l1: f64, m1: f64, b1: f64) -> Pendulum {
        Pendulum::new(vec![1.0, m1], vec![1.0, l1], vec![0.05, b1], 9.81, SEED_DT)
    }

    /// Sweep a grid of `(link-2 length, payload mass, friction)` and seed each
    /// arm's signature → {params, K, e_up}. Whitening stats are computed from the
    /// grid first, then every seed is inserted in the shared whitened space.
    pub fn seed_grid(&mut self) -> Result<usize, Box<dyn std::error::Error>> {
        let lengths = [0.6, 1.0, 1.5, 2.0, 2.5];
        let masses = [1.0, 2.0, 3.0];
        let frictions = [0.05, 0.30];

        // The probe gain every signature is fingerprinted under (offline here,
        // online in the estimator) — must be identical on both sides.
        let k_probe = nominal_probe_gain(SEED_DT);

        // Pass 1: compute every config + its closed-loop signature.
        let mut configs: Vec<(f64, f64, f64, Signature, Vec4, f64)> = Vec::new();
        for &l1 in &lengths {
            for &m1 in &masses {
                for &b1 in &frictions {
                    let sim = Self::arm(l1, m1, b1);
                    let sig = closed_loop_signature(&sim, &k_probe);
                    let k = balance_gain(&sim, SEED_DT);
                    let e_up = upright_energy(&sim);
                    configs.push((l1, m1, b1, sig, k, e_up));
                }
            }
        }

        // Compute whitening stats (per-dimension mean / std) from the grid.
        let n = configs.len() as f64;
        let mut mean = [0.0f64; SIG_DIM];
        for c in &configs {
            for i in 0..SIG_DIM {
                mean[i] += c.3[i];
            }
        }
        for m in &mut mean {
            *m /= n;
        }
        let mut std = [0.0f64; SIG_DIM];
        for c in &configs {
            for i in 0..SIG_DIM {
                let d = c.3[i] - mean[i];
                std[i] += d * d;
            }
        }
        for s in &mut std {
            *s = (*s / n).sqrt();
        }
        // Floor each std relative to the *largest* spread. Some signature dims
        // (the damping/velocity coefficients) barely vary across the grid yet
        // are noisy to identify online; without a floor, whitening would divide
        // their estimation error by ~0 and let a non-discriminative dimension
        // dominate the nearest-neighbor distance. The floor caps that.
        let max_std = std.iter().cloned().fold(0.0f64, f64::max).max(1e-9);
        let floor = 0.1 * max_std;
        for s in &mut std {
            *s = s.max(floor);
        }
        self.mean = mean;
        self.std = std;

        // Pass 2: insert every seed in the whitened space (grid, not learned).
        for (l1, m1, b1, sig, k, e_up) in configs {
            self.insert(sig, l1, m1, b1, k, e_up, false)?;
        }
        Ok(self.next_id)
    }

    /// The exact probe gain the seeds were fingerprinted under. Online probing
    /// **must** apply this gain (not one recomputed at a different control dt),
    /// or the measured closed-loop signature won't live in the seeded space.
    pub fn probe_gain(&self) -> Vec4 {
        nominal_probe_gain(SEED_DT)
    }

    /// Whiten a raw signature into the embedding actually stored / queried.
    fn whiten(&self, sig: &Signature) -> Vec<f32> {
        (0..SIG_DIM)
            .map(|i| ((sig[i] - self.mean[i]) / self.std[i]) as f32)
            .collect()
    }

    /// Insert a config (used by both `seed_grid` and the self-learning loop).
    /// `learned` marks entries written from a real catch (vs offline grid seeds).
    pub fn insert(
        &mut self,
        sig: Signature,
        l1: f64,
        m1: f64,
        b1: f64,
        k: Vec4,
        e_up: f64,
        learned: bool,
    ) -> Result<String, Box<dyn std::error::Error>> {
        let id = format!("cfg{}", self.next_id);
        self.next_id += 1;
        let mut md = HashMap::new();
        md.insert("l1".to_string(), serde_json::json!(l1));
        md.insert("m1".to_string(), serde_json::json!(m1));
        md.insert("b1".to_string(), serde_json::json!(b1));
        md.insert("k".to_string(), serde_json::json!(k));
        md.insert("e_up".to_string(), serde_json::json!(e_up));
        md.insert("learned".to_string(), serde_json::json!(learned));
        self.db.insert(VectorEntry {
            id: Some(id.clone()),
            vector: self.whiten(&sig),
            metadata: Some(md),
        })?;
        Ok(id)
    }

    /// Convenience: learn a config straight from its model (recompute K / e_up).
    /// Used after a successful catch to remember the arm we just stabilized.
    pub fn learn_from_arm(
        &mut self,
        sig: Signature,
        l1: f64,
        m1: f64,
        b1: f64,
    ) -> Result<String, Box<dyn std::error::Error>> {
        let sim = Self::arm(l1, m1, b1);
        let k = balance_gain(&sim, SEED_DT);
        let e_up = upright_energy(&sim);
        self.insert(sig, l1, m1, b1, k, e_up, true)
    }

    /// Recall the nearest seeded config to a measured signature.
    pub fn recall(&self, sig: &Signature) -> Result<Option<RecalledConfig>, Box<dyn std::error::Error>> {
        let results = self.db.search(SearchQuery {
            vector: self.whiten(sig),
            k: 1,
            filter: None,
            ef_search: None,
        })?;
        let Some(top) = results.into_iter().next() else {
            return Ok(None);
        };
        let md = top.metadata.unwrap_or_default();
        let getf = |key: &str| md.get(key).and_then(|v| v.as_f64()).unwrap_or(0.0);
        let k_arr = md
            .get("k")
            .and_then(|v| v.as_array())
            .map(|a| {
                let mut k = [0.0f64; 4];
                for (i, x) in a.iter().take(4).enumerate() {
                    k[i] = x.as_f64().unwrap_or(0.0);
                }
                k
            })
            .unwrap_or([0.0; 4]);
        let learned = md.get("learned").and_then(|v| v.as_bool()).unwrap_or(false);
        Ok(Some(RecalledConfig {
            id: top.id,
            score: top.score,
            l1: getf("l1"),
            m1: getf("m1"),
            b1: getf("b1"),
            k: k_arr,
            e_up: getf("e_up"),
            learned,
        }))
    }

    /// **GNN interpolation over the config graph** (Phase 3). Nearest-neighbour
    /// recall *snaps* an unseen arm to one seeded config; this instead blends the
    /// `k` nearest seeds, generalizing to arms between grid points.
    ///
    /// The seeded configs form a graph (nodes = arms, edges to nearby arms in
    /// signature space). For a query we gather its `k` nearest seeds and message-
    /// pass the query node over that neighbourhood with a real `ruvector-gnn`
    /// `RuvectorLayer` (attention + weighted aggregation), producing a context
    /// embedding. The adopted gain is the attention-weighted blend of the
    /// neighbours' gains, `K = Σ wᵢ·Kᵢ`, with `wᵢ = softmax(−distanceᵢ/τ)`.
    ///
    /// Honest note: the layer ships **untrained**, so we do not route the gains
    /// through its random projection (that would scramble them) — we use the
    /// graph's attention weights to interpolate, and the layer's message-pass to
    /// embed/contextualize the neighbourhood. The interpolation is the win: for a
    /// between-seed arm the blended gain lands closer to the true gain than any
    /// single neighbour's.
    #[cfg(feature = "gnn")]
    pub fn recall_interpolated(
        &self,
        sig: &Signature,
        k: usize,
    ) -> Result<Option<InterpResult>, Box<dyn std::error::Error>> {
        use ruvector_gnn::RuvectorLayer;
        let results = self.db.search(SearchQuery {
            vector: self.whiten(sig),
            k,
            filter: None,
            ef_search: None,
        })?;
        if results.is_empty() {
            return Ok(None);
        }

        // Softmax-over-(-distance) attention weights across the neighbourhood.
        let tau = 0.5f32;
        let min_score = results.iter().map(|r| r.score).fold(f32::INFINITY, f32::min);
        let exps: Vec<f32> = results.iter().map(|r| (-(r.score - min_score) / tau).exp()).collect();
        let z: f32 = exps.iter().sum::<f32>().max(1e-9);
        let weights: Vec<f32> = exps.iter().map(|e| e / z).collect();

        // Message-pass the query node over its neighbours with a real GNN layer
        // (demonstrates ruvector-gnn operating on the config graph). The context
        // embedding is a diagnostic; the gain comes from the attention blend.
        let layer = RuvectorLayer::new(SIG_DIM, 16, 2, 0.0).map_err(|e| format!("gnn init: {e:?}"))?;
        let query_vec = self.whiten(sig);
        let neighbor_vecs: Vec<Vec<f32>> =
            results.iter().filter_map(|r| r.vector.clone()).collect();
        let embedding = layer.forward(&query_vec, &neighbor_vecs, &weights);

        // Attention-weighted blend of the neighbours' gains and e_up.
        let mut k_blend = [0.0f64; 4];
        let mut e_up = 0.0f64;
        let mut contributors = Vec::with_capacity(results.len());
        for (r, &w) in results.iter().zip(&weights) {
            let md = r.metadata.as_ref();
            let getf = |key: &str| md.and_then(|m| m.get(key)).and_then(|v| v.as_f64()).unwrap_or(0.0);
            let kn = md
                .and_then(|m| m.get("k"))
                .and_then(|v| v.as_array())
                .map(|a| {
                    let mut kk = [0.0f64; 4];
                    for (i, x) in a.iter().take(4).enumerate() {
                        kk[i] = x.as_f64().unwrap_or(0.0);
                    }
                    kk
                })
                .unwrap_or([0.0; 4]);
            for i in 0..4 {
                k_blend[i] += w as f64 * kn[i];
            }
            e_up += w as f64 * getf("e_up");
            contributors.push((getf("l1"), w));
        }

        Ok(Some(InterpResult {
            k: k_blend,
            e_up,
            contributors,
            embedding_dim: embedding.len(),
        }))
    }
}

/// Result of [`ConfigMemory::recall_interpolated`]: a gain blended across config-
/// graph neighbours, plus which arms contributed and at what weight.
#[cfg(feature = "gnn")]
#[derive(Debug, Clone)]
pub struct InterpResult {
    /// The interpolated balance gain.
    pub k: Vec4,
    /// The interpolated upright-energy target.
    pub e_up: f64,
    /// `(link-2 length, attention weight)` for each contributing seed.
    pub contributors: Vec<(f64, f32)>,
    /// Width of the GNN context embedding produced by the message-pass.
    pub embedding_dim: usize,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::closed_loop_signature;

    #[test]
    fn recall_finds_exact_and_nearest() {
        // Use a unique temp path so the test is hermetic.
        let path = std::env::temp_dir()
            .join("pendulum_phase2_test.db")
            .to_string_lossy()
            .into_owned();
        let mut mem = ConfigMemory::new(&path).unwrap();
        mem.seed_grid().unwrap();
        let k_probe = nominal_probe_gain(SEED_DT);

        // An exact grid config (l1=2.0, m1=1, b1=0.05) must come back as itself.
        let exact = closed_loop_signature(&ConfigMemory::arm(2.0, 1.0, 0.05), &k_probe);
        let hit = mem.recall(&exact).unwrap().unwrap();
        assert!((hit.l1 - 2.0).abs() < 1e-9, "exact recall got l1={}", hit.l1);
        assert!(hit.score < 1e-3, "exact recall distance should be ~0, got {}", hit.score);
        assert!(!hit.learned, "grid seeds are not learned");

        // An off-grid arm (l1=2.25) must snap to its nearest seeded neighbour
        // (2.0 or 2.5), not something far away.
        let off = closed_loop_signature(&ConfigMemory::arm(2.25, 1.0, 0.05), &k_probe);
        let near = mem.recall(&off).unwrap().unwrap();
        assert!(
            (near.l1 - 2.0).abs() < 1e-9 || (near.l1 - 2.5).abs() < 1e-9,
            "off-grid 2.25 should snap to 2.0 or 2.5, got {}",
            near.l1
        );
    }

    /// Phase 3: GNN interpolation should beat nearest-neighbour *snapping* for an
    /// arm that lies between seeded grid points — the blended gain lands closer
    /// to the true gain than the single nearest seed's gain.
    #[cfg(feature = "gnn")]
    #[test]
    fn gnn_interpolation_beats_snapping() {
        let path = std::env::temp_dir()
            .join("pendulum_phase3_gnn.db")
            .to_string_lossy()
            .into_owned();
        let mut mem = ConfigMemory::new(&path).unwrap();
        mem.seed_grid().unwrap();
        let k_probe = nominal_probe_gain(SEED_DT);

        // A clearly between-seed arm (1.75 m sits midway between 1.5 and 2.0).
        let off_arm = ConfigMemory::arm(1.75, 1.0, 0.05);
        let true_k = balance_gain(&off_arm, SEED_DT);
        let sig = closed_loop_signature(&off_arm, &k_probe);

        let nearest = mem.recall(&sig).unwrap().unwrap();
        let interp = mem.recall_interpolated(&sig, 4).unwrap().unwrap();

        let dist = |a: &Vec4, b: &Vec4| -> f64 {
            (0..4).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt()
        };
        let err_snap = dist(&nearest.k, &true_k);
        let err_interp = dist(&interp.k, &true_k);

        assert!(interp.embedding_dim > 0, "GNN message-pass produced an embedding");
        assert!(interp.contributors.len() >= 2, "interpolation blends multiple seeds");
        assert!(
            err_interp < err_snap,
            "interpolated gain (err {err_interp:.2}) should beat snapped (err {err_snap:.2})"
        );
    }
}
