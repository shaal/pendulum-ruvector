//! Rust n-link pendulum: Lagrangian dynamics -> Rerun visualization, with an
//! optional in-process RuVector loop (vector DB insert + GNN message passing).
//!
//! Build variants
//! --------------
//!   cargo run --release                       # sim + Rerun only (fast build)
//!   cargo run --release --features gnn         # + GNN over the link graph
//!   cargo run --release --features vectordb    # + insert state vectors into RuVector
//!   cargo run --release --features ruvector    # the whole unified loop
//!
//! By default it writes an `.rrd` recording you open with `rerun pendulum_rs.rrd`.
//! Pass `--spawn` to launch the viewer live instead.

mod simulator;

use simulator::Pendulum;
use std::f64::consts::PI;
use std::io::Write;

#[cfg(feature = "vectordb")]
use ruvector_core::types::DbOptions; // DbOptions isn't re-exported at the crate root
#[cfg(feature = "vectordb")]
use ruvector_core::{DistanceMetric, SearchQuery, VectorDB, VectorEntry};
#[cfg(feature = "gnn")]
use ruvector_gnn::RuvectorLayer;

struct Args {
    links: usize,
    duration: f64,
    fps: f64,
    damping: f64,
    spawn: bool,
    out: String,
    /// Optional CSV dump of joint positions per frame (for external plotting).
    csv: Option<String>,
    /// "passive" (free swing) or "actuated" (PD controller drives links upright).
    mode: String,
}

/// PD controller that drives every link toward upright (theta = pi).
/// Holding a multi-link pendulum inverted is genuinely hard, so this is lively
/// rather than perfectly stable — the point is to demonstrate torque actuation.
/// Output is clamped so a divergent link can't produce NaN-inducing torques.
fn pd_control(theta: &[f64], omega: &[f64], kp: f64, kd: f64) -> Vec<f64> {
    theta
        .iter()
        .zip(omega)
        .map(|(&th, &om)| {
            // Wrap the angle error into (-pi, pi] so control takes the short way.
            let err = (PI - th + PI).rem_euclid(2.0 * PI) - PI;
            (kp * err - kd * om).clamp(-40.0, 40.0)
        })
        .collect()
}

/// Flatten the live state into RuVector's embedding layout: [sinθ | cosθ | ω | τ].
#[cfg(feature = "vectordb")]
fn state_vector(sim: &Pendulum) -> Vec<f32> {
    let n = sim.n;
    let mut v: Vec<f32> = Vec::with_capacity(4 * n);
    v.extend((0..n).map(|i| sim.theta[i].sin() as f32));
    v.extend((0..n).map(|i| sim.theta[i].cos() as f32));
    v.extend((0..n).map(|i| sim.omega[i] as f32));
    v.extend((0..n).map(|i| sim.tau[i] as f32));
    v
}

fn parse_args() -> Args {
    let mut a = Args {
        links: 2,
        duration: 12.0,
        fps: 60.0,
        damping: 0.0,
        spawn: false,
        out: "pendulum_rs.rrd".to_string(),
        csv: None,
        mode: "passive".to_string(),
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        match flag.as_str() {
            "--links" => a.links = it.next().unwrap().parse().unwrap(),
            "--duration" => a.duration = it.next().unwrap().parse().unwrap(),
            "--fps" => a.fps = it.next().unwrap().parse().unwrap(),
            "--damping" => a.damping = it.next().unwrap().parse().unwrap(),
            "--out" => a.out = it.next().unwrap(),
            "--csv" => a.csv = Some(it.next().unwrap()),
            "--mode" => a.mode = it.next().unwrap(),
            "--spawn" => a.spawn = true,
            other => eprintln!("(ignoring unknown arg: {other})"),
        }
    }
    a
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args();
    let n = args.links;
    let dt = 1.0 / args.fps;

    let masses = vec![1.0; n];
    let lengths = vec![1.0 / n as f64; n]; // total reach ~1 m as links grow
    let damping = vec![args.damping; n];
    let mut sim = Pendulum::new(masses, lengths, damping, 9.81, dt);

    // Near-horizontal release with a tiny per-link offset -> lively chaos.
    let theta0: Vec<f64> = (0..n).map(|i| PI / 2.0 + 0.1 * i as f64 / n as f64).collect();
    sim.reset(theta0, vec![0.0; n]);

    // --- Rerun recording stream -------------------------------------------
    let builder = rerun::RecordingStreamBuilder::new("pendulum_rs");
    let rec = if args.spawn {
        builder.spawn()?
    } else {
        builder.save(&args.out)?
    };

    // --- Optional RuVector: vector DB (state-vector index) -----------------
    #[cfg(feature = "vectordb")]
    let db = {
        let opts = DbOptions {
            dimensions: 4 * n, // [sin θ | cos θ | ω | τ]
            distance_metric: DistanceMetric::Cosine,
            storage_path: "pendulum_ruvector.db".to_string(),
            ..Default::default()
        };
        VectorDB::new(opts)?
    };
    #[cfg(feature = "vectordb")]
    let mut inserted = 0usize;

    // --- Optional RuVector: a real GNN layer for the link graph ------------
    // input_dim = 9 (node feature width, matching the Python graph), hidden 16,
    // 2 attention heads, no dropout (deterministic demo).
    #[cfg(feature = "gnn")]
    let gnn = RuvectorLayer::new(9, 16, 2, 0.0).map_err(|e| format!("gnn init: {e:?}"))?;

    // Optional CSV writer: one row per frame = x0,y0,x1,y1,...,xn,yn.
    let mut csv_w = args
        .csv
        .as_ref()
        .map(|p| std::io::BufWriter::new(std::fs::File::create(p).expect("create csv")));

    let actuated = args.mode == "actuated";
    let n_steps = (args.duration * args.fps) as usize;
    for step in 0..n_steps {
        // Passive: free swing. Actuated: PD torques computed from the live state.
        let tau = if actuated {
            pd_control(&sim.theta, &sim.omega, 12.0, 3.0)
        } else {
            vec![0.0; n]
        };
        sim.step(&tau);

        if let Some(w) = csv_w.as_mut() {
            let row: Vec<String> = sim
                .link_positions()
                .iter()
                .flat_map(|&(x, y)| [format!("{x:.6}"), format!("{y:.6}")])
                .collect();
            writeln!(w, "{}", row.join(",")).expect("write csv");
        }

        // ---- Visualization: arm + per-joint plots + energy ----
        rec.set_time_sequence("step", step as i64);
        let pts: Vec<[f32; 2]> = sim
            .link_positions()
            .iter()
            .map(|&(x, y)| [x as f32, y as f32])
            .collect();

        rec.log(
            "world/arm/links",
            &rerun::LineStrips2D::new([pts.clone()])
                .with_colors([rerun::Color::from_rgb(30, 144, 255)])
                .with_radii([0.02]),
        )?;
        rec.log(
            "world/arm/joints",
            &rerun::Points2D::new(pts.clone())
                .with_colors([rerun::Color::from_rgb(255, 215, 0)])
                .with_radii([0.05]),
        )?;
        for i in 0..n {
            rec.log(
                format!("plots/theta/joint_{i}"),
                &rerun::Scalars::new([sim.theta[i]]),
            )?;
        }
        rec.log("plots/energy", &rerun::Scalars::new([sim.total_energy()]))?;

        // ---- RuVector vector DB: index the state vector (every 5 steps) ----
        #[cfg(feature = "vectordb")]
        if step % 5 == 0 {
            let mut md = std::collections::HashMap::new();
            md.insert("t".to_string(), serde_json::json!(sim.t));
            md.insert("step".to_string(), serde_json::json!(step));
            db.insert(VectorEntry {
                id: Some(format!("s{step}")),
                vector: state_vector(&sim),
                metadata: Some(md),
            })?;
            inserted += 1;
        }

        // ---- RuVector search: "have I been in a state like this before?" ----
        // Query the index with the current state. This is the retrieval half of
        // a calibration loop: at run time you fetch the nearest *past* states
        // (and their known-good parameters / corrections) to warm-start an
        // estimate. We skip the very first steps so there's history to match.
        #[cfg(feature = "vectordb")]
        if step % 30 == 0 && inserted > 5 {
            // Simulate an imperfect *observation* of the current state by adding
            // small deterministic noise, then ask RuVector for the nearest clean
            // indexed state. Non-zero score = how far the noisy reading sits from
            // the closest thing we've actually seen — the retrieval half of a
            // calibration loop.
            let mut query = state_vector(&sim);
            for (j, x) in query.iter_mut().enumerate() {
                *x += 0.05 * (((step * 7 + j * 13) % 11) as f32 - 5.0) / 5.0;
            }
            let results = db.search(SearchQuery {
                vector: query,
                k: 3,
                filter: None,
                ef_search: None,
            })?;
            if let Some(top) = results.first() {
                rec.log("plots/nearest_score", &rerun::Scalars::new([top.score as f64]))?;
                let when = top
                    .metadata
                    .as_ref()
                    .and_then(|m| m.get("t"))
                    .map(|v| v.to_string())
                    .unwrap_or_default();
                eprintln!(
                    "[search] step {step}: noisy obs -> nearest clean state id={} score={:.4} (t={})",
                    top.id, top.score, when
                );
            }
        }

        // ---- RuVector GNN: message-pass over the link/joint graph ----
        // The pendulum *is* a graph (links=nodes, joints=edges). We hand the
        // real RuVector GNN layer each node's features + its chain neighbors.
        #[cfg(feature = "gnn")]
        if step % 120 == 0 {
            let pos = sim.link_positions();
            let node_feat = |i: usize| -> Vec<f32> {
                let (tx, ty) = pos[i + 1];
                vec![
                    sim.m[i] as f32,
                    sim.l[i] as f32,
                    sim.theta[i] as f32,
                    sim.omega[i] as f32,
                    sim.tau[i] as f32,
                    sim.theta[i].sin() as f32,
                    sim.theta[i].cos() as f32,
                    tx as f32,
                    ty as f32,
                ]
            };
            let feats: Vec<Vec<f32>> = (0..n).map(node_feat).collect();
            // Node 0's updated embedding after one message-passing round.
            let mut neighbors = Vec::new();
            if n > 1 {
                neighbors.push(feats[1].clone());
            }
            let weights = vec![1.0f32; neighbors.len()];
            let h0 = gnn.forward(&feats[0], &neighbors, &weights);
            let norm: f32 = h0.iter().map(|x| x * x).sum::<f32>().sqrt();
            eprintln!(
                "[gnn] step {step}: node0 -> embedding dim {}, |h0|={:.4}",
                h0.len(),
                norm
            );
        }
    }

    #[cfg(feature = "vectordb")]
    eprintln!("[vectordb] inserted {inserted} state vectors (dim {})", 4 * n);

    if !args.spawn {
        eprintln!(
            "Wrote Rerun recording -> {}\nView it with:  rerun {}",
            args.out, args.out
        );
    }
    Ok(())
}
