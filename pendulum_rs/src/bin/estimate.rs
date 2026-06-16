//! Phase 2 — RuVector becomes the estimator (the oracle is gone).
//!
//! Two underactuated arms balance straight up. At `t = 1s` link-2 grows on both
//! (a tool extends). The **naive** arm keeps its old gain and topples. The
//! **adaptive** arm does *not* get told the new length: it runs a short dithered
//! probe, identifies its live dynamics signature from the motion it produces,
//! asks RuVector for the nearest arm it has seen, and adopts that arm's gain.
//! The delay between the change and the adoption is the honest **recognition
//! lag** — printed every run.
//!
//! Then the self-learning loop: after a successful catch the measured signature
//! is inserted back into RuVector. The *same* disturbance, thrown again, is
//! recognized from a rougher/earlier estimate — so the lag shrinks. That curve
//! is the learning.
//!
//!   cargo run --release --features vectordb --bin estimate
//!
//! It writes `estimate.rrd` (open with `rerun estimate.rrd`) plus a console log.

use pendulum_rs::control::{balance_gain, balance_torque, nominal_probe_gain, Vec4};
use pendulum_rs::estimator::{OnlineEstimator, Signature, SIG_DIM};
use pendulum_rs::memory::ConfigMemory;
use pendulum_rs::simulator::Pendulum;
use std::f64::consts::PI;

const DT: f64 = 0.005; // 200 Hz control
const U_MAX: f64 = 150.0;
const DISTURB_T: f64 = 1.0; // when link-2 grows
const SETTLE_T: f64 = 6.0; // total scenario length

// Probe / recognition tuning.
const MIN_SAMPLES: usize = 25; // ~0.12s of clean motion before the first recognition attempt
const PROBE_WINDOW: usize = 240; // sliding window cap (~1.2s)
const CHECK_EVERY: usize = 5; // re-evaluate recall this often during the probe
const FREEZE_TIP: f64 = 0.18; // stop collecting once the arm leaves the linear regime
// Commit gates. A COLD grid lookup is approximate, so it must converge tightly
// (small distance) or hold a plausible neighbour for several checks. A LEARNED
// match — a config we have actually caught before — can be trusted from a
// rougher, earlier estimate, because its stored signature is *this arm's own*
// measured fingerprint. That asymmetry is what makes the lag shrink on a repeat.
const GRID_TIGHT: f32 = 1.5; // cold lookup: instant-commit distance
const GRID_LOOSE: f32 = 5.0; // cold lookup: plausible-neighbour distance (needs consensus)
const GRID_CONSENSUS: usize = 3;
const LEARNED_THRESH: f32 = 13.0; // learned recall: tolerated distance from a rough early estimate
const LEARNED_CONSENSUS: usize = 2; // learned recall: fewer agreeing checks needed
const EMA_ALPHA: f64 = 0.5; // smoothing on the measured signature (tames per-check flicker)

fn nominal_arm() -> Pendulum {
    Pendulum::new(vec![1.0, 1.0], vec![1.0, 1.0], vec![0.05, 0.05], 9.81, DT)
}

fn tip_error(sim: &Pendulum) -> f64 {
    let w = |a: f64| (a + PI).rem_euclid(2.0 * PI) - PI;
    w(sim.theta[0] - PI).abs() + w(sim.theta[1] - PI).abs()
}

/// Dithered stabilizing torque: stale gain `k` plus an exogenous multi-sine
/// probe. Returns `(total_torque_applied, dither_component)` — the dither is
/// what the estimator records as the independent input.
fn probe_torque(k: &Vec4, sim: &Pendulum, t: f64) -> (f64, f64) {
    let e0 = (sim.theta[0] - PI + PI).rem_euclid(2.0 * PI) - PI;
    let e1 = (sim.theta[1] - PI + PI).rem_euclid(2.0 * PI) - PI;
    let u_fb = -(k[0] * e0 + k[1] * e1 + k[2] * sim.omega[0] + k[3] * sim.omega[1]);
    let dither = 6.0 * (2.0 * PI * 1.7 * t).sin() + 4.0 * (2.0 * PI * 3.3 * t).sin();
    ((u_fb + dither).clamp(-U_MAX, U_MAX), dither)
}

/// The arm the disturbance turns the nominal arm into: a new link-2 length
/// (a tool extending) and/or mass (a payload picked up), plus friction.
#[derive(Clone, Copy)]
struct Disturb {
    l1: f64,
    m1: f64,
    b1: f64,
}

struct RunResult {
    lag: Option<f64>,
    recalled_id: String,
    recalled_l1: f64,
    naive_err: f64,
    adaptive_err: f64,
}

/// Run one disturbance scenario. `learn` controls whether a successful catch is
/// written back into RuVector (the self-learning step).
fn run_scenario(
    mem: &mut ConfigMemory,
    dist: Disturb,
    rec: &rerun::RecordingStream,
    tag: &str,
    learn: bool,
) -> Result<RunResult, Box<dyn std::error::Error>> {
    let mut naive = nominal_arm();
    let mut adaptive = nominal_arm();
    let theta0 = vec![PI - 0.05, PI + 0.05];
    naive.reset(theta0.clone(), vec![0.0; 2]);
    adaptive.reset(theta0, vec![0.0; 2]);

    let k0 = balance_gain(&naive, DT); // stale gain (for the original arm)
    let k_naive = k0;
    let mut k_adaptive = k0;

    // The probe gain is the stale gain itself: while recognizing, the adaptive
    // arm keeps running the controller the naive arm uses and measures the
    // *closed-loop* response under it (the leftover wobble supplies excitation).
    // Seeds were fingerprinted under this same gain, so the measured and seeded
    // closed-loop signatures live in the same space. (`k0` equals this.)
    let k_probe = nominal_probe_gain(DT);

    let mut est = OnlineEstimator::new(PROBE_WINDOW, 1e-4);
    let mut probing = false;
    let mut committed = false;
    let mut lag: Option<f64> = None;
    let mut recalled_id = String::new();
    let mut recalled_l1 = 0.0;
    let mut measured_sig = None;
    // Consensus tracking: same nearest config for several checks in a row.
    let mut agree_id = String::new();
    let mut agree_count = 0usize;
    // EMA of the measured signature — recall on the smoothed value, not the raw
    // (noisy) per-check estimate, so the nearest neighbour stops flickering.
    let mut smoothed: Option<Signature> = None;

    let disturb_step = (DISTURB_T / DT) as usize;
    let total = (SETTLE_T / DT) as usize;
    let (mut naive_late, mut adaptive_late) = (0.0f64, 0.0f64);
    let late_start = total.saturating_sub((1.0 / DT) as usize);

    for step in 0..total {
        let t = step as f64 * DT;

        if step == disturb_step {
            for arm in [&mut naive, &mut adaptive] {
                arm.set_length(1, dist.l1);
                arm.set_mass(1, dist.m1);
                arm.set_damping(1, dist.b1);
            }
            probing = true; // adaptive starts its recognition probe
            est.clear();
            eprintln!(
                "[{tag}] t={DISTURB_T:.1}s: link-2 -> {:.2} m / {:.1} kg. adaptive begins probing…",
                dist.l1, dist.m1
            );
        }

        // --- naive: stale gain forever ---
        let un = balance_torque(&k_naive, &naive.theta, &naive.omega, U_MAX);
        naive.step(&[un, 0.0]);

        // --- adaptive ---
        if probing && !committed {
            // Probe: keep running the stale gain + dither, recording motion for
            // identification — but only while the arm is still in the clean
            // near-upright linear regime. Once it starts to topple, freeze the
            // window so divergent (nonlinear) samples can't corrupt the estimate.
            let theta_before = adaptive.theta.clone();
            let omega_before = adaptive.omega.clone();
            let clean = tip_error(&adaptive) < FREEZE_TIP;
            let (u, dither) = probe_torque(&k_probe, &adaptive, t);
            adaptive.step(&[u, 0.0]);
            if clean {
                est.observe(&theta_before, &omega_before, dither, &adaptive.omega, DT);
            }

            if est.len() >= MIN_SAMPLES && step % CHECK_EVERY == 0 {
                if let Some(raw) = est.estimate() {
                    // Smooth the raw estimate before recall.
                    let sig: Signature = match smoothed {
                        Some(prev) => {
                            let mut s = [0.0f64; SIG_DIM];
                            for i in 0..SIG_DIM {
                                s[i] = EMA_ALPHA * raw[i] + (1.0 - EMA_ALPHA) * prev[i];
                            }
                            smoothed = Some(s);
                            s
                        }
                        None => {
                            smoothed = Some(raw);
                            raw
                        }
                    };
                    if let Some(rc) = mem.recall(&sig)? {
                        rec.set_time_sequence("step", step as i64);
                        rec.log(
                            format!("{tag}/recall_distance"),
                            &rerun::Scalars::new([rc.score as f64]),
                        )?;
                        if std::env::var("PROBE_DEBUG").is_ok() {
                            eprintln!(
                                "[{tag}] t={:.2} n={} dist={:.3} -> {} (l1≈{:.2}) agree={}  (tip err {:.3})",
                                t - DISTURB_T, est.len(), rc.score, rc.id, rc.l1, agree_count, tip_error(&adaptive)
                            );
                        }
                        // Track consensus (same config winning consecutive checks).
                        if rc.id == agree_id {
                            agree_count += 1;
                        } else {
                            agree_id = rc.id.clone();
                            agree_count = 1;
                        }
                        // A verified (learned) match is trusted from a rough,
                        // early estimate; a cold grid match must converge tightly
                        // or hold for several checks.
                        let commit = if rc.learned {
                            rc.score < LEARNED_THRESH && agree_count >= LEARNED_CONSENSUS
                        } else {
                            rc.score < GRID_TIGHT
                                || (rc.score < GRID_LOOSE && agree_count >= GRID_CONSENSUS)
                        };
                        if commit {
                            k_adaptive = rc.k;
                            committed = true;
                            lag = Some(t - DISTURB_T);
                            recalled_id = rc.id.clone();
                            recalled_l1 = rc.l1;
                            measured_sig = Some(sig);
                            eprintln!(
                                "[{tag}] recognized after {:.2}s probe: nearest = {} (l1≈{:.2}, dist={:.3}, {}) -> adopting its gain",
                                t - DISTURB_T,
                                rc.id,
                                rc.l1,
                                rc.score,
                                if rc.learned { "learned recall" } else { "cold grid lookup" }
                            );
                        }
                    }
                }
            }
        } else {
            // Either still nominal (pre-disturbance) or already recognized:
            // run the current gain straight, no dither.
            let ua = balance_torque(&k_adaptive, &adaptive.theta, &adaptive.omega, U_MAX);
            adaptive.step(&[ua, 0.0]);
        }

        if step >= late_start {
            naive_late = naive_late.max(tip_error(&naive));
            adaptive_late = adaptive_late.max(tip_error(&adaptive));
        }

        // --- Rerun: both arms + their tip errors ---
        rec.set_time_sequence("step", step as i64);
        log_arm(rec, &format!("{tag}/naive"), &naive, -3.0, (220, 70, 70))?;
        log_arm(rec, &format!("{tag}/adaptive"), &adaptive, 3.0, (70, 200, 110))?;
        rec.log(format!("{tag}/err/naive"), &rerun::Scalars::new([tip_error(&naive)]))?;
        rec.log(format!("{tag}/err/adaptive"), &rerun::Scalars::new([tip_error(&adaptive)]))?;
    }

    // Self-learning: remember the arm we just stabilized so it is recognized
    // faster next time. We insert the *measured* signature (what this arm
    // actually produces), tagged with the true config for clarity.
    if learn && committed {
        if let Some(sig) = measured_sig {
            let id = mem.learn_from_arm(sig, dist.l1, dist.m1, dist.b1)?;
            eprintln!("[{tag}] learned this arm as {id} (measured signature inserted into RuVector)");
        }
    }

    Ok(RunResult {
        lag,
        recalled_id,
        recalled_l1,
        naive_err: naive_late,
        adaptive_err: adaptive_late,
    })
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun::RecordingStreamBuilder::new("phase2_estimate").save("estimate.rrd")?;

    // Seed RuVector with a grid of arms keyed by dynamics signature.
    let mut mem = ConfigMemory::new("phase2_configs.db")?;
    let seeded = mem.seed_grid()?;
    eprintln!("Seeded RuVector with {seeded} arm configs (signature -> params, K, e_up).\n");

    // The disturbance: link-2 extends to an OFF-GRID 2.2 m (between seeded 2.0
    // and 2.5), past where the stale gain copes (so naive falls). Cold recall
    // snaps to the nearest grid neighbour (2.0 m), whose gain still catches it;
    // learning the exact arm then speeds up the repeat. ~2.2 m is near the top
    // of the recognition envelope — beyond it the arm topples faster than the
    // probe can identify it (that regime is Phase-3 swing-up).
    //
    // The signature here keys on *structural* (link-length) change. Mass/payload
    // changes shift the same gravity-stiffness terms and are confounded with
    // length under measurement noise — generalizing across the full config space
    // is exactly what Phase-3's GNN interpolation is for.
    let l1 = std::env::var("NEW_L1").ok().and_then(|s| s.parse().ok()).unwrap_or(2.2);
    let dist = Disturb { l1, m1: 1.0, b1: 0.05 };

    eprintln!("=== Encounter 1: a config we've only seen *near* (cold memory) ===");
    let r1 = run_scenario(&mut mem, dist, &rec, "run1", true)?;

    eprintln!("\n=== Encounter 2: same disturbance, now that we've felt it once ===");
    let r2 = run_scenario(&mut mem, dist, &rec, "run2", false)?;

    let verdict = |e: f64| {
        if e < 0.2 {
            "holds ✅"
        } else if e < 0.7 {
            "wobbling ⚠️"
        } else {
            "FELL ❌"
        }
    };

    eprintln!("\n────────────────────────── RESULTS ──────────────────────────");
    eprintln!(
        "Naive arm (stale gain):       last-second tip error {:.2} rad  {}",
        r1.naive_err,
        verdict(r1.naive_err)
    );
    eprintln!(
        "Adaptive arm (RuVector recall): last-second tip error {:.2} rad  {}",
        r1.adaptive_err,
        verdict(r1.adaptive_err)
    );
    eprintln!(
        "  └─ recalled config {} (l1≈{:.2} m) as the nearest match to the true {:.2} m / {:.1} kg arm",
        r1.recalled_id, r1.recalled_l1, dist.l1, dist.m1
    );
    match (r1.lag, r2.lag) {
        (Some(l1), Some(l2)) => {
            eprintln!("\nRecognition lag:");
            eprintln!("  encounter 1 (cold):   {:.2} s", l1);
            eprintln!("  encounter 2 (learned): {:.2} s", l2);
            if l2 < l1 {
                eprintln!(
                    "  → lag shrank by {:.0}% after learning the arm once.",
                    100.0 * (l1 - l2) / l1
                );
            } else {
                eprintln!("  → no shrink this run (see notes).");
            }
        }
        _ => eprintln!("\n(at least one encounter did not recognize — tune the probe.)"),
    }

    // Phase 3: GNN interpolation over the config graph (only with `--features
    // ruvector`/`gnn`). Instead of snapping the off-grid arm to one neighbour,
    // message-pass over its neighbourhood and blend their gains.
    #[cfg(feature = "gnn")]
    {
        use pendulum_rs::control::nominal_probe_gain;
        use pendulum_rs::estimator::closed_loop_signature;
        let true_arm = Pendulum::new(vec![1.0, dist.m1], vec![1.0, dist.l1], vec![0.05, dist.b1], 9.81, DT);
        let sig = closed_loop_signature(&true_arm, &nominal_probe_gain(DT));
        if let Some(interp) = mem.recall_interpolated(&sig, 4)? {
            eprintln!("\nGNN interpolation (between-seed generalization):");
            let blend: Vec<String> = interp
                .contributors
                .iter()
                .map(|(l1, w)| format!("{l1:.2}m×{:.0}%", w * 100.0))
                .collect();
            eprintln!(
                "  message-passed over {} graph neighbours (embed dim {}) → blended {}",
                interp.contributors.len(),
                interp.embedding_dim,
                blend.join(" + ")
            );
            eprintln!("  → interpolated gain for the true {:.2} m arm, instead of snapping to one seed.", dist.l1);
        }
    }

    eprintln!("\nRerun:  rerun estimate.rrd");
    Ok(())
}

fn log_arm(
    rec: &rerun::RecordingStream,
    path: &str,
    sim: &Pendulum,
    x_shift: f32,
    color: (u8, u8, u8),
) -> Result<(), Box<dyn std::error::Error>> {
    let pts: Vec<[f32; 2]> = sim
        .link_positions()
        .iter()
        .map(|&(x, y)| [x as f32 + x_shift, y as f32])
        .collect();
    rec.log(
        format!("{path}/links"),
        &rerun::LineStrips2D::new([pts.clone()])
            .with_colors([rerun::Color::from_rgb(color.0, color.1, color.2)])
            .with_radii([0.03]),
    )?;
    rec.log(format!("{path}/joints"), &rerun::Points2D::new(pts).with_radii([0.06]))?;
    Ok(())
}
