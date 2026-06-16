//! Stage 2.6 — a per-arm champion library stored in RuVector and recalled per
//! arm. Pins the honest decomposition of the cross-arm ceiling: per-arm
//! *selection* has real headroom over any single policy (the oracle), while
//! signature-keyed *recall* captures only part of it.
#![cfg(feature = "vectordb")]

use pendulum_rs::learn::{
    held_out_configs, knockdown_starts, recovery_rate_over, rollout_config, rollout_recalling_policy,
    EnergyShapingPolicy, PflBaseline, DR_CHAMPION, NOMINAL_CHAMPION, POLICY_LIBRARY,
};
use pendulum_rs::memory::ConfigMemory;

const SECS: f64 = 15.0;

#[test]
fn per_arm_library_beats_single_policies() {
    let path = std::env::temp_dir()
        .join("pendulum_policy_library.db")
        .to_string_lossy()
        .into_owned();
    let mut mem = ConfigMemory::new(&path).unwrap();
    mem.seed_grid().unwrap();

    // Store the evolved per-arm library, keyed by each anchor's config signature.
    for &(l1, m1, b1, params) in POLICY_LIBRARY.iter() {
        let sig = mem.config_signature(l1, m1, b1);
        mem.insert_policy(&sig, &params, l1, m1, b1).unwrap();
    }

    let configs = held_out_configs();
    let starts = knockdown_starts();
    let total = configs.len() * starts.len();

    // Best single global policy (baseline / nominal / DR).
    let base = recovery_rate_over(&PflBaseline, &configs, SECS).0;
    let nom = recovery_rate_over(&EnergyShapingPolicy { p: NOMINAL_CHAMPION }, &configs, SECS).0;
    let dr = recovery_rate_over(&EnergyShapingPolicy { p: DR_CHAMPION }, &configs, SECS).0;
    let best_single = base.max(nom).max(dr);

    // Per-arm recall: recall the nearest stored champion for each held-out arm.
    let mut recall = 0usize;
    for &(l1, m1, b1) in &configs {
        for (_, theta0) in &starts {
            if rollout_recalling_policy(&mem, l1, m1, b1, theta0, SECS).caught {
                recall += 1;
            }
        }
    }

    // Per-arm oracle: the best library champion *for each arm* — the ceiling for
    // anything keyed on arm config.
    let mut oracle = 0usize;
    for &(l1, m1, b1) in &configs {
        let best_for_arm = POLICY_LIBRARY
            .iter()
            .map(|&(_, _, _, p)| {
                starts
                    .iter()
                    .filter(|(_, t)| rollout_config(l1, m1, b1, t, &EnergyShapingPolicy { p }, SECS).caught)
                    .count()
            })
            .max()
            .unwrap_or(0);
        oracle += best_for_arm;
    }

    // Findings: per-arm SELECTION clearly beats any single policy (the oracle has
    // real headroom), and per-arm RECALL at least matches the best single one.
    assert!(
        oracle > best_single + 5,
        "per-arm oracle ({oracle}/{total}) should clearly beat the best single policy ({best_single})"
    );
    assert!(
        recall >= best_single,
        "per-arm recall ({recall}/{total}) should at least match the best single policy ({best_single})"
    );
}
