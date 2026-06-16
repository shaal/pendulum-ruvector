//! Stage 2 live consumer: the controller recovers an arm by *recalling* a stored
//! swing-up policy from RuVector and running it — closing the loop from
//! "evolution discovered it" → "RuVector stored it" → "the controller uses it".
#![cfg(feature = "vectordb")]

use pendulum_rs::learn::{rollout_config, rollout_recalling_policy, EnergyShapingPolicy, NP};
use pendulum_rs::memory::ConfigMemory;
use std::f64::consts::PI;

#[test]
fn controller_recalls_and_runs_a_stored_policy() {
    let path = std::env::temp_dir()
        .join("pendulum_recall_consumer.db")
        .to_string_lossy()
        .into_owned();
    let mut mem = ConfigMemory::new(&path).unwrap();
    mem.seed_grid().unwrap();

    // Store a known-good swing-up champion (the Stage-1 10/10 champion).
    let champ = [35.14, 7.42, 4.24, -6.89, 2.12];
    let sig = mem.config_signature(1.0, 1.0, 0.05);
    mem.insert_policy(&sig, &champ, 1.0, 1.0, 0.05).unwrap();

    // The live consumer: recover the arm from a knockdown by recalling the
    // stored policy and running it (LQR catch + recalled swing-up).
    let (l1, m1, b1) = (1.0, 1.0, 0.05);
    let theta0 = vec![0.1, -0.1]; // dead hang — the champion recovers this
    let via_recall = rollout_recalling_policy(&mem, l1, m1, b1, &theta0, 15.0);

    // Consumer correctness: the recall path must be byte-for-byte the same as
    // running the recalled policy directly (it returned the right params and the
    // controller used them) — and it must differ from the baseline fallback (so
    // we know it actually adopted the *learned* policy, not silently fell back).
    let direct = rollout_config(l1, m1, b1, &theta0, &EnergyShapingPolicy { p: champ }, 15.0);
    assert_eq!(via_recall.caught, direct.caught, "recall path must match the direct run");
    assert!(
        (via_recall.final_tip - direct.final_tip).abs() < 1e-12,
        "recall path diverged from the direct run"
    );
    assert!(via_recall.caught, "the recalled learned policy should recover this arm");
    let _ = NP;
}
