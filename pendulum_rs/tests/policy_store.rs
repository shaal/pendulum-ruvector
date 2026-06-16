//! Stage 1 step 4: a swing-up champion stored in RuVector and recalled by config
//! signature must rebuild into a working policy that beats the hand-tuned
//! baseline — the round trip from "evolution discovered it" to "the controller
//! recalls and uses it".
#![cfg(feature = "vectordb")]

use pendulum_rs::learn::{recovery_count, EnergyShapingPolicy, PflBaseline, NP};
use pendulum_rs::memory::ConfigMemory;

#[test]
fn recalled_learned_policy_beats_baseline() {
    let path = std::env::temp_dir()
        .join("pendulum_policy_store.db")
        .to_string_lossy()
        .into_owned();
    let mut mem = ConfigMemory::new(&path).unwrap();
    mem.seed_grid().unwrap(); // establishes the shared whitening

    // The champion the `evolve` search finds at the default seed (10/10).
    let champion = [35.14, 7.42, 4.24, -6.89, 2.12];
    let sig = mem.config_signature(1.0, 1.0, 0.05);
    mem.insert_policy(&sig, &champion, 1.0, 1.0, 0.05).unwrap();

    // Recall it by the same signature and confirm a clean round-trip.
    let hit = mem.recall_policy(&sig).unwrap().expect("a stored policy");
    assert!(hit.score < 1e-3, "exact-signature recall distance ~0, got {}", hit.score);
    assert_eq!(hit.params.len(), NP, "recalled the full parameter vector");
    for (a, b) in hit.params.iter().zip(&champion) {
        assert!((a - b).abs() < 1e-9, "param round-trip mismatch");
    }

    // Rebuild the policy from the *recalled* params and confirm it still wins.
    let mut p = [0.0; NP];
    p.copy_from_slice(&hit.params);
    let recalled_policy = EnergyShapingPolicy { p };

    let base = recovery_count(&PflBaseline, 15.0);
    let learned = recovery_count(&recalled_policy, 15.0);
    assert!(
        learned > base,
        "recalled learned policy ({learned}/10) should beat the hand-tuned baseline ({base}/10)"
    );
}

/// With several policies stored under different config signatures, recall must
/// return the one keyed nearest to the query — the reason the store is a vector
/// DB and not a single slot.
#[test]
fn recall_policy_picks_nearest() {
    let path = std::env::temp_dir()
        .join("pendulum_policy_nearest.db")
        .to_string_lossy()
        .into_owned();
    let mut mem = ConfigMemory::new(&path).unwrap();
    mem.seed_grid().unwrap();

    // Two distinct configs, two distinct (marker) policy vectors.
    let sig_short = mem.config_signature(0.6, 1.0, 0.05);
    let sig_long = mem.config_signature(2.5, 1.0, 0.05);
    let short_params = [1.0, 2.0, 3.0, 4.0, 5.0];
    let long_params = [9.0, 8.0, 7.0, 6.0, 5.0];
    mem.insert_policy(&sig_short, &short_params, 0.6, 1.0, 0.05).unwrap();
    mem.insert_policy(&sig_long, &long_params, 2.5, 1.0, 0.05).unwrap();

    // Each config's signature recalls its own policy, not the other's.
    let got_short = mem.recall_policy(&sig_short).unwrap().unwrap();
    let got_long = mem.recall_policy(&sig_long).unwrap().unwrap();
    assert_eq!(got_short.params, short_params, "short config recalled the wrong policy");
    assert_eq!(got_long.params, long_params, "long config recalled the wrong policy");
    assert!((got_short.l1 - 0.6).abs() < 1e-9 && (got_long.l1 - 2.5).abs() < 1e-9);

    // A between-config query (1.5 m) snaps to whichever seeded policy is nearer.
    let sig_mid = mem.config_signature(1.5, 1.0, 0.05);
    let got_mid = mem.recall_policy(&sig_mid).unwrap().unwrap();
    assert!(
        got_mid.params == short_params || got_mid.params == long_params,
        "between-config query must snap to one of the stored policies"
    );
    let _ = NP;
}
