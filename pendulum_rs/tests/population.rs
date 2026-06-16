//! Stage 4 — a competing population of CEM islands sharing discoveries through
//! RuVector reaches population-wide competence in fewer total rollouts than the
//! same islands run independently. Same seed in both conditions, so the only
//! difference is the sharing.
#![cfg(feature = "vectordb")]

use pendulum_rs::learn::population_run;

#[test]
fn ruvector_sharing_accelerates_the_population() {
    let p_indep = std::env::temp_dir().join("pop_indep_test.db").to_string_lossy().into_owned();
    let p_shared = std::env::temp_dir().join("pop_shared_test.db").to_string_lossy().into_owned();

    // Weak islands (small population): the laggards struggle to reach
    // population-wide competence alone. With migration through RuVector, the best
    // discovery propagates and pulls everyone up fast.
    let (seed, n, pop, cases, gens, migrate, target) = (7u64, 6, 8, 5, 25, 2, 56.0);
    let indep = population_run(seed, false, n, pop, cases, gens, migrate, target, &p_indep);
    let shared = population_run(seed, true, n, pop, cases, gens, migrate, target, &p_shared);

    // Sharing brings ALL islands to the target...
    assert!(shared.reached, "RuVector-shared population should reach the target");
    // ...in far fewer rollouts than the independent islands consume (which here
    // don't even get every island there within the budget).
    assert!(
        2 * shared.rollouts < indep.rollouts,
        "sharing should reach competence in far fewer rollouts (shared {} vs independent {})",
        shared.rollouts,
        indep.rollouts
    );
}
