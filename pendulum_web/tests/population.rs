//! Native test of the wasm `Population` handle (the Compete / popviz station):
//! the frame-sliced evolution must actually advance generations and produce a
//! champion per island, and the per-arm render buffer must have the right shape.

use pendulum_web::Population;

#[test]
fn population_evolves_and_renders() {
    let mut p = Population::new(true);
    let n = p.n_islands();
    assert!(n >= 4, "expected several islands, got {n}");
    assert_eq!(p.positions_all().len(), n * 6, "6 floats per 2-link arm (3 points)");

    let g0 = p.generation();
    // One island is evolved per evolve_islands(1), so ~6 generations needs ~6n calls.
    for _ in 0..(n * 6) {
        p.tick_arms(2);
        p.evolve_islands(1);
    }

    assert!(p.generation() > g0, "generations should advance: {g0} -> {}", p.generation());
    assert!(p.rollouts() > 0, "rollouts should be counted");

    let fits = p.fitnesses();
    assert_eq!(fits.len(), n);
    assert!(fits.iter().all(|f| f.is_finite()), "every island should have a champion fitness");
    assert!(p.best_island() < n, "best island in range");
    assert_eq!(p.positions_all().len(), n * 6, "render buffer stays well-formed");
}

#[test]
fn sharing_toggle_is_respected() {
    let mut p = Population::new(false);
    assert!(!p.sharing());
    p.set_sharing(true);
    assert!(p.sharing());
}
