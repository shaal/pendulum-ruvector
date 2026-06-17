//! Native test of the Compete-station handles. `Evolver` (worker side) must
//! advance generations and produce a champion per island; `PopArms` (main side)
//! must drive the display arms from those champions and produce a well-formed
//! render buffer.

use pendulum_web::{Evolver, PopArms};

#[test]
fn population_evolves_and_renders() {
    let mut ev = Evolver::new(true, 8);
    let n = ev.n_islands();
    assert!(n >= 4, "expected several islands, got {n}");

    let g0 = ev.generation();
    // One island per evolve_islands(1); ~6 generations needs ~6n calls.
    for _ in 0..(n * 6) {
        ev.evolve_islands(1);
    }
    assert!(ev.generation() > g0, "generations should advance: {g0} -> {}", ev.generation());
    assert!(ev.rollouts() > 0, "rollouts should be counted");

    let fits = ev.fitnesses();
    assert_eq!(fits.len(), n);
    assert!(fits.iter().all(|f| f.is_finite()), "every island should have a champion fitness");
    assert!(ev.best_island() < n, "best island in range");

    let champions = ev.champions_flat();
    assert_eq!(champions.len(), n * pendulum_web::np(), "n_islands * NP champion params");

    let mut arms = PopArms::new(n);
    assert_eq!(arms.positions_all().len(), n * 6, "6 floats per 2-link arm");
    arms.tick(5, &champions);
    assert_eq!(arms.positions_all().len(), n * 6, "render buffer stays well-formed");
}

#[test]
fn sharing_toggle_is_respected() {
    let mut ev = Evolver::new(false, 8);
    assert!(!ev.sharing());
    ev.set_sharing(true);
    assert!(ev.sharing());
}
