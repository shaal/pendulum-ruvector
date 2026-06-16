//! Native test of the wasm `Recalibrator` handle: the in-browser recognize loop
//! must (1) recognize the changed arm via RuVector recall, and (2) recognize it
//! *faster* on a repeat once it has been learned — the same behavior the native
//! `estimate` binary demonstrates (lag ~0.38s cold → ~0.15s learned).
//!
//! Runs natively (the wasm-bindgen methods are plain Rust methods off-wasm), so it
//! guards the browser experience without a browser.

use pendulum_web::Recalibrator;

fn run_one_encounter(r: &mut Recalibrator) {
    // Step until it commits a recall, or give up after ~10s of scenario time.
    let mut steps = 0;
    while !r.committed() && steps < 2000 {
        r.tick(5);
        steps += 5;
    }
}

#[test]
fn recognition_works_and_lag_shrinks_on_repeat() {
    let mut r = Recalibrator::new(2.2);
    r.set_learning(true);

    // Encounter 1 — cold memory.
    run_one_encounter(&mut r);
    assert!(r.committed(), "encounter 1 should recognize the changed arm");
    let lag1 = r.lag();
    assert!(lag1 > 0.0, "encounter 1 lag should be positive, got {lag1}");

    // Encounter 2 — same disturbance, now learned.
    r.next_encounter();
    run_one_encounter(&mut r);
    assert!(r.committed(), "encounter 2 should recognize the changed arm");
    let lag2 = r.lag();
    assert!(lag2 > 0.0, "encounter 2 lag should be positive, got {lag2}");

    assert!(
        lag2 < lag1,
        "recognition lag should shrink after learning: encounter1={lag1:.3}s encounter2={lag2:.3}s"
    );
}

#[test]
fn naive_arm_falls_when_link_grows() {
    // Sanity: with a big enough length change, the stale-gain (naive) arm topples
    // while the adaptive arm recovers — the contrast the station is built on.
    let mut r = Recalibrator::new(2.2);
    run_one_encounter(&mut r);
    // run a few more seconds past recognition to let both settle
    r.tick(1000);
    assert!(r.tip_error_naive() > 0.7, "naive arm should fall, tip err {}", r.tip_error_naive());
    assert!(r.tip_error_adaptive() < 0.3, "adaptive arm should hold, tip err {}", r.tip_error_adaptive());
}
