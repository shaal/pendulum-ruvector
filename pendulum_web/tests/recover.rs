//! Native test of the Recover handle (swing-up): at least one of the named
//! knockdown starts must hoist itself back upright (the harness recovers ~7/10),
//! and the knockdown catalogue is exposed for the UI.

use pendulum_web::Recover;

const SEC: usize = 200; // DT = 0.005 → 200 steps per second

#[test]
fn at_least_one_knockdown_recovers() {
    let mut r = Recover::new();
    let kinds = r.num_kinds();
    assert!(kinds >= 5, "expected several knockdown starts, got {kinds}");

    let mut any = false;
    for i in 0..kinds {
        r.knock(i);
        for _ in 0..(15 * SEC) {
            r.step(1);
            match r.outcome() {
                1 => {
                    any = true;
                    break;
                }
                2 => break, // didn't catch this one — try the next
                _ => {}
            }
        }
        if any {
            break;
        }
    }
    assert!(any, "at least one knockdown start should recover upright");
}

#[test]
fn catalogue_and_render_buffer_exposed() {
    let r = Recover::new();
    assert!(!r.name_at(0).is_empty(), "first knockdown should be named");
    assert!(!r.current_name().is_empty());
    assert_eq!(r.positions().len(), 6, "2-link arm = 3 points");
}
