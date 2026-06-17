//! Native test of the Duel handle (You vs RuVector): the auto arm balances itself,
//! the hand arm falls without input, and the length disturbance triggers a live
//! RuVector recall — the same flow as the in-browser station.

use pendulum_web::Duel;

const SEC: usize = 250; // DUEL_DT = 0.004 → 250 steps per second

#[test]
fn auto_balances_and_human_falls_without_input() {
    let mut d = Duel::new();
    for _ in 0..(3 * SEC) {
        d.step(1, 0.0); // no human input
    }
    assert!(d.auto_up(), "auto arm should keep itself balanced");
    assert!(!d.you_up(), "hand arm falls with no input");
    assert_eq!(d.you_positions().len(), 6);
    assert_eq!(d.auto_positions().len(), 6);
}

#[test]
fn disturbance_triggers_ruvector_recall() {
    let mut d = Duel::new();
    for _ in 0..SEC {
        d.step(1, 0.0); // settle
    }
    d.disturb();
    assert!(d.disturbed());
    for _ in 0..(2 * SEC) {
        d.step(1, 0.0);
    }
    assert!(!d.recog_active(), "recognition probe should finish");
    let status = d.recog_status();
    assert!(
        status.contains("RECALLED") || status.contains("recognized"),
        "status should report a recall/recognition, got: {status:?}"
    );
    assert!(d.auto_up(), "auto arm should recover after recalling its new gain");
}

#[test]
fn pokes_and_reset_dont_crash() {
    let mut d = Duel::new();
    d.poke_auto(-1.0);
    d.poke_auto(1.0);
    d.toggle_wind();
    assert!(d.wind_on());
    d.add_payload();
    for _ in 0..(SEC / 2) {
        d.step(1, 1.0); // also drive the hand arm
    }
    d.reset();
    assert!(!d.disturbed());
    assert!(!d.wind_on());
}
