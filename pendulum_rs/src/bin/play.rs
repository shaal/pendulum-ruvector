//! "You vs RuVector" — an interactive arm-balancing duel.
//!
//! Two underactuated 2-link arms (only joint 0 is motorized) stand side by side.
//! LEFT is yours: press **A / D** to drive the base motor left / right and try
//! to keep the arm balanced straight up. RIGHT is the auto arm: LQR balance that
//! recalibrates on disturbance, plus a Phase-3 collocated-PFL swing-up that
//! hoists it back up from most full knockdowns (the gain it adopts is recalled
//! from RuVector by recognition in the `estimate` demo; here it uses the oracle).
//!
//! Press **SPACE** to fire a disturbance (both arms' second link extends): watch
//! the auto arm recover while you fight to keep yours up. **R** resets.
//!
//!   cargo run --release --features game --bin play                 # oracle recalibration
//!   cargo run --release --features "game vectordb" --bin play      # LIVE RuVector recognition
//!
//! With `vectordb` the auto arm earns its recalibration: on the length
//! disturbance it runs a short dithered recognition probe, identifies its new
//! dynamics signature, and recalls the matching gain from a seeded RuVector
//! store (the Phase-2 pipeline, live in the game). Without it, it uses the
//! oracle for an instant gain.
//!
//! Note: balancing an underactuated double pendulum by hand is *hard* — that's
//! the point. RuVector makes it look easy.

use macroquad::prelude::*;
use pendulum_rs::control::{balance_gain, recover_torque, upright_energy, Vec4};
use pendulum_rs::simulator::Pendulum;
use std::f64::consts::PI;

#[cfg(feature = "vectordb")]
use pendulum_rs::estimator::{OnlineEstimator, Signature};
#[cfg(feature = "vectordb")]
use pendulum_rs::memory::ConfigMemory;

const DT: f64 = 0.004;
const U_MAX: f64 = 150.0;
const HUMAN_TORQUE: f64 = 90.0; // torque you apply per key hold
const SCALE: f32 = 90.0; // pixels per meter
const NEW_LEN: f64 = 2.0;

/// Live RuVector recognition for the in-game length disturbance (Phase 2,
/// wired into the duel). Seeds a config grid once, then on a disturbance runs a
/// dithered probe on the auto arm, identifies its signature, and recalls the
/// matching gain — replacing the oracle.
#[cfg(feature = "vectordb")]
struct Recognizer {
    mem: ConfigMemory,
    est: OnlineEstimator,
    k_probe: Vec4,
    active: bool,
    t_start: f64,
    smoothed: Option<Signature>,
    /// Human-readable status for the HUD.
    status: String,
    /// Recognition lag once committed (seconds from disturbance to adoption).
    lag: Option<f64>,
}

#[cfg(feature = "vectordb")]
impl Recognizer {
    // Probe tuning mirrors the `estimate` binary.
    const MIN_SAMPLES: usize = 25;
    const FREEZE_TIP: f64 = 0.18;
    const EMA: f64 = 0.5;
    const COMMIT: f32 = 5.0; // whitened-L2 distance under which we adopt the recall
    const TIMEOUT: f64 = 0.9; // give up to the oracle if not recognized by then

    fn new() -> Self {
        let mut mem = ConfigMemory::new("play_configs.db").expect("open RuVector store");
        mem.seed_grid().expect("seed config grid");
        // Probe with the *seed's* gain (built at the memory's seed dt), not one
        // recomputed at the game's control dt — the signature must match.
        let k_probe = mem.probe_gain();
        Recognizer {
            mem,
            est: OnlineEstimator::new(240, 1e-4),
            k_probe,
            active: false,
            t_start: 0.0,
            smoothed: None,
            status: String::new(),
            lag: None,
        }
    }

    fn start(&mut self, t: f64) {
        self.active = true;
        self.t_start = t;
        self.est.clear();
        self.smoothed = None;
        self.status = "RECOGNIZING…".to_string();
        self.lag = None;
    }

    /// Probe torque while recognizing: the stale gain plus an exogenous dither.
    fn probe_torque(&self, auto: &Pendulum, t: f64) -> (f64, f64) {
        let e0 = (auto.theta[0] - PI + PI).rem_euclid(2.0 * PI) - PI;
        let e1 = (auto.theta[1] - PI + PI).rem_euclid(2.0 * PI) - PI;
        let k = &self.k_probe;
        let u_fb = -(k[0] * e0 + k[1] * e1 + k[2] * auto.omega[0] + k[3] * auto.omega[1]);
        let dither = 6.0 * (2.0 * PI * 1.7 * t).sin() + 4.0 * (2.0 * PI * 3.3 * t).sin();
        ((u_fb + dither).clamp(-U_MAX, U_MAX), dither)
    }

    /// Record one probe step and try to commit a recall. On success it adopts
    /// the recalled gain into `k_auto`/`e_up` and ends the probe; on timeout it
    /// falls back to the oracle so the arm never just gives up.
    #[allow(clippy::too_many_arguments)]
    fn observe(
        &mut self,
        theta_before: &[f64],
        omega_before: &[f64],
        dither: f64,
        omega_after: &[f64],
        t: f64,
        auto: &Pendulum,
        k_auto: &mut Vec4,
        e_up: &mut f64,
    ) {
        let tip = {
            let w = |a: f64| (a + PI).rem_euclid(2.0 * PI) - PI;
            w(theta_before[0] - PI).abs() + w(theta_before[1] - PI).abs()
        };
        if tip < Self::FREEZE_TIP {
            self.est.observe(theta_before, omega_before, dither, omega_after, DT);
        }
        if self.est.len() >= Self::MIN_SAMPLES {
            if let Some(raw) = self.est.estimate() {
                let sig: Signature = match self.smoothed {
                    Some(prev) => {
                        let s = std::array::from_fn(|i| Self::EMA * raw[i] + (1.0 - Self::EMA) * prev[i]);
                        self.smoothed = Some(s);
                        s
                    }
                    None => {
                        self.smoothed = Some(raw);
                        raw
                    }
                };
                if let Ok(Some(rc)) = self.mem.recall(&sig) {
                    if rc.score < Self::COMMIT {
                        *k_auto = rc.k;
                        *e_up = rc.e_up;
                        self.lag = Some(t - self.t_start);
                        self.status = format!("RECALLED l1≈{:.1}m in {:.2}s", rc.l1, t - self.t_start);
                        self.active = false;
                        return;
                    }
                }
            }
        }
        if t - self.t_start > Self::TIMEOUT {
            // Couldn't recognize in time — fall back to the oracle.
            *k_auto = balance_gain(auto, DT);
            *e_up = upright_energy(auto);
            self.status = "recognition timed out → oracle".to_string();
            self.active = false;
        }
    }
}

struct Game {
    you: Pendulum,
    auto: Pendulum,
    k_auto: Vec4,
    e_up: f64, // auto arm's upright energy target (for swing-up); refreshed on change
    t: f64,
    disturbed: bool,
    you_up: bool,       // currently near upright (for display only — never locks input)
    you_balanced: f64,  // total time you've kept it up
    auto_up: bool,
    auto_wind_on: bool,
    #[cfg(feature = "vectordb")]
    recog: Recognizer,
}

fn fresh_arm() -> Pendulum {
    let mut a = Pendulum::new(vec![1.0, 1.0], vec![1.0, 1.0], vec![0.05, 0.05], 9.81, DT);
    // Start a touch off-upright so there is immediately something to control.
    a.reset(vec![PI + 0.12, PI - 0.10], vec![0.0, 0.0]);
    a
}

impl Game {
    fn new() -> Self {
        let you = fresh_arm();
        let auto = fresh_arm();
        let k_auto = balance_gain(&auto, DT);
        let e_up = upright_energy(&auto);
        Game {
            you,
            auto,
            k_auto,
            e_up,
            t: 0.0,
            disturbed: false,
            you_up: true,
            you_balanced: 0.0,
            auto_up: true,
            auto_wind_on: false,
            #[cfg(feature = "vectordb")]
            recog: Recognizer::new(),
        }
    }

    fn tip_error(sim: &Pendulum) -> f64 {
        let w = |a: f64| (a + PI).rem_euclid(2.0 * PI) - PI;
        w(sim.theta[0] - PI).abs() + w(sim.theta[1] - PI).abs()
    }

    fn disturb(&mut self) {
        if self.disturbed {
            return;
        }
        self.you.set_length(1, NEW_LEN);
        self.auto.set_length(1, NEW_LEN);
        self.e_up = upright_energy(&self.auto);
        // With RuVector: kick off a live recognition probe (the auto arm keeps
        // its stale gain while it identifies the new arm, then adopts the recall).
        // Without it: the oracle hands over an instant gain.
        #[cfg(feature = "vectordb")]
        {
            self.recog.start(self.t);
        }
        #[cfg(not(feature = "vectordb"))]
        {
            self.k_auto = balance_gain(&self.auto, DT);
        }
        self.disturbed = true;
    }

    fn step(&mut self, human_torque: f64) {
        // You: A/D ALWAYS drives joint 0 — even when tipped over, so you can try
        // to fight the arm back up. There is no "game over".
        self.you.step(&[human_torque.clamp(-U_MAX, U_MAX), 0.0]);
        self.you_up = Self::tip_error(&self.you) < 1.4;
        if self.you_up {
            self.you_balanced += DT;
        }

        // Auto: ALWAYS tries to recover — LQR balance within its basin (catches
        // pokes, even big ones), and Phase-3 collocated-PFL energy swing-up when
        // knocked further out. It now hoists itself back up from most full
        // knockdowns, including a dead hang (≈7/10 of the `check` harness starts);
        // a few worst-case configurations still defeat it (chaotic, unsolved).
        //
        // While a RuVector recognition probe is active, the auto arm runs the
        // dithered probe controller and identifies its new gain instead of
        // balancing; once it commits (or times out) it returns to recover_torque.
        #[cfg(feature = "vectordb")]
        if self.recog.active {
            let theta_before = self.auto.theta.clone();
            let omega_before = self.auto.omega.clone();
            let (u, dither) = self.recog.probe_torque(&self.auto, self.t);
            self.auto.step(&[u, 0.0]);
            let omega_after = self.auto.omega.clone();
            // Disjoint field borrows: `recog` (mut), `auto` (shared), `k_auto`/
            // `e_up` (mut) are distinct fields of `self`, so this is allowed.
            self.recog.observe(
                &theta_before,
                &omega_before,
                dither,
                &omega_after,
                self.t,
                &self.auto,
                &mut self.k_auto,
                &mut self.e_up,
            );
            self.auto_up = Self::tip_error(&self.auto) < 1.4;
            self.t += DT;
            return;
        }

        let u = recover_torque(&self.auto, &self.k_auto, self.e_up, U_MAX);
        self.auto.step(&[u, 0.0]);
        self.auto_up = Self::tip_error(&self.auto) < 1.4;
        self.t += DT;
    }

    // --- "bother the RuVector arm" disturbances ---------------------------
    /// Shove the auto arm's elbow with a velocity impulse (recoverable kick).
    fn poke_auto(&mut self, dir: f64) {
        self.auto.omega[1] += dir * 3.0;
    }
    /// Toggle a steady wind on the auto arm (a sustained force it must fight).
    fn toggle_wind(&mut self) {
        self.auto_wind_on = !self.auto_wind_on;
        self.auto.wind = if self.auto_wind_on { 5.0 } else { 0.0 };
    }
    /// Hang an extra 1 kg on the auto arm's tip (cumulative).
    fn add_payload(&mut self) {
        let m = self.auto.m[1] + 1.0;
        self.auto.set_mass(1, m);
        self.e_up = upright_energy(&self.auto);
    }
}

fn to_screen(p: (f64, f64), base: (f32, f32)) -> (f32, f32) {
    (base.0 + p.0 as f32 * SCALE, base.1 - p.1 as f32 * SCALE)
}

fn draw_arm(sim: &Pendulum, base: (f32, f32), color: Color, alive: bool) {
    let col = if alive { color } else { Color::new(0.45, 0.45, 0.45, 1.0) };
    let pts = sim.link_positions();
    // ground / base marker
    draw_line(base.0 - 60.0, base.1, base.0 + 60.0, base.1, 3.0, DARKGRAY);
    for w in pts.windows(2) {
        let (x1, y1) = to_screen(w[0], base);
        let (x2, y2) = to_screen(w[1], base);
        draw_line(x1, y1, x2, y2, 7.0, col);
    }
    for p in &pts {
        let (x, y) = to_screen(*p, base);
        draw_circle(x, y, 8.0, col);
    }
}

fn window_conf() -> Conf {
    Conf {
        window_title: "Arm Duel: You vs RuVector".to_owned(),
        window_width: 1000,
        window_height: 640,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut game = Game::new();
    let mut acc = 0.0f32;

    loop {
        // --- input ---
        if is_key_pressed(KeyCode::R) {
            game = Game::new();
        }
        if is_key_pressed(KeyCode::Space) {
            game.disturb();
        }
        // "Bother the RuVector arm" controls.
        if is_key_pressed(KeyCode::Left) {
            game.poke_auto(-1.0);
        }
        if is_key_pressed(KeyCode::Right) {
            game.poke_auto(1.0);
        }
        if is_key_pressed(KeyCode::W) {
            game.toggle_wind();
        }
        if is_key_pressed(KeyCode::M) {
            game.add_payload();
        }
        let mut torque = 0.0;
        if is_key_down(KeyCode::A) {
            torque -= HUMAN_TORQUE;
        }
        if is_key_down(KeyCode::D) {
            torque += HUMAN_TORQUE;
        }

        // --- fixed-step physics ---
        acc += get_frame_time().min(0.05);
        while acc as f64 >= DT {
            game.step(torque);
            acc -= DT as f32;
        }

        // --- draw ---
        clear_background(Color::new(0.07, 0.08, 0.10, 1.0));
        let h = screen_height();
        let you_base = (screen_width() * 0.27, h * 0.78);
        let auto_base = (screen_width() * 0.73, h * 0.78);
        draw_arm(&game.you, you_base, Color::new(0.86, 0.27, 0.27, 1.0), game.you_up);
        draw_arm(&game.auto, auto_base, Color::new(0.27, 0.78, 0.43, 1.0), game.auto_up);

        // labels
        let you_status = if game.you_up { "BALANCING" } else { "DOWN — fight it back up!" };
        let auto_status = if game.auto_up { "BALANCING" } else { "RECOVERING…" };
        draw_text("YOU  (A / D to rotate motor)", you_base.0 - 130.0, 40.0, 26.0, WHITE);
        draw_text(
            &format!("{}   balanced {:.1}s", you_status, game.you_balanced),
            you_base.0 - 130.0,
            68.0,
            22.0,
            if game.you_up { GREEN } else { ORANGE },
        );
        draw_text("RuVector  (auto-balance + recalibrate)", auto_base.0 - 150.0, 40.0, 26.0, WHITE);
        draw_text(auto_status, auto_base.0 - 150.0, 68.0, 22.0, if game.auto_up { GREEN } else { ORANGE });
        // Live recognition status (only with the `vectordb` build).
        #[cfg(feature = "vectordb")]
        if !game.recog.status.is_empty() {
            let col = if game.recog.active { YELLOW } else { SKYBLUE };
            draw_text(&game.recog.status, auto_base.0 - 150.0, 92.0, 20.0, col);
        }

        // Controls for bothering the RuVector arm (with live wind indicator).
        let wind_tag = if game.auto_wind_on { "ON" } else { "off" };
        draw_text(
            &format!("Bother RuVector:  \u{2190}/\u{2192} poke    W wind [{wind_tag}]    M +payload"),
            30.0,
            h - 52.0,
            22.0,
            SKYBLUE,
        );
        let banner = if !game.disturbed {
            "SPACE = extend both arms    R = reset"
        } else {
            "DISTURBANCE FIRED — keep yours up!    R = reset"
        };
        draw_text(banner, 30.0, h - 24.0, 24.0, YELLOW);
        draw_text(&format!("t = {:.1}s", game.t), screen_width() - 120.0, h - 24.0, 24.0, WHITE);

        next_frame().await
    }
}
