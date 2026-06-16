//! "You vs RuVector" — an interactive arm-balancing duel.
//!
//! Two underactuated 2-link arms (only joint 0 is motorized) stand side by side.
//! LEFT is yours: press **A / D** to drive the base motor left / right and try
//! to keep the arm balanced straight up. RIGHT is the auto arm (LQR balance that
//! recalibrates on disturbance — Phase 2 will make this RuVector-driven).
//!
//! Press **SPACE** to fire a disturbance (both arms' second link extends): watch
//! the auto arm recover while you fight to keep yours up. **R** resets.
//!
//!   cargo run --release --features game --bin play
//!
//! Note: balancing an underactuated double pendulum by hand is *hard* — that's
//! the point. RuVector makes it look easy.

use macroquad::prelude::*;
use pendulum_rs::control::{balance_gain, balance_torque, Vec4};
use pendulum_rs::simulator::Pendulum;
use std::f64::consts::PI;

const DT: f64 = 0.004;
const U_MAX: f64 = 150.0;
const HUMAN_TORQUE: f64 = 90.0; // torque you apply per key hold
const SCALE: f32 = 90.0; // pixels per meter
const NEW_LEN: f64 = 2.0;

struct Game {
    you: Pendulum,
    auto: Pendulum,
    k_auto: Vec4,
    t: f64,
    disturbed: bool,
    you_alive: bool,
    you_survived: f64,
    auto_alive: bool,
    auto_wind_on: bool,
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
        Game {
            you,
            auto,
            k_auto,
            t: 0.0,
            disturbed: false,
            you_alive: true,
            you_survived: 0.0,
            auto_alive: true,
            auto_wind_on: false,
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
        // The auto arm recalibrates (oracle now; RuVector in Phase 2).
        self.k_auto = balance_gain(&self.auto, DT);
        self.disturbed = true;
    }

    fn step(&mut self, human_torque: f64) {
        // You: your key torque drives joint 0 — but ONLY while alive. Once fallen,
        // input is locked out (torque forced to 0) and the arm flops passively.
        let ht = if self.you_alive {
            human_torque.clamp(-U_MAX, U_MAX)
        } else {
            0.0
        };
        self.you.step(&[ht, 0.0]);
        if self.you_alive {
            if Self::tip_error(&self.you) > 1.4 {
                self.you_alive = false;
            } else {
                self.you_survived = self.t;
            }
        }

        // Auto: LQR balance while alive; passive flop after it loses.
        if self.auto_alive {
            let u = balance_torque(&self.k_auto, &self.auto.theta, &self.auto.omega, U_MAX);
            self.auto.step(&[u, 0.0]);
            if Self::tip_error(&self.auto) > 1.4 {
                self.auto_alive = false;
            }
        } else {
            self.auto.step(&[0.0, 0.0]);
        }
        self.t += DT;
    }

    // --- "bother the RuVector arm" disturbances ---------------------------
    /// Shove the auto arm's elbow with a velocity impulse (recoverable kick).
    fn poke_auto(&mut self, dir: f64) {
        if self.auto_alive {
            self.auto.omega[1] += dir * 3.0;
        }
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
        draw_arm(&game.you, you_base, Color::new(0.86, 0.27, 0.27, 1.0), game.you_alive);
        draw_arm(&game.auto, auto_base, Color::new(0.27, 0.78, 0.43, 1.0), game.auto_alive);

        // labels
        let you_status = if game.you_alive { "BALANCING" } else { "FELL" };
        let auto_status = if game.auto_alive { "BALANCING" } else { "FELL" };
        draw_text("YOU  (A / D to rotate motor)", you_base.0 - 130.0, 40.0, 26.0, WHITE);
        draw_text(
            &format!("{}   survived {:.1}s", you_status, game.you_survived),
            you_base.0 - 130.0,
            68.0,
            22.0,
            if game.you_alive { GREEN } else { RED },
        );
        draw_text("RuVector  (auto-balance + recalibrate)", auto_base.0 - 150.0, 40.0, 26.0, WHITE);
        draw_text(auto_status, auto_base.0 - 150.0, 68.0, 22.0, if game.auto_alive { GREEN } else { RED });

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
