//! Live visualization of the Stage-4 competing population.
//!
//! `cargo run --release --features "game vectordb" --bin popviz`
//!
//! A background thread continuously evolves a population of weak CEM islands
//! (sharing discoveries through RuVector). The window shows one **live arm per
//! island**, driven in real time by that island's *current* champion swing-up
//! policy — so you watch them compete and improve. Press **S** to toggle sharing
//! on/off: with it on, a laggard arm visibly inherits the global-best policy the
//! instant a migration fires; with it off, the stragglers keep flailing. **R**
//! restarts the population. **H** toggles a plain-language overlay (on at launch)
//! that explains what every label, colour, and the gold box mean.

use macroquad::prelude::*;
use pendulum_rs::control::{balance_gain, upright_energy};
use pendulum_rs::learn::{recover_torque_with_policy, EnergyShapingPolicy, PopulationSim, NP};
use pendulum_rs::simulator::Pendulum;
use std::f64::consts::PI;
use std::sync::{Arc, Mutex};

const N: usize = 8; // islands (= live arms)
const POP: usize = 10; // candidates per island generation
const CASES: usize = 6; // training knockdowns per fitness eval
const MIGRATE: usize = 3; // share/migrate every N generations
const SEED: u64 = 7;
const DT: f64 = 0.005;
const U_MAX: f64 = 150.0;

/// Snapshot the evolution thread publishes for the render loop to read.
#[derive(Default)]
struct Shared {
    champions: Vec<[f64; NP]>,
    fitnesses: Vec<f64>,
    generation: usize,
    rollouts: usize,
    best_island: usize,
    sharing: bool,
    migrated_pulse: bool,
    want_sharing: bool,
    want_reset: bool,
    ready: bool,
}

fn window_conf() -> Conf {
    Conf { window_title: "RuVector — competing population".to_owned(), window_width: 1100, window_height: 680, ..Default::default() }
}

#[macroquad::main(window_conf)]
async fn main() {
    let shared = Arc::new(Mutex::new(Shared { want_sharing: true, ..Default::default() }));

    // --- evolution thread: owns the PopulationSim, publishes snapshots ---
    {
        let bg = shared.clone();
        std::thread::spawn(move || {
            let make = || PopulationSim::new(SEED, N, POP, CASES, MIGRATE, true, "popviz_pop.db");
            let mut sim = make();
            loop {
                let (reset, want) = {
                    let s = bg.lock().unwrap();
                    (s.want_reset, s.want_sharing)
                };
                if reset {
                    sim = make();
                    bg.lock().unwrap().want_reset = false;
                }
                sim.set_sharing(want);
                let migrated = sim.step_generation();
                {
                    let mut s = bg.lock().unwrap();
                    s.champions = (0..sim.n_islands()).map(|i| sim.champion(i)).collect();
                    s.fitnesses = (0..sim.n_islands()).map(|i| sim.fitness(i)).collect();
                    s.generation = sim.generation();
                    s.rollouts = sim.total_rollouts();
                    s.best_island = sim.best_island();
                    s.sharing = sim.sharing();
                    if migrated {
                        s.migrated_pulse = true;
                    }
                    s.ready = true;
                }
                // Throttle so the search advances at a watchable pace.
                std::thread::sleep(std::time::Duration::from_millis(180));
            }
        });
    }

    // --- live arms (one per island) ---
    let nominal = Pendulum::new(vec![1.0, 1.0], vec![1.0, 1.0], vec![0.05, 0.05], 9.81, DT);
    let k = balance_gain(&nominal, DT);
    let e_up = upright_energy(&nominal);
    let mut arms: Vec<Pendulum> = (0..N).map(|_| fresh_arm()).collect();
    let mut up_timer = vec![0.0f64; N]; // seconds each arm has held near upright
    let mut champions = vec![EnergyShapingPolicy::baseline().p; N];
    let mut acc = 0.0f32;
    let mut flash = 0.0f32; // migration flash timer
    let mut show_help = true; // plain-language "what am I looking at?" panel, on at launch

    loop {
        // --- input ---
        if is_key_pressed(KeyCode::S) {
            let mut s = shared.lock().unwrap();
            s.want_sharing = !s.want_sharing;
        }
        if is_key_pressed(KeyCode::R) {
            shared.lock().unwrap().want_reset = true;
            arms = (0..N).map(|_| fresh_arm()).collect();
            up_timer = vec![0.0; N];
        }
        if is_key_pressed(KeyCode::H) {
            show_help = !show_help;
        }

        // --- pull the latest snapshot ---
        let (fits, gen, rollouts, best, sharing) = {
            let mut s = shared.lock().unwrap();
            if s.ready && s.champions.len() == N {
                champions.copy_from_slice(&s.champions);
            }
            if s.migrated_pulse {
                s.migrated_pulse = false;
                flash = 0.6;
            }
            (s.fitnesses.clone(), s.generation, s.rollouts, s.best_island, s.want_sharing)
        };

        // --- fixed-step physics: each arm driven by its island's champion ---
        acc += get_frame_time().min(0.05);
        while acc as f64 >= DT {
            for i in 0..N {
                let policy = EnergyShapingPolicy { p: champions[i] };
                let u = recover_torque_with_policy(&arms[i], &policy, &k, e_up, U_MAX);
                arms[i].step(&[u, 0.0]);
                let tip = tip_error(&arms[i]);
                if tip < 0.3 {
                    up_timer[i] += DT;
                } else {
                    up_timer[i] = 0.0;
                }
                // Once it has held upright a moment, knock it down again so we
                // keep watching each champion *earn* the recovery.
                if up_timer[i] > 1.5 {
                    knock_down(&mut arms[i]);
                    up_timer[i] = 0.0;
                }
            }
            acc -= DT as f32;
        }
        if flash > 0.0 {
            flash -= get_frame_time();
        }

        // --- draw ---
        clear_background(Color::new(0.06, 0.07, 0.09, 1.0));
        let cols = 4;
        let rows = N.div_ceil(cols);
        let cw = screen_width() / cols as f32;
        let ch = (screen_height() - 70.0) / rows as f32;
        for i in 0..N {
            let cx = (i % cols) as f32 * cw + cw * 0.5;
            let cy = 60.0 + (i / cols) as f32 * ch + ch * 0.55;
            let fit = fits.get(i).copied().unwrap_or(f64::NEG_INFINITY);
            let col = fitness_color(fit);
            let is_best = i == best && fits.get(i).map(|f| f.is_finite()).unwrap_or(false);
            // cell highlight for the global best
            if is_best {
                draw_rectangle_lines((i % cols) as f32 * cw + 3.0, 60.0 + (i / cols) as f32 * ch + 3.0, cw - 6.0, ch - 6.0, 3.0, GOLD);
            }
            draw_arm(&arms[i], (cx, cy), col);
            let label = if fit.is_finite() { format!("island {i}   fit {fit:.0}") } else { format!("island {i}   …") };
            draw_text(&label, (i % cols) as f32 * cw + 10.0, 60.0 + (i / cols) as f32 * ch + 18.0, 20.0, if is_best { GOLD } else { LIGHTGRAY });
        }

        // migration flash overlay
        if flash > 0.0 {
            let a = (flash / 0.6).clamp(0.0, 1.0) * 0.25;
            draw_rectangle(0.0, 0.0, screen_width(), screen_height(), Color::new(0.4, 0.9, 0.5, a));
        }

        // --- HUD ---
        draw_text("RuVector — competing population (each arm = one island's champion)", 16.0, 26.0, 26.0, WHITE);
        let share_tag = if sharing { "ON" } else { "off" };
        let share_col = if sharing { GREEN } else { ORANGE };
        draw_text(
            &format!("gen {gen}   rollouts {rollouts}   sharing [{share_tag}]   (S toggle · R restart · H help)"),
            16.0,
            48.0,
            22.0,
            share_col,
        );

        if show_help {
            draw_help_panel();
        }

        next_frame().await
    }
}

/// Plain-language overlay explaining what the screen is showing. Shown at launch
/// (a first-time viewer has no way to guess what "island 7 fit 81" means) and
/// toggled with H so it gets out of the way once you know.
fn draw_help_panel() {
    let lines: [&str; 18] = [
        "What am I looking at?",
        "",
        "Each box is an \"island\" - its own trial-and-error search for how to",
        "swing this arm up and balance it. The arm runs that island's best try so far.",
        "",
        "fit = fitness: how well that best try recovers after being knocked down.",
        "Higher is better. Arms shade red (weak) -> green (strong).",
        "The gold box marks the best island in the whole population right now.",
        "",
        "Arms get knocked down again and again on purpose, so you can",
        "watch each one earn its recovery.",
        "",
        "gen = rounds of improvement.   rollouts = total practice attempts.",
        "sharing (S): islands post their best find into RuVector and copy the",
        "overall best back. A weak arm can jump ahead the instant a share fires",
        "(the green flash). Turn sharing off to watch them struggle alone.",
        "",
        "S = sharing on/off     R = restart     H = hide / show this",
    ];
    let pad = 24.0;
    let line_h = 24.0;
    let w = 900.0_f32.min(screen_width() - 40.0);
    let h = pad * 2.0 + line_h * lines.len() as f32;
    let x = (screen_width() - w) * 0.5;
    let y = (screen_height() - h) * 0.5;
    draw_rectangle(x, y, w, h, Color::new(0.04, 0.05, 0.07, 0.93));
    draw_rectangle_lines(x, y, w, h, 2.0, GOLD);
    for (i, ln) in lines.iter().enumerate() {
        let (size, color) = if i == 0 { (26.0, GOLD) } else { (20.0, LIGHTGRAY) };
        draw_text(ln, x + pad, y + pad + line_h * (i as f32 + 1.0) - 6.0, size, color);
    }
}

fn fresh_arm() -> Pendulum {
    let mut a = Pendulum::new(vec![1.0, 1.0], vec![1.0, 1.0], vec![0.05, 0.05], 9.81, DT);
    a.reset(vec![0.1, -0.1], vec![0.0, 0.0]); // start from a dead hang
    a
}

/// Knock an arm down to a random-ish sprawl so we keep watching it recover.
fn knock_down(arm: &mut Pendulum) {
    let r = || (rand::gen_range(-1.0f64, 1.0)) * PI;
    arm.reset(vec![PI + r(), PI + r()], vec![0.0, 0.0]);
}

fn tip_error(sim: &Pendulum) -> f64 {
    let w = |a: f64| (a + PI).rem_euclid(2.0 * PI) - PI;
    w(sim.theta[0] - PI).abs() + w(sim.theta[1] - PI).abs()
}

/// Red (weak) → green (strong) by champion fitness.
fn fitness_color(fit: f64) -> Color {
    if !fit.is_finite() {
        return Color::new(0.45, 0.45, 0.5, 1.0);
    }
    let t = ((fit + 60.0) / 160.0).clamp(0.0, 1.0) as f32; // ~[-60,100] → [0,1]
    Color::new(0.9 - 0.7 * t, 0.3 + 0.6 * t, 0.3, 1.0)
}

fn draw_arm(sim: &Pendulum, base: (f32, f32), color: Color) {
    let scale = 32.0f32;
    let to_screen = |p: (f64, f64)| (base.0 + p.0 as f32 * scale, base.1 - p.1 as f32 * scale);
    let pts = sim.link_positions();
    draw_circle(base.0, base.1, 3.0, DARKGRAY);
    for w in pts.windows(2) {
        let (x1, y1) = to_screen(w[0]);
        let (x2, y2) = to_screen(w[1]);
        draw_line(x1, y1, x2, y2, 4.0, color);
    }
    for p in &pts {
        let (x, y) = to_screen(*p);
        draw_circle(x, y, 4.0, color);
    }
}
