//! Watch reactive vs. predictive swing-up, side by side, in Rerun.
//!
//! Two identical knocked-down arms recover to upright at the same time:
//!
//!   * **left (red)**  — the evolved energy-shaping policy: *reactive*, it looks
//!     only at the current state and chatters the motor back and forth.
//!   * **right (green)** — the CEM model-predictive controller: *predictive*, it
//!     rolls the real dynamics forward, plans a smooth pump, and commits.
//!
//! Both share the identical LQR catch and basin threshold — the only difference
//! is the swing-up brain. The viewer shows the arms plus live time-series of tip
//! error, applied torque, and the cumulative **torque-reversal** count — so the
//! "fewer moves" claim is visible as it happens (the red trace flips sign
//! constantly; the green one barely moves).
//!
//!   cargo run --release --bin mpc_viz -- --spawn --start 7
//!   cargo run --release --bin mpc_viz -- --start 5 --duration 12 --out swingup.rrd
//!
//! `--start` indexes `learn::knockdown_starts()` (default 7 = dead hang).

use pendulum_rs::control::{balance_gain, balance_torque, upright_energy, Vec4};
use pendulum_rs::learn::{knockdown_starts, EnergyShapingPolicy, SwingUpPolicy, NOMINAL_CHAMPION};
use pendulum_rs::mpc::{MpcConfig, MpcSwingUp};
use pendulum_rs::simulator::Pendulum;
use std::f64::consts::PI;

const DT: f64 = 0.005;
const U_MAX: f64 = 150.0;

struct Args {
    start: usize,
    duration: f64,
    out: String,
    spawn: bool,
}

fn parse_args() -> Args {
    let mut a = Args { start: 7, duration: 12.0, out: "swingup.rrd".to_string(), spawn: false };
    let mut it = std::env::args().skip(1);
    while let Some(f) = it.next() {
        match f.as_str() {
            "--start" => a.start = it.next().unwrap().parse().unwrap(),
            "--duration" => a.duration = it.next().unwrap().parse().unwrap(),
            "--out" => a.out = it.next().unwrap(),
            "--spawn" => a.spawn = true,
            other => eprintln!("(ignoring {other})"),
        }
    }
    a
}

fn tip_error(sim: &Pendulum) -> f64 {
    let w = |a: f64| (a + PI).rem_euclid(2.0 * PI) - PI;
    w(sim.theta[0] - PI).abs() + w(sim.theta[1] - PI).abs()
}

/// One recovering arm with its swing-up brain and live "fewer moves" tallies.
struct Arm {
    sim: Pendulum,
    policy: Box<dyn SwingUpPolicy>,
    k: Vec4,
    e_up: f64,
    effort: f64,
    reversals: usize,
    last_sign: i32,
    hold: f64,
    time_to_catch: f64,
    caught_once: bool,
}

impl Arm {
    fn new(policy: Box<dyn SwingUpPolicy>, theta0: &[f64], secs: f64) -> Self {
        let mut sim = Pendulum::new(vec![1.0, 1.0], vec![1.0, 1.0], vec![0.05, 0.05], 9.81, DT);
        sim.reset(theta0.to_vec(), vec![0.0, 0.0]);
        let k = balance_gain(&sim, DT);
        let e_up = upright_energy(&sim);
        Arm { sim, policy, k, e_up, effort: 0.0, reversals: 0, last_sign: 0, hold: 0.0, time_to_catch: secs, caught_once: false }
    }

    /// Advance one control step (LQR catch inside the basin, swing-up outside),
    /// update the tallies, and return the torque applied (for plotting).
    fn step(&mut self, step: usize) -> f64 {
        let tip = tip_error(&self.sim);
        if tip < 0.2 {
            self.hold += DT;
            if self.hold >= 1.0 && !self.caught_once {
                self.caught_once = true;
                self.time_to_catch = step as f64 * DT;
            }
        } else {
            self.hold = 0.0;
        }
        let u = if tip < 1.0 {
            balance_torque(&self.k, &self.sim.theta, &self.sim.omega, U_MAX)
        } else {
            self.policy.torque(&self.sim, self.e_up, U_MAX)
        };
        self.effort += u.abs() * DT;
        let sign = if u > 1e-6 { 1 } else if u < -1e-6 { -1 } else { 0 };
        if sign != 0 {
            if self.last_sign != 0 && sign != self.last_sign {
                self.reversals += 1;
            }
            self.last_sign = sign;
        }
        self.sim.step(&[u, 0.0]);
        u
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args();
    let starts = knockdown_starts();
    let idx = args.start.min(starts.len() - 1);
    let (label, theta0) = &starts[idx];

    let mut reactive = Arm::new(Box::new(EnergyShapingPolicy { p: NOMINAL_CHAMPION }), theta0, args.duration);
    let mut predictive = Arm::new(Box::new(MpcSwingUp::new(MpcConfig::default())), theta0, args.duration);

    let builder = rerun::RecordingStreamBuilder::new("mpc_swingup");
    let rec = if args.spawn { builder.spawn()? } else { builder.save(&args.out)? };

    eprintln!("Swing-up from '{}' — left: reactive energy-shaping, right: predictive MPC.", label.trim());

    let total = (args.duration / DT) as usize;
    for step in 0..total {
        let u_r = reactive.step(step);
        let u_p = predictive.step(step);

        rec.set_time_sequence("step", step as i64);
        log_arm(&rec, "world/reactive", &reactive.sim, -2.0, (220, 70, 70))?;
        log_arm(&rec, "world/predictive", &predictive.sim, 2.0, (70, 200, 110))?;

        rec.log("tip_error/reactive", &rerun::Scalars::new([tip_error(&reactive.sim)]))?;
        rec.log("tip_error/predictive", &rerun::Scalars::new([tip_error(&predictive.sim)]))?;
        rec.log("torque/reactive", &rerun::Scalars::new([u_r]))?;
        rec.log("torque/predictive", &rerun::Scalars::new([u_p]))?;
        rec.log("reversals/reactive", &rerun::Scalars::new([reactive.reversals as f64]))?;
        rec.log("reversals/predictive", &rerun::Scalars::new([predictive.reversals as f64]))?;
    }

    let report = |name: &str, a: &Arm| {
        let caught = if a.caught_once { format!("caught at {:.1}s", a.time_to_catch) } else { "did not catch".to_string() };
        eprintln!("  {name:<22} {caught:<18} effort {:>5.0}  reversals {}", a.effort, a.reversals);
    };
    eprintln!("\nFrom '{}':", label.trim());
    report("reactive (energy-shape)", &reactive);
    report("predictive (MPC)", &predictive);
    if !args.spawn {
        eprintln!("\nRerun: rerun {}", args.out);
    }
    Ok(())
}

fn log_arm(
    rec: &rerun::RecordingStream,
    path: &str,
    sim: &Pendulum,
    x_shift: f32,
    color: (u8, u8, u8),
) -> Result<(), Box<dyn std::error::Error>> {
    let pts: Vec<[f32; 2]> = sim.link_positions().iter().map(|&(x, y)| [x as f32 + x_shift, y as f32]).collect();
    rec.log(
        format!("{path}/links"),
        &rerun::LineStrips2D::new([pts.clone()])
            .with_colors([rerun::Color::from_rgb(color.0, color.1, color.2)])
            .with_radii([0.03]),
    )?;
    rec.log(format!("{path}/joints"), &rerun::Points2D::new(pts).with_radii([0.06]))?;
    Ok(())
}
