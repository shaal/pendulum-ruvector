//! Phase 1 — underactuated arm balance, naive vs adaptive, side by side.
//!
//! Two identical 2-link arms (only joint 0 motorized) balance straight up. At
//! t = 2s the **second link changes length** on BOTH arms (e.g. a tool extends).
//! The true dynamics shift: the *naive* arm keeps its old balance gain and
//! topples; the *adaptive* arm recomputes its gain for the new arm and stays up.
//!
//! In Phase 1 the adaptive arm is handed the new length by an oracle — this
//! validates the control. Phase 2 replaces the oracle with RuVector estimating
//! the change from the observed motion.
//!
//!   cargo run --release --bin arm -- --duration 8 --newlen 2.0 --out arm.rrd --csv arm.csv
//!
//! CSV columns per row: naive joint positions, then adaptive joint positions
//! (x0,y0,...,xn,yn, X0,Y0,...,Xn,Yn) — for side-by-side rendering.

use pendulum_rs::control::{balance_gain, balance_torque, Vec4};
use pendulum_rs::simulator::Pendulum;
use std::f64::consts::PI;
use std::io::Write;

struct Args {
    duration: f64,
    out: String,
    csv: Option<String>,
    new_len: f64, // link-2 length after the change
}

fn parse_args() -> Args {
    let mut a = Args {
        duration: 8.0,
        out: "arm.rrd".to_string(),
        csv: None,
        new_len: 2.0,
    };
    let mut it = std::env::args().skip(1);
    while let Some(f) = it.next() {
        match f.as_str() {
            "--duration" => a.duration = it.next().unwrap().parse().unwrap(),
            "--out" => a.out = it.next().unwrap(),
            "--csv" => a.csv = Some(it.next().unwrap()),
            "--newlen" => a.new_len = it.next().unwrap().parse().unwrap(),
            other => eprintln!("(ignoring {other})"),
        }
    }
    a
}

const DT: f64 = 0.005; // 200 Hz control
const U_MAX: f64 = 150.0;

fn new_arm() -> Pendulum {
    Pendulum::new(vec![1.0, 1.0], vec![1.0, 1.0], vec![0.05, 0.05], 9.81, DT)
}

fn tip_error(sim: &Pendulum) -> f64 {
    let w = |a: f64| (a + PI).rem_euclid(2.0 * PI) - PI;
    w(sim.theta[0] - PI).abs() + w(sim.theta[1] - PI).abs()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args();

    let mut naive = new_arm();
    let mut adaptive = new_arm();
    let theta0 = vec![PI - 0.2, PI + 0.15];
    naive.reset(theta0.clone(), vec![0.0; 2]);
    adaptive.reset(theta0.clone(), vec![0.0; 2]);

    // Both start with the gain for the original arm.
    let k0: Vec4 = balance_gain(&naive, DT);
    let mut k_naive = k0;
    let mut k_adaptive = k0;

    let rec = rerun::RecordingStreamBuilder::new("arm_balance").save(&args.out)?;
    let mut csv = args
        .csv
        .as_ref()
        .map(|p| std::io::BufWriter::new(std::fs::File::create(p).unwrap()));

    let disturb_step = (2.0 / DT) as usize;
    let total = (args.duration / DT) as usize;
    let late_start = total.saturating_sub((1.5 / DT) as usize);
    let (mut naive_late, mut adaptive_late) = (0.0f64, 0.0f64);

    for step in 0..total {
        // --- the world changes: link 2 changes length on both arms ---
        if step == disturb_step {
            naive.set_length(1, args.new_len);
            adaptive.set_length(1, args.new_len);
            // Adaptive arm recalibrates: recompute the gain for the NEW arm.
            // (Phase 1 oracle; Phase 2 = RuVector estimates this length.)
            k_adaptive = balance_gain(&adaptive, DT);
            let _ = &mut k_naive; // naive deliberately keeps its stale gain
            eprintln!("t=2.0s: link-2 length -> {} m. adaptive recomputed its gain.", args.new_len);
        }

        let un = balance_torque(&k_naive, &naive.theta, &naive.omega, U_MAX);
        naive.step(&[un, 0.0]);
        let ua = balance_torque(&k_adaptive, &adaptive.theta, &adaptive.omega, U_MAX);
        adaptive.step(&[ua, 0.0]);

        if step >= late_start {
            naive_late = naive_late.max(tip_error(&naive));
            adaptive_late = adaptive_late.max(tip_error(&adaptive));
        }

        // --- Rerun: draw both arms (naive left, adaptive right) ---
        rec.set_time_sequence("step", step as i64);
        log_arm(&rec, "world/naive", &naive, -2.0, (220, 70, 70))?;
        log_arm(&rec, "world/adaptive", &adaptive, 2.0, (70, 200, 110))?;
        rec.log("error/naive", &rerun::Scalars::new([tip_error(&naive)]))?;
        rec.log("error/adaptive", &rerun::Scalars::new([tip_error(&adaptive)]))?;

        // --- CSV for side-by-side GIF rendering ---
        if let Some(w) = csv.as_mut() {
            let mut row: Vec<String> = Vec::new();
            for p in naive.link_positions() {
                row.push(format!("{:.5}", p.0));
                row.push(format!("{:.5}", p.1));
            }
            for p in adaptive.link_positions() {
                row.push(format!("{:.5}", p.0));
                row.push(format!("{:.5}", p.1));
            }
            writeln!(w, "{}", row.join(","))?;
        }
    }

    let verdict = |late: f64| {
        if late < 0.15 {
            "holds straight ✅"
        } else if late < 0.6 {
            "wobbling ⚠️"
        } else {
            "FELL ❌"
        }
    };
    eprintln!(
        "\nRESULT (max error in last 1.5s after the change, radians):\n  naive arm:    {:.2} rad  {}\n  adaptive arm: {:.2} rad  {}",
        naive_late,
        verdict(naive_late),
        adaptive_late,
        verdict(adaptive_late),
    );
    eprintln!("Rerun: rerun {}", args.out);
    Ok(())
}

fn log_arm(
    rec: &rerun::RecordingStream,
    path: &str,
    sim: &Pendulum,
    x_shift: f32,
    color: (u8, u8, u8),
) -> Result<(), Box<dyn std::error::Error>> {
    let pts: Vec<[f32; 2]> = sim
        .link_positions()
        .iter()
        .map(|&(x, y)| [x as f32 + x_shift, y as f32])
        .collect();
    rec.log(
        format!("{path}/links"),
        &rerun::LineStrips2D::new([pts.clone()])
            .with_colors([rerun::Color::from_rgb(color.0, color.1, color.2)])
            .with_radii([0.03]),
    )?;
    rec.log(
        format!("{path}/joints"),
        &rerun::Points2D::new(pts).with_radii([0.06]),
    )?;
    Ok(())
}
