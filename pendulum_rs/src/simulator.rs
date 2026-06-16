//! N-link pendulum dynamics — a direct Rust port of `pendulum_sim/simulator.py`.
//!
//! Same model: a planar chain of point masses `m_i` on massless rods `l_i`,
//! absolute angles `theta_i` from the downward vertical, integrated with a
//! fixed-step RK4. The manipulator equation is
//!
//! ```text
//! M(theta) * theta_ddot = tau - C(theta) * omega^2 - G(theta) - b * omega
//! ```
//!
//! We keep this dependency-free (a tiny Gaussian-elimination solver instead of
//! pulling `nalgebra`) so the crate's base build stays fast and has no version
//! overlap with the RuVector crates' pinned `nalgebra`.

/// A configurable n-link pendulum with torque actuation.
pub struct Pendulum {
    pub n: usize,
    /// Per-link masses. Read by the GNN feature; kept for completeness otherwise.
    #[allow(dead_code)]
    pub m: Vec<f64>,
    pub l: Vec<f64>,
    pub b: Vec<f64>,
    pub g: f64,
    pub dt: f64,
    /// `mu[i][j] = sum_{k >= max(i,j)} m_k` — constant tail-mass coupling.
    mu: Vec<Vec<f64>>,
    /// `cum_tail[i] = sum_{k >= i} m_k`.
    cum_tail: Vec<f64>,
    pub theta: Vec<f64>,
    pub omega: Vec<f64>,
    pub tau: Vec<f64>,
    pub t: f64,
}

impl Pendulum {
    pub fn new(masses: Vec<f64>, lengths: Vec<f64>, damping: Vec<f64>, gravity: f64, dt: f64) -> Self {
        let n = masses.len();
        assert_eq!(lengths.len(), n);
        assert_eq!(damping.len(), n);

        // cum_tail[i] = sum of masses from i to the tip.
        let mut cum_tail = vec![0.0; n];
        let mut acc = 0.0;
        for i in (0..n).rev() {
            acc += masses[i];
            cum_tail[i] = acc;
        }
        let mut mu = vec![vec![0.0; n]; n];
        for i in 0..n {
            for j in 0..n {
                mu[i][j] = cum_tail[i.max(j)];
            }
        }

        Self {
            n,
            m: masses,
            l: lengths,
            b: damping,
            g: gravity,
            dt,
            mu,
            cum_tail,
            theta: vec![0.0; n],
            omega: vec![0.0; n],
            tau: vec![0.0; n],
            t: 0.0,
        }
    }

    pub fn reset(&mut self, theta0: Vec<f64>, omega0: Vec<f64>) {
        self.theta = theta0;
        self.omega = omega0;
        self.tau = vec![0.0; self.n];
        self.t = 0.0;
    }

    /// Solve `M * x = rhs` for the angular accelerations.
    fn angular_acceleration(&self, theta: &[f64], omega: &[f64], tau: &[f64]) -> Vec<f64> {
        let n = self.n;
        let mut m_mat = vec![vec![0.0; n]; n];
        let mut rhs = vec![0.0; n];

        for i in 0..n {
            // Gravity + actuation + damping on the RHS.
            let g_i = self.g * self.cum_tail[i] * self.l[i] * theta[i].sin();
            rhs[i] = tau[i] - g_i - self.b[i] * omega[i];
            for j in 0..n {
                let d = theta[i] - theta[j];
                let coupling = self.mu[i][j] * self.l[i] * self.l[j];
                m_mat[i][j] = coupling * d.cos();
                // Centrifugal term C[i][j] * omega_j^2 moves to the RHS.
                rhs[i] -= coupling * d.sin() * omega[j] * omega[j];
            }
        }
        gauss_solve(m_mat, rhs)
    }

    /// State derivative for RK4: `y = [theta, omega] -> [omega, theta_ddot]`.
    fn deriv(&self, y: &[f64], tau: &[f64]) -> Vec<f64> {
        let n = self.n;
        let theta = &y[..n];
        let omega = &y[n..];
        let acc = self.angular_acceleration(theta, omega, tau);
        let mut out = vec![0.0; 2 * n];
        out[..n].copy_from_slice(omega);
        out[n..].copy_from_slice(&acc);
        out
    }

    /// Advance one timestep with fixed-step RK4. `tau` = applied joint torques.
    pub fn step(&mut self, tau: &[f64]) {
        let n = self.n;
        self.tau = tau.to_vec();
        let dt = self.dt;

        let mut y = vec![0.0; 2 * n];
        y[..n].copy_from_slice(&self.theta);
        y[n..].copy_from_slice(&self.omega);

        let k1 = self.deriv(&y, tau);
        let y2: Vec<f64> = y.iter().zip(&k1).map(|(a, b)| a + 0.5 * dt * b).collect();
        let k2 = self.deriv(&y2, tau);
        let y3: Vec<f64> = y.iter().zip(&k2).map(|(a, b)| a + 0.5 * dt * b).collect();
        let k3 = self.deriv(&y3, tau);
        let y4: Vec<f64> = y.iter().zip(&k3).map(|(a, b)| a + dt * b).collect();
        let k4 = self.deriv(&y4, tau);

        for idx in 0..2 * n {
            y[idx] += dt / 6.0 * (k1[idx] + 2.0 * k2[idx] + 2.0 * k3[idx] + k4[idx]);
        }
        self.theta.copy_from_slice(&y[..n]);
        self.omega.copy_from_slice(&y[n..]);
        self.t += dt;
    }

    /// Cartesian joint positions including the anchor: `(n+1)` points.
    pub fn link_positions(&self) -> Vec<(f64, f64)> {
        let mut pts = Vec::with_capacity(self.n + 1);
        pts.push((0.0, 0.0));
        let (mut x, mut y) = (0.0, 0.0);
        for i in 0..self.n {
            x += self.l[i] * self.theta[i].sin();
            y -= self.l[i] * self.theta[i].cos();
            pts.push((x, y));
        }
        pts
    }

    /// Total mechanical energy KE + PE — ~constant for the passive system.
    pub fn total_energy(&self) -> f64 {
        let n = self.n;
        let mut ke = 0.0;
        for i in 0..n {
            for j in 0..n {
                let d = self.theta[i] - self.theta[j];
                let m_ij = self.mu[i][j] * self.l[i] * self.l[j] * d.cos();
                ke += 0.5 * m_ij * self.omega[i] * self.omega[j];
            }
        }
        let mut pe = 0.0;
        for i in 0..n {
            pe -= self.g * self.cum_tail[i] * self.l[i] * self.theta[i].cos();
        }
        ke + pe
    }
}

/// Dense Gaussian elimination with partial pivoting. Fine for the small `n`
/// (2–6) we use; avoids a linear-algebra dependency entirely.
fn gauss_solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Vec<f64> {
    let n = b.len();
    for col in 0..n {
        // Partial pivot: swap in the row with the largest magnitude in `col`.
        let mut pivot = col;
        for r in (col + 1)..n {
            if a[r][col].abs() > a[pivot][col].abs() {
                pivot = r;
            }
        }
        a.swap(col, pivot);
        b.swap(col, pivot);

        let diag = a[col][col];
        for r in (col + 1)..n {
            let factor = a[r][col] / diag;
            for c in col..n {
                a[r][c] -= factor * a[col][c];
            }
            b[r] -= factor * b[col];
        }
    }
    // Back-substitution.
    let mut x = vec![0.0; n];
    for col in (0..n).rev() {
        let mut s = b[col];
        for c in (col + 1)..n {
            s -= a[col][c] * x[c];
        }
        x[col] = s / a[col][col];
    }
    x
}
