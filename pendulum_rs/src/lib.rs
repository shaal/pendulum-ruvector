//! pendulum_rs library: shared physics + control used by the binaries.
//!
//! - [`simulator`] — n-link pendulum dynamics (RK4).
//! - [`control`] — linearization + in-Rust discrete LQR + balance controller
//!   for the underactuated arm (only joint 0 driven).

pub mod control;
pub mod simulator;
