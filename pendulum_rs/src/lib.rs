//! pendulum_rs library: shared physics + control used by the binaries.
//!
//! - [`simulator`] — n-link pendulum dynamics (RK4).
//! - [`control`] — linearization + in-Rust discrete LQR + balance controller
//!   for the underactuated arm (only joint 0 driven).
//! - [`estimator`] — Phase 2: identify the live dynamics signature from motion
//!   (the offline/online halves of replacing the oracle with a RuVector recall).
//! - [`memory`] — Phase 2 (behind `vectordb`): seed/recall arm configs in
//!   RuVector keyed by their dynamics signature.

pub mod control;
pub mod estimator;
pub mod learn;
pub mod mpc;
pub mod simulator;

#[cfg(feature = "vectordb")]
pub mod memory;
