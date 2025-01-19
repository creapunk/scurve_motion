//! # scurve_motion
//!
//! A small library for computing S-curve motion profiles in Rust.
//!
//! This library provides the following modules:
//! - `scurve` for managing an S-curve with given constraints and computing the profile.
//! - `scurve_task_executor` for storing and operating on the computed motion tasks.
//! - `motion_task` for describing a single motion segment in time.
//! - `buffer_fifo` for a simple FIFO buffer implementation.
//! - `motion_executor` for executing motion tasks in discrete ticks.
//!
//! Author: Anton Khrustalev, creapunk

pub mod scurve;
pub mod scurve_task;
pub mod motion_polynomial;
mod buffer_fifo;
pub mod motion_polynomial_executor;

// Re-export main structs for convenience:
pub use scurve::*;
pub use scurve_task::*;
pub use motion_polynomial::*;
pub use motion_polynomial_executor::*;
