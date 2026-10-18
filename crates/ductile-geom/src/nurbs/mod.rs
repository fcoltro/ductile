// SPDX-License-Identifier: GPL-3.0-only
//! Non-uniform rational B-splines.

pub mod basis;
pub mod curve;
pub mod surface;

pub use curve::NurbsCurve;
pub use surface::{make_compatible, NurbsSurface};
