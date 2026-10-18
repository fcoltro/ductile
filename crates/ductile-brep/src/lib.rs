// SPDX-License-Identifier: GPL-3.0-only
//! Topology and modeling operations of the Ductile kernel.

pub mod ops;
pub mod primitives;
pub mod curves;
pub mod profile;
pub mod ssi;
pub mod tess;
pub mod topo;

pub use tess::{tessellate, BodyMesh, TessOptions};
pub use topo::{Body, BodyKind, Coedge, EdgeId, FaceId, LoopId, VertexId};
