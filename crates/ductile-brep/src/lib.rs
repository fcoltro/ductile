// SPDX-License-Identifier: GPL-3.0-only
//! Topology and modeling operations of the Ductile kernel.

pub mod primitives;
pub mod tess;
pub mod topo;

pub use tess::{tessellate, BodyMesh, TessOptions};
pub use topo::{Body, BodyKind, Coedge, EdgeId, FaceId, LoopId, VertexId};
