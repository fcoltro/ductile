// SPDX-License-Identifier: GPL-3.0-only
//! The native `.ductile` document format: JSON with exact B-rep geometry.

use ductile_brep::Body;
use serde::{Deserialize, Serialize};

pub const FORMAT_VERSION: u32 = 1;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NativeObject {
    pub name: String,
    pub body: Body,
    #[serde(default)]
    pub color: Option<[f32; 3]>,
    #[serde(default = "yes")]
    pub visible: bool,
    #[serde(default)]
    pub locked: bool,
    /// Id of its group in the document's `groups`.
    #[serde(default)]
    pub group: Option<u64>,
}

/// An Outliner group.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NativeGroup {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub parent: Option<u64>,
    #[serde(default = "yes")]
    pub visible: bool,
    #[serde(default)]
    pub locked: bool,
}

fn yes() -> bool {
    true
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NativeDocument {
    pub format: String,
    pub version: u32,
    pub objects: Vec<NativeObject>,
    #[serde(default)]
    pub groups: Vec<NativeGroup>,
}

impl NativeDocument {
    pub fn new(objects: Vec<NativeObject>) -> Self {
        Self { format: "ductile".into(), version: FORMAT_VERSION, objects, groups: vec![] }
    }

    pub fn to_json(&self) -> crate::Result<String> {
        Ok(serde_json::to_string(self)?)
    }

    pub fn from_json(s: &str) -> crate::Result<Self> {
        let d: NativeDocument = serde_json::from_str(s)?;
        if d.format != "ductile" {
            return Err(crate::IoError::Parse("not a Ductile document".into()));
        }
        Ok(d)
    }
}
