// SPDX-License-Identifier: GPL-3.0-only
//! User preferences, kept as `key = value` lines in
//! `~/.config/ductile/prefs.txt`.

use std::path::PathBuf;

/// How mouse buttons and modifiers drive the camera.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NavPreset {
    /// Right or middle drag orbits, with Shift pans, with Ctrl zooms.
    Plasticity,
    /// Middle drag orbits, Shift+middle pans, Ctrl+middle zooms.
    Blender,
    /// Alt+left orbits, Alt+middle pans, Alt+right zooms.
    Maya,
    /// Right drag orbits, Shift+right pans, Ctrl+right zooms.
    Rhino,
}

impl NavPreset {
    pub const ALL: [NavPreset; 4] = [NavPreset::Plasticity, NavPreset::Blender, NavPreset::Maya, NavPreset::Rhino];

    pub fn label(self) -> &'static str {
        match self {
            NavPreset::Plasticity => "Plasticity",
            NavPreset::Blender => "Blender",
            NavPreset::Maya => "Maya",
            NavPreset::Rhino => "Rhino",
        }
    }
}

/// What a viewport drag does to the camera.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Nav {
    Orbit,
    Pan,
    Zoom,
}

impl NavPreset {
    /// The camera move for a drag with these buttons held, if any.
    pub fn drag(self, left: bool, middle: bool, right: bool, m: egui::Modifiers) -> Option<Nav> {
        let with_mods = |down: bool| {
            down.then(|| if m.shift { Nav::Pan } else if m.ctrl { Nav::Zoom } else { Nav::Orbit })
        };
        match self {
            NavPreset::Plasticity => with_mods(middle || right),
            NavPreset::Blender => with_mods(middle),
            NavPreset::Rhino => with_mods(right),
            NavPreset::Maya if m.alt => {
                if left {
                    Some(Nav::Orbit)
                } else if middle {
                    Some(Nav::Pan)
                } else if right {
                    Some(Nav::Zoom)
                } else {
                    None
                }
            }
            NavPreset::Maya => None,
        }
    }

    /// Whether a left drag belongs to the camera (so it neither box-selects
    /// nor drives a tool).
    pub fn takes_left(self, m: egui::Modifiers) -> bool {
        self == NavPreset::Maya && m.alt
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Prefs {
    /// Selecting a face starts Push face, an edge Fillet.
    pub auto_invoke: bool,
    /// In object mode a second click on a selected solid picks the face or
    /// edge under the cursor.
    pub drill_down: bool,
    pub nav: NavPreset,
    /// Wheel zoom goes toward the point under the cursor.
    pub zoom_to_cursor: bool,
    /// Minutes between autosaves of an unsaved document; 0 turns it off.
    pub autosave_minutes: u32,
    /// Open the autosaved session on startup when no file is given.
    pub load_last_session: bool,
}

impl Default for Prefs {
    fn default() -> Self {
        Prefs { auto_invoke: true, drill_down: true, nav: NavPreset::Plasticity, zoom_to_cursor: true, autosave_minutes: 2, load_last_session: false }
    }
}

fn path() -> Option<PathBuf> {
    let base = std::env::var_os("XDG_CONFIG_HOME").map(PathBuf::from).or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
    Some(base.join("ductile").join("prefs.txt"))
}

impl Prefs {
    pub fn parse(text: &str) -> Prefs {
        let mut p = Prefs::default();
        for line in text.lines() {
            let Some((k, v)) = line.split_once('=') else { continue };
            let (k, v) = (k.trim(), v.trim());
            let flag = v == "true";
            match k {
                "auto_invoke" => p.auto_invoke = flag,
                "drill_down" => p.drill_down = flag,
                "zoom_to_cursor" => p.zoom_to_cursor = flag,
                "load_last_session" => p.load_last_session = flag,
                "autosave_minutes" => p.autosave_minutes = v.parse().unwrap_or(p.autosave_minutes),
                "nav" => p.nav = NavPreset::ALL.into_iter().find(|n| n.label().eq_ignore_ascii_case(v)).unwrap_or(p.nav),
                _ => {}
            }
        }
        p
    }

    pub fn text(&self) -> String {
        format!(
            "auto_invoke = {}\ndrill_down = {}\nnav = {}\nzoom_to_cursor = {}\nautosave_minutes = {}\nload_last_session = {}\n",
            self.auto_invoke,
            self.drill_down,
            self.nav.label(),
            self.zoom_to_cursor,
            self.autosave_minutes,
            self.load_last_session
        )
    }

    pub fn load() -> Prefs {
        path().and_then(|p| std::fs::read_to_string(p).ok()).map(|t| Prefs::parse(&t)).unwrap_or_default()
    }

    pub fn save(&self) {
        if let Some(p) = path() {
            if let Some(dir) = p.parent() {
                let _ = std::fs::create_dir_all(dir);
            }
            let _ = std::fs::write(p, self.text());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_and_maps_drags() {
        let p = Prefs { auto_invoke: false, nav: NavPreset::Maya, autosave_minutes: 5, ..Prefs::default() };
        assert_eq!(Prefs::parse(&p.text()), p);
        let none = egui::Modifiers::NONE;
        assert_eq!(NavPreset::Plasticity.drag(false, false, true, none), Some(Nav::Orbit));
        assert_eq!(NavPreset::Blender.drag(false, false, true, none), None);
        assert_eq!(NavPreset::Maya.drag(true, false, false, egui::Modifiers::ALT), Some(Nav::Orbit));
        assert_eq!(NavPreset::Rhino.drag(false, false, true, egui::Modifiers::SHIFT), Some(Nav::Pan));
    }
}
