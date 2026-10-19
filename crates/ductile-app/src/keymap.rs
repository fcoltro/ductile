// SPDX-License-Identifier: GPL-3.0-only
//! Keyboard shortcuts. The defaults follow Plasticity's keymap, so its
//! users feel at home; any binding can be changed from the command palette
//! (right-click a command), and changes are kept in
//! `~/.config/ductile/keymap.txt` as lines like `Fillet = B, Shift+F`
//! (`Fillet =` unbinds it).

use crate::commands::Command;
use egui::{Key, Modifiers};
use std::path::PathBuf;
use std::sync::{LazyLock, RwLock};

/// A key with the modifiers held for it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Chord {
    pub key: Key,
    pub ctrl: bool,
    pub shift: bool,
    pub alt: bool,
}

impl Chord {
    pub fn new(key: Key, m: Modifiers) -> Chord {
        Chord { key, ctrl: m.ctrl || m.command, shift: m.shift, alt: m.alt }
    }

    /// Parses `Ctrl+Shift+S`, `Alt+.`, `B` and the like.
    pub fn parse(s: &str) -> Option<Chord> {
        let s = s.trim();
        // The key itself may be `+`, so split off modifiers from the left.
        let mut c = Chord { key: Key::A, ctrl: false, shift: false, alt: false };
        let mut rest = s;
        loop {
            let lower = rest.to_ascii_lowercase();
            let take = |p: &str| lower.starts_with(p) && rest.len() > p.len();
            if take("ctrl+") {
                c.ctrl = true;
                rest = &rest[5..];
            } else if take("shift+") {
                c.shift = true;
                rest = &rest[6..];
            } else if take("alt+") {
                c.alt = true;
                rest = &rest[4..];
            } else {
                break;
            }
        }
        c.key = Key::from_name(rest.trim())?;
        Some(c)
    }

    pub fn text(&self) -> String {
        let mut s = String::new();
        for (on, name) in [(self.ctrl, "Ctrl+"), (self.alt, "Alt+"), (self.shift, "Shift+")] {
            if on {
                s.push_str(name);
            }
        }
        s.push_str(match self.key {
            Key::Minus => "-",
            k => k.symbol_or_name(),
        });
        s
    }
}

/// Plasticity's default bindings, plus Ductile's own for commands
/// Plasticity has no key for, where the key is free.
const DEFAULTS: &[(Command, &str)] = {
    use Command::*;
    &[
        (Palette, "F"),
        (New, "Ctrl+N"),
        (Open, "Ctrl+O"),
        (Save, "Ctrl+S"),
        (SaveAs, "Ctrl+Shift+S"),
        (ImportStep, "Ctrl+Shift+O"),
        (ExportStep, "Ctrl+Shift+E"),
        (Undo, "Ctrl+Z"),
        (Redo, "Ctrl+Shift+Z"),
        (Redo, "Ctrl+Y"),
        (Repeat, "Shift+R"),
        (Delete, "X"),
        (Delete, "Delete"),
        (Delete, "Backspace"),
        (Delete, "Shift+X"),
        (Duplicate, "Shift+D"),
        (Instance, "Alt+D"),
        (SlideCv, "Shift+G"),
        (SelectPrevious, "Ctrl+T"),
        (Place, "Ctrl+D"),
        (SelectAll, "A"),
        (SelectAll, "Ctrl+A"),
        (InvertSelection, "Alt+A"),
        (DeselectAll, "Escape"),
        (Hide, "H"),
        (Isolate, "Shift+H"),
        (Isolate, "."),
        (UnhideAll, "Alt+H"),
        (ModeVertex, "1"),
        (ModeEdge, "2"),
        (ModeFace, "3"),
        (ModeObject, "4"),
        (Curve, "Shift+A"),
        (Extrude, "E"),
        (Revolve, "V"),
        (Loft, "L"),
        (Sweep, "Shift+P"),
        (Pipe, "P"),
        (Boolean, "Q"),
        (Cut, "C"),
        (Fillet, "B"),
        (Chamfer, "Shift+K"),
        (Offset, "O"),
        (Shell, "Alt+S"),
        (JoinCurves, "J"),
        (Trim, "T"),
        (ProjectCurve, "I"),
        (Imprint, "Shift+I"),
        (Patch, "Alt+P"),
        (Thicken, "Alt+K"),
        (BridgeCurve, "Alt+B"),
        (Move, "G"),
        (Rotate, "R"),
        (Scale, "S"),
        (Mirror, "Alt+X"),
        (FrameSelection, "/"),
        (SetCPlane, "Space"),
        (SetCPlaneOnly, "Shift+Space"),
        (CPlaneView, "Ctrl+Space"),
        (ToggleXray, "Alt+Z"),
        (ToggleZebra, "Alt+R"),
        (Help, "F1"),
        (Preferences, "Ctrl+,"),
        (Group, "Ctrl+G"),
        (Isoparam, "Ctrl+R"),
        (Untrim, "Alt+T"),
        (Unjoin, "Alt+J"),
        (Ungroup, "Alt+G"),
    ]
};

pub struct Keymap {
    binds: Vec<(Command, Chord)>,
}

static KEYMAP: LazyLock<RwLock<Keymap>> = LazyLock::new(|| RwLock::new(Keymap::load()));

/// The keymap in use.
pub fn keymap() -> std::sync::RwLockReadGuard<'static, Keymap> {
    KEYMAP.read().unwrap_or_else(|e| e.into_inner())
}

pub fn keymap_mut() -> std::sync::RwLockWriteGuard<'static, Keymap> {
    KEYMAP.write().unwrap_or_else(|e| e.into_inner())
}

fn config_path() -> Option<PathBuf> {
    let base = std::env::var_os("XDG_CONFIG_HOME").map(PathBuf::from).or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
    Some(base.join("ductile").join("keymap.txt"))
}

impl Keymap {
    pub fn defaults() -> Keymap {
        Keymap { binds: DEFAULTS.iter().filter_map(|(c, s)| Chord::parse(s).map(|k| (*c, k))).collect() }
    }

    /// The defaults with the user's changes applied.
    pub fn load() -> Keymap {
        let mut k = Keymap::defaults();
        if let Some(text) = config_path().and_then(|p| std::fs::read_to_string(p).ok()) {
            k.apply(&text);
        }
        k
    }

    /// Applies `Command = Chord, Chord` lines; each names all of that
    /// command's keys, and takes them from any other command.
    pub fn apply(&mut self, text: &str) {
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let Some((name, keys)) = line.split_once('=') else { continue };
            let Some(cmd) = Command::from_id(name.trim()) else { continue };
            let chords: Vec<Chord> = keys.split(',').filter(|s| !s.trim().is_empty()).filter_map(Chord::parse).collect();
            self.binds.retain(|(c, k)| *c != cmd && !chords.contains(k));
            self.binds.extend(chords.into_iter().map(|k| (cmd, k)));
        }
    }

    pub fn lookup(&self, key: Key, m: Modifiers) -> Option<Command> {
        let chord = Chord::new(key, m);
        self.binds.iter().find(|(_, k)| *k == chord).map(|(c, _)| *c)
    }

    pub fn chords(&self, cmd: Command) -> Vec<Chord> {
        self.binds.iter().filter(|(c, _)| *c == cmd).map(|(_, k)| *k).collect()
    }

    pub fn text(&self, cmd: Command) -> String {
        self.chords(cmd).iter().map(Chord::text).collect::<Vec<_>>().join(" / ")
    }

    /// Makes `chord` the only key of `cmd`, taking it from any other
    /// command, and saves the change.
    pub fn assign(&mut self, cmd: Command, chord: Chord) -> Option<Command> {
        let taken = self.binds.iter().find(|(c, k)| *k == chord && *c != cmd).map(|(c, _)| *c);
        self.binds.retain(|(c, k)| *c != cmd && *k != chord);
        self.binds.push((cmd, chord));
        self.save();
        taken
    }

    /// Puts `cmd` back on its default keys.
    pub fn reset(&mut self, cmd: Command) {
        let defaults: Vec<Chord> = Keymap::defaults().chords(cmd);
        self.binds.retain(|(c, k)| *c != cmd && !defaults.contains(k));
        self.binds.extend(defaults.into_iter().map(|k| (cmd, k)));
        self.save();
    }

    /// The lines that turn the defaults into this keymap.
    pub fn changes(&self) -> String {
        let defaults = Keymap::defaults();
        let mut out = String::new();
        for &cmd in Command::ALL {
            let (mine, theirs) = (self.chords(cmd), defaults.chords(cmd));
            if mine.len() != theirs.len() || mine.iter().any(|k| !theirs.contains(k)) {
                out.push_str(&format!("{} = {}\n", cmd.id(), mine.iter().map(Chord::text).collect::<Vec<_>>().join(", ")));
            }
        }
        out
    }

    fn save(&self) {
        let Some(p) = config_path() else { return };
        if let Some(dir) = p.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        let _ = std::fs::write(p, format!("# Ductile shortcuts that differ from the defaults.\n{}", self.changes()));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_parse_and_are_unique() {
        let k = Keymap::defaults();
        assert_eq!(k.binds.len(), DEFAULTS.len());
        for (i, (_, a)) in k.binds.iter().enumerate() {
            assert!(!k.binds[i + 1..].iter().any(|(_, b)| b == a), "{} bound twice", a.text());
        }
        assert_eq!(k.lookup(Key::B, Modifiers::NONE), Some(Command::Fillet));
        assert_eq!(k.lookup(Key::Period, Modifiers::NONE), Some(Command::Isolate));
        assert_eq!(k.lookup(Key::D, Modifiers::SHIFT), Some(Command::Duplicate));
    }

    #[test]
    fn changes_round_trip_through_text() {
        let mut k = Keymap::defaults();
        k.apply("Box = B\nPalette = Space, Ctrl+P\nPipe =\n");
        assert_eq!(k.lookup(Key::B, Modifiers::NONE), Some(Command::Box));
        assert_eq!(k.lookup(Key::Space, Modifiers::NONE), Some(Command::Palette));
        assert_eq!(k.lookup(Key::P, Modifiers::NONE), None);
        assert!(k.chords(Command::Fillet).is_empty());
        let mut again = Keymap::defaults();
        again.apply(&k.changes());
        for &c in Command::ALL {
            assert_eq!(again.text(c), k.text(c), "{c:?}");
        }
        assert_eq!(Chord::parse("Ctrl+Shift+S").unwrap().text(), "Ctrl+Shift+S");
        assert_eq!(Chord::parse("Alt+.").unwrap().key, Key::Period);
    }
}

#[cfg(test)]
mod prefs_key_tests {
    use super::*;
    #[test]
    fn ctrl_comma_opens_preferences() {
        let k = Keymap::defaults();
        assert_eq!(k.lookup(Key::Comma, egui::Modifiers { ctrl: true, command: true, ..Default::default() }), Some(Command::Preferences));
    }
}
