// SPDX-License-Identifier: GPL-3.0-only
//! Recently opened files and the autosaved session, kept under the XDG
//! config and state directories.

use std::path::{Path, PathBuf};

const KEEP: usize = 10;

fn home_dir(var: &str, fallback: &str) -> Option<PathBuf> {
    std::env::var_os(var).map(PathBuf::from).or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(fallback)))
}

fn recent_file() -> Option<PathBuf> {
    Some(home_dir("XDG_CONFIG_HOME", ".config")?.join("ductile").join("recent.txt"))
}

/// Where the open document is saved every little while.
pub fn autosave_path() -> Option<PathBuf> {
    Some(home_dir("XDG_STATE_HOME", ".local/state")?.join("ductile").join("autosave.ductile"))
}

/// Recently opened or saved files, newest first, that still exist.
pub fn list() -> Vec<PathBuf> {
    let Some(text) = recent_file().and_then(|p| std::fs::read_to_string(p).ok()) else { return vec![] };
    text.lines().map(PathBuf::from).filter(|p| p.exists()).take(KEEP).collect()
}

/// Puts `path` at the top of the recent files.
pub fn push(path: &Path) {
    let Some(file) = recent_file() else { return };
    let path = path.canonicalize().unwrap_or_else(|_| path.to_path_buf());
    let mut all = vec![path.clone()];
    all.extend(list().into_iter().filter(|p| *p != path));
    all.truncate(KEEP);
    if let Some(dir) = file.parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    let text: String = all.iter().map(|p| format!("{}\n", p.display())).collect();
    let _ = std::fs::write(file, text);
}
