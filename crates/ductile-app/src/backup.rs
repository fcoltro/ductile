// SPDX-License-Identifier: GPL-3.0-only
//! Backups of saved files and crash detection.
//!
//! Before a save overwrites a file, the old copy goes to a backup folder
//! (`<name>.backups` next to the file unless Preferences name another
//! folder) as `<name>-YYYYMMDD-HHMMSS.ductile`. The folder is then pruned:
//! the newest few stay, plus the newest of each of the last hours, days and
//! weeks the policy asks for.
//!
//! A marker file in the state directory says Ductile is running; finding it
//! at startup means the last run did not close cleanly, and the autosave
//! is offered back.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::time::UNIX_EPOCH;

/// How many backups to keep, as in Preferences.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Policy {
    pub recent: usize,
    pub hourly: usize,
    pub daily: usize,
    pub weekly: usize,
}

impl Default for Policy {
    fn default() -> Self {
        Policy { recent: 3, hourly: 2, daily: 2, weekly: 1 }
    }
}

/// Which of the backups taken at `times` (seconds since the epoch, any
/// order) to keep, as indices into `times`.
pub fn keep(times: &[u64], p: &Policy) -> BTreeSet<usize> {
    let mut order: Vec<usize> = (0..times.len()).collect();
    order.sort_by(|a, b| times[*b].cmp(&times[*a]));
    let mut out: BTreeSet<usize> = order.iter().take(p.recent).copied().collect();
    for (period, count) in [(3600, p.hourly), (86_400, p.daily), (7 * 86_400, p.weekly)] {
        let mut buckets = BTreeSet::new();
        for &i in &order {
            if buckets.len() >= count {
                break;
            }
            // The newest backup in each period stands for it.
            if buckets.insert(times[i] / period) {
                out.insert(i);
            }
        }
    }
    out
}

/// `YYYYMMDD-HHMMSS` in UTC.
fn stamp(secs: u64) -> String {
    let days = (secs / 86_400) as i64;
    let rem = secs % 86_400;
    // Civil date from days since 1970-01-01 (Howard Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!("{y:04}{m:02}{d:02}-{:02}{:02}{:02}", rem / 3600, rem % 3600 / 60, rem % 60)
}

fn modified_secs(p: &Path) -> Option<u64> {
    p.metadata().ok()?.modified().ok()?.duration_since(UNIX_EPOCH).ok().map(|d| d.as_secs())
}

/// The backup folder for `file`.
pub fn folder(file: &Path, custom: &str) -> PathBuf {
    let stem = file.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "untitled".into());
    if custom.trim().is_empty() {
        file.with_file_name(format!("{stem}.backups"))
    } else {
        PathBuf::from(custom.trim()).join(stem)
    }
}

/// Copies `file` (about to be overwritten) into its backup folder and
/// prunes the folder. Does nothing when there is no old copy.
pub fn back_up(file: &Path, custom: &str, p: &Policy) -> std::io::Result<()> {
    let Some(secs) = modified_secs(file) else { return Ok(()) };
    let dir = folder(file, custom);
    std::fs::create_dir_all(&dir)?;
    let stem = file.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
    let ext = file.extension().map(|e| e.to_string_lossy().into_owned()).unwrap_or_else(|| "ductile".into());
    std::fs::copy(file, dir.join(format!("{stem}-{}.{ext}", stamp(secs))))?;
    let prefix = format!("{stem}-");
    let files: Vec<(PathBuf, u64)> = std::fs::read_dir(&dir)?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.file_name().is_some_and(|n| n.to_string_lossy().starts_with(&prefix)))
        .filter_map(|p| modified_secs(&p).map(|t| (p, t)))
        .collect();
    let times: Vec<u64> = files.iter().map(|f| f.1).collect();
    let kept = keep(&times, p);
    for (i, (path, _)) in files.iter().enumerate() {
        if !kept.contains(&i) {
            std::fs::remove_file(path)?;
        }
    }
    Ok(())
}

fn marker() -> Option<PathBuf> {
    crate::recent::autosave_path().map(|p| p.with_file_name("running"))
}

/// Marks Ductile as running; true when the previous run never cleared its
/// marker (it crashed or was killed).
pub fn start_session() -> bool {
    let Some(m) = marker() else { return false };
    let crashed = m.exists();
    if let Some(dir) = m.parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    let _ = std::fs::write(&m, std::process::id().to_string());
    crashed
}

/// Clears the running marker on a clean exit.
pub fn end_session() {
    if let Some(m) = marker() {
        let _ = std::fs::remove_file(m);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keeps_recent_and_one_per_period() {
        let h = 3600;
        // Ten backups ten minutes apart, then one a day earlier and one a
        // week earlier.
        let now = 100 * 7 * 86_400 + 12 * h;
        let mut times: Vec<u64> = (0..10).map(|i| now - i * 600).collect();
        times.push(now - 86_400 - h);
        times.push(now - 8 * 86_400);
        let p = Policy { recent: 2, hourly: 2, daily: 2, weekly: 3 };
        let k = keep(&times, &p);
        assert!(k.contains(&0) && k.contains(&1), "the newest two stay");
        assert!(k.contains(&10), "yesterday's stands for its day");
        assert!(k.contains(&11), "last week's stands for its week");
        assert!(k.len() < times.len());
        assert_eq!(keep(&times, &Policy { recent: 0, hourly: 0, daily: 0, weekly: 0 }).len(), 0);
    }

    #[test]
    fn backs_up_the_old_copy_next_to_the_file() {
        let dir = std::env::temp_dir().join(format!("ductile-backup-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("part.ductile");
        back_up(&file, "", &Policy::default()).unwrap();
        assert!(!folder(&file, "").exists(), "nothing to keep before the first save");
        std::fs::write(&file, "old").unwrap();
        back_up(&file, "", &Policy::default()).unwrap();
        let kept: Vec<_> = std::fs::read_dir(folder(&file, "")).unwrap().filter_map(|e| e.ok()).collect();
        assert_eq!(kept.len(), 1);
        assert_eq!(std::fs::read_to_string(kept[0].path()).unwrap(), "old");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn stamps_civil_dates() {
        assert_eq!(stamp(0), "19700101-000000");
        assert_eq!(stamp(951_782_400 + 3661), "20000229-010101");
    }
}
