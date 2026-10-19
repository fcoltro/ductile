// SPDX-License-Identifier: GPL-3.0-only
//! Display units. The model is always in millimetres; the unit chosen in
//! Preferences only changes how lengths are shown and how a bare typed
//! number is read.

use std::cell::Cell;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unit {
    Millimetre,
    Centimetre,
    Metre,
    Inch,
    Foot,
}

impl Unit {
    pub const ALL: [Unit; 5] = [Unit::Millimetre, Unit::Centimetre, Unit::Metre, Unit::Inch, Unit::Foot];

    /// Millimetres in one of this unit.
    pub fn mm(self) -> f64 {
        match self {
            Unit::Millimetre => 1.0,
            Unit::Centimetre => 10.0,
            Unit::Metre => 1000.0,
            Unit::Inch => 25.4,
            Unit::Foot => 304.8,
        }
    }

    pub fn suffix(self) -> &'static str {
        match self {
            Unit::Millimetre => "mm",
            Unit::Centimetre => "cm",
            Unit::Metre => "m",
            Unit::Inch => "in",
            Unit::Foot => "ft",
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Unit::Millimetre => "Millimetres",
            Unit::Centimetre => "Centimetres",
            Unit::Metre => "Metres",
            Unit::Inch => "Inches",
            Unit::Foot => "Feet",
        }
    }

    pub fn from_suffix(s: &str) -> Option<Unit> {
        Unit::ALL.into_iter().find(|u| u.suffix() == s)
    }
}

thread_local! {
    // The UI thread sets it each frame from the preferences.
    static CURRENT: Cell<Unit> = const { Cell::new(Unit::Millimetre) };
}

pub fn current() -> Unit {
    CURRENT.with(Cell::get)
}

pub fn set(u: Unit) {
    CURRENT.with(|c| c.set(u));
}

/// A length in millimetres as a number in the display unit, trimmed.
pub fn num(mm: f64) -> String {
    let s = format!("{:.4}", mm / current().mm());
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.to_string() }
}

/// A length with its unit, as `12.5 mm`.
pub fn len(mm: f64) -> String {
    format!("{} {}", num(mm), current().suffix())
}

/// An area in square millimetres, in the display unit squared.
pub fn area(mm2: f64) -> String {
    let k = current().mm();
    let s = format!("{:.4}", mm2 / (k * k));
    format!("{} {}²", s.trim_end_matches('0').trim_end_matches('.'), current().suffix())
}

/// A volume in cubic millimetres, in the display unit cubed.
pub fn volume(mm3: f64) -> String {
    let k = current().mm();
    let s = format!("{:.4}", mm3 / (k * k * k));
    format!("{} {}³", s.trim_end_matches('0').trim_end_matches('.'), current().suffix())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shows_lengths_in_the_chosen_unit() {
        set(Unit::Centimetre);
        assert_eq!(len(25.0), "2.5 cm");
        assert_eq!(area(200.0), "2 cm²");
        set(Unit::Inch);
        assert_eq!(num(50.8), "2");
        set(Unit::Millimetre);
        assert_eq!(len(3.0), "3 mm");
    }
}
