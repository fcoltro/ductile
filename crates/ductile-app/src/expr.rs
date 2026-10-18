// SPDX-License-Identifier: GPL-3.0-only
//! Typed values: arithmetic with units, as in Plasticity's numeric input
//! (`10+2.5`, `3*4mm`, `(1in+2)/2`, `90deg`, `pi/4rad`). Lengths are in
//! millimetres and angles in degrees unless a unit says otherwise.

/// The value of `s`, or None if it doesn't parse.
pub fn eval(s: &str) -> Option<f64> {
    let mut p = Parser { s: s.as_bytes(), i: 0 };
    let v = p.sum()?;
    p.space();
    (p.i == p.s.len() && v.is_finite()).then_some(v)
}

struct Parser<'a> {
    s: &'a [u8],
    i: usize,
}

impl Parser<'_> {
    fn space(&mut self) {
        while self.s.get(self.i).is_some_and(|c| c.is_ascii_whitespace()) {
            self.i += 1;
        }
    }

    fn eat(&mut self, c: u8) -> bool {
        self.space();
        let hit = self.s.get(self.i) == Some(&c);
        if hit {
            self.i += 1;
        }
        hit
    }

    fn sum(&mut self) -> Option<f64> {
        let mut v = self.product()?;
        loop {
            if self.eat(b'+') {
                v += self.product()?;
            } else if self.eat(b'-') {
                v -= self.product()?;
            } else {
                return Some(v);
            }
        }
    }

    fn product(&mut self) -> Option<f64> {
        let mut v = self.power()?;
        loop {
            if self.eat(b'*') || self.eat(b'x') {
                v *= self.power()?;
            } else if self.eat(b'/') {
                v /= self.power()?;
            } else {
                return Some(v);
            }
        }
    }

    fn power(&mut self) -> Option<f64> {
        let b = self.unary()?;
        if self.eat(b'^') {
            return Some(b.powf(self.power()?));
        }
        Some(b)
    }

    fn unary(&mut self) -> Option<f64> {
        if self.eat(b'-') {
            return Some(-self.unary()?);
        }
        if self.eat(b'+') {
            return self.unary();
        }
        let v = self.primary()?;
        Some(v * self.unit())
    }

    fn primary(&mut self) -> Option<f64> {
        if self.eat(b'(') {
            let v = self.sum()?;
            return self.eat(b')').then_some(v);
        }
        self.space();
        let start = self.i;
        while self.s.get(self.i).is_some_and(|c| c.is_ascii_digit() || *c == b'.') {
            self.i += 1;
        }
        if self.i > start {
            return std::str::from_utf8(&self.s[start..self.i]).ok()?.parse().ok();
        }
        if self.word("pi") {
            return Some(std::f64::consts::PI);
        }
        None
    }

    fn word(&mut self, w: &str) -> bool {
        self.space();
        let hit = self.s[self.i..].starts_with(w.as_bytes()) && !self.s.get(self.i + w.len()).is_some_and(|c| c.is_ascii_alphabetic());
        if hit {
            self.i += w.len();
        }
        hit
    }

    /// The factor of a unit following a number, 1 when none does.
    fn unit(&mut self) -> f64 {
        // Longest names first, so `mm` isn't read as `m`.
        const UNITS: &[(&str, f64)] = &[
            ("mm", 1.0),
            ("cm", 10.0),
            ("m", 1000.0),
            ("in", 25.4),
            ("\"", 25.4),
            ("ft", 304.8),
            ("'", 304.8),
            ("deg", 1.0),
            ("°", 1.0),
            ("rad", 180.0 / std::f64::consts::PI),
        ];
        for (name, f) in UNITS {
            if self.word(name) {
                return *f;
            }
        }
        1.0
    }
}

#[cfg(test)]
mod tests {
    use super::eval;

    #[test]
    fn arithmetic_and_units() {
        let near = |s: &str, v: f64| assert!((eval(s).unwrap() - v).abs() < 1e-9, "{s} = {:?}", eval(s));
        near("12", 12.0);
        near("-2.5", -2.5);
        near("10+2*3", 16.0);
        near("(10+2)*3", 36.0);
        near("2^3^2", 512.0);
        near("1in", 25.4);
        near("1in + 2mm", 27.4);
        near("3 x 4cm", 120.0);
        near("1.5m/3", 500.0);
        near("pi rad", 180.0);
        near("90deg-45", 45.0);
        near("6'", 1828.8);
        assert_eq!(eval("2+"), None);
        assert_eq!(eval("(1"), None);
        assert_eq!(eval("1/0"), None);
        assert_eq!(eval("5 apples"), None);
    }
}
