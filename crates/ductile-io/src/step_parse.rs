// SPDX-License-Identifier: GPL-3.0-only
//! Reader for the ISO 10303-21 exchange structure ("STEP physical file"):
//! a map from instance numbers to typed parameter lists.

use std::collections::HashMap;

/// A parameter value.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Int(i64),
    Real(f64),
    Str(String),
    Enum(String),
    Ref(u64),
    List(Vec<Value>),
    /// Typed parameter such as `LENGTH_MEASURE(1.0)`.
    Typed(String, Vec<Value>),
    /// `$`
    Unset,
    /// `*`
    Derived,
}

impl Value {
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Real(x) => Some(*x),
            Value::Int(i) => Some(*i as f64),
            Value::Typed(_, v) if v.len() == 1 => v[0].as_f64(),
            _ => None,
        }
    }
    pub fn as_ref(&self) -> Option<u64> {
        match self {
            Value::Ref(r) => Some(*r),
            _ => None,
        }
    }
    pub fn as_list(&self) -> Option<&[Value]> {
        match self {
            Value::List(l) => Some(l),
            _ => None,
        }
    }
    pub fn as_enum(&self) -> Option<&str> {
        match self {
            Value::Enum(e) => Some(e),
            _ => None,
        }
    }
    pub fn as_bool(&self) -> Option<bool> {
        match self.as_enum()? {
            "T" => Some(true),
            "F" => Some(false),
            _ => None,
        }
    }
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::Str(s) => Some(s),
            _ => None,
        }
    }
}

/// One instance: a simple entity is a single (name, params) part; a complex
/// entity has several parts.
#[derive(Clone, Debug, PartialEq)]
pub struct Entity {
    pub parts: Vec<(String, Vec<Value>)>,
}

impl Entity {
    /// Name of a simple entity (the first part of a complex one).
    pub fn name(&self) -> &str {
        &self.parts[0].0
    }
    pub fn params(&self) -> &[Value] {
        &self.parts[0].1
    }
    pub fn part(&self, name: &str) -> Option<&[Value]> {
        self.parts.iter().find(|p| p.0 == name).map(|p| p.1.as_slice())
    }
    pub fn is_complex(&self) -> bool {
        self.parts.len() > 1
    }
}

pub struct StepFile {
    pub entities: HashMap<u64, Entity>,
    /// Instance numbers in file order.
    pub order: Vec<u64>,
}

struct Lexer<'a> {
    s: &'a [u8],
    i: usize,
}

impl<'a> Lexer<'a> {
    fn skip_ws(&mut self) {
        loop {
            while self.i < self.s.len() && self.s[self.i].is_ascii_whitespace() {
                self.i += 1;
            }
            if self.s[self.i..].starts_with(b"/*") {
                match find(&self.s[self.i + 2..], b"*/") {
                    Some(k) => self.i += k + 4,
                    None => self.i = self.s.len(),
                }
            } else {
                return;
            }
        }
    }
    fn peek(&mut self) -> Option<u8> {
        self.skip_ws();
        self.s.get(self.i).copied()
    }
    fn expect(&mut self, c: u8) -> Result<(), String> {
        if self.peek() == Some(c) {
            self.i += 1;
            Ok(())
        } else {
            Err(format!("expected '{}' at byte {}", c as char, self.i))
        }
    }
    fn keyword(&mut self) -> String {
        self.skip_ws();
        let st = self.i;
        while self.i < self.s.len() && (self.s[self.i].is_ascii_alphanumeric() || self.s[self.i] == b'_' || self.s[self.i] == b'-') {
            self.i += 1;
        }
        String::from_utf8_lossy(&self.s[st..self.i]).to_ascii_uppercase()
    }
    fn number(&mut self) -> Value {
        let st = self.i;
        if matches!(self.s[self.i], b'+' | b'-') {
            self.i += 1;
        }
        let mut real = false;
        while self.i < self.s.len() {
            let c = self.s[self.i];
            if c.is_ascii_digit() {
                self.i += 1;
            } else if c == b'.' || c == b'E' || c == b'e' {
                real = true;
                self.i += 1;
                if (c == b'E' || c == b'e') && self.i < self.s.len() && matches!(self.s[self.i], b'+' | b'-') {
                    self.i += 1;
                }
            } else {
                break;
            }
        }
        let t = std::str::from_utf8(&self.s[st..self.i]).unwrap_or("0");
        if real {
            // "1." and "1.E-07" are valid STEP reals; Rust wants a digit after '.'.
            let fixed = t.replace(".E", ".0E").replace(".e", ".0e");
            let fixed = if fixed.ends_with('.') { fixed + "0" } else { fixed };
            Value::Real(fixed.parse().unwrap_or(0.0))
        } else {
            Value::Int(t.parse().unwrap_or(0))
        }
    }
    fn string(&mut self) -> String {
        // Opening quote already consumed.
        let mut out = Vec::new();
        while self.i < self.s.len() {
            let c = self.s[self.i];
            self.i += 1;
            if c == b'\'' {
                if self.s.get(self.i) == Some(&b'\'') {
                    out.push(b'\'');
                    self.i += 1;
                } else {
                    break;
                }
            } else {
                out.push(c);
            }
        }
        decode_string(&String::from_utf8_lossy(&out))
    }
    fn value(&mut self) -> Result<Value, String> {
        match self.peek().ok_or("unexpected end")? {
            b'#' => {
                self.i += 1;
                let st = self.i;
                while self.i < self.s.len() && self.s[self.i].is_ascii_digit() {
                    self.i += 1;
                }
                Ok(Value::Ref(std::str::from_utf8(&self.s[st..self.i]).unwrap_or("0").parse().unwrap_or(0)))
            }
            b'\'' => {
                self.i += 1;
                Ok(Value::Str(self.string()))
            }
            b'.' => {
                self.i += 1;
                let k = self.keyword();
                self.expect(b'.')?;
                Ok(Value::Enum(k))
            }
            b'$' => {
                self.i += 1;
                Ok(Value::Unset)
            }
            b'*' => {
                self.i += 1;
                Ok(Value::Derived)
            }
            b'(' => Ok(Value::List(self.list()?)),
            b'"' => {
                // Binary: keep as a string.
                self.i += 1;
                let st = self.i;
                while self.i < self.s.len() && self.s[self.i] != b'"' {
                    self.i += 1;
                }
                let v = String::from_utf8_lossy(&self.s[st..self.i]).to_string();
                self.i += 1;
                Ok(Value::Str(v))
            }
            c if c.is_ascii_digit() || c == b'-' || c == b'+' => Ok(self.number()),
            c if c.is_ascii_alphabetic() => {
                let k = self.keyword();
                let args = self.list()?;
                Ok(Value::Typed(k, args))
            }
            c => Err(format!("unexpected '{}' at byte {}", c as char, self.i)),
        }
    }
    fn list(&mut self) -> Result<Vec<Value>, String> {
        self.expect(b'(')?;
        let mut out = Vec::new();
        if self.peek() == Some(b')') {
            self.i += 1;
            return Ok(out);
        }
        loop {
            out.push(self.value()?);
            match self.peek() {
                Some(b',') => self.i += 1,
                Some(b')') => {
                    self.i += 1;
                    return Ok(out);
                }
                _ => return Err(format!("expected ',' or ')' at byte {}", self.i)),
            }
        }
    }
}

fn find(h: &[u8], n: &[u8]) -> Option<usize> {
    h.windows(n.len()).position(|w| w == n)
}

/// Decodes the `\X2\...\X0\` (UCS-2 hex), `\X\hh` and `\S\c` escapes.
fn decode_string(s: &str) -> String {
    let mut out = String::new();
    let mut rest = s;
    while let Some(k) = rest.find('\\') {
        out.push_str(&rest[..k]);
        rest = &rest[k..];
        if let Some(r) = rest.strip_prefix("\\X2\\") {
            let end = r.find("\\X0\\").unwrap_or(r.len());
            let hex = &r[..end];
            for chunk in hex.as_bytes().chunks(4) {
                if let Some(c) = std::str::from_utf8(chunk).ok().and_then(|h| u32::from_str_radix(h, 16).ok()).and_then(char::from_u32) {
                    out.push(c);
                }
            }
            rest = r.get(end + 4..).unwrap_or("");
        } else if let Some(r) = rest.strip_prefix("\\X\\") {
            if let Some(c) = r.get(..2).and_then(|h| u8::from_str_radix(h, 16).ok()) {
                out.push(c as char);
            }
            rest = r.get(2..).unwrap_or("");
        } else if let Some(r) = rest.strip_prefix("\\S\\") {
            if let Some(c) = r.chars().next() {
                out.push(char::from_u32(c as u32 + 128).unwrap_or('?'));
                rest = &r[c.len_utf8()..];
            } else {
                rest = "";
            }
        } else if let Some(r) = rest.strip_prefix("\\\\") {
            out.push('\\');
            rest = r;
        } else {
            out.push('\\');
            rest = &rest[1..];
        }
    }
    out.push_str(rest);
    out
}

/// Parses the DATA section of a STEP file.
pub fn parse(text: &str) -> Result<StepFile, String> {
    let s = text.as_bytes();
    let start = find(s, b"DATA;").ok_or("no DATA section")? + 5;
    let mut lx = Lexer { s, i: start };
    let mut entities = HashMap::new();
    let mut order = Vec::new();
    loop {
        match lx.peek() {
            None => break,
            Some(b'#') => {
                lx.i += 1;
                let st = lx.i;
                while lx.i < s.len() && s[lx.i].is_ascii_digit() {
                    lx.i += 1;
                }
                let id: u64 = std::str::from_utf8(&s[st..lx.i]).unwrap_or("0").parse().map_err(|_| format!("bad instance number at byte {st}"))?;
                lx.expect(b'=')?;
                let mut parts = Vec::new();
                if lx.peek() == Some(b'(') {
                    lx.i += 1;
                    while lx.peek() != Some(b')') {
                        let name = lx.keyword();
                        if name.is_empty() {
                            return Err(format!("bad complex entity #{id}"));
                        }
                        parts.push((name, lx.list()?));
                    }
                    lx.i += 1;
                } else {
                    let name = lx.keyword();
                    parts.push((name, lx.list()?));
                }
                lx.expect(b';')?;
                entities.insert(id, Entity { parts });
                order.push(id);
            }
            Some(_) => {
                // ENDSEC or another section.
                let k = lx.keyword();
                if k == "ENDSEC" {
                    break;
                }
                if k.is_empty() {
                    lx.i += 1;
                }
                let _ = lx.expect(b';');
            }
        }
    }
    Ok(StepFile { entities, order })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_simple_and_complex_instances() {
        let f = parse(
            "ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('X'));\nENDSEC;\nDATA;\n\
             #1=CARTESIAN_POINT('p\\X2\\00E9\\X0\\',(1.,-2.5E-01,3));\n\
             /* comment */ #2=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
             #3=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#2,'it''s','');\n\
             ENDSEC;\nEND-ISO-10303-21;\n",
        )
        .unwrap();
        assert_eq!(f.order, vec![1, 2, 3]);
        let p = &f.entities[&1];
        assert_eq!(p.name(), "CARTESIAN_POINT");
        assert_eq!(p.params()[0], Value::Str("pé".into()));
        let xyz: Vec<f64> = p.params()[1].as_list().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
        assert_eq!(xyz, vec![1.0, -0.25, 3.0]);
        let u = &f.entities[&2];
        assert!(u.is_complex());
        assert_eq!(u.part("SI_UNIT").unwrap()[0], Value::Enum("MILLI".into()));
        assert_eq!(u.part("NAMED_UNIT").unwrap()[0], Value::Derived);
        let m = &f.entities[&3];
        assert_eq!(m.params()[0].as_f64(), Some(1e-7));
        assert_eq!(m.params()[2], Value::Str("it's".into()));
    }
}
