//! Subset of KiCad custom design rules (`.kicad_dru`): clearance rules between tracks of two
//! nets. Conditions are parsed into a small expression tree and evaluated for every pair of
//! nets. Position dependent functions (courtyards, footprints) evaluate to false, which keeps
//! a rule like "... && !A.intersectsCourtyard('U1')" in force everywhere (on the safe side).

use std::collections::HashMap;

use gummiband_core::board::{Board, NetId};

use crate::sexpr::{self, Node};

#[derive(Debug, Clone, PartialEq)]
enum Expr {
    Or(Box<Expr>, Box<Expr>),
    And(Box<Expr>, Box<Expr>),
    Not(Box<Expr>),
    Eq(Operand, Operand),
    Ne(Operand, Operand),
    /// A function call on an item: always false here.
    Call,
    Bool(bool),
}

#[derive(Debug, Clone, PartialEq)]
enum Operand {
    Prop(char, String),
    Str(String),
}

#[derive(Debug, Clone)]
pub struct ClearanceRule {
    pub name: String,
    pub min: f64,
    cond: Expr,
}

/// Item properties a condition can ask for.
struct ItemProps<'a> {
    kind: &'a str,
    net: &'a str,
    class: &'a str,
}

pub fn parse_rules(text: &str) -> Vec<ClearanceRule> {
    // comments start with '#' and run to the end of the line
    let text: String = text
        .lines()
        .filter(|l| !l.trim_start().starts_with('#'))
        .collect::<Vec<_>>()
        .join("\n");
    let wrapped = format!("(rules {text})");
    let Ok(root) = sexpr::parse(&wrapped) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for r in root.children("rule") {
        let name = r.args().first().and_then(|x| x.as_str()).unwrap_or("").to_string();
        let Some(cond) = r.child_str("condition") else { continue };
        let Some(expr) = parse_expr(cond) else { continue };
        for c in r.children("constraint") {
            if c.args().first().and_then(|x| x.as_str()) != Some("clearance") {
                continue;
            }
            if let Some(min) = constraint_min(c) {
                out.push(ClearanceRule {
                    name: name.clone(),
                    min,
                    cond: expr.clone(),
                });
            }
        }
    }
    out
}

fn constraint_min(c: &Node) -> Option<f64> {
    let m = c.child("min")?;
    let v = m.args().first()?.as_str()?;
    let v = v.trim_end_matches("mm");
    v.parse().ok()
}

/// Minimum track widths per net class (`A.NetClass == 'X'` conditions) and the footprints
/// whose courtyards are exempt (`!A.intersectsCourtyard('REF')`).
pub fn apply_width_rules(board: &mut Board, text: &str) {
    let text: String = text
        .lines()
        .filter(|l| !l.trim_start().starts_with('#'))
        .collect::<Vec<_>>()
        .join("\n");
    let wrapped = format!("(rules {text})");
    let Ok(root) = sexpr::parse(&wrapped) else { return };
    for r in root.children("rule") {
        let Some(cond) = r.child_str("condition") else { continue };
        for c in r.children("constraint") {
            if c.args().first().and_then(|x| x.as_str()) != Some("track_width") {
                continue;
            }
            let Some(min) = constraint_min(c) else { continue };
            let classes = quoted_after(cond, "A.NetClass == ");
            let refs = quoted_after(cond, "!A.intersectsCourtyard(");
            for cls in board.classes.iter_mut().filter(|k| classes.contains(&k.name)) {
                cls.min_width = Some(cls.min_width.map_or(min, |m: f64| m.max(min)));
                cls.neck_refs.extend(refs.iter().cloned());
            }
        }
    }
}

/// The quoted strings that follow each occurrence of `prefix`.
fn quoted_after(s: &str, prefix: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut rest = s;
    while let Some(i) = rest.find(prefix) {
        rest = &rest[i + prefix.len()..];
        let r = rest.trim_start();
        if let Some(q) = r.chars().next().filter(|c| *c == '\'' || *c == '"')
            && let Some(end) = r[1..].find(q)
        {
            out.push(r[1..1 + end].to_string());
        }
    }
    out
}

/// Track to track clearance per net pair (only pairs above the netclass clearance matter).
pub fn pair_clearances(board: &Board, rules: &[ClearanceRule]) -> HashMap<(NetId, NetId), f64> {
    let mut out = HashMap::new();
    if rules.is_empty() {
        return out;
    }
    let n = board.nets.len();
    for a in 0..n {
        for b in a + 1..n {
            let (na, nb) = (&board.nets[a], &board.nets[b]);
            let pa = ItemProps {
                kind: "Track",
                net: &na.name,
                class: &board.classes[na.class as usize].name,
            };
            let pb = ItemProps {
                kind: "Track",
                net: &nb.name,
                class: &board.classes[nb.class as usize].name,
            };
            let mut best: f64 = 0.0;
            for r in rules {
                if eval(&r.cond, &pa, &pb) || eval(&r.cond, &pb, &pa) {
                    best = best.max(r.min);
                }
            }
            if best > board.clearance(Some(a as NetId), Some(b as NetId)) {
                out.insert((a as NetId, b as NetId), best);
            }
        }
    }
    out
}

fn glob(pat: &str, s: &str) -> bool {
    fn rec(p: &[u8], s: &[u8]) -> bool {
        match (p.first(), s.first()) {
            (None, None) => true,
            (Some(b'*'), _) => rec(&p[1..], s) || (!s.is_empty() && rec(p, &s[1..])),
            (Some(b'?'), Some(_)) => rec(&p[1..], &s[1..]),
            (Some(a), Some(b)) if a == b => rec(&p[1..], &s[1..]),
            _ => false,
        }
    }
    rec(pat.as_bytes(), s.as_bytes())
}

fn prop<'a>(p: &'a ItemProps, name: &str) -> Option<&'a str> {
    match name {
        "Type" => Some(p.kind),
        "NetName" => Some(p.net),
        "NetClass" => Some(p.class),
        _ => None,
    }
}

fn eval(e: &Expr, a: &ItemProps, b: &ItemProps) -> bool {
    let value = |o: &Operand| -> Option<String> {
        match o {
            Operand::Str(s) => Some(s.clone()),
            Operand::Prop(w, name) => prop(if *w == 'A' { a } else { b }, name).map(str::to_string),
        }
    };
    let eq = |x: &Operand, y: &Operand| -> bool {
        match (x, y, value(x), value(y)) {
            // string literals act as wildcard patterns, as in KiCad
            (_, Operand::Str(pat), Some(v), _) => glob(pat, &v),
            (Operand::Str(pat), _, _, Some(v)) => glob(pat, &v),
            (_, _, Some(v), Some(w)) => v == w,
            _ => false,
        }
    };
    match e {
        Expr::Or(x, y) => eval(x, a, b) || eval(y, a, b),
        Expr::And(x, y) => eval(x, a, b) && eval(y, a, b),
        Expr::Not(x) => !eval(x, a, b),
        Expr::Eq(x, y) => eq(x, y),
        Expr::Ne(x, y) => !eq(x, y),
        Expr::Call => false,
        Expr::Bool(v) => *v,
    }
}

// --- expression parser (recursive descent over a token list)

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Ident(String),
    Str(String),
    Op(&'static str),
    LParen,
    RParen,
    Dot,
    Comma,
}

fn tokenize(s: &str) -> Option<Vec<Tok>> {
    let b = s.as_bytes();
    let mut i = 0;
    let mut out = Vec::new();
    while i < b.len() {
        let c = b[i];
        match c {
            b' ' | b'\t' | b'\n' => i += 1,
            b'(' => {
                out.push(Tok::LParen);
                i += 1
            }
            b')' => {
                out.push(Tok::RParen);
                i += 1
            }
            b'.' => {
                out.push(Tok::Dot);
                i += 1
            }
            b',' => {
                out.push(Tok::Comma);
                i += 1
            }
            b'\'' | b'"' => {
                let q = c;
                let start = i + 1;
                i += 1;
                while i < b.len() && b[i] != q {
                    i += 1;
                }
                out.push(Tok::Str(s[start..i].to_string()));
                i += 1;
            }
            _ => {
                if let Some(op) = ["&&", "||", "==", "!="].into_iter().find(|op| s[i..].starts_with(op)) {
                    out.push(Tok::Op(op));
                    i += op.len();
                    continue;
                }
                if c == b'!' {
                    out.push(Tok::Op("!"));
                    i += 1;
                    continue;
                }
                let start = i;
                while i < b.len() && (b[i].is_ascii_alphanumeric() || b[i] == b'_') {
                    i += 1;
                }
                if i == start {
                    return None;
                }
                out.push(Tok::Ident(s[start..i].to_string()));
            }
        }
    }
    Some(out)
}

fn parse_expr(s: &str) -> Option<Expr> {
    let toks = tokenize(s)?;
    let mut p = Parser { t: toks, i: 0 };
    let e = p.or()?;
    if p.i != p.t.len() {
        return None;
    }
    Some(e)
}

struct Parser {
    t: Vec<Tok>,
    i: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.t.get(self.i)
    }
    fn eat(&mut self, t: &Tok) -> bool {
        if self.peek() == Some(t) {
            self.i += 1;
            true
        } else {
            false
        }
    }
    fn or(&mut self) -> Option<Expr> {
        let mut e = self.and()?;
        while self.eat(&Tok::Op("||")) {
            e = Expr::Or(Box::new(e), Box::new(self.and()?));
        }
        Some(e)
    }
    fn and(&mut self) -> Option<Expr> {
        let mut e = self.unary()?;
        while self.eat(&Tok::Op("&&")) {
            e = Expr::And(Box::new(e), Box::new(self.unary()?));
        }
        Some(e)
    }
    fn unary(&mut self) -> Option<Expr> {
        if self.eat(&Tok::Op("!")) {
            return Some(Expr::Not(Box::new(self.unary()?)));
        }
        if self.eat(&Tok::LParen) {
            let e = self.or()?;
            if !self.eat(&Tok::RParen) {
                return None;
            }
            return Some(e);
        }
        let lhs = self.operand()?;
        let lhs = match lhs {
            Some(o) => o,
            None => return Some(Expr::Call),
        };
        if self.eat(&Tok::Op("==")) {
            let rhs = self.operand()??;
            return Some(Expr::Eq(lhs, rhs));
        }
        if self.eat(&Tok::Op("!=")) {
            let rhs = self.operand()??;
            return Some(Expr::Ne(lhs, rhs));
        }
        match lhs {
            Operand::Str(s) => Some(Expr::Bool(!s.is_empty())),
            Operand::Prop(..) => Some(Expr::Bool(false)),
        }
    }
    /// Some(Some(operand)), or Some(None) for a function call (consumed).
    fn operand(&mut self) -> Option<Option<Operand>> {
        match self.peek()?.clone() {
            Tok::Str(s) => {
                self.i += 1;
                Some(Some(Operand::Str(s)))
            }
            Tok::Ident(w) => {
                self.i += 1;
                if !self.eat(&Tok::Dot) {
                    return Some(Some(Operand::Str(w)));
                }
                let Some(Tok::Ident(name)) = self.peek().cloned() else {
                    return None;
                };
                self.i += 1;
                if self.eat(&Tok::LParen) {
                    // function call: skip arguments
                    let mut depth = 1;
                    while depth > 0 {
                        match self.peek()? {
                            Tok::LParen => depth += 1,
                            Tok::RParen => depth -= 1,
                            _ => {}
                        }
                        self.i += 1;
                    }
                    return Some(None);
                }
                let who = w.chars().next().unwrap_or('A');
                Some(Some(Operand::Prop(who, name)))
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_3w_rule() {
        let text = r#"(version 1)
(rule "LED data 3W spacing"
  (constraint clearance (min 0.5mm))
  (condition "A.Type == 'Track' && B.Type == 'Track' && A.NetClass == 'LEDData' && B.NetClass == 'LEDData' && A.NetName != B.NetName && (A.NetName == '*OUT?_DATA' || A.NetName == '*DATA_IN') && (B.NetName == '*OUT?_DATA' || B.NetName == '*DATA_IN') && !A.intersectsCourtyard('U21')"))"#;
        let rules = parse_rules(text);
        assert_eq!(rules.len(), 1);
        assert_eq!(rules[0].min, 0.5);
        let a = ItemProps {
            kind: "Track",
            net: "/Outputs/OUT1_DATA",
            class: "LEDData",
        };
        let b = ItemProps {
            kind: "Track",
            net: "/Outputs/OUT2_DATA",
            class: "LEDData",
        };
        let c = ItemProps {
            kind: "Track",
            net: "LED_D1",
            class: "LEDData",
        };
        assert!(eval(&rules[0].cond, &a, &b));
        assert!(!eval(&rules[0].cond, &a, &c));
        assert!(!eval(&rules[0].cond, &a, &a));
    }
}
