//! Subset of KiCad custom design rules (`.kicad_dru`): clearance rules between tracks of two
//! nets. Conditions are parsed into a small expression tree and evaluated for every pair of
//! nets. Position dependent functions (courtyards, footprints) evaluate to false, which keeps
//! a rule like "... && !A.intersectsCourtyard('U1')" in force everywhere (on the safe side).

use std::collections::HashMap;

use gummiband_core::board::{Board, MatchKind, MatchRule, MinOptMax, NetId};

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
    /// `A.inDiffPair('pattern')`: the net of the item is part of a differential pair whose base
    /// name matches.
    InDiffPair(char, String),
    /// `A.hasNetclass('name')`.
    HasNetclass(char, String),
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
    /// The rule holds only outside coupled pairs (`!AB.isCoupledDiffPair()`).
    not_coupled: bool,
}

/// Item properties a condition can ask for.
struct ItemProps<'a> {
    kind: &'a str,
    net: &'a str,
    class: &'a str,
    /// Base name of the differential pair the net belongs to (`USB_D` of `USB_DP`), if any.
    pair: Option<&'a str>,
}

pub fn parse_rules(text: &str) -> Vec<ClearanceRule> {
    parse_constraints(text, "clearance")
}

fn parse_constraints(text: &str, what: &str) -> Vec<ClearanceRule> {
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
            if c.args().first().and_then(|x| x.as_str()) != Some(what) {
                continue;
            }
            if let Some(min) = constraint_min(c) {
                out.push(ClearanceRule {
                    name: name.clone(),
                    min,
                    cond: expr.clone(),
                    not_coupled: cond.contains("!AB.isCoupledDiffPair()"),
                });
            }
        }
    }
    out
}

fn constraint_min(c: &Node) -> Option<f64> {
    constraint_value(c, "min")
}

fn constraint_value(c: &Node, key: &str) -> Option<f64> {
    let m = c.child(key)?;
    let v = m.args().first()?.as_str()?;
    let v = v.trim_end_matches("mm");
    v.parse().ok()
}

/// Minimum and target track widths per net class (`A.NetClass == 'X'` conditions, the `min` and
/// the `opt` of a `track_width` constraint) and the footprints whose courtyards are exempt from
/// the minimum (`!A.intersectsCourtyard('REF')`).
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
        // the width of a pair rule is the width of the pair, not a target of the class
        let pair_rule = r
            .children("constraint")
            .any(|c| c.args().first().and_then(|x| x.as_str()) == Some("diff_pair_gap"));
        for c in r.children("constraint") {
            if c.args().first().and_then(|x| x.as_str()) != Some("track_width") {
                continue;
            }
            let classes = quoted_after(cond, "A.NetClass == ");
            if let Some(opt) = constraint_value(c, "opt").filter(|_| !pair_rule) {
                for cls in board.classes.iter_mut().filter(|k| classes.contains(&k.name)) {
                    cls.target_width = Some(opt);
                }
            }
            let Some(min) = constraint_min(c) else { continue };
            let refs = quoted_after(cond, "!A.intersectsCourtyard(");
            for cls in board.classes.iter_mut().filter(|k| classes.contains(&k.name)) {
                cls.min_width = Some(cls.min_width.map_or(min, |m: f64| m.max(min)));
                cls.neck_refs.extend(refs.iter().cloned());
            }
        }
    }
}

/// The preferred value of a constraint, else its minimum.
fn constraint_opt(c: &Node) -> Option<f64> {
    let v = c.child("opt").or_else(|| c.child("min"))?;
    let v = v.args().first()?.as_str()?;
    v.trim_end_matches("mm").parse().ok()
}

/// Width and gap of differential pairs per net class, from rules with a `diff_pair_gap`
/// constraint: `A.NetClass == 'X'` or `A.inDiffPair('pattern')` conditions, for all layers or
/// for inner or outer layers. Rules further down win, as in KiCad. Skew limits are match rules
/// ([`match_rules`]).
pub fn apply_pair_rules(board: &mut Board, text: &str) {
    let text: String = text
        .lines()
        .filter(|l| !l.trim_start().starts_with('#'))
        .collect::<Vec<_>>()
        .join("\n");
    let wrapped = format!("(rules {text})");
    let Ok(root) = sexpr::parse(&wrapped) else { return };
    let pairs = board.diff_pairs();
    for r in root.children("rule") {
        let Some(cond) = r.child_str("condition") else { continue };
        let of = |name: &str| {
            r.children("constraint")
                .find(|c| c.args().first().and_then(|x| x.as_str()) == Some(name))
        };
        let gap = of("diff_pair_gap").and_then(constraint_opt);
        let width = of("track_width").and_then(constraint_opt);
        if gap.is_none() {
            continue;
        }
        let mut classes = quoted_after(cond, "A.NetClass == ");
        for pat in quoted_after(cond, "A.inDiffPair(") {
            for &(p, n) in &pairs {
                let (a, b) = (&board.nets[p as usize], &board.nets[n as usize]);
                let base = &a.name[..a.name.len() - 1];
                if glob(&pat, &a.name) || glob(&pat, &b.name) || glob(&pat, base) || glob(&pat, base.trim_end_matches('_')) {
                    classes.push(board.classes[a.class as usize].name.clone());
                }
            }
        }
        let layer = r.child_str("layer").unwrap_or("");
        let (outer, inner) = match layer {
            "inner" => (false, true),
            "outer" | "F.Cu" | "B.Cu" => (true, false),
            "" => (true, true),
            _ => (false, true),
        };
        for cls in board.classes.iter_mut().filter(|k| classes.contains(&k.name)) {
            let (w, g) = (width.unwrap_or(0.0), gap.unwrap_or(0.0));
            if gap.is_some() {
                if outer {
                    cls.diff_outer = Some((w, g));
                }
                if inner {
                    cls.diff_inner = Some((w, g));
                }
            }
        }
    }
}

/// A length value of a constraint in mm; time values (`ps`) are not lengths.
fn length_value(c: &Node, key: &str) -> Option<f64> {
    let v = c.child(key)?.args().first()?.as_str()?.trim().to_string();
    if v.ends_with("ps") || v.ends_with("fs") || v.ends_with("ns") {
        return None;
    }
    let (num, scale) = if let Some(x) = v.strip_suffix("mil") {
        (x, 0.0254)
    } else if let Some(x) = v.strip_suffix("in") {
        (x, 25.4)
    } else {
        (v.trim_end_matches("mm"), 1.0)
    };
    num.trim().parse::<f64>().ok().map(|x| x * scale)
}

/// The `length`, `skew` and `diff_pair_uncoupled` constraints of the rules, grouped as KiCad's
/// DRC groups them: per net and kind of constraint the last rule in the file whose condition
/// holds for the net's tracks wins, and a rule holds for the nets it wins.
pub fn match_rules(board: &Board, text: &str) -> Vec<MatchRule> {
    let text: String = text
        .lines()
        .filter(|l| !l.trim_start().starts_with('#'))
        .collect::<Vec<_>>()
        .join("\n");
    let wrapped = format!("(rules {text})");
    let Ok(root) = sexpr::parse(&wrapped) else {
        return Vec::new();
    };
    let mut base: HashMap<NetId, String> = HashMap::new();
    for (p, n) in board.diff_pairs() {
        for x in [p, n] {
            let name = &board.nets[x as usize].name;
            base.insert(x, name[..name.len() - 1].to_string());
        }
    }
    // candidates in file order: (kind, rule name, value, within pairs, condition)
    let mut cands: Vec<(MatchKind, String, MinOptMax, bool, Expr)> = Vec::new();
    for r in root.children("rule") {
        let name = r.args().first().and_then(|x| x.as_str()).unwrap_or("").to_string();
        let Some(cond) = r.child_str("condition") else { continue };
        let Some(expr) = parse_expr(cond) else { continue };
        for c in r.children("constraint") {
            let kind = match c.args().first().and_then(|x| x.as_str()) {
                Some("length") => MatchKind::Length,
                Some("skew") => MatchKind::Skew,
                Some("diff_pair_uncoupled") => MatchKind::Uncoupled,
                Some("diff_pair_gap") => MatchKind::Gap,
                _ => continue,
            };
            let value = MinOptMax {
                min: length_value(c, "min"),
                opt: length_value(c, "opt"),
                max: length_value(c, "max"),
            };
            if value == MinOptMax::default() {
                continue;
            }
            let within = c.child("within_diff_pairs").is_some()
                || c.args().iter().any(|x| x.as_str() == Some("within_diff_pairs"));
            cands.push((kind, name.clone(), value, within, expr.clone()));
        }
    }
    let none = ItemProps {
        kind: "",
        net: "",
        class: "",
        pair: None,
    };
    // the winning candidate per kind and net
    let mut won: Vec<Vec<NetId>> = vec![Vec::new(); cands.len()];
    for (i, net) in board.nets.iter().enumerate() {
        if net.pins.len() < 2 {
            continue;
        }
        let a = ItemProps {
            kind: "Track",
            net: &net.name,
            class: &board.classes[net.class as usize].name,
            pair: base.get(&(i as NetId)).map(String::as_str),
        };
        for kind in [MatchKind::Length, MatchKind::Skew, MatchKind::Uncoupled, MatchKind::Gap] {
            if matches!(kind, MatchKind::Uncoupled | MatchKind::Gap) && a.pair.is_none() {
                continue;
            }
            if let Some(c) = (0..cands.len()).rev().find(|&c| cands[c].0 == kind && eval(&cands[c].4, &a, &none)) {
                won[c].push(i as NetId);
            }
        }
    }
    cands
        .into_iter()
        .zip(won)
        .filter(|(_, nets)| !nets.is_empty())
        .map(|((kind, name, value, within_pairs, _), nets)| MatchRule {
            name,
            kind,
            value,
            within_pairs,
            nets,
        })
        .collect()
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
    clearances(board, rules, "Track")
}

/// The same for copper of any kind: a rule that asks for tracks does not hold here. Vias stand in
/// for pads as well, the rules of a board do not tell them apart.
pub fn item_clearances(board: &Board, rules: &[ClearanceRule]) -> HashMap<(NetId, NetId), f64> {
    clearances(board, rules, "Via")
}

/// Hole clearance per net (`hole_clearance` constraints) above the one of the board.
pub fn hole_clearances(board: &Board, text: &str) -> HashMap<NetId, f64> {
    let mut out = HashMap::new();
    for r in parse_constraints(text, "hole_clearance") {
        for (i, net) in board.nets.iter().enumerate() {
            let a = ItemProps {
                kind: "Via",
                net: &net.name,
                class: &board.classes[net.class as usize].name,
                pair: None,
            };
            let hole = ItemProps {
                kind: "Hole",
                net: "",
                class: "",
                pair: None,
            };
            if r.min > board.rules.hole_clearance && (eval(&r.cond, &a, &hole) || eval(&r.cond, &hole, &a)) {
                let e = out.entry(i as NetId).or_insert(0.0_f64);
                *e = e.max(r.min);
            }
        }
    }
    out
}

fn clearances(board: &Board, rules: &[ClearanceRule], kind: &str) -> HashMap<(NetId, NetId), f64> {
    let mut out = HashMap::new();
    if rules.is_empty() {
        return out;
    }
    let n = board.nets.len();
    let partners: std::collections::HashSet<(NetId, NetId)> = board
        .diff_pairs()
        .into_iter()
        .map(|(a, b)| (a.min(b), a.max(b)))
        .collect();
    for a in 0..n {
        for b in a + 1..n {
            let (na, nb) = (&board.nets[a], &board.nets[b]);
            let coupled = partners.contains(&(a as NetId, b as NetId));
            let pa = ItemProps {
                kind,
                net: &na.name,
                class: &board.classes[na.class as usize].name,
                pair: None,
            };
            let pb = ItemProps {
                kind,
                net: &nb.name,
                class: &board.classes[nb.class as usize].name,
                pair: None,
            };
            let mut best: f64 = 0.0;
            for r in rules {
                if !(coupled && r.not_coupled) && (eval(&r.cond, &pa, &pb) || eval(&r.cond, &pb, &pa)) {
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
        Expr::InDiffPair(w, pat) => {
            let it = if *w == 'B' { b } else { a };
            it.pair.is_some_and(|base| {
                // KiCad matches the base name, with or without a trailing '_'
                glob(pat, base) || glob(pat, base.trim_end_matches('_')) || glob(pat, it.net)
            })
        }
        Expr::HasNetclass(w, name) => {
            let it = if *w == 'B' { b } else { a };
            glob(name, it.class)
        }
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
    let mut p = Parser { t: toks, i: 0, known: None };
    let e = p.or()?;
    if p.i != p.t.len() {
        return None;
    }
    Some(e)
}

struct Parser {
    t: Vec<Tok>,
    i: usize,
    /// A function call the last operand was, if it is one evaluated here.
    known: Option<Expr>,
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
            None => return Some(self.known.take().unwrap_or(Expr::Call)),
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
                    let who = w.chars().next().unwrap_or('A');
                    let known = matches!(name.as_str(), "inDiffPair" | "hasNetclass" | "hasNetClass");
                    if known
                        && let (Some(Tok::Str(arg)), Some(Tok::RParen)) =
                            (self.t.get(self.i).cloned(), self.t.get(self.i + 1).cloned())
                    {
                        self.i += 2;
                        self.known = Some(if name == "inDiffPair" {
                            Expr::InDiffPair(who, arg)
                        } else {
                            Expr::HasNetclass(who, arg)
                        });
                        return Some(None);
                    }
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
    fn pair_rules_by_class_and_layer() {
        use gummiband_core::board::{Layer, Net, NetClass};
        let mut b = Board::default();
        b.rules.min_clearance = 0.1;
        b.layers = ["F.Cu", "In1.Cu", "B.Cu"]
            .iter()
            .map(|n| Layer {
                name: n.to_string(),
                is_plane: false,
            })
            .collect();
        b.classes = vec![NetClass {
            name: "USB".into(),
            track_width: 0.2,
            clearance: 0.1,
            ..Default::default()
        }];
        for name in ["D_P", "D_N"] {
            b.nets.push(Net {
                name: name.into(),
                class: 0,
                pins: vec![0, 1],
                planes: Default::default(),
                priority: 0,
            });
        }
        apply_pair_rules(
            &mut b,
            r#"(version 1)
(rule a (layer outer) (condition "(A.NetClass == 'USB')")
  (constraint track_width (opt 0.16mm)) (constraint diff_pair_gap (opt 0.138mm)))
(rule b (layer inner) (condition "A.NetClass == 'USB'")
  (constraint track_width (opt 0.155mm)) (constraint diff_pair_gap (opt 0.144mm)))
(rule c (condition "A.inDiffPair('D_*')") (constraint skew (max 0.5mm)))"#,
        );
        let c = &b.classes[0];
        assert_eq!(c.diff_outer, Some((0.16, 0.138)));
        assert_eq!(c.diff_inner, Some((0.155, 0.144)));
        assert_eq!(b.pair_shape(0, 1, 0), (0.16, 0.138, 0.138));
        assert_eq!(b.pair_shape(0, 1, 1), (0.155, 0.144, 0.144));
    }

    #[test]
    fn width_rules_give_minimum_and_target() {
        use gummiband_core::board::NetClass;
        let mut b = Board::default();
        b.classes = ["Default", "Power", "USB"]
            .iter()
            .map(|n| NetClass {
                name: n.to_string(),
                track_width: 0.3,
                ..Default::default()
            })
            .collect();
        apply_width_rules(
            &mut b,
            r#"(version 1)
(rule p (condition "A.NetClass == 'Power' && !A.intersectsCourtyard('U1')")
  (constraint track_width (min 0.2mm) (opt 0.8mm)))
(rule u (condition "A.NetClass == 'USB'")
  (constraint track_width (opt 0.16mm)) (constraint diff_pair_gap (opt 0.14mm)))"#,
        );
        let [d, p, u] = [&b.classes[0], &b.classes[1], &b.classes[2]];
        assert_eq!((d.min_width, d.target_width), (None, None));
        assert_eq!((p.min_width, p.target_width), (Some(0.2), Some(0.8)));
        assert_eq!(p.neck_refs, vec!["U1".to_string()]);
        // the width of a pair rule is no target of the class
        assert_eq!((u.min_width, u.target_width), (None, None));
    }

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
