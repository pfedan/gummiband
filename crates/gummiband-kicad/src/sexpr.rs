//! Minimal s-expression parser for KiCad files. Keeps byte spans so that a writer can patch the
//! original text instead of re-serializing it.

use std::ops::Range;

#[derive(Debug, Clone, PartialEq)]
pub enum Atom {
    Symbol(String),
    Str(String),
}

#[derive(Debug, Clone)]
pub enum Node {
    Atom(Atom, Range<usize>),
    List(Vec<Node>, Range<usize>),
}

#[derive(Debug, thiserror::Error)]
pub enum ParseError {
    #[error("unexpected end of input")]
    Eof,
    #[error("unexpected ')' at byte {0}")]
    UnexpectedClose(usize),
    #[error("trailing data at byte {0}")]
    Trailing(usize),
}

impl Node {
    pub fn span(&self) -> Range<usize> {
        match self {
            Node::Atom(_, s) | Node::List(_, s) => s.clone(),
        }
    }

    /// Head symbol of a list, e.g. `segment` for `(segment ...)`.
    pub fn head(&self) -> Option<&str> {
        match self {
            Node::List(items, _) => match items.first() {
                Some(Node::Atom(Atom::Symbol(s), _)) => Some(s),
                _ => None,
            },
            _ => None,
        }
    }

    pub fn items(&self) -> &[Node] {
        match self {
            Node::List(items, _) => items,
            Node::Atom(..) => &[],
        }
    }

    /// Arguments after the head.
    pub fn args(&self) -> &[Node] {
        let items = self.items();
        if items.is_empty() { items } else { &items[1..] }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Node::Atom(Atom::Symbol(s), _) | Node::Atom(Atom::Str(s), _) => Some(s),
            _ => None,
        }
    }

    pub fn as_f64(&self) -> Option<f64> {
        self.as_str()?.parse().ok()
    }

    /// First child list with the given head.
    pub fn child(&self, head: &str) -> Option<&Node> {
        self.args().iter().find(|n| n.head() == Some(head))
    }

    pub fn children<'a>(&'a self, head: &'a str) -> impl Iterator<Item = &'a Node> + 'a {
        self.args().iter().filter(move |n| n.head() == Some(head))
    }

    /// `(head a b c)` -> a, b, c as f64.
    pub fn child_f64s(&self, head: &str) -> Option<Vec<f64>> {
        let c = self.child(head)?;
        c.args().iter().map(|a| a.as_f64()).collect()
    }

    pub fn child_f64(&self, head: &str) -> Option<f64> {
        self.child(head)?.args().first()?.as_f64()
    }

    pub fn child_str(&self, head: &str) -> Option<&str> {
        self.child(head)?.args().first()?.as_str()
    }

    /// True for `(head yes)` and for a bare `(head)` or bare symbol `head` among the args.
    pub fn flag(&self, head: &str) -> bool {
        for a in self.args() {
            match a {
                Node::Atom(Atom::Symbol(s), _) if s == head => return true,
                Node::List(..) if a.head() == Some(head) => {
                    return match a.args().first().and_then(|v| v.as_str()) {
                        None => true,
                        Some(v) => v == "yes" || v == "true",
                    };
                }
                _ => {}
            }
        }
        false
    }
}

pub fn parse(text: &str) -> Result<Node, ParseError> {
    let bytes = text.as_bytes();
    let mut pos = 0;
    skip_ws(bytes, &mut pos);
    let node = parse_node(text, bytes, &mut pos)?;
    skip_ws(bytes, &mut pos);
    if pos != bytes.len() {
        return Err(ParseError::Trailing(pos));
    }
    Ok(node)
}

fn skip_ws(b: &[u8], pos: &mut usize) {
    while *pos < b.len() && b[*pos].is_ascii_whitespace() {
        *pos += 1;
    }
}

fn parse_node(text: &str, b: &[u8], pos: &mut usize) -> Result<Node, ParseError> {
    // Iterative to survive deeply nested files without stack overflow.
    let mut stack: Vec<(Vec<Node>, usize)> = Vec::new();
    loop {
        skip_ws(b, pos);
        if *pos >= b.len() {
            return Err(ParseError::Eof);
        }
        let node = match b[*pos] {
            b'(' => {
                stack.push((Vec::new(), *pos));
                *pos += 1;
                continue;
            }
            b')' => {
                let Some((items, start)) = stack.pop() else {
                    return Err(ParseError::UnexpectedClose(*pos));
                };
                *pos += 1;
                Node::List(items, start..*pos)
            }
            b'"' => {
                let start = *pos;
                *pos += 1;
                let mut s = String::new();
                let mut run = *pos;
                loop {
                    if *pos >= b.len() {
                        return Err(ParseError::Eof);
                    }
                    match b[*pos] {
                        b'"' => break,
                        b'\\' => {
                            s.push_str(&text[run..*pos]);
                            *pos += 1;
                            if *pos >= b.len() {
                                return Err(ParseError::Eof);
                            }
                            match b[*pos] {
                                b'n' => s.push('\n'),
                                b't' => s.push('\t'),
                                c => s.push(c as char),
                            }
                            *pos += 1;
                            run = *pos;
                        }
                        _ => *pos += 1,
                    }
                }
                s.push_str(&text[run..*pos]);
                *pos += 1;
                Node::Atom(Atom::Str(s), start..*pos)
            }
            _ => {
                let start = *pos;
                while *pos < b.len()
                    && !b[*pos].is_ascii_whitespace()
                    && b[*pos] != b'('
                    && b[*pos] != b')'
                {
                    *pos += 1;
                }
                Node::Atom(Atom::Symbol(text[start..*pos].to_string()), start..*pos)
            }
        };
        match stack.last_mut() {
            Some((items, _)) => items.push(node),
            None => return Ok(node),
        }
    }
}

pub fn quote(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            _ => out.push(c),
        }
    }
    out.push('"');
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_nested_lists_and_strings() {
        let t = r#"(kicad_pcb (version 1) (net "a \"b\"") (locked yes) hide)"#;
        let n = parse(t).unwrap();
        assert_eq!(n.head(), Some("kicad_pcb"));
        assert_eq!(n.child_f64("version"), Some(1.0));
        assert_eq!(n.child_str("net"), Some("a \"b\""));
        assert!(n.flag("locked"));
        assert!(n.flag("hide"));
        assert!(!n.flag("missing"));
        let net = n.child("net").unwrap();
        assert_eq!(&t[net.span()], r#"(net "a \"b\"")"#);
    }
}
