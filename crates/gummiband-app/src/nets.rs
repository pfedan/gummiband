//! What a user prescribes for single nets: widths, where a track may neck, priority, and nets
//! left out. One type for every front end: the CLI reads it from the file of `--net-widths`, the
//! MCP server keeps it per session (`set_net_options`, `power_plan`), and both hand it to
//! [`apply_net_options`] through [`RouteOptions`](crate::RouteOptions).

use std::collections::BTreeMap;
use std::path::Path;

use anyhow::{Context, Result, bail};
use gummiband_core::board::{Board, ClassId, NetId};

/// The options of one net. What is `None` stays as the net class and the rules have it.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NetOptions {
    /// Track width in mm the net is routed with.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub width: Option<f64>,
    /// Minimum width in mm: the track never necks below it, except inside the courtyards of
    /// `neck_refs`. A larger minimum of the class or the rules stays.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_width: Option<f64>,
    /// References of the parts in whose courtyard the track may neck below the minimum width.
    /// None: the parts whose pad on this net is narrower than `min_width`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub neck_refs: Option<Vec<String>>,
    /// Routing priority, lower routes first. None: the priority of the net class.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub priority: Option<i32>,
    /// Leave the net unrouted.
    #[serde(default)]
    pub skip: bool,
}

impl NetOptions {
    /// Nothing is prescribed.
    pub fn is_empty(&self) -> bool {
        *self == NetOptions::default()
    }

    /// What `over` prescribes wins, the rest stays.
    pub fn merge(&mut self, over: &NetOptions) {
        if over.width.is_some() {
            self.width = over.width;
        }
        if over.min_width.is_some() {
            self.min_width = over.min_width;
        }
        if over.neck_refs.is_some() {
            self.neck_refs = over.neck_refs.clone();
        }
        if over.priority.is_some() {
            self.priority = over.priority;
        }
        self.skip |= over.skip;
    }

    /// The widths make sense on their own.
    fn check(&self, net: &str) -> Result<()> {
        for (name, w) in [("width", self.width), ("min_width", self.min_width)] {
            if w.is_some_and(|w| w.is_nan() || w <= 0.0) {
                bail!("net {net}: {name} must be above 0");
            }
        }
        if let (Some(w), Some(m)) = (self.width, self.min_width)
            && m > w + 1e-9
        {
            bail!("net {net}: min_width {m} is above width {w}");
        }
        Ok(())
    }
}

/// Per net options from JSON text: an object with one entry per net name, either a number (the
/// track width in mm, `{"GND": 0.5}`) or an object with the fields of [`NetOptions`]
/// (`{"VBUS": {"width": 0.8, "min_width": 0.8, "priority": 1}}`).
pub fn parse_net_options(text: &str) -> Result<BTreeMap<String, NetOptions>> {
    let map: BTreeMap<String, serde_json::Value> =
        serde_json::from_str(text).context("expected a JSON object with one entry per net name")?;
    let mut out = BTreeMap::new();
    for (net, v) in map {
        let o = match v {
            serde_json::Value::Number(n) => NetOptions {
                width: n.as_f64(),
                ..Default::default()
            },
            v @ serde_json::Value::Object(_) => serde_json::from_value(v)
                .with_context(|| format!("net {net}: the fields are width, min_width, neck_refs, priority and skip"))?,
            other => bail!("net {net}: expected a width in mm or an object of options, not {other}"),
        };
        o.check(&net)?;
        out.insert(net, o);
    }
    Ok(out)
}

/// [`parse_net_options`] of a file.
pub fn read_net_options(path: &Path) -> Result<BTreeMap<String, NetOptions>> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    parse_net_options(&text).with_context(|| path.display().to_string())
}

/// Priorities per net from a JSON file `{"net name": priority}`.
pub fn read_net_priorities(path: &Path) -> Result<BTreeMap<String, NetOptions>> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let map: BTreeMap<String, i32> = serde_json::from_str(&text)
        .with_context(|| format!("{}: expected {{\"net name\": priority}}", path.display()))?;
    Ok(map
        .into_iter()
        .map(|(net, p)| {
            (
                net,
                NetOptions {
                    priority: Some(p),
                    ..Default::default()
                },
            )
        })
        .collect())
}

/// What [`apply_net_options`] did.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AppliedNetOptions {
    /// Entries whose net the board has (the others are passed over: the widths of another
    /// routing may name nets this board lacks).
    pub nets: usize,
    /// Nets left out of the route, in board order: skipped ones, and with `only` every net
    /// that is not named there. Nets with fewer than two pins have nothing to route and are
    /// not listed.
    pub skipped: Vec<String>,
}

/// Put the options on the board as the router sees it: widths become a net class of the net's
/// own, priorities replace the class priority, and a skipped net loses its pins, so it gets no
/// connections while its pads stay obstacles. `only` names the nets of this route (empty: all)
/// and only adds to the list of nets left out; the search itself narrows the board to them.
pub fn apply_net_options(
    b: &mut Board,
    options: &BTreeMap<String, NetOptions>,
    only: &[String],
) -> Result<AppliedNetOptions> {
    let mut done = AppliedNetOptions::default();
    for (name, o) in options {
        o.check(name)?;
        if let Some(id) = b.net_id(name) {
            done.nets += 1;
            set_widths(b, id, o)?;
        }
    }
    for n in b.nets.iter_mut() {
        let o = options.get(&n.name);
        if let Some(p) = o.and_then(|o| o.priority) {
            n.priority = p;
        }
        let skip = o.is_some_and(|o| o.skip);
        if (skip || (!only.is_empty() && !only.contains(&n.name))) && n.pins.len() >= 2 {
            done.skipped.push(n.name.clone());
        }
        if skip {
            n.pins.clear();
        }
    }
    Ok(done)
}

/// Give a net the class its widths ask for: a copy of its class, shared with the nets that ask
/// for the same.
fn set_widths(b: &mut Board, net: NetId, o: &NetOptions) -> Result<()> {
    if o.min_width.is_none() && o.neck_refs.is_none() {
        // the width alone: the class's other values stay as they are
        if let Some(w) = o.width {
            b.set_net_width(net, w);
        }
        return Ok(());
    }
    let ci = b.nets[net as usize].class as usize;
    let mut c = b.classes[ci].clone();
    if let Some(w) = o.width {
        c.track_width = w;
        c.name = format!("{}@{w}", c.name);
    }
    if let Some(m) = o.min_width {
        if m > c.track_width + 1e-9 {
            bail!(
                "net {}: min_width {m} is above its track width {}",
                b.nets[net as usize].name,
                c.track_width
            );
        }
        c.name = format!("{} min {m}", c.name);
        // a larger minimum of the rules stays, and a preferred width below the new one goes
        c.min_width = Some(c.min_width.map_or(m, |old| old.max(m)));
        c.target_width = c.target_width.filter(|t| *t > m);
    }
    match &o.neck_refs {
        Some(refs) => {
            for r in refs {
                if !b.courtyards.contains_key(r) {
                    bail!(
                        "net {}: neck_refs names {r}, which is no part with a courtyard on this board",
                        b.nets[net as usize].name
                    );
                }
                if !c.neck_refs.contains(r) {
                    c.neck_refs.push(r.clone());
                }
            }
        }
        // pads narrower than the minimum: the track necks inside their part's courtyard
        None => {
            if let Some(m) = o.min_width {
                for &pin in &b.nets[net as usize].pins {
                    let p = &b.pins[pin];
                    if p.size.0.min(p.size.1) < m && !c.neck_refs.contains(&p.footprint) {
                        c.neck_refs.push(p.footprint.clone());
                    }
                }
            }
        }
    }
    // two nets share a class only if all of it agrees: the parts to neck at differ per net
    let same = |k: &gummiband_core::board::NetClass| {
        k.name == c.name
            && k.track_width == c.track_width
            && k.min_width == c.min_width
            && k.target_width == c.target_width
            && k.neck_refs == c.neck_refs
    };
    let i = match b.classes.iter().position(same) {
        Some(i) => i,
        None => {
            if b.classes.iter().any(|k| k.name == c.name) {
                c.name = format!("{} ({})", c.name, b.nets[net as usize].name);
            }
            b.classes.push(c);
            b.classes.len() - 1
        }
    };
    b.nets[net as usize].class = i as ClassId;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_number_is_a_width_and_an_object_has_the_fields() {
        let m = parse_net_options(
            r#"{"GND": 0.5, "VBUS": {"width": 0.8, "min_width": 0.6, "neck_refs": ["U1"], "priority": 1},
                "NC": {"skip": true}}"#,
        )
        .unwrap();
        assert_eq!(m["GND"].width, Some(0.5));
        assert_eq!(m["GND"].min_width, None);
        assert_eq!(
            m["VBUS"],
            NetOptions {
                width: Some(0.8),
                min_width: Some(0.6),
                neck_refs: Some(vec!["U1".into()]),
                priority: Some(1),
                skip: false,
            }
        );
        assert!(m["NC"].skip);
        // a field that does not exist, a width that is none, a minimum above the width
        let e = format!("{:#}", parse_net_options(r#"{"A": {"widht": 1}}"#).unwrap_err());
        assert!(e.contains("net A") && e.contains("min_width"), "{e}");
        assert!(parse_net_options(r#"{"A": "wide"}"#).is_err());
        assert!(parse_net_options(r#"{"A": 0}"#).is_err());
        let e = parse_net_options(r#"{"A": {"width": 0.2, "min_width": 0.3}}"#).unwrap_err();
        assert_eq!(e.to_string(), "net A: min_width 0.3 is above width 0.2");
    }

    #[test]
    fn what_is_given_later_wins_field_by_field() {
        let mut o = NetOptions {
            width: Some(0.5),
            priority: Some(3),
            ..Default::default()
        };
        o.merge(&NetOptions {
            min_width: Some(0.4),
            priority: Some(1),
            ..Default::default()
        });
        assert_eq!(
            (o.width, o.min_width, o.priority, o.skip),
            (Some(0.5), Some(0.4), Some(1), false)
        );
        assert!(!o.is_empty() && NetOptions::default().is_empty());
    }
}
