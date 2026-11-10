//! Create a board from a KiCad netlist (s-expression export): footprints from the KiCad
//! footprint libraries with the nets on their pads, a rectangular outline, the stackup,
//! mounting holes, and the net classes in the `.kicad_pro`. Parts are laid out in rows inside
//! the outline, unlocked, for `place_parts` / `move_part` to take over.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::fmt::Write as _;
use std::ops::Range;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow, bail};
use gummiband_core::knobs;
use serde_json::{Value, json};

use crate::read::glob_match;
use crate::sexpr::{self, Node, quote};
use crate::write::UuidGen;

/// Board file version of KiCad 9; boards with only KiCad 9 (or older) footprints get it.
pub const KICAD9_VERSION: u64 = 20241229;
/// Board file version of KiCad 10 (nets by name, no net table); used when a footprint library
/// is newer than KiCad 9.
pub const KICAD10_VERSION: u64 = 20260206;

// ------------------------------------------------------------------------------ netlist

#[derive(Debug, Clone, Default)]
pub struct Component {
    pub reference: String,
    pub value: String,
    /// Library id, e.g. `Resistor_SMD:R_0603_1608Metric`; empty if the symbol has none.
    pub footprint: String,
    pub datasheet: String,
    pub description: String,
    /// User fields (LCSC, MPN, ...) without Reference, Value, Footprint, Datasheet, Description.
    pub fields: Vec<(String, String)>,
    pub dnp: bool,
    pub exclude_from_bom: bool,
    pub exclude_from_board: bool,
    pub sheetname: String,
    pub sheetfile: String,
    /// KIID path of the symbol (sheet uuids and symbol uuid), links the footprint to it.
    pub path: String,
}

#[derive(Debug, Clone, Default)]
pub struct NetNode {
    pub reference: String,
    pub pin: String,
    pub pinfunction: String,
    pub pintype: String,
}

#[derive(Debug, Clone, Default)]
pub struct NetlistNet {
    pub name: String,
    pub class: String,
    pub nodes: Vec<NetNode>,
}

#[derive(Debug, Clone, Default)]
pub struct Netlist {
    pub components: Vec<Component>,
    pub nets: Vec<NetlistNet>,
    /// The schematic the netlist was exported from.
    pub source: Option<String>,
}

const RESERVED_FIELDS: [&str; 5] = ["Reference", "Value", "Footprint", "Datasheet", "Description"];

pub fn parse_netlist(text: &str) -> Result<Netlist> {
    let root = sexpr::parse(text).map_err(|e| anyhow!("netlist parse error: {e}"))?;
    if root.head() != Some("export") {
        bail!("not a KiCad netlist (expected `(export ...)`, the s-expression format of File > Export > Netlist)");
    }
    let s = |n: &Node, k: &str| n.child_str(k).unwrap_or("").to_string();
    let mut out = Netlist {
        source: root
            .child("design")
            .and_then(|d| d.child_str("source"))
            .map(str::to_string),
        ..Default::default()
    };
    for c in root.child("components").into_iter().flat_map(|c| c.children("comp")) {
        let mut comp = Component {
            reference: s(c, "ref"),
            value: s(c, "value"),
            footprint: s(c, "footprint"),
            datasheet: s(c, "datasheet"),
            description: s(c, "description"),
            ..Default::default()
        };
        if comp.datasheet == "~" {
            comp.datasheet.clear();
        }
        for f in c.child("fields").into_iter().flat_map(|f| f.children("field")) {
            let name = f.child_str("name").unwrap_or("").to_string();
            let value = f.args().iter().find_map(|a| a.as_str()).unwrap_or("").to_string();
            if !name.is_empty() && !RESERVED_FIELDS.contains(&name.as_str()) {
                comp.fields.push((name, value));
            }
        }
        for p in c.children("property") {
            let name = p.child_str("name").unwrap_or("");
            let value = p.child_str("value").unwrap_or("");
            match name {
                "dnp" => comp.dnp = true,
                "exclude_from_bom" => comp.exclude_from_bom = true,
                "exclude_from_board" => comp.exclude_from_board = true,
                "Sheetname" => comp.sheetname = value.to_string(),
                "Sheetfile" => comp.sheetfile = value.to_string(),
                _ => {}
            }
        }
        let sheet = c.child("sheetpath");
        let names = sheet.and_then(|x| x.child_str("names")).unwrap_or("/");
        if comp.sheetname.is_empty() {
            comp.sheetname = names.trim_matches('/').rsplit('/').next().unwrap_or("").to_string();
        }
        let mut sheet_ids = sheet.and_then(|x| x.child_str("tstamps")).unwrap_or("/").to_string();
        if !sheet_ids.ends_with('/') {
            sheet_ids.push('/');
        }
        let own = c
            .child_str("tstamps")
            .or_else(|| c.child_str("tstamp"))
            .and_then(|t| t.split_whitespace().next())
            .unwrap_or("");
        if !own.is_empty() {
            comp.path = format!("{sheet_ids}{own}");
        }
        if !comp.reference.is_empty() {
            out.components.push(comp);
        }
    }
    for n in root.child("nets").into_iter().flat_map(|n| n.children("net")) {
        let net = NetlistNet {
            name: s(n, "name"),
            class: s(n, "class"),
            nodes: n
                .children("node")
                .map(|x| NetNode {
                    reference: s(x, "ref"),
                    pin: s(x, "pin"),
                    pinfunction: s(x, "pinfunction"),
                    pintype: s(x, "pintype"),
                })
                .collect(),
        };
        if !net.name.is_empty() {
            out.nets.push(net);
        }
    }
    Ok(out)
}

// ------------------------------------------------------------------------------ libraries

/// Finds `.pretty` footprint libraries by nickname: explicit directories first, then the
/// project's `fp-lib-table`, the KiCad environment variables, the user's global
/// `fp-lib-table` and the usual install places.
pub struct FootprintLibs {
    explicit: Vec<PathBuf>,
    project_tables: Vec<(String, PathBuf)>,
    env_dirs: Vec<PathBuf>,
    global_tables: Vec<(String, PathBuf)>,
    install_dirs: Vec<PathBuf>,
    cache: HashMap<String, Option<PathBuf>>,
}

const ENV_VARS: [&str; 5] = [
    "KICAD10_FOOTPRINT_DIR",
    "KICAD9_FOOTPRINT_DIR",
    "KICAD8_FOOTPRINT_DIR",
    "KICAD7_FOOTPRINT_DIR",
    "KICAD_FOOTPRINT_DIR",
];

fn install_dirs() -> Vec<PathBuf> {
    let mut v: Vec<PathBuf> = [
        "/usr/share/kicad/footprints",
        "/usr/local/share/kicad/footprints",
        "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints",
        "/var/lib/flatpak/app/org.kicad.KiCad/current/active/files/share/kicad/footprints",
        "C:\\Program Files\\KiCad\\10.0\\share\\kicad\\footprints",
        "C:\\Program Files\\KiCad\\9.0\\share\\kicad\\footprints",
    ]
    .iter()
    .map(PathBuf::from)
    .collect();
    if let Some(home) = knobs::foreign("HOME") {
        v.push(
            PathBuf::from(home)
                .join(".local/share/flatpak/app/org.kicad.KiCad/current/active/files/share/kicad/footprints"),
        );
    }
    v.retain(|d| d.is_dir());
    v
}

fn global_table_files() -> Vec<PathBuf> {
    let mut bases = Vec::new();
    if let Some(home) = knobs::foreign("HOME") {
        let home = PathBuf::from(home);
        bases.push(home.join(".config/kicad"));
        bases.push(home.join("Library/Preferences/kicad"));
    }
    if let Some(app) = knobs::foreign("APPDATA") {
        bases.push(PathBuf::from(app).join("kicad"));
    }
    let mut out = Vec::new();
    for b in bases {
        for v in ["10.0", "9.0", "8.0"] {
            let f = b.join(v).join("fp-lib-table");
            if f.is_file() {
                out.push(f);
            }
        }
    }
    out
}

impl FootprintLibs {
    /// `explicit`: `.pretty` directories or directories that contain them. `project_dirs`:
    /// directories whose `fp-lib-table` counts as the project's (`${KIPRJMOD}` is the
    /// directory itself). `system`: also look at the environment, the global table and the
    /// install places.
    pub fn new(explicit: &[PathBuf], project_dirs: &[PathBuf], system: bool) -> Self {
        let env_dirs: Vec<PathBuf> = if system {
            ENV_VARS
                .iter()
                .filter_map(|v| knobs::foreign(v))
                .filter(|s| !s.is_empty())
                .map(PathBuf::from)
                .filter(|d| d.is_dir())
                .collect()
        } else {
            Vec::new()
        };
        let install = if system { install_dirs() } else { Vec::new() };
        let kicad_dir = env_dirs.first().or(install.first()).cloned();
        let mut project_tables = Vec::new();
        let mut seen = BTreeSet::new();
        for d in project_dirs {
            let f = d.join("fp-lib-table");
            if seen.insert(f.clone())
                && let Ok(text) = std::fs::read_to_string(&f)
            {
                project_tables.extend(parse_lib_table(&text, d, kicad_dir.as_deref()));
            }
        }
        let mut global_tables = Vec::new();
        if system {
            let prj = project_dirs.first().cloned().unwrap_or_default();
            for f in global_table_files() {
                if let Ok(text) = std::fs::read_to_string(&f) {
                    global_tables.extend(parse_lib_table(&text, &prj, kicad_dir.as_deref()));
                }
            }
        }
        FootprintLibs {
            explicit: explicit.to_vec(),
            project_tables,
            env_dirs,
            global_tables,
            install_dirs: install,
            cache: HashMap::new(),
        }
    }

    /// The `.pretty` directory of a library nickname.
    pub fn library_dir(&mut self, lib: &str) -> Option<PathBuf> {
        if let Some(d) = self.cache.get(lib) {
            return d.clone();
        }
        let pretty = format!("{lib}.pretty");
        let in_dir = |d: &Path| -> Option<PathBuf> {
            if d.extension().is_some_and(|e| e == "pretty") && d.file_stem().is_some_and(|s| s == lib) {
                return Some(d.to_path_buf());
            }
            let p = d.join(&pretty);
            p.is_dir().then_some(p)
        };
        let table = |t: &[(String, PathBuf)]| t.iter().find(|(n, d)| n == lib && d.is_dir()).map(|(_, d)| d.clone());
        let found = self
            .explicit
            .iter()
            .find_map(|d| in_dir(d))
            .or_else(|| table(&self.project_tables))
            .or_else(|| self.env_dirs.iter().find_map(|d| in_dir(d)))
            .or_else(|| table(&self.global_tables))
            .or_else(|| self.install_dirs.iter().find_map(|d| in_dir(d)));
        self.cache.insert(lib.to_string(), found.clone());
        found
    }

    /// The `.kicad_mod` of `lib:name`. A footprint that is not in a library of its nickname is
    /// also looked for in the explicit `.pretty` directories of another name.
    pub fn find(&mut self, lib: &str, name: &str) -> Option<PathBuf> {
        let file = format!("{name}.kicad_mod");
        if let Some(d) = self.library_dir(lib) {
            let p = d.join(&file);
            if p.is_file() {
                return Some(p);
            }
        }
        self.explicit
            .iter()
            .filter(|d| d.extension().is_some_and(|e| e == "pretty"))
            .map(|d| d.join(&file))
            .find(|p| p.is_file())
    }

    /// Where it looks, for error messages.
    pub fn searched(&self) -> Vec<String> {
        let mut v: Vec<String> = self.explicit.iter().map(|d| d.display().to_string()).collect();
        if !self.project_tables.is_empty() {
            v.push(format!(
                "project fp-lib-table ({} libraries)",
                self.project_tables.len()
            ));
        }
        v.extend(self.env_dirs.iter().map(|d| d.display().to_string()));
        if !self.global_tables.is_empty() {
            v.push(format!("global fp-lib-table ({} libraries)", self.global_tables.len()));
        }
        v.extend(self.install_dirs.iter().map(|d| d.display().to_string()));
        v
    }
}

/// Entries of an `fp-lib-table` of type KiCad, with `${VAR}` expanded; entries with variables
/// that cannot be resolved are left out.
fn parse_lib_table(text: &str, prjmod: &Path, kicad_dir: Option<&Path>) -> Vec<(String, PathBuf)> {
    let Ok(root) = sexpr::parse(text) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for lib in root.children("lib") {
        let (Some(name), Some(uri)) = (lib.child_str("name"), lib.child_str("uri")) else {
            continue;
        };
        if lib.child_str("type").is_some_and(|t| !t.eq_ignore_ascii_case("kicad")) {
            continue;
        }
        if let Some(p) = expand_vars(uri, prjmod, kicad_dir) {
            out.push((name.to_string(), PathBuf::from(p)));
        }
    }
    out
}

fn expand_vars(s: &str, prjmod: &Path, kicad_dir: Option<&Path>) -> Option<String> {
    let mut out = String::new();
    let mut rest = s;
    while let Some(i) = rest.find('$') {
        out.push_str(&rest[..i]);
        let after = &rest[i + 1..];
        let close = match after.chars().next() {
            Some('{') => '}',
            Some('(') => ')',
            _ => {
                out.push('$');
                rest = after;
                continue;
            }
        };
        let end = after.find(close)?;
        let var = &after[1..end];
        let value = if var == "KIPRJMOD" {
            prjmod.display().to_string()
        } else if let Some(v) = knobs::foreign(var) {
            v
        } else if var.starts_with("KICAD") && var.ends_with("FOOTPRINT_DIR") {
            kicad_dir?.display().to_string()
        } else {
            return None;
        };
        out.push_str(&value);
        rest = &after[end + 1..];
    }
    out.push_str(rest);
    Some(out)
}

// ------------------------------------------------------------------------------ footprints

struct LibFootprint {
    text: String,
    root: Node,
    version: u64,
    /// Local box [x0, y0, x1, y1]: courtyard, else pads.
    bbox: [f64; 4],
    /// Distinct non-empty pad numbers in file order.
    pads: Vec<String>,
}

fn load_footprint(path: &Path) -> Result<LibFootprint> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let root = sexpr::parse(&text).map_err(|e| anyhow!("{}: parse error: {e}", path.display()))?;
    if !matches!(root.head(), Some("footprint") | Some("module")) {
        bail!("{}: not a footprint", path.display());
    }
    let version = root.child_f64("version").unwrap_or(0.0) as u64;
    let mut pads = Vec::new();
    for p in root.children("pad") {
        let n = p.args().first().and_then(|x| x.as_str()).unwrap_or("");
        if !n.is_empty() && !pads.iter().any(|q| q == n) {
            pads.push(n.to_string());
        }
    }
    let bbox = footprint_bbox(&root);
    Ok(LibFootprint {
        text,
        root,
        version,
        bbox,
        pads,
    })
}

fn xy(n: &Node, k: &str) -> Option<(f64, f64)> {
    let v = n.child_f64s(k)?;
    Some((*v.first()?, *v.get(1)?))
}

fn footprint_bbox(root: &Node) -> [f64; 4] {
    let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    let mut add = |x: f64, y: f64, rx: f64, ry: f64| {
        b[0] = b[0].min(x - rx);
        b[1] = b[1].min(y - ry);
        b[2] = b[2].max(x + rx);
        b[3] = b[3].max(y + ry);
    };
    let mut any = false;
    for g in root.args() {
        let Some(h) = g.head() else { continue };
        if !matches!(h, "fp_line" | "fp_arc" | "fp_rect" | "fp_circle" | "fp_poly") {
            continue;
        }
        if !matches!(g.child_str("layer"), Some("F.CrtYd") | Some("B.CrtYd")) {
            continue;
        }
        if h == "fp_circle" {
            if let (Some(c), Some(e)) = (xy(g, "center"), xy(g, "end")) {
                let r = (e.0 - c.0).hypot(e.1 - c.1);
                add(c.0, c.1, r, r);
                any = true;
            }
            continue;
        }
        for k in ["start", "mid", "end"] {
            if let Some((x, y)) = xy(g, k) {
                add(x, y, 0.0, 0.0);
                any = true;
            }
        }
        for p in g.child("pts").into_iter().flat_map(|p| p.children("xy")) {
            let v: Vec<f64> = p.args().iter().filter_map(|a| a.as_f64()).collect();
            if v.len() >= 2 {
                add(v[0], v[1], 0.0, 0.0);
                any = true;
            }
        }
    }
    if !any {
        for p in root.children("pad") {
            let at = p.child_f64s("at").unwrap_or_default();
            let size = p.child_f64s("size").unwrap_or_default();
            let (x, y) = (at.first().copied().unwrap_or(0.0), at.get(1).copied().unwrap_or(0.0));
            let a = at.get(2).copied().unwrap_or(0.0).to_radians();
            let (w, h) = (
                size.first().copied().unwrap_or(0.0) / 2.0,
                size.get(1).copied().unwrap_or(0.0) / 2.0,
            );
            add(
                x,
                y,
                (w * a.cos()).abs() + (h * a.sin()).abs(),
                (w * a.sin()).abs() + (h * a.cos()).abs(),
            );
            any = true;
        }
    }
    if !any {
        return [-0.5, -0.5, 0.5, 0.5];
    }
    b
}

/// The net of a pad on the board.
#[derive(Debug, Clone)]
struct PadNet {
    name: String,
    code: usize,
    pinfunction: String,
    pintype: String,
}

struct Instance<'a> {
    lib_id: &'a str,
    reference: &'a str,
    value: &'a str,
    datasheet: &'a str,
    description: &'a str,
    fields: &'a [(String, String)],
    path: &'a str,
    sheetname: &'a str,
    sheetfile: &'a str,
    at: (f64, f64),
    locked: bool,
    dnp: bool,
    exclude_from_bom: bool,
    pad_nets: &'a HashMap<String, PadNet>,
    /// `(net "name")` (KiCad 10) instead of `(net 3 "name")`.
    net_by_name: bool,
}

fn net_text(n: &PadNet, by_name: bool) -> String {
    if by_name {
        format!("(net {})", quote(&n.name))
    } else {
        format!("(net {} {})", n.code, quote(&n.name))
    }
}

/// Start of the whitespace in front of byte `i`.
fn ws_start(text: &str, i: usize) -> usize {
    let b = text.as_bytes();
    let mut s = i;
    while s > 0 && b[s - 1].is_ascii_whitespace() {
        s -= 1;
    }
    s
}

fn apply_edits(text: &str, mut edits: Vec<(Range<usize>, String)>) -> String {
    edits.sort_by_key(|(r, _)| (r.start, r.end));
    let mut out = String::with_capacity(text.len() + 1024);
    let mut pos = 0;
    for (r, s) in edits {
        if r.start < pos {
            continue;
        }
        out.push_str(&text[pos..r.start]);
        out.push_str(&s);
        pos = r.end;
    }
    out.push_str(&text[pos..]);
    out
}

/// Fresh uuids for every `(uuid ..)` / `(tstamp ..)` below `n`.
fn fresh_uuids(n: &Node, ids: &mut UuidGen, edits: &mut Vec<(Range<usize>, String)>) {
    for c in n.args() {
        match c.head() {
            Some("uuid") | Some("tstamp") => edits.push((c.span(), format!("(uuid \"{}\")", ids.next()))),
            Some(_) => fresh_uuids(c, ids, edits),
            None => {}
        }
    }
}

fn hidden_property(name: &str, value: &str, ids: &mut UuidGen) -> String {
    format!(
        "\n\t(property {} {}\n\t\t(at 0 0 0)\n\t\t(layer \"F.Fab\")\n\t\t(hide yes)\n\t\t(uuid \"{}\")\n\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1 1)\n\t\t\t\t(thickness 0.15)\n\t\t\t)\n\t\t)\n\t)",
        quote(name),
        quote(value),
        ids.next()
    )
}

/// Board text of one footprint: the library text with the board's fields, one tab deeper.
fn instantiate(fp: &LibFootprint, inst: &Instance, ids: &mut UuidGen) -> String {
    let text = &fp.text;
    let root = &fp.root;
    let mut edits: Vec<(Range<usize>, String)> = Vec::new();
    let items = root.items();
    if root.head() == Some("module") {
        edits.push((items[0].span(), "footprint".into()));
    }
    let args = root.args();
    if let Some(name) = args.first() {
        edits.push((name.span(), quote(inst.lib_id)));
    }
    let mut header_at = args.first().map(|n| n.span().end).unwrap_or(root.span().start + 1);
    let mut has_layer = false;
    let mut props_end = None;
    let mut meta_end = None;
    let mut have_props: BTreeSet<String> = BTreeSet::new();
    let mut attr: Option<&Node> = None;
    let remove = |n: &Node, edits: &mut Vec<(Range<usize>, String)>| {
        edits.push((ws_start(text, n.span().start)..n.span().end, String::new()));
    };
    for n in args.iter().skip(1) {
        match n.head() {
            None => {
                if matches!(n.as_str(), Some("locked") | Some("placed")) {
                    remove(n, &mut edits);
                }
            }
            Some(
                "version" | "generator" | "generator_version" | "at" | "path" | "sheetname" | "sheetfile" | "uuid"
                | "tstamp" | "tedit" | "locked" | "placed",
            ) => remove(n, &mut edits),
            Some("layer") => {
                header_at = n.span().end;
                has_layer = true;
                meta_end.get_or_insert(n.span().end);
            }
            Some("descr") | Some("tags") => meta_end = Some(n.span().end),
            Some("property") => {
                let a = n.args();
                let key = a.first().and_then(|x| x.as_str()).unwrap_or("");
                let new = match key {
                    "Reference" => Some(inst.reference),
                    "Value" => Some(inst.value),
                    "Datasheet" if !inst.datasheet.is_empty() => Some(inst.datasheet),
                    "Description" if !inst.description.is_empty() => Some(inst.description),
                    _ => inst.fields.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str()),
                };
                if let (Some(v), Some(old)) = (new, a.get(1)) {
                    edits.push((old.span(), quote(v)));
                }
                have_props.insert(key.to_string());
                props_end = Some(n.span().end);
                fresh_uuids(n, ids, &mut edits);
            }
            Some("fp_text") => {
                let a = n.args();
                let new = match a.first().and_then(|x| x.as_str()) {
                    Some("reference") => Some(inst.reference),
                    Some("value") => Some(inst.value),
                    _ => None,
                };
                if let (Some(v), Some(old)) = (new, a.get(1)) {
                    edits.push((old.span(), quote(v)));
                }
                fresh_uuids(n, ids, &mut edits);
            }
            Some("attr") => {
                attr = Some(n);
            }
            Some("pad") => {
                let number = n.args().first().and_then(|x| x.as_str()).unwrap_or("");
                let mut last = n.span().end - 1;
                for c in n.args() {
                    last = c.span().end;
                    match c.head() {
                        Some("net" | "uuid" | "tstamp" | "pinfunction" | "pintype") => remove(c, &mut edits),
                        Some(_) => fresh_uuids(c, ids, &mut edits),
                        None => {}
                    }
                }
                let mut ins = String::new();
                if let Some(pn) = (!number.is_empty()).then(|| inst.pad_nets.get(number)).flatten() {
                    let _ = write!(ins, "\n\t\t{}", net_text(pn, inst.net_by_name));
                    if !pn.pinfunction.is_empty() {
                        let _ = write!(ins, "\n\t\t(pinfunction {})", quote(&pn.pinfunction));
                    }
                    if !pn.pintype.is_empty() {
                        let _ = write!(ins, "\n\t\t(pintype {})", quote(&pn.pintype));
                    }
                }
                let _ = write!(ins, "\n\t\t(uuid \"{}\")", ids.next());
                edits.push((last..last, ins));
            }
            Some(_) => fresh_uuids(n, ids, &mut edits),
        }
    }
    // header: locked, layer, uuid, at
    let mut head = String::new();
    if inst.locked {
        head.push_str("\n\t(locked yes)");
    }
    if !has_layer {
        head.push_str("\n\t(layer \"F.Cu\")");
    }
    let _ = write!(
        head,
        "\n\t(uuid \"{}\")\n\t(at {} {})",
        ids.next(),
        f(inst.at.0),
        f(inst.at.1)
    );
    edits.push((header_at..header_at, head));
    // properties the footprint does not have, then the link to the symbol
    let mut tail = String::new();
    for (k, v) in [("Datasheet", inst.datasheet), ("Description", inst.description)] {
        if !v.is_empty() && !have_props.contains(k) {
            tail.push_str(&hidden_property(k, v, ids));
        }
    }
    for (k, v) in inst.fields {
        if !have_props.contains(k) && !k.starts_with("ki_") {
            tail.push_str(&hidden_property(k, v, ids));
        }
    }
    if !inst.path.is_empty() {
        let _ = write!(tail, "\n\t(path {})", quote(inst.path));
    }
    if !inst.sheetname.is_empty() || !inst.sheetfile.is_empty() {
        let _ = write!(
            tail,
            "\n\t(sheetname {})\n\t(sheetfile {})",
            quote(inst.sheetname),
            quote(inst.sheetfile)
        );
    }
    let mut flags: Vec<&str> = Vec::new();
    if inst.exclude_from_bom {
        flags.push("exclude_from_bom");
    }
    if inst.dnp {
        flags.push("dnp");
    }
    match attr {
        Some(a) => {
            let have: Vec<&str> = a.args().iter().filter_map(|x| x.as_str()).collect();
            let add: String = flags
                .iter()
                .filter(|fl| !have.contains(fl))
                .map(|fl| format!(" {fl}"))
                .collect();
            if !add.is_empty() {
                let end = a.span().end - 1;
                edits.push((end..end, add));
            }
        }
        None if !flags.is_empty() => {
            let _ = write!(tail, "\n\t(attr {})", flags.join(" "));
        }
        None => {}
    }
    let at = props_end.or(meta_end).unwrap_or(header_at);
    edits.push((at..at, tail));
    let body = apply_edits(text, edits);
    let span = root.span();
    let shift = body.len() as isize - text.len() as isize;
    let body = &body[span.start..(span.end as isize + shift) as usize];
    format!("\t{}", body.replace('\n', "\n\t"))
}

fn f(v: f64) -> String {
    let s = format!("{:.6}", (v * 1e6).round() / 1e6);
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.to_string() }
}

// ------------------------------------------------------------------------------ options

#[derive(Debug, Clone)]
pub struct HoleSpec {
    /// Drill diameter in mm.
    pub diameter: f64,
    /// Distance of the hole centre from the two edges of its corner.
    pub inset: f64,
    /// Library footprint instead of a plain non-plated hole, e.g.
    /// `MountingHole:MountingHole_3.2mm_M3_Pad_Via`.
    pub footprint: Option<String>,
    /// Net for the pads of a plated hole footprint.
    pub net: Option<String>,
}

#[derive(Debug, Clone, Default)]
pub struct ClassSpec {
    pub name: String,
    pub clearance: Option<f64>,
    pub track_width: Option<f64>,
    pub via_diameter: Option<f64>,
    pub via_drill: Option<f64>,
    pub diff_pair_width: Option<f64>,
    pub diff_pair_gap: Option<f64>,
    pub priority: Option<i32>,
    /// Net name patterns (`*`, `?`) of the class.
    pub patterns: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct CreateOptions {
    pub width: f64,
    pub height: f64,
    pub x0: f64,
    pub y0: f64,
    pub corner_radius: f64,
    /// Copper layer count (even, 2 to 32).
    pub layers: usize,
    /// Inner layers of KiCad type "power" (planes, not routed by Gummiband).
    pub plane_layers: Vec<String>,
    pub thickness: f64,
    /// Copper weight per layer from F.Cu to B.Cu, or one value for all.
    pub copper_oz: Vec<f64>,
    /// Prepreg thickness for boards with 4 or more layers.
    pub prepreg: f64,
    pub mounting_holes: Option<HoleSpec>,
    pub net_classes: Vec<ClassSpec>,
    /// Design rules for `board.design_settings.rules` in the `.kicad_pro`.
    pub rules: BTreeMap<String, f64>,
    pub footprint_dirs: Vec<PathBuf>,
    /// Reference -> (netlist pin -> footprint pad).
    pub pin_aliases: BTreeMap<String, BTreeMap<String, String>>,
    /// Also search the KiCad environment variables, global table and install places.
    pub system_libraries: bool,
    /// Put every loose part in rows below the board instead of inside the outline, so that
    /// copper drawn before the placement (add_locked_copper, zones) meets no pile of parts.
    pub parts_outside: bool,
}

impl Default for CreateOptions {
    fn default() -> Self {
        CreateOptions {
            width: 100.0,
            height: 80.0,
            x0: 100.0,
            y0: 100.0,
            corner_radius: 0.0,
            layers: 2,
            plane_layers: Vec::new(),
            thickness: 1.6,
            copper_oz: vec![1.0],
            prepreg: 0.21,
            mounting_holes: None,
            net_classes: Vec::new(),
            rules: BTreeMap::new(),
            footprint_dirs: Vec::new(),
            pin_aliases: BTreeMap::new(),
            system_libraries: true,
            parts_outside: false,
        }
    }
}

// ------------------------------------------------------------------------------ report

#[derive(Debug, Clone)]
pub struct MissingFootprint {
    pub reference: String,
    pub footprint: String,
    pub reason: String,
}

#[derive(Debug, Clone)]
pub struct Unmatched {
    pub reference: String,
    pub footprint: String,
    /// Pad numbers without a netlist pin (they stay without a net).
    pub pads: Vec<String>,
    /// Netlist pins without a pad, with their net (the net misses that connection).
    pub pins: Vec<(String, String)>,
    /// (pin, pad) when exactly one of each is left: the likely `pin_aliases` entry.
    pub suggested_alias: Option<(String, String)>,
}

#[derive(Debug, Clone, Default)]
pub struct CreateReport {
    pub pcb_path: PathBuf,
    pub pro_path: PathBuf,
    /// The `.kicad_pro` existed and was merged instead of written new.
    pub pro_merged: bool,
    pub version: u64,
    /// Footprints from the netlist on the board.
    pub footprints: usize,
    pub holes: usize,
    pub nets: usize,
    pub pads: usize,
    pub pads_with_net: usize,
    pub missing_footprints: Vec<MissingFootprint>,
    pub unmatched: Vec<Unmatched>,
    /// Library nickname -> directory it was read from.
    pub libraries: BTreeMap<String, String>,
    /// Symbols marked "exclude from board".
    pub excluded: Vec<String>,
    /// Parts that did not fit in rows inside the outline; they sit in rows below the board.
    pub overflow: Vec<String>,
    /// With `parts_outside`: the parts put in rows below the board on purpose.
    pub parked: Vec<String>,
    /// With overwrite: stale working copies of an earlier board that were removed.
    pub removed: Vec<PathBuf>,
    /// Gap between parts in the rows.
    pub grid_gap: f64,
    pub warnings: Vec<String>,
}

// ------------------------------------------------------------------------------ board

fn copper_names(n: usize) -> Vec<String> {
    let mut v = vec!["F.Cu".to_string()];
    v.extend((1..n - 1).map(|i| format!("In{i}.Cu")));
    v.push("B.Cu".into());
    v
}

fn layers_block(names: &[String], planes: &[String]) -> String {
    let mut s = String::from("\t(layers\n");
    let n = names.len();
    for (i, name) in names.iter().enumerate() {
        let id = if i == 0 {
            0
        } else if i == n - 1 {
            2
        } else {
            2 + 2 * i
        };
        let kind = if planes.contains(name) { "power" } else { "signal" };
        let _ = writeln!(s, "\t\t({id} {} {kind})", quote(name));
    }
    s.push_str(
        "\t\t(9 \"F.Adhes\" user \"F.Adhesive\")\n\t\t(11 \"B.Adhes\" user \"B.Adhesive\")\n\
         \t\t(13 \"F.Paste\" user)\n\t\t(15 \"B.Paste\" user)\n\
         \t\t(5 \"F.SilkS\" user \"F.Silkscreen\")\n\t\t(7 \"B.SilkS\" user \"B.Silkscreen\")\n\
         \t\t(1 \"F.Mask\" user)\n\t\t(3 \"B.Mask\" user)\n\
         \t\t(17 \"Dwgs.User\" user \"User.Drawings\")\n\t\t(19 \"Cmts.User\" user \"User.Comments\")\n\
         \t\t(21 \"Eco1.User\" user \"User.Eco1\")\n\t\t(23 \"Eco2.User\" user \"User.Eco2\")\n\
         \t\t(25 \"Edge.Cuts\" user)\n\t\t(27 \"Margin\" user)\n\
         \t\t(31 \"F.CrtYd\" user \"F.Courtyard\")\n\t\t(29 \"B.CrtYd\" user \"B.Courtyard\")\n\
         \t\t(35 \"F.Fab\" user)\n\t\t(33 \"B.Fab\" user)\n\
         \t\t(39 \"User.1\" user)\n\t\t(41 \"User.2\" user)\n\t\t(43 \"User.3\" user)\n\t\t(45 \"User.4\" user)\n\t)\n",
    );
    s
}

const MASK: f64 = 0.01;

fn stackup_block(names: &[String], opts: &CreateOptions) -> Result<String> {
    let n = names.len();
    let oz: Vec<f64> = match opts.copper_oz.len() {
        0 => vec![1.0; n],
        1 => vec![opts.copper_oz[0]; n],
        k if k == n => opts.copper_oz.clone(),
        k => bail!("copper_oz has {k} values; give one for all layers or one per layer ({n})"),
    };
    if oz.iter().any(|o| *o <= 0.0) {
        bail!("copper_oz must be positive");
    }
    let cu: Vec<f64> = oz.iter().map(|o| (o * 0.035 * 1e4).round() / 1e4).collect();
    let rest = opts.thickness - 2.0 * MASK - cu.iter().sum::<f64>();
    let prepregs = if n == 2 { 0 } else { n / 2 };
    let cores = if n == 2 { 1 } else { n / 2 - 1 };
    let core = (rest - prepregs as f64 * opts.prepreg) / cores as f64;
    if core < 0.05 {
        bail!(
            "thickness {} mm is too thin for {n} layers with {} mm prepreg (core would be {:.3} mm)",
            opts.thickness,
            opts.prepreg,
            core
        );
    }
    let core = (core * 1e4).round() / 1e4;
    let mut s = String::from(
        "\t\t(stackup\n\t\t\t(layer \"F.SilkS\" (type \"Top Silk Screen\"))\n\t\t\t(layer \"F.Paste\" (type \"Top Solder Paste\"))\n",
    );
    let _ = writeln!(
        s,
        "\t\t\t(layer \"F.Mask\" (type \"Top Solder Mask\") (thickness {}))",
        f(MASK)
    );
    for (i, name) in names.iter().enumerate() {
        let _ = writeln!(
            s,
            "\t\t\t(layer {} (type \"copper\") (thickness {}))",
            quote(name),
            f(cu[i])
        );
        if i + 1 < n {
            let d = i + 1;
            let (kind, t, eps) = if n > 2 && d % 2 == 1 {
                ("prepreg", opts.prepreg, 4.4)
            } else {
                ("core", core, 4.5)
            };
            let _ = writeln!(
                s,
                "\t\t\t(layer \"dielectric {d}\" (type \"{kind}\") (thickness {}) (material \"FR4\") (epsilon_r {eps}) (loss_tangent 0.02))",
                f(t)
            );
        }
    }
    let _ = writeln!(
        s,
        "\t\t\t(layer \"B.Mask\" (type \"Bottom Solder Mask\") (thickness {}))",
        f(MASK)
    );
    s.push_str(
        "\t\t\t(layer \"B.Paste\" (type \"Bottom Solder Paste\"))\n\t\t\t(layer \"B.SilkS\" (type \"Bottom Silk Screen\"))\n\
         \t\t\t(copper_finish \"None\")\n\t\t\t(dielectric_constraints no)\n\t\t)\n",
    );
    Ok(s)
}

fn edge_items(opts: &CreateOptions, ids: &mut UuidGen) -> String {
    let (x0, y0) = (opts.x0, opts.y0);
    let (x1, y1) = (x0 + opts.width, y0 + opts.height);
    let r = opts.corner_radius.clamp(0.0, opts.width.min(opts.height) / 2.0);
    let stroke = "\t\t(stroke\n\t\t\t(width 0.05)\n\t\t\t(type default)\n\t\t)\n\t\t(layer \"Edge.Cuts\")\n";
    let mut s = String::new();
    let mut line = |a: (f64, f64), b: (f64, f64), s: &mut String| {
        if (a.0 - b.0).abs() + (a.1 - b.1).abs() < 1e-9 {
            return;
        }
        let _ = writeln!(
            s,
            "\t(gr_line\n\t\t(start {} {})\n\t\t(end {} {})\n{stroke}\t\t(uuid \"{}\")\n\t)",
            f(a.0),
            f(a.1),
            f(b.0),
            f(b.1),
            ids.next()
        );
    };
    line((x0 + r, y0), (x1 - r, y0), &mut s);
    line((x1, y0 + r), (x1, y1 - r), &mut s);
    line((x1 - r, y1), (x0 + r, y1), &mut s);
    line((x0, y1 - r), (x0, y0 + r), &mut s);
    if r > 0.0 {
        let k = r * (1.0 - std::f64::consts::FRAC_1_SQRT_2);
        // start, mid, end clockwise in board coordinates (y down)
        let arcs = [
            ((x0, y0 + r), (x0 + k, y0 + k), (x0 + r, y0)),
            ((x1 - r, y0), (x1 - k, y0 + k), (x1, y0 + r)),
            ((x1, y1 - r), (x1 - k, y1 - k), (x1 - r, y1)),
            ((x0 + r, y1), (x0 + k, y1 - k), (x0, y1 - r)),
        ];
        for (a, m, b) in arcs {
            let _ = writeln!(
                s,
                "\t(gr_arc\n\t\t(start {} {})\n\t\t(mid {} {})\n\t\t(end {} {})\n{stroke}\t\t(uuid \"{}\")\n\t)",
                f(a.0),
                f(a.1),
                f(m.0),
                f(m.1),
                f(b.0),
                f(b.1),
                ids.next()
            );
        }
    }
    s
}

/// A plain non-plated mounting hole like KiCad's `MountingHole_3.2mm_M3`.
fn hole_footprint(d: f64) -> LibFootprint {
    let head = d;
    let court = d + 0.25;
    let text = format!(
        "(footprint \"MountingHole_{d}mm\"\n\t(layer \"F.Cu\")\n\t(descr \"Mounting hole {d} mm, no annular ring\")\n\t(tags \"mounting hole\")\n\
         \t(property \"Reference\" \"REF**\"\n\t\t(at 0 {} 0)\n\t\t(layer \"F.SilkS\")\n\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1 1)\n\t\t\t\t(thickness 0.15)\n\t\t\t)\n\t\t)\n\t)\n\
         \t(property \"Value\" \"MountingHole\"\n\t\t(at 0 {} 0)\n\t\t(layer \"F.Fab\")\n\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1 1)\n\t\t\t\t(thickness 0.15)\n\t\t\t)\n\t\t)\n\t)\n\
         \t(attr exclude_from_pos_files exclude_from_bom)\n\
         \t(fp_circle\n\t\t(center 0 0)\n\t\t(end {} 0)\n\t\t(stroke\n\t\t\t(width 0.15)\n\t\t\t(type solid)\n\t\t)\n\t\t(fill no)\n\t\t(layer \"Cmts.User\")\n\t)\n\
         \t(fp_circle\n\t\t(center 0 0)\n\t\t(end {} 0)\n\t\t(stroke\n\t\t\t(width 0.05)\n\t\t\t(type solid)\n\t\t)\n\t\t(fill no)\n\t\t(layer \"F.CrtYd\")\n\t)\n\
         \t(pad \"\" np_thru_hole circle\n\t\t(at 0 0)\n\t\t(size {d} {d})\n\t\t(drill {d})\n\t\t(layers \"*.Cu\" \"*.Mask\")\n\t)\n)\n",
        f(-(court + 0.75)),
        f(court + 0.75),
        f(head),
        f(court),
    );
    let root = sexpr::parse(&text).expect("hole footprint");
    let bbox = footprint_bbox(&root);
    LibFootprint {
        text,
        root,
        version: 0,
        bbox,
        pads: Vec::new(),
    }
}

/// Board text of a locked, plain non-plated mounting hole of `d` mm at `at` (one tab deep, as
/// the root items of a board).
pub(crate) fn hole_text(d: f64, reference: &str, at: (f64, f64), seed: u64) -> String {
    let fp = hole_footprint(d);
    let lib_id = format!("MountingHole:MountingHole_{}mm", f(d));
    let nets = HashMap::new();
    let inst = Instance {
        lib_id: &lib_id,
        reference,
        value: "MountingHole",
        datasheet: "",
        description: "",
        fields: &[],
        path: "",
        sheetname: "",
        sheetfile: "",
        at,
        locked: true,
        dnp: false,
        exclude_from_bom: false,
        pad_nets: &nets,
        net_by_name: false,
    };
    instantiate(&fp, &inst, &mut UuidGen::new(seed))
}

/// Parts inside `area` around the `keepouts`, each at the topmost, then leftmost free spot
/// next to the parts placed before it (larger parts first); None if a part does not fit.
/// Items: (width, height). Returns the top left corner of each.
fn pack(items: &[(f64, f64)], area: [f64; 4], keepouts: &[[f64; 4]], gap: f64) -> Vec<Option<(f64, f64)>> {
    const EPS: f64 = 1e-9;
    let mut order: Vec<usize> = (0..items.len()).collect();
    order.sort_by(|&a, &b| {
        let (wa, ha) = items[a];
        let (wb, hb) = items[b];
        hb.total_cmp(&ha).then((wb * hb).total_cmp(&(wa * ha))).then(a.cmp(&b))
    });
    let mut out = vec![None; items.len()];
    // placed parts grown by the gap, and the keepouts
    let mut blocked: Vec<[f64; 4]> = keepouts.to_vec();
    let mut xs: Vec<f64> = vec![area[0]];
    let mut ys: Vec<f64> = vec![area[1]];
    for k in keepouts {
        xs.push(k[2]);
        ys.push(k[3]);
    }
    for i in order {
        let (w, h) = items[i];
        let mut best: Option<(f64, f64)> = None;
        for &y in &ys {
            if y < area[1] - EPS || y + h > area[3] + EPS || best.is_some_and(|b| b.1 < y - EPS) {
                continue;
            }
            for &x in &xs {
                if x < area[0] - EPS || x + w > area[2] + EPS {
                    continue;
                }
                if best.is_some_and(|b| (b.1, b.0) <= (y, x)) {
                    continue;
                }
                let r = [x, y, x + w, y + h];
                if blocked
                    .iter()
                    .any(|k| r[0] < k[2] - EPS && r[2] > k[0] + EPS && r[1] < k[3] - EPS && r[3] > k[1] + EPS)
                {
                    continue;
                }
                best = Some((x, y));
            }
        }
        if let Some((x, y)) = best {
            out[i] = Some((x, y));
            blocked.push([x - gap, y - gap, x + w + gap, y + h + gap]);
            xs.push(x + w + gap);
            ys.push(y + h + gap);
        }
    }
    out
}

/// The board text and its report (without paths). `seed` makes the uuids differ per netlist.
pub fn build_board(
    netlist: &Netlist,
    libs: &mut FootprintLibs,
    opts: &CreateOptions,
    seed: u64,
) -> Result<(String, CreateReport, BTreeMap<String, String>)> {
    if !(opts.width > 0.0 && opts.height > 0.0) {
        bail!("width and height must be positive");
    }
    if opts.layers < 2 || opts.layers > 32 || !opts.layers.is_multiple_of(2) {
        bail!("layers must be an even number from 2 to 32, got {}", opts.layers);
    }
    let names = copper_names(opts.layers);
    for p in &opts.plane_layers {
        if !names.contains(p) {
            bail!(
                "plane_layers: {p} is not a copper layer of this board ({})",
                names.join(", ")
            );
        }
        if p == "F.Cu" || p == "B.Cu" {
            bail!("plane_layers: {p} is an outer layer; only inner layers can be power planes");
        }
    }
    let mut ids = UuidGen::new(seed);
    let mut rep = CreateReport::default();

    // nets: code by netlist order, pins by (reference, pin)
    let mut pin_net: HashMap<(&str, &str), PadNet> = HashMap::new();
    let mut comp_pins: HashMap<&str, Vec<&str>> = HashMap::new();
    for (i, n) in netlist.nets.iter().enumerate() {
        for node in &n.nodes {
            let mut pintype = node.pintype.clone();
            if n.name.starts_with("unconnected-(") && !pintype.is_empty() && !pintype.contains("no_connect") {
                pintype.push_str("+no_connect");
            }
            pin_net.insert(
                (node.reference.as_str(), node.pin.as_str()),
                PadNet {
                    name: n.name.clone(),
                    code: i + 1,
                    pinfunction: node.pinfunction.clone(),
                    pintype,
                },
            );
            comp_pins
                .entry(node.reference.as_str())
                .or_default()
                .push(node.pin.as_str());
        }
    }
    rep.nets = netlist.nets.len();
    for r in opts.pin_aliases.keys() {
        if !netlist.components.iter().any(|c| &c.reference == r) {
            rep.warnings
                .push(format!("pin_aliases: no component {r} in the netlist"));
        }
    }

    // footprints
    let mut loaded: HashMap<String, Option<usize>> = HashMap::new();
    let mut fps: Vec<LibFootprint> = Vec::new();
    struct Part<'a> {
        comp: Option<&'a Component>,
        fp: usize,
        lib_id: String,
        reference: String,
        value: String,
        pad_nets: HashMap<String, PadNet>,
        at: Option<(f64, f64)>,
        locked: bool,
    }
    let mut parts: Vec<Part> = Vec::new();
    for c in &netlist.components {
        if c.exclude_from_board {
            rep.excluded.push(c.reference.clone());
            continue;
        }
        let missing = |reason: String| MissingFootprint {
            reference: c.reference.clone(),
            footprint: c.footprint.clone(),
            reason,
        };
        let Some((lib, name)) = c.footprint.split_once(':') else {
            let reason = if c.footprint.is_empty() {
                "the symbol has no footprint".to_string()
            } else {
                "no library nickname (expected Library:Footprint)".to_string()
            };
            rep.missing_footprints.push(missing(reason));
            continue;
        };
        let idx = match loaded.get(&c.footprint) {
            Some(i) => *i,
            None => {
                let i = match libs.find(lib, name) {
                    Some(path) => match load_footprint(&path) {
                        Ok(fp) => {
                            if let Some(d) = path.parent() {
                                rep.libraries.insert(lib.to_string(), d.display().to_string());
                            }
                            fps.push(fp);
                            Some(fps.len() - 1)
                        }
                        Err(e) => {
                            rep.warnings.push(format!("{}: {e:#}", c.footprint));
                            None
                        }
                    },
                    None => None,
                };
                loaded.insert(c.footprint.clone(), i);
                i
            }
        };
        let Some(idx) = idx else {
            let reason = match libs.library_dir(lib) {
                Some(d) => format!("{name}.kicad_mod is not in {}", d.display()),
                None => format!("library {lib} not found"),
            };
            rep.missing_footprints.push(missing(reason));
            continue;
        };
        let fp = &fps[idx];
        // pads: pin numbers, with the aliases
        let aliases = opts.pin_aliases.get(&c.reference);
        let pins: Vec<&str> = comp_pins.get(c.reference.as_str()).cloned().unwrap_or_default();
        let mut pad_nets: HashMap<String, PadNet> = HashMap::new();
        let mut used_pins: BTreeSet<&str> = BTreeSet::new();
        if let Some(a) = aliases {
            for (pin, pad) in a {
                if !pins.contains(&pin.as_str()) {
                    rep.warnings.push(format!(
                        "pin_aliases: {} has no netlist pin {pin} (pins: {})",
                        c.reference,
                        pins.join(", ")
                    ));
                    continue;
                }
                if !fp.pads.contains(pad) {
                    rep.warnings.push(format!(
                        "pin_aliases: footprint {} of {} has no pad {pad} (pads: {})",
                        c.footprint,
                        c.reference,
                        fp.pads.join(", ")
                    ));
                    continue;
                }
                if let Some(n) = pin_net.get(&(c.reference.as_str(), pin.as_str())) {
                    pad_nets.insert(pad.clone(), n.clone());
                    used_pins.insert(pin.as_str());
                }
            }
        }
        let mut lone_pads = Vec::new();
        for pad in &fp.pads {
            if pad_nets.contains_key(pad) {
                continue;
            }
            match pin_net.get(&(c.reference.as_str(), pad.as_str())) {
                Some(n) => {
                    pad_nets.insert(pad.clone(), n.clone());
                    used_pins.insert(pins.iter().find(|p| *p == pad).copied().unwrap_or(""));
                }
                None => lone_pads.push(pad.clone()),
            }
        }
        let mut lone_pins: Vec<(String, String)> = Vec::new();
        for p in &pins {
            if !used_pins.contains(p) && !lone_pins.iter().any(|(q, _)| q == p) {
                let net = pin_net
                    .get(&(c.reference.as_str(), *p))
                    .map(|n| n.name.clone())
                    .unwrap_or_default();
                lone_pins.push((p.to_string(), net));
            }
        }
        if !lone_pads.is_empty() || !lone_pins.is_empty() {
            let suggested =
                (lone_pads.len() == 1 && lone_pins.len() == 1).then(|| (lone_pins[0].0.clone(), lone_pads[0].clone()));
            let mut w = format!("{} ({}):", c.reference, c.footprint);
            if !lone_pads.is_empty() {
                let _ = write!(
                    w,
                    " pad{} {} {} no netlist pin and stay{} without a net;",
                    if lone_pads.len() > 1 { "s" } else { "" },
                    lone_pads.join(", "),
                    if lone_pads.len() > 1 { "have" } else { "has" },
                    if lone_pads.len() > 1 { "" } else { "s" },
                );
            }
            if !lone_pins.is_empty() {
                let list: Vec<String> = lone_pins.iter().map(|(p, n)| format!("{p} ({n})")).collect();
                let _ = write!(
                    w,
                    " netlist pin{} {} without a pad, the net misses {};",
                    if lone_pins.len() > 1 { "s" } else { "" },
                    list.join(", "),
                    if lone_pins.len() > 1 {
                        "these connections"
                    } else {
                        "this connection"
                    },
                );
            }
            if let Some((pin, pad)) = &suggested {
                let _ = write!(
                    w,
                    " if pin {pin} is pad {pad}, pass pin_aliases {{\"{}\": {{\"{pin}\": \"{pad}\"}}}}",
                    c.reference
                );
            }
            rep.warnings.push(w.trim_end_matches(';').to_string());
            rep.unmatched.push(Unmatched {
                reference: c.reference.clone(),
                footprint: c.footprint.clone(),
                pads: lone_pads,
                pins: lone_pins,
                suggested_alias: suggested,
            });
        }
        parts.push(Part {
            comp: Some(c),
            fp: idx,
            lib_id: c.footprint.clone(),
            reference: c.reference.clone(),
            value: c.value.clone(),
            pad_nets,
            at: None,
            locked: false,
        });
    }
    rep.footprints = parts.len();

    // mounting holes at the corners, locked
    let (x0, y0, x1, y1) = (opts.x0, opts.y0, opts.x0 + opts.width, opts.y0 + opts.height);
    let mut keepouts: Vec<[f64; 4]> = Vec::new();
    if let Some(h) = &opts.mounting_holes {
        if h.diameter <= 0.0 {
            bail!("mounting_holes: diameter must be positive");
        }
        if 2.0 * h.inset >= opts.width.min(opts.height) {
            bail!("mounting_holes: inset {} mm leaves no board between the holes", h.inset);
        }
        let (idx, lib_id) = match &h.footprint {
            Some(id) => {
                let (lib, name) = id
                    .split_once(':')
                    .ok_or_else(|| anyhow!("mounting_holes: footprint {id:?} is not Library:Footprint"))?;
                let path = libs
                    .find(lib, name)
                    .ok_or_else(|| anyhow!("mounting_holes: footprint {id} not found"))?;
                fps.push(load_footprint(&path)?);
                (fps.len() - 1, id.clone())
            }
            None => {
                fps.push(hole_footprint(h.diameter));
                (fps.len() - 1, format!("MountingHole:MountingHole_{}mm", f(h.diameter)))
            }
        };
        let net = match &h.net {
            Some(n) => match netlist.nets.iter().position(|x| &x.name == n) {
                Some(i) => Some(PadNet {
                    name: n.clone(),
                    code: i + 1,
                    pinfunction: String::new(),
                    pintype: String::new(),
                }),
                None => {
                    rep.warnings.push(format!(
                        "mounting_holes: net {n} is not in the netlist; the holes have no net"
                    ));
                    None
                }
            },
            None => None,
        };
        let pad_nets: HashMap<String, PadNet> = match &net {
            Some(n) => fps[idx].pads.iter().map(|p| (p.clone(), n.clone())).collect(),
            None => HashMap::new(),
        };
        let used: BTreeSet<&str> = netlist.components.iter().map(|c| c.reference.as_str()).collect();
        let mut k = 0;
        let b = fps[idx].bbox;
        for (x, y) in [
            (x0 + h.inset, y0 + h.inset),
            (x1 - h.inset, y0 + h.inset),
            (x0 + h.inset, y1 - h.inset),
            (x1 - h.inset, y1 - h.inset),
        ] {
            let reference = loop {
                k += 1;
                let r = format!("H{k}");
                if !used.contains(r.as_str()) {
                    break r;
                }
            };
            keepouts.push([x + b[0], y + b[1], x + b[2], y + b[3]]);
            parts.push(Part {
                comp: None,
                fp: idx,
                lib_id: lib_id.clone(),
                reference,
                value: "MountingHole".into(),
                pad_nets: pad_nets.clone(),
                at: Some((x, y)),
                locked: true,
            });
            rep.holes += 1;
        }
    }

    // rows inside the outline
    let margin = 1.0 + opts.corner_radius.max(0.0) * (1.0 - std::f64::consts::FRAC_1_SQRT_2);
    let area = [x0 + margin, y0 + margin, x1 - margin, y1 - margin];
    let loose: Vec<usize> = (0..parts.len()).filter(|&i| parts[i].at.is_none()).collect();
    let sizes: Vec<(f64, f64)> = loose
        .iter()
        .map(|&i| {
            let b = fps[parts[i].fp].bbox;
            (b[2] - b[0], b[3] - b[1])
        })
        .collect();
    type Placement = Vec<Option<(f64, f64)>>;
    let mut best: Option<(f64, Placement)> = None;
    for gap in [1.0, 0.5, 0.25, 0.0] {
        if opts.parts_outside {
            best = Some((1.0, vec![None; sizes.len()]));
            break;
        }
        let k: Vec<[f64; 4]> = keepouts
            .iter()
            .map(|k| [k[0] - gap, k[1] - gap, k[2] + gap, k[3] + gap])
            .collect();
        let pos = pack(&sizes, area, &k, gap);
        let fit = pos.iter().all(|p| p.is_some());
        let better = best
            .as_ref()
            .is_none_or(|(_, b)| pos.iter().filter(|p| p.is_some()).count() > b.iter().filter(|p| p.is_some()).count());
        if fit || better {
            best = Some((gap, pos));
        }
        if fit {
            break;
        }
    }
    let (gap, pos) = best.unwrap_or((0.0, Vec::new()));
    rep.grid_gap = gap;
    let centre = ((x0 + x1) / 2.0, (y0 + y1) / 2.0);
    // what does not fit goes into rows below the board, for place_parts (scope "outside")
    let rest: Vec<usize> = (0..loose.len())
        .filter(|&j| pos.get(j).copied().flatten().is_none())
        .collect();
    let widest = rest.iter().map(|&j| sizes[j].0).fold(0.0, f64::max);
    let below = pack(
        &rest.iter().map(|&j| sizes[j]).collect::<Vec<_>>(),
        [x0, y1 + 5.0, x0 + opts.width.max(widest), f64::INFINITY],
        &[],
        1.0,
    );
    for (j, &i) in loose.iter().enumerate() {
        let b = fps[parts[i].fp].bbox;
        let at = match pos.get(j).copied().flatten() {
            Some(p) => Some(p),
            None => {
                rep.overflow.push(parts[i].reference.clone());
                rest.iter().position(|&k| k == j).and_then(|k| below[k])
            }
        };
        parts[i].at = Some(match at {
            Some((x, y)) => (x - b[0], y - b[1]),
            None => (centre.0 - (b[0] + b[2]) / 2.0, centre.1 - (b[1] + b[3]) / 2.0),
        });
    }
    if opts.parts_outside {
        rep.parked = std::mem::take(&mut rep.overflow);
    } else if !rep.overflow.is_empty() {
        let court: f64 = sizes.iter().map(|(w, h)| w * h).sum();
        rep.warnings.push(format!(
            "{} parts did not fit in rows inside the outline (the courtyards of the loose parts take {:.0} mm2, the board has {:.0} mm2); \
             they sit in rows below the board, for move_part or place_parts with scope \"outside\": {}",
            rep.overflow.len(),
            court,
            opts.width * opts.height,
            rep.overflow.join(", ")
        ));
    }

    // file version: KiCad 9 unless a footprint is newer
    let newest = fps.iter().map(|f| f.version).max().unwrap_or(0);
    rep.version = if newest <= KICAD9_VERSION {
        KICAD9_VERSION
    } else {
        newest.max(KICAD10_VERSION)
    };
    let by_name = rep.version > KICAD9_VERSION;

    let mut out = String::new();
    let _ = write!(
        out,
        "(kicad_pcb\n\t(version {})\n\t(generator \"gummiband\")\n\t(generator_version \"{}\")\n\t(general\n\t\t(thickness {})\n\t\t(legacy_teardrops no)\n\t)\n",
        rep.version,
        if by_name { "10.0" } else { "9.0" },
        f(opts.thickness)
    );
    let paper = [
        ("A4", 297.0, 210.0),
        ("A3", 420.0, 297.0),
        ("A2", 594.0, 420.0),
        ("A1", 841.0, 594.0),
    ]
    .iter()
    .find(|(_, w, h)| x1 + 10.0 <= *w && y1 + 10.0 <= *h)
    .map(|(n, _, _)| *n)
    .unwrap_or("A0");
    let _ = writeln!(out, "\t(paper \"{paper}\")");
    out.push_str(&layers_block(&names, &opts.plane_layers));
    out.push_str("\t(setup\n");
    out.push_str(&stackup_block(&names, opts)?);
    out.push_str("\t\t(pad_to_mask_clearance 0)\n\t\t(allow_soldermask_bridges_in_footprints no)\n\t)\n");
    if !by_name {
        out.push_str("\t(net 0 \"\")\n");
        for (i, n) in netlist.nets.iter().enumerate() {
            let _ = writeln!(out, "\t(net {} {})", i + 1, quote(&n.name));
        }
    }
    for p in &parts {
        let fp = &fps[p.fp];
        let c = p.comp;
        let empty: &[(String, String)] = &[];
        let inst = Instance {
            lib_id: &p.lib_id,
            reference: &p.reference,
            value: &p.value,
            datasheet: c.map(|c| c.datasheet.as_str()).unwrap_or(""),
            description: c.map(|c| c.description.as_str()).unwrap_or(""),
            fields: c.map(|c| c.fields.as_slice()).unwrap_or(empty),
            path: c.map(|c| c.path.as_str()).unwrap_or(""),
            sheetname: c.map(|c| c.sheetname.as_str()).unwrap_or(""),
            sheetfile: c.map(|c| c.sheetfile.as_str()).unwrap_or(""),
            at: p.at.unwrap_or(centre),
            locked: p.locked,
            dnp: c.is_some_and(|c| c.dnp),
            exclude_from_bom: c.is_some_and(|c| c.exclude_from_bom),
            pad_nets: &p.pad_nets,
            net_by_name: by_name,
        };
        out.push_str(&instantiate(fp, &inst, &mut ids));
        out.push('\n');
        let count = fp
            .root
            .children("pad")
            .filter(|x| x.args().first().and_then(|n| n.as_str()).is_some_and(|n| !n.is_empty()))
            .count();
        let with_net = fp
            .root
            .children("pad")
            .filter(|x| {
                x.args()
                    .first()
                    .and_then(|n| n.as_str())
                    .is_some_and(|n| p.pad_nets.contains_key(n))
            })
            .count();
        rep.pads += count;
        rep.pads_with_net += with_net;
    }
    out.push_str(&edge_items(opts, &mut ids));
    out.push_str("\t(embedded_fonts no)\n)\n");

    // schematic net classes the patterns do not already give
    let mut assignments = BTreeMap::new();
    let known: BTreeSet<&str> = opts
        .net_classes
        .iter()
        .map(|c| c.name.as_str())
        .chain(["Default"])
        .collect();
    let mut undefined: BTreeMap<&str, usize> = BTreeMap::new();
    for n in &netlist.nets {
        if n.class.is_empty() || n.class == "Default" {
            continue;
        }
        let by_pattern = opts
            .net_classes
            .iter()
            .find(|c| c.patterns.iter().any(|p| glob_match(p, &n.name)))
            .map(|c| c.name.as_str());
        if by_pattern == Some(n.class.as_str()) {
            continue;
        }
        if known.contains(n.class.as_str()) {
            assignments.insert(n.name.clone(), n.class.clone());
        } else {
            *undefined.entry(n.class.as_str()).or_default() += 1;
        }
    }
    for (cls, k) in undefined {
        rep.warnings.push(format!(
            "the schematic puts {k} nets in class {cls}, which net_classes does not define; they fall back to Default"
        ));
    }
    Ok((out, rep, assignments))
}

/// The `.kicad_pro`: a new one, or `existing` with the board rules and net classes merged in.
pub fn project_json(
    existing: Option<Value>,
    file_name: &str,
    opts: &CreateOptions,
    assignments: &BTreeMap<String, String>,
) -> Value {
    let mut pro = existing.unwrap_or_else(|| json!({}));
    if !pro.is_object() {
        pro = json!({});
    }
    let obj = |v: &mut Value, k: &str| -> Value {
        let e = v
            .as_object_mut()
            .unwrap()
            .entry(k.to_string())
            .or_insert_with(|| json!({}));
        if !e.is_object() {
            *e = json!({});
        }
        e.take()
    };
    // board.design_settings.rules
    let mut board = obj(&mut pro, "board");
    let mut ds = obj(&mut board, "design_settings");
    let mut rules = obj(&mut ds, "rules");
    let r = rules.as_object_mut().unwrap();
    for (k, v) in [
        ("min_copper_edge_clearance", 0.5),
        ("min_hole_clearance", 0.25),
        ("min_hole_to_hole", 0.25),
        ("min_via_diameter", 0.5),
        ("min_via_annular_width", 0.1),
        ("min_through_hole_diameter", 0.3),
    ] {
        r.entry(k.to_string()).or_insert(json!(v));
    }
    for (k, v) in &opts.rules {
        r.insert(k.clone(), json!(v));
    }
    ds["rules"] = rules;
    board["design_settings"] = ds;
    pro["board"] = board;
    let mut meta = obj(&mut pro, "meta");
    meta.as_object_mut()
        .unwrap()
        .entry("filename".to_string())
        .or_insert(json!(file_name));
    meta.as_object_mut()
        .unwrap()
        .entry("version".to_string())
        .or_insert(json!(3));
    pro["meta"] = meta;

    // net classes
    let mut ns = obj(&mut pro, "net_settings");
    let mut classes: Vec<Value> = ns
        .get("classes")
        .and_then(|c| c.as_array())
        .cloned()
        .unwrap_or_default();
    let default = opts.net_classes.iter().find(|c| c.name == "Default");
    let g = |c: Option<&ClassSpec>, k: fn(&ClassSpec) -> Option<f64>, d: f64| c.and_then(k).unwrap_or(d);
    let base = (
        g(default, |c| c.track_width, 0.2),
        g(default, |c| c.clearance, 0.2),
        g(default, |c| c.via_diameter, 0.6),
        g(default, |c| c.via_drill, 0.3),
        g(default, |c| c.diff_pair_width, 0.2),
        g(default, |c| c.diff_pair_gap, 0.25),
    );
    let class_json = |c: Option<&ClassSpec>, name: &str, prio: i64| {
        json!({
            "name": name,
            "track_width": g(c, |c| c.track_width, base.0),
            "clearance": g(c, |c| c.clearance, base.1),
            "via_diameter": g(c, |c| c.via_diameter, base.2),
            "via_drill": g(c, |c| c.via_drill, base.3),
            "microvia_diameter": 0.3,
            "microvia_drill": 0.1,
            "diff_pair_width": g(c, |c| c.diff_pair_width, base.4),
            "diff_pair_gap": g(c, |c| c.diff_pair_gap, base.5),
            "diff_pair_via_gap": 0.25,
            "line_style": 0,
            "pcb_color": "rgba(0, 0, 0, 0.000)",
            "schematic_color": "rgba(0, 0, 0, 0.000)",
            "wire_width": 6,
            "bus_width": 12,
            "priority": prio,
        })
    };
    let need_default = default.is_some() || !classes.iter().any(|c| c["name"] == "Default");
    let mut put = |v: Value| {
        let name = v["name"].clone();
        match classes.iter_mut().find(|c| c["name"] == name) {
            Some(c) => *c = v,
            None => classes.push(v),
        }
    };
    if need_default {
        put(class_json(default, "Default", i32::MAX as i64));
    }
    for (k, c) in opts.net_classes.iter().filter(|c| c.name != "Default").enumerate() {
        put(class_json(
            Some(c),
            &c.name,
            c.priority.map(i64::from).unwrap_or(k as i64),
        ));
    }
    ns["classes"] = json!(classes);
    let mut patterns: Vec<Value> = ns
        .get("netclass_patterns")
        .and_then(|c| c.as_array())
        .cloned()
        .unwrap_or_default();
    let ours: BTreeSet<&str> = opts.net_classes.iter().map(|c| c.name.as_str()).collect();
    patterns.retain(|p| !p["netclass"].as_str().is_some_and(|n| ours.contains(n)));
    for c in &opts.net_classes {
        for p in &c.patterns {
            patterns.push(json!({"netclass": c.name, "pattern": p}));
        }
    }
    ns["netclass_patterns"] = json!(patterns);
    if !assignments.is_empty() {
        let mut a = obj(&mut ns, "netclass_assignments");
        for (net, cls) in assignments {
            a[net.as_str()] = json!([cls]);
        }
        ns["netclass_assignments"] = a;
    }
    let mut nm = obj(&mut ns, "meta");
    nm.as_object_mut()
        .unwrap()
        .entry("version".to_string())
        .or_insert(json!(4));
    ns["meta"] = nm;
    pro["net_settings"] = ns;
    pro
}

fn seed_of(text: &str) -> u64 {
    // FNV-1a
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in text.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Read the netlist, write `output` (.kicad_pcb) and the `.kicad_pro` next to it (merged into
/// an existing one). The project `fp-lib-table` is looked for next to the output, the netlist
/// and the schematic the netlist names.
pub fn create_board_files(
    netlist: &Path,
    output: &Path,
    opts: &CreateOptions,
    overwrite: bool,
) -> Result<CreateReport> {
    if output.extension().is_none_or(|e| e != "kicad_pcb") {
        bail!("output must end in .kicad_pcb: {}", output.display());
    }
    if output.exists() && !overwrite {
        bail!("{} exists; pass overwrite: true to replace it", output.display());
    }
    let text = std::fs::read_to_string(netlist).with_context(|| format!("reading {}", netlist.display()))?;
    let nl = parse_netlist(&text)?;
    if nl.components.is_empty() {
        bail!("{}: the netlist has no components", netlist.display());
    }
    let mut project_dirs: Vec<PathBuf> = Vec::new();
    for d in [output.parent(), netlist.parent()].into_iter().flatten() {
        project_dirs.push(d.to_path_buf());
    }
    if let Some(src) = &nl.source
        && let Some(d) = Path::new(src).parent()
    {
        project_dirs.push(d.to_path_buf());
    }
    let mut libs = FootprintLibs::new(&opts.footprint_dirs, &project_dirs, opts.system_libraries);
    let (pcb, mut rep, assignments) = build_board(&nl, &mut libs, opts, seed_of(&text))?;
    if rep.footprints == 0 {
        bail!(
            "none of the {} footprints was found (looked in: {}); pass footprint_dirs",
            rep.missing_footprints.len(),
            libs.searched().join(", ")
        );
    }
    if !rep.missing_footprints.is_empty() {
        let mut libs_missing: Vec<&str> = rep
            .missing_footprints
            .iter()
            .filter_map(|m| m.footprint.split_once(':').map(|x| x.0))
            .collect();
        libs_missing.sort();
        libs_missing.dedup();
        rep.warnings.insert(
            0,
            format!(
                "{} parts left off the board, their footprints were not found (libraries: {}; looked in: {}); pass footprint_dirs",
                rep.missing_footprints.len(),
                libs_missing.join(", "),
                libs.searched().join(", ")
            ),
        );
    }
    if let Some(d) = output.parent()
        && !d.as_os_str().is_empty()
    {
        std::fs::create_dir_all(d).with_context(|| format!("creating {}", d.display()))?;
    }
    std::fs::write(output, pcb).with_context(|| format!("writing {}", output.display()))?;
    if overwrite {
        rep.removed = remove_working_copies(output);
    }
    let pro_path = output.with_extension("kicad_pro");
    let existing = std::fs::read_to_string(&pro_path)
        .ok()
        .and_then(|s| serde_json::from_str::<Value>(&s).ok());
    rep.pro_merged = existing.is_some();
    let name = pro_path
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("board.kicad_pro");
    let pro = project_json(existing, name, opts, &assignments);
    std::fs::write(&pro_path, serde_json::to_string_pretty(&pro)? + "\n")
        .with_context(|| format!("writing {}", pro_path.display()))?;
    rep.pcb_path = output.to_path_buf();
    rep.pro_path = pro_path;
    Ok(rep)
}

/// The working copy of a board (`<stem>.prerouted.kicad_pcb`) and its project files belong to
/// the board they were made from: a new board under the same name must not continue on them.
pub fn remove_working_copies(pcb: &Path) -> Vec<PathBuf> {
    let Some(stem) = pcb.file_stem().and_then(|s| s.to_str()) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for ext in ["kicad_pcb", "kicad_pro", "kicad_dru", "kicad_prl"] {
        let f = pcb.with_file_name(format!("{stem}.prerouted.{ext}"));
        if f.is_file() && std::fs::remove_file(&f).is_ok() {
            out.push(f);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::read::{ReadOptions, parse_board};

    const NETLIST: &str = r#"(export (version "E")
  (design (source "/nowhere/t.kicad_sch"))
  (components
    (comp (ref "R1") (value "10k") (footprint "GbTest:R_0603")
      (fields (field (name "LCSC") "C25804") (field (name "Footprint") "GbTest:R_0603") (field (name "Datasheet")))
      (property (name "Sheetname") (value "Power")) (property (name "Sheetfile") (value "power.kicad_sch"))
      (sheetpath (names "/Power/") (tstamps "/aaaa/")) (tstamps "bbbb"))
    (comp (ref "R2") (value "DNP") (footprint "GbTest:R_0603")
      (property (name "dnp")) (sheetpath (names "/") (tstamps "/")) (tstamps "cccc"))
    (comp (ref "J1") (value "USB") (footprint "GbTest:Shielded")
      (sheetpath (names "/") (tstamps "/")) (tstamps "dddd"))
    (comp (ref "U9") (value "X") (footprint "Nope:Missing")))
  (nets
    (net (code "1") (name "GND") (class "Default")
      (node (ref "R1") (pin "2") (pintype "passive"))
      (node (ref "J1") (pin "2") (pinfunction "GND") (pintype "power_in"))
      (node (ref "J1") (pin "S1") (pinfunction "SHIELD") (pintype "passive")))
    (net (code "2") (name "/Power/VBUS") (class "Power")
      (node (ref "R1") (pin "1") (pintype "passive"))
      (node (ref "R2") (pin "1") (pintype "passive"))
      (node (ref "J1") (pin "1") (pinfunction "VBUS") (pintype "power_out")))
    (net (code "3") (name "unconnected-(R2-Pad2)") (class "Default")
      (node (ref "R2") (pin "2") (pintype "passive")))))
"#;

    const R0603: &str = r#"(footprint "R_0603"
	(version 20241229)
	(generator "kicad-footprint-generator")
	(layer "F.Cu")
	(descr "test resistor")
	(property "Reference" "REF**"
		(at 0 -1.43 0)
		(layer "F.SilkS")
		(uuid "11111111-1111-4111-8111-111111111111")
		(effects (font (size 1 1) (thickness 0.15)))
	)
	(property "Value" "R_0603"
		(at 0 1.43 0)
		(layer "F.Fab")
		(uuid "22222222-2222-4222-8222-222222222222")
		(effects (font (size 1 1) (thickness 0.15)))
	)
	(attr smd)
	(fp_rect (start -1.48 -0.73) (end 1.48 0.73) (stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd") (uuid "33333333-3333-4333-8333-333333333333"))
	(pad "1" smd roundrect (at -0.7875 0) (size 0.875 0.95) (layers "F.Cu" "F.Mask" "F.Paste") (roundrect_rratio 0.25) (uuid "44444444-4444-4444-8444-444444444444"))
	(pad "2" smd roundrect (at 0.7875 0) (size 0.875 0.95) (layers "F.Cu" "F.Mask" "F.Paste") (roundrect_rratio 0.25) (uuid "55555555-5555-4555-8555-555555555555"))
	(embedded_fonts no)
)
"#;

    const SHIELDED: &str = r#"(footprint "Shielded" (version 20241229) (layer "F.Cu")
  (property "Reference" "REF**" (at 0 -3 0) (layer "F.SilkS"))
  (property "Value" "Shielded" (at 0 3 0) (layer "F.Fab"))
  (attr smd)
  (fp_line (start -3 -2) (end 3 2) (stroke (width 0.05) (type solid)) (layer "F.CrtYd"))
  (pad "" np_thru_hole circle (at 0 -1) (size 0.6 0.6) (drill 0.6) (layers "*.Cu" "*.Mask"))
  (pad "1" smd rect (at -1 0) (size 0.6 1) (layers "F.Cu" "F.Mask" "F.Paste"))
  (pad "2" smd rect (at 1 0) (size 0.6 1) (layers "F.Cu" "F.Mask" "F.Paste"))
  (pad "SH" thru_hole oval (at -2.5 0) (size 1 1.6) (drill 0.6) (layers "*.Cu" "*.Mask"))
  (pad "SH" thru_hole oval (at 2.5 0) (size 1 1.6) (drill 0.6) (layers "*.Cu" "*.Mask")))
"#;

    fn lib_dir(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("gb-create-{tag}-{}", std::process::id()));
        let lib = d.join("GbTest.pretty");
        std::fs::create_dir_all(&lib).unwrap();
        std::fs::write(lib.join("R_0603.kicad_mod"), R0603).unwrap();
        std::fs::write(lib.join("Shielded.kicad_mod"), SHIELDED).unwrap();
        d
    }

    fn build(opts: &CreateOptions) -> (String, CreateReport, BTreeMap<String, String>) {
        let nl = parse_netlist(NETLIST).unwrap();
        let mut libs = FootprintLibs::new(&opts.footprint_dirs, &[], false);
        build_board(&nl, &mut libs, opts, 7).unwrap()
    }

    #[test]
    fn parses_the_netlist() {
        let nl = parse_netlist(NETLIST).unwrap();
        assert_eq!(nl.components.len(), 4);
        let r1 = &nl.components[0];
        assert_eq!(r1.path, "/aaaa/bbbb");
        assert_eq!(r1.sheetname, "Power");
        assert_eq!(r1.fields, vec![("LCSC".to_string(), "C25804".to_string())]);
        assert!(nl.components[1].dnp);
        assert_eq!(nl.components[1].path, "/cccc");
        assert_eq!(nl.nets.len(), 3);
        assert_eq!(nl.nets[1].class, "Power");
    }

    #[test]
    fn parts_outside_leave_the_board_empty() {
        let dir = lib_dir("outside");
        let opts = CreateOptions {
            width: 40.0,
            height: 30.0,
            mounting_holes: Some(HoleSpec {
                diameter: 3.2,
                inset: 4.0,
                footprint: None,
                net: None,
            }),
            footprint_dirs: vec![dir.clone()],
            parts_outside: true,
            ..Default::default()
        };
        let (text, rep, _) = build(&opts);
        assert!(rep.overflow.is_empty() && rep.warnings.iter().all(|w| !w.contains("did not fit")));
        let mut parked = rep.parked.clone();
        parked.sort();
        assert_eq!(parked, vec!["J1", "R1", "R2"]);
        let kb = parse_board(text, None, &ReadOptions::default()).unwrap();
        for r in ["J1", "R1", "R2"] {
            let c = kb.board.courtyards[r];
            assert!(c.min.y > opts.y0 + opts.height, "{r} {c:?}");
        }
        // the holes stay at the corners
        assert!(kb.board.courtyards["H1"].max.y < opts.y0 + opts.height);
        std::fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn overwrite_removes_the_old_working_copy() {
        let dir = std::env::temp_dir().join(format!("gb-create-work-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let pcb = dir.join("b.kicad_pcb");
        for f in [
            "b.prerouted.kicad_pcb",
            "b.prerouted.kicad_pro",
            "b.prerouted.kicad_dru",
            "b.routed.kicad_pcb",
        ] {
            std::fs::write(dir.join(f), "x").unwrap();
        }
        let gone = remove_working_copies(&pcb);
        assert_eq!(gone.len(), 3, "{gone:?}");
        assert!(!dir.join("b.prerouted.kicad_pcb").exists());
        assert!(
            dir.join("b.routed.kicad_pcb").exists(),
            "a routed result is not a working copy"
        );
        std::fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn builds_a_board_the_reader_loads() {
        let dir = lib_dir("build");
        let opts = CreateOptions {
            width: 40.0,
            height: 30.0,
            layers: 4,
            plane_layers: vec!["In1.Cu".into()],
            corner_radius: 2.0,
            mounting_holes: Some(HoleSpec {
                diameter: 3.2,
                inset: 4.0,
                footprint: None,
                net: None,
            }),
            footprint_dirs: vec![dir.clone()],
            ..Default::default()
        };
        let (text, rep, _) = build(&opts);
        assert_eq!(rep.version, KICAD9_VERSION);
        assert_eq!(rep.footprints, 3);
        assert_eq!(rep.holes, 4);
        assert_eq!(rep.missing_footprints.len(), 1);
        assert_eq!(rep.missing_footprints[0].reference, "U9");
        assert!(rep.overflow.is_empty(), "{:?}", rep.overflow);
        // J1: SH has no pin, S1 has no pad
        assert_eq!(rep.unmatched.len(), 1);
        let u = &rep.unmatched[0];
        assert_eq!(u.pads, vec!["SH"]);
        assert_eq!(u.pins, vec![("S1".to_string(), "GND".to_string())]);
        assert_eq!(u.suggested_alias, Some(("S1".to_string(), "SH".to_string())));
        assert!(
            rep.warnings.iter().any(|w| w.contains(r#"{"J1": {"S1": "SH"}}"#)),
            "{:?}",
            rep.warnings
        );
        assert!(text.contains("(net 2 \"/Power/VBUS\")"));
        assert!(text.contains("(path \"/aaaa/bbbb\")"));
        assert!(text.contains("(property \"LCSC\" \"C25804\""));
        assert!(text.contains("(attr smd dnp)"));
        assert!(text.contains("(pintype \"passive+no_connect\")"));
        assert!(text.contains("(4 \"In1.Cu\" power)"));
        assert!(text.contains("(gr_arc"));
        assert!(!text.contains("REF**"));
        // every uuid once
        let mut uuids: Vec<&str> = text
            .match_indices("(uuid \"")
            .map(|(i, _)| &text[i + 7..i + 43])
            .collect();
        let n = uuids.len();
        uuids.sort();
        uuids.dedup();
        assert_eq!(uuids.len(), n);

        let kb = parse_board(text, None, &ReadOptions::default()).unwrap();
        let b = &kb.board;
        assert_eq!(b.layers.len(), 4);
        assert!(b.layers[1].is_plane);
        let bb = b.bbox();
        assert!((bb.width() - 40.0).abs() < 1e-6 && (bb.height() - 30.0).abs() < 1e-6);
        let vbus = b.nets.iter().find(|n| n.name == "/Power/VBUS").unwrap();
        assert_eq!(vbus.pins.len(), 3);
        // the stackup: 4 copper and 3 dielectric layers summing to 1.6 mm with the masks
        assert_eq!(b.stackup.len(), 7);
        let total: f64 = b.stackup.iter().map(|s| s.thickness).sum::<f64>() + 2.0 * MASK;
        assert!((total - 1.6).abs() < 1e-3, "{total}");
        // parts inside the outline
        for (r, c) in &b.courtyards {
            assert!(
                c.min.x >= bb.min.x && c.max.x <= bb.max.x && c.min.y >= bb.min.y && c.max.y <= bb.max.y,
                "{r}"
            );
        }
        std::fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn an_alias_connects_the_shield() {
        let dir = lib_dir("alias");
        let mut opts = CreateOptions {
            width: 40.0,
            height: 30.0,
            footprint_dirs: vec![dir.join("GbTest.pretty")],
            ..Default::default()
        };
        opts.pin_aliases.insert(
            "J1".into(),
            [("S1".to_string(), "SH".to_string())].into_iter().collect(),
        );
        let (text, rep, _) = build(&opts);
        assert!(rep.unmatched.is_empty(), "{:?}", rep.unmatched);
        let kb = parse_board(text, None, &ReadOptions::default()).unwrap();
        let gnd = kb.board.nets.iter().find(|n| n.name == "GND").unwrap();
        // R1.2, J1.2 and the two SH pads
        assert_eq!(gnd.pins.len(), 4);
        std::fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn a_newer_footprint_makes_a_kicad10_board() {
        let dir = lib_dir("v10");
        let lib = dir.join("GbTest.pretty");
        std::fs::write(
            lib.join("R_0603.kicad_mod"),
            R0603.replace("(version 20241229)", "(version 20260206)"),
        )
        .unwrap();
        let opts = CreateOptions {
            footprint_dirs: vec![dir.clone()],
            ..Default::default()
        };
        let (text, rep, _) = build(&opts);
        assert_eq!(rep.version, KICAD10_VERSION);
        assert!(text.contains("(net \"/Power/VBUS\")"));
        assert!(!text.contains("(net 0 \"\")"));
        let kb = parse_board(text, None, &ReadOptions::default()).unwrap();
        assert_eq!(
            kb.board
                .nets
                .iter()
                .find(|n| n.name == "/Power/VBUS")
                .unwrap()
                .pins
                .len(),
            3
        );
        std::fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn the_project_gets_classes_and_rules() {
        let opts = CreateOptions {
            net_classes: vec![ClassSpec {
                name: "Power".into(),
                track_width: Some(0.5),
                patterns: vec!["+5V*".into()],
                ..Default::default()
            }],
            rules: [("min_clearance".to_string(), 0.127)].into_iter().collect(),
            ..Default::default()
        };
        let existing = json!({"schematic": {"x": 1}, "net_settings": {"classes": [{"name": "Default", "clearance": 0.3}],
            "netclass_patterns": [{"netclass": "Power", "pattern": "old"}, {"netclass": "Other", "pattern": "keep"}]}});
        let pro = project_json(Some(existing), "t.kicad_pro", &opts, &BTreeMap::new());
        assert_eq!(pro["schematic"]["x"], 1);
        assert_eq!(pro["board"]["design_settings"]["rules"]["min_clearance"], 0.127);
        let classes = pro["net_settings"]["classes"].as_array().unwrap();
        assert_eq!(classes.len(), 2);
        assert_eq!(classes[0]["clearance"], 0.3, "an existing Default stays");
        assert_eq!(classes[1]["track_width"], 0.5);
        let pats = pro["net_settings"]["netclass_patterns"].as_array().unwrap();
        assert_eq!(pats.len(), 2);
        assert!(pats.iter().any(|p| p["pattern"] == "keep"));
        assert!(pats.iter().any(|p| p["pattern"] == "+5V*"));
    }

    #[test]
    fn schematic_classes_become_assignments() {
        let dir = lib_dir("cls");
        let opts = CreateOptions {
            footprint_dirs: vec![dir.clone()],
            net_classes: vec![ClassSpec {
                name: "Power".into(),
                ..Default::default()
            }],
            ..Default::default()
        };
        let (_, _, a) = build(&opts);
        assert_eq!(a.get("/Power/VBUS").map(String::as_str), Some("Power"));
        let (_, rep, a) = build(&CreateOptions {
            footprint_dirs: vec![dir.clone()],
            ..Default::default()
        });
        assert!(a.is_empty());
        assert!(rep.warnings.iter().any(|w| w.contains("class Power")));
        std::fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn lib_table_paths_expand() {
        let t = r#"(fp_lib_table (version 7)
          (lib (name "Mine") (type "KiCad") (uri "${KIPRJMOD}/libs/Mine.pretty") (options "") (descr ""))
          (lib (name "Std") (type "KiCad") (uri "${KICAD9_FOOTPRINT_DIR}/Std.pretty") (options "") (descr ""))
          (lib (name "Odd") (type "KiCad") (uri "${GB_SURELY_UNSET_VAR}/Odd.pretty") (options "") (descr "")))"#;
        let v = parse_lib_table(t, Path::new("/prj"), Some(Path::new("/kicad/fp")));
        assert_eq!(v[0], ("Mine".to_string(), PathBuf::from("/prj/libs/Mine.pretty")));
        assert_eq!(
            v.len(),
            2 + usize::from(knobs::foreign("GB_SURELY_UNSET_VAR").is_some())
        );
    }

    #[test]
    fn rows_avoid_the_keepouts() {
        let items = vec![(5.0, 5.0); 6];
        let pos = pack(&items, [0.0, 0.0, 20.0, 20.0], &[[0.0, 0.0, 6.0, 6.0]], 1.0);
        for p in pos.iter().flatten() {
            assert!(!(p.0 < 6.0 && p.1 < 6.0), "{p:?}");
        }
        assert!(pos.iter().all(|p| p.is_some()));
        let pos = pack(&[(15.0, 15.0); 2], [0.0, 0.0, 20.0, 20.0], &[], 1.0);
        assert!(pos[0].is_some() && pos[1].is_none());
    }
}
