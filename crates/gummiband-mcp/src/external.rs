//! External KiCad tools: `kicad-cli` (DRC, SVG export) and `rsvg-convert` (SVG to PNG).

use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{Context, Result, bail};
use serde::Serialize;

use rmcp::schemars::{self, JsonSchema};

/// Find an executable: the environment variable, then PATH, then the usual install places.
/// MCP clients often start servers with a minimal PATH, hence the fallbacks.
pub fn find_tool(name: &str, env: &str, fallbacks: &[&str]) -> Result<PathBuf> {
    if let Ok(p) = std::env::var(env) {
        let p = PathBuf::from(p);
        if p.is_file() {
            return Ok(p);
        }
        bail!("{env} points to {}, which is not a file", p.display());
    }
    if let Some(path) = std::env::var_os("PATH") {
        for dir in std::env::split_paths(&path) {
            let p = dir.join(name);
            if p.is_file() {
                return Ok(p);
            }
        }
    }
    for f in fallbacks {
        let p = PathBuf::from(f);
        if p.is_file() {
            return Ok(p);
        }
    }
    bail!("{name} not found: install it, put it on PATH or set {env}")
}

pub fn kicad_cli() -> Result<PathBuf> {
    find_tool(
        "kicad-cli",
        "GUMMIBAND_KICAD_CLI",
        &[
            "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli",
            "/opt/homebrew/bin/kicad-cli",
            "/usr/local/bin/kicad-cli",
            "/usr/bin/kicad-cli",
        ],
    )
}

pub fn rsvg_convert() -> Result<PathBuf> {
    find_tool(
        "rsvg-convert",
        "GUMMIBAND_RSVG_CONVERT",
        &[
            "/opt/homebrew/bin/rsvg-convert",
            "/usr/local/bin/rsvg-convert",
            "/usr/bin/rsvg-convert",
        ],
    )
}

fn run(cmd: &mut Command) -> Result<std::process::Output> {
    let out = cmd.output().with_context(|| format!("running {cmd:?}"))?;
    if !out.status.success() {
        bail!(
            "{:?} failed ({}): {}{}",
            cmd.get_program(),
            out.status,
            String::from_utf8_lossy(&out.stderr).trim(),
            String::from_utf8_lossy(&out.stdout).trim()
        );
    }
    Ok(out)
}

/// A scratch file in the server's temp folder.
pub fn temp_file(stem: &str, ext: &str) -> Result<PathBuf> {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static N: AtomicUsize = AtomicUsize::new(0);
    let dir = std::env::temp_dir().join(format!("gummiband-mcp-{}", std::process::id()));
    std::fs::create_dir_all(&dir)?;
    Ok(dir.join(format!("{stem}-{}.{ext}", N.fetch_add(1, Ordering::Relaxed))))
}

/// Copy `<src>.kicad_pro` and `<src>.kicad_dru` next to `dst` (same base name) if they are
/// missing there: kicad-cli needs the project for net classes and rules, and the reader
/// takes net classes and custom rules from them too. Returns the copied files.
pub fn copy_project_files(src_pcb: &Path, dst_pcb: &Path) -> Vec<PathBuf> {
    let mut copied = Vec::new();
    for ext in ["kicad_pro", "kicad_dru"] {
        let s = src_pcb.with_extension(ext);
        let d = dst_pcb.with_extension(ext);
        if s != d && s.is_file() && !d.exists() && std::fs::copy(&s, &d).is_ok() {
            copied.push(d);
        }
    }
    copied
}

/// Make sure `<pcb>.kicad_pro` exists: otherwise copy it from a project whose base name is a
/// prefix of the board's (`x.routed.kicad_pcb` -> `x.kicad_pro`) or from the only project in
/// the folder. Returns the source of the copy, if any.
pub fn ensure_project(pcb: &Path) -> Result<Option<PathBuf>> {
    if pcb.with_extension("kicad_pro").is_file() {
        return Ok(None);
    }
    let dir = pcb.parent().unwrap_or(Path::new("."));
    let stem = pcb.file_stem().and_then(|s| s.to_str()).unwrap_or("").to_string();
    let mut base = stem.as_str();
    while let Some(i) = base.rfind(['.', '-', '_']) {
        base = &base[..i];
        let cand = dir.join(format!("{base}.kicad_pro"));
        if cand.is_file() {
            copy_project_files(&cand.with_extension("kicad_pcb"), pcb);
            return Ok(Some(cand));
        }
    }
    let pros: Vec<PathBuf> = std::fs::read_dir(dir)?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e == "kicad_pro"))
        .collect();
    if pros.len() == 1 {
        copy_project_files(&pros[0].with_extension("kicad_pcb"), pcb);
        return Ok(Some(pros[0].clone()));
    }
    bail!(
        "{} is missing and no single project file was found in {} to copy: kicad-cli needs the \
         project for net classes and rules",
        pcb.with_extension("kicad_pro").display(),
        dir.display()
    )
}

// ------------------------------------------------------------------------------------- DRC

#[derive(Debug, Serialize, JsonSchema)]
pub struct DrcItem {
    pub description: String,
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct DrcClearance {
    /// KiCad violation type (clearance, hole_clearance, hole_to_hole, ...).
    #[serde(rename = "type")]
    pub kind: String,
    pub required_mm: f64,
    pub actual_mm: f64,
    pub deficit_mm: f64,
    pub items: Vec<DrcItem>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct DrcUnconnected {
    pub items: Vec<DrcItem>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct DrcOut {
    pub path: String,
    /// The full kicad-cli JSON report.
    pub report_path: String,
    pub kicad_version: String,
    /// Project file copied next to the board for the check, if one was missing.
    pub project_copied_from: Option<String>,
    /// Violations per type, without silkscreen (silk*) and library (lib_*) checks.
    pub counts: std::collections::BTreeMap<String, usize>,
    /// Sum of `counts`.
    pub violations: usize,
    /// Silkscreen and library findings that were left out of `counts`.
    pub ignored: usize,
    /// Number of unconnected item pairs.
    pub unconnected: usize,
    /// The first unconnected pairs (up to max_items).
    pub unconnected_items: Vec<DrcUnconnected>,
    /// Clearance type violations by deficit: [<2 um, <10 um, <30 um, <100 um, more].
    pub deficit_histogram: [usize; 5],
    /// The worst clearance type violations, largest deficit first (up to max_items).
    pub worst_clearance: Vec<DrcClearance>,
}

fn items_of(v: &serde_json::Value) -> Vec<DrcItem> {
    v["items"]
        .as_array()
        .map(|a| {
            a.iter()
                .map(|i| DrcItem {
                    description: i["description"].as_str().unwrap_or("").to_string(),
                    x: i["pos"]["x"].as_f64().unwrap_or(f64::NAN),
                    y: i["pos"]["y"].as_f64().unwrap_or(f64::NAN),
                })
                .collect()
        })
        .unwrap_or_default()
}

/// "... clearance 0.2000 mm; actual 0.1710 mm)" -> (0.2, 0.171).
fn required_actual(desc: &str) -> Option<(f64, f64)> {
    let i = desc.find("mm; actual ")?;
    let required = desc[..i].trim_end().rsplit(' ').next()?.parse().ok()?;
    let rest = &desc[i + "mm; actual ".len()..];
    let actual = rest.split(' ').next()?.parse().ok()?;
    Some((required, actual))
}

pub fn drc(pcb: &Path, max_items: usize) -> Result<DrcOut> {
    if !pcb.is_file() {
        bail!("{} does not exist", pcb.display());
    }
    let copied = ensure_project(pcb)?;
    let report = pcb.with_extension("drc.json");
    let cli = kicad_cli()?;
    run(Command::new(&cli)
        .args([
            "pcb",
            "drc",
            "--refill-zones",
            "--format",
            "json",
            "--units",
            "mm",
            "-o",
        ])
        .arg(&report)
        .arg(pcb))?;
    let d: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(&report).with_context(|| format!("reading {}", report.display()))?,
    )?;
    let mut counts = std::collections::BTreeMap::new();
    let mut ignored = 0;
    let mut worst = Vec::new();
    let mut hist = [0usize; 5];
    for v in d["violations"].as_array().into_iter().flatten() {
        let kind = v["type"].as_str().unwrap_or("").to_string();
        if kind.starts_with("silk") || kind.starts_with("lib_") {
            ignored += 1;
            continue;
        }
        if kind.contains("clearance") || kind == "hole_to_hole" {
            let parsed = v["description"].as_str().and_then(required_actual);
            if let Some((required, actual)) = parsed {
                let deficit = required - actual;
                let b = [0.002, 0.01, 0.03, 0.1].iter().position(|&x| deficit < x).unwrap_or(4);
                hist[b] += 1;
                worst.push(DrcClearance {
                    kind: kind.clone(),
                    required_mm: required,
                    actual_mm: actual,
                    deficit_mm: (deficit * 1e4).round() / 1e4,
                    items: items_of(v),
                });
            }
        }
        *counts.entry(kind).or_insert(0) += 1;
    }
    worst.sort_by(|a, b| b.deficit_mm.total_cmp(&a.deficit_mm));
    worst.truncate(max_items);
    let unconnected = d["unconnected_items"].as_array().map(|a| a.len()).unwrap_or(0);
    let unconnected_items = d["unconnected_items"]
        .as_array()
        .into_iter()
        .flatten()
        .take(max_items)
        .map(|u| DrcUnconnected { items: items_of(u) })
        .collect();
    Ok(DrcOut {
        path: pcb.display().to_string(),
        report_path: report.display().to_string(),
        kicad_version: d["kicad_version"].as_str().unwrap_or("").to_string(),
        project_copied_from: copied.map(|p| p.display().to_string()),
        violations: counts.values().sum(),
        counts,
        ignored,
        unconnected,
        unconnected_items,
        deficit_histogram: hist,
        worst_clearance: worst,
    })
}

// ---------------------------------------------------------------------------------- render

/// Set an attribute on the first element that starts at `tag_start` (the `<svg` root).
fn set_attr(svg: &mut String, tag_start: usize, name: &str, value: &str) {
    let tag_end = svg[tag_start..].find('>').map(|i| tag_start + i).unwrap_or(svg.len());
    let key = format!(" {name}=\"");
    let tag = &svg[tag_start..tag_end];
    // attributes may follow a newline instead of a space
    let found = tag
        .find(&key)
        .or_else(|| tag.find(&format!("\n{name}=\"")))
        .or_else(|| tag.find(&format!("\t{name}=\"")));
    match found {
        Some(i) => {
            let vstart = tag_start + i + key.len();
            let vend = vstart + svg[vstart..].find('"').unwrap_or(0);
            svg.replace_range(vstart..vend, value);
        }
        None => svg.insert_str(tag_start + 4, &format!(" {name}=\"{value}\"")),
    }
}

pub struct Rendered {
    pub png: PathBuf,
    pub width_px: u32,
    pub height_px: u32,
}

/// Plot `layers` of the board to SVG with kicad-cli (page coordinates in mm equal board
/// coordinates), crop the SVG to `region` (x0, y0, x1, y1 in mm) and rasterize it.
pub fn render(
    pcb: &Path,
    layers: &[String],
    region: (f64, f64, f64, f64),
    width_px: u32,
    background: &str,
    png: &Path,
) -> Result<Rendered> {
    let (x0, y0, x1, y1) = region;
    let (w, h) = (x1 - x0, y1 - y0);
    if !(w > 0.0 && h > 0.0) {
        bail!("empty region");
    }
    let svg_path = temp_file("plot", "svg")?;
    run(Command::new(kicad_cli()?)
        .args([
            "pcb",
            "export",
            "svg",
            "--mode-single",
            "--exclude-drawing-sheet",
            "--page-size-mode",
            "0",
        ])
        .args(["--drill-shape-opt", "2", "-l"])
        .arg(layers.join(","))
        .arg("-o")
        .arg(&svg_path)
        .arg(pcb))?;
    let mut svg = std::fs::read_to_string(&svg_path)?;
    let _ = std::fs::remove_file(&svg_path);
    let root = svg.find("<svg").context("kicad-cli wrote no <svg> element")?;
    set_attr(&mut svg, root, "viewBox", &format!("{x0:.4} {y0:.4} {w:.4} {h:.4}"));
    set_attr(&mut svg, root, "width", &format!("{w:.4}mm"));
    set_attr(&mut svg, root, "height", &format!("{h:.4}mm"));
    let cropped = temp_file("crop", "svg")?;
    std::fs::write(&cropped, svg)?;
    let height_px = ((width_px as f64) * h / w).round().max(1.0) as u32;
    let res = run(Command::new(rsvg_convert()?)
        .args([
            "-w",
            &width_px.to_string(),
            "-h",
            &height_px.to_string(),
            "-b",
            background,
            "-f",
            "png",
            "-o",
        ])
        .arg(png)
        .arg(&cropped));
    let _ = std::fs::remove_file(&cropped);
    res?;
    Ok(Rendered {
        png: png.to_path_buf(),
        width_px,
        height_px,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_drc_numbers() {
        assert_eq!(
            required_actual("Clearance violation ( clearance 0.2000 mm; actual 0.1710 mm)"),
            Some((0.2, 0.171))
        );
        assert_eq!(
            required_actual(
                "Drilled hole too close to other hole (board setup constraints min 0.2500 mm; actual 0.1233 mm)"
            ),
            Some((0.25, 0.1233))
        );
        assert_eq!(required_actual("Tracks crossing"), None);
    }

    #[test]
    fn crops_svg_root() {
        let mut s = String::from(
            "<?xml?>\n<svg\n  version=\"1.1\"\n  width=\"297mm\" height=\"210mm\" viewBox=\"0 0 297 210\">\n<g/></svg>",
        );
        let r = s.find("<svg").unwrap();
        set_attr(&mut s, r, "viewBox", "10 20 5 5");
        set_attr(&mut s, r, "width", "5mm");
        set_attr(&mut s, r, "height", "5mm");
        assert!(s.contains("width=\"5mm\" height=\"5mm\" viewBox=\"10 20 5 5\""), "{s}");
    }
}
