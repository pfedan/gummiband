//! Debug rendering of a board, its substrate and routed geometry.

use std::fmt::Write as _;

use gummiband_core::board::{Board, ObstacleKind};
use gummiband_core::check::Violation;
use gummiband_core::geom::{Point, Polygon};
use gummiband_core::realize::Geometry;
use gummiband_core::topo::{NONE, Substrate};

const LAYER_COLORS: [&str; 4] = ["#c83434", "#3a7bd5", "#2a9d5c", "#d59a3a"];

fn poly_path(p: &Polygon) -> String {
    let mut s = String::new();
    for (i, q) in p.pts.iter().enumerate() {
        let _ = write!(s, "{}{:.4},{:.4} ", if i == 0 { "M" } else { "L" }, q.x, q.y);
    }
    s.push('Z');
    s
}

pub fn render(board: &Board, sub: Option<&Substrate>, geo: &Geometry, viol: &[Violation], show_tris: bool) -> String {
    let bb = board.bbox().grow(2.0);
    let mut s = String::new();
    let _ = writeln!(
        s,
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="{} {} {} {}" width="{}" height="{}">"#,
        bb.min.x,
        bb.min.y,
        bb.width(),
        bb.height(),
        bb.width() * 20.0,
        bb.height() * 20.0
    );
    let _ = writeln!(
        s,
        r##"<rect x="{}" y="{}" width="{}" height="{}" fill="#111"/>"##,
        bb.min.x,
        bb.min.y,
        bb.width(),
        bb.height()
    );
    for o in &board.outline {
        let _ = writeln!(
            s,
            r##"<path d="{}" fill="#1c2420" stroke="#ddd" stroke-width="0.05"/>"##,
            poly_path(o)
        );
    }
    let routing: Vec<u8> = board.routing_layers().iter().collect();
    if let (Some(sub), true) = (sub, show_tris) {
        for (mi, m) in sub.m.iter().enumerate() {
            let mut d = String::new();
            for t in &m.tris {
                let p: Vec<Point> = t.iter().map(|&v| m.verts[v as usize]).collect();
                let _ = write!(
                    d,
                    "M{:.3},{:.3} L{:.3},{:.3} L{:.3},{:.3} Z ",
                    p[0].x, p[0].y, p[1].x, p[1].y, p[2].x, p[2].y
                );
            }
            let color = if mi == 0 { "#5a3a3a" } else { "#3a4a5a" };
            let _ = writeln!(s, r#"<path d="{d}" fill="none" stroke="{color}" stroke-width="0.01"/>"#);
        }
        let _ = NONE;
    }
    // obstacles: back layer first
    for &l in routing.iter().rev() {
        let li = routing.iter().position(|&x| x == l).unwrap();
        let color = LAYER_COLORS[li.min(3) * 3 % 4];
        for o in &board.obstacles {
            if !o.layers.has(l) {
                continue;
            }
            let (fill, op) = match o.kind {
                ObstacleKind::Keepout { .. } => ("#888", 0.15),
                ObstacleKind::Hole => ("#000", 1.0),
                _ => (color, 0.55),
            };
            let p = o.shape.to_polygon(0.01);
            let _ = writeln!(s, r#"<path d="{}" fill="{fill}" fill-opacity="{op}"/>"#, poly_path(&p));
        }
    }
    for &l in routing.iter().rev() {
        let li = routing.iter().position(|&x| x == l).unwrap();
        let color = LAYER_COLORS[li.min(3) * 3 % 4];
        for t in geo.tracks.iter().filter(|t| t.layer == l) {
            let mut d = String::new();
            for (i, q) in t.pts.iter().enumerate() {
                let _ = write!(d, "{}{:.4},{:.4} ", if i == 0 { "M" } else { "L" }, q.x, q.y);
            }
            let _ = writeln!(
                s,
                r#"<path d="{d}" fill="none" stroke="{color}" stroke-opacity="0.85" stroke-width="{}" stroke-linecap="round" stroke-linejoin="round"/>"#,
                t.width
            );
        }
    }
    for v in &geo.vias {
        let _ = writeln!(
            s,
            r##"<circle cx="{}" cy="{}" r="{}" fill="#e8e8e8"/><circle cx="{}" cy="{}" r="{}" fill="#111"/>"##,
            v.at.x,
            v.at.y,
            v.diameter / 2.0,
            v.at.x,
            v.at.y,
            v.drill / 2.0
        );
    }
    if let Ok(t) = std::fs::read_to_string("/tmp/gb-why.txt")
        && std::env::var("GB_WHY").is_ok()
    {
        for l in t.lines() {
            let v: Vec<f64> = l.split(' ').filter_map(|x| x.parse().ok()).collect();
            if v.len() == 3 {
                let c = if v[2] == 0.0 { "#0ff" } else { "#f0f" };
                let _ = writeln!(s, r#"<circle cx="{}" cy="{}" r="0.04" fill="{c}"/>"#, v[0], v[1]);
            }
        }
    }
    for v in viol {
        let _ = writeln!(
            s,
            r##"<circle cx="{}" cy="{}" r="0.25" fill="none" stroke="#ff0" stroke-width="0.06"/>"##,
            v.at.x, v.at.y
        );
    }
    s.push_str("</svg>\n");
    s
}
