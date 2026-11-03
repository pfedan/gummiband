//! Board outline of a variant and the arrangement of the fixed mechanical parts on its edge.

use anyhow::{Result, bail};
use gummiband_core::geom::{Point, pt};
use gummiband_kicad::place_write::kicad_rot;

use crate::model::{Design, R};

#[derive(Debug, Clone, Copy)]
pub enum Outline {
    Rect { x0: f64, y0: f64, x1: f64, y1: f64 },
    Circle { c: Point, r: f64 },
}

impl Outline {
    /// True if `p` lies inside the outline shrunk by `margin`.
    pub fn inside(&self, p: Point, margin: f64) -> bool {
        match *self {
            Outline::Rect { x0, y0, x1, y1 } => {
                p.x >= x0 + margin && p.x <= x1 - margin && p.y >= y0 + margin && p.y <= y1 - margin
            }
            Outline::Circle { c, r } => p.dist(c) <= r - margin,
        }
    }
    pub fn bbox(&self) -> R {
        match *self {
            Outline::Rect { x0, y0, x1, y1 } => R { x0, y0, x1, y1 },
            Outline::Circle { c, r } => R {
                x0: c.x - r,
                y0: c.y - r,
                x1: c.x + r,
                y1: c.y + r,
            },
        }
    }
    /// Polygon of the outline (vertices on the outline).
    pub fn polygon(&self) -> Vec<Point> {
        match *self {
            Outline::Rect { x0, y0, x1, y1 } => vec![pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)],
            Outline::Circle { c, r } => (0..180)
                .map(|i| {
                    let a = i as f64 * std::f64::consts::TAU / 180.0;
                    c + pt(a.cos(), a.sin()) * r
                })
                .collect(),
        }
    }
    pub fn describe(&self) -> String {
        match *self {
            Outline::Rect { x0, y0, x1, y1 } => format!("rectangle {:.1} x {:.1} mm", x1 - x0, y1 - y0),
            Outline::Circle { r, .. } => format!("circle, diameter {:.1} mm", 2.0 * r),
        }
    }
    pub fn area(&self) -> f64 {
        match *self {
            Outline::Rect { x0, y0, x1, y1 } => (x1 - x0) * (y1 - y0),
            Outline::Circle { r, .. } => std::f64::consts::PI * r * r,
        }
    }
}

/// A part inside a unit: offset and rotation relative to the unit pose.
#[derive(Debug, Clone)]
pub struct Member {
    pub part: usize,
    pub rel: Point,
    pub rel_rot: f64,
}

/// Parts that move together (a single part, a power stage macro, a plug with its PTC and cap).
#[derive(Debug, Clone)]
pub struct Unit {
    pub name: String,
    pub members: Vec<Member>,
    pub pos: Point,
    pub rot: f64,
}

impl Unit {
    pub fn single(d: &Design, part: usize) -> Unit {
        let p = &d.parts[part];
        Unit {
            name: p.reference.clone(),
            members: vec![Member {
                part,
                rel: pt(0.0, 0.0),
                rel_rot: p.rot,
            }],
            pos: p.pos,
            rot: 0.0,
        }
    }

    /// Rigid group with the original relative placement; unit rotation 0 = original orientation.
    pub fn rigid(d: &Design, name: &str, parts: &[usize]) -> Unit {
        let mut c = pt(0.0, 0.0);
        for &i in parts {
            c = c + d.parts[i].pos;
        }
        c = c * (1.0 / parts.len() as f64);
        Unit {
            name: name.to_string(),
            members: parts
                .iter()
                .map(|&i| Member {
                    part: i,
                    rel: d.parts[i].pos - c,
                    rel_rot: d.parts[i].rot,
                })
                .collect(),
            pos: c,
            rot: 0.0,
        }
    }

    /// Rigid group expressed in the frame of its first part (the anchor): unit pose = anchor pose.
    pub fn anchored(d: &Design, name: &str, anchor: usize, others: &[usize]) -> Unit {
        let a = &d.parts[anchor];
        let mut members = vec![Member {
            part: anchor,
            rel: pt(0.0, 0.0),
            rel_rot: 0.0,
        }];
        for &i in others {
            let p = &d.parts[i];
            members.push(Member {
                part: i,
                rel: kicad_rot(p.pos - a.pos, -a.rot),
                rel_rot: p.rot - a.rot,
            });
        }
        Unit {
            name: name.to_string(),
            members,
            pos: a.pos,
            rot: a.rot,
        }
    }

    pub fn member_pose(&self, m: &Member) -> (Point, f64) {
        (self.pos + kicad_rot(m.rel, self.rot), m.rel_rot + self.rot)
    }

    /// Courtyard rectangles as (part pose, local rect).
    pub fn quads(&self, d: &Design) -> Vec<Quad> {
        let mut out = Vec::new();
        for m in &self.members {
            let (pos, rot) = self.member_pose(m);
            for r in &d.parts[m.part].court {
                out.push(Quad { pos, rot, r: *r });
            }
        }
        out
    }
}

/// A courtyard rectangle of a part at some pose (any angle).
#[derive(Debug, Clone, Copy)]
pub struct Quad {
    pub pos: Point,
    pub rot: f64,
    pub r: R,
}

impl Quad {
    pub fn corners(&self) -> [Point; 4] {
        self.r.corners().map(|c| self.pos + kicad_rot(c, self.rot))
    }
    /// Point inside the rectangle grown by `g`.
    pub fn contains(&self, p: Point, g: f64) -> bool {
        let l = kicad_rot(p - self.pos, -self.rot);
        l.x >= self.r.x0 - g && l.x <= self.r.x1 + g && l.y >= self.r.y0 - g && l.y <= self.r.y1 + g
    }
    pub fn bbox(&self) -> R {
        self.r.transformed_bbox(self.pos, self.rot)
    }
    /// Lower bound of the distance to `o` (largest separation along the edge normals; negative if
    /// they overlap).
    pub fn separation(&self, o: &Quad) -> f64 {
        let a = self.corners();
        let b = o.corners();
        let mut best = f64::NEG_INFINITY;
        for poly in [&a, &b] {
            for i in 0..2 {
                let e = poly[i + 1] - poly[i];
                let n = pt(-e.y, e.x).norm();
                let (amin, amax) = proj(&a, n);
                let (bmin, bmax) = proj(&b, n);
                let s = (bmin - amax).max(amin - bmax);
                best = best.max(s);
            }
        }
        best
    }
}

fn proj(p: &[Point; 4], n: Point) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for q in p {
        let v = q.dot(n);
        lo = lo.min(v);
        hi = hi.max(v);
    }
    (lo, hi)
}

pub fn units_separation(d: &Design, a: &Unit, b: &Unit) -> f64 {
    let qa = a.quads(d);
    let qb = b.quads(d);
    let mut s = f64::INFINITY;
    for x in &qa {
        for y in &qb {
            s = s.min(x.separation(y));
        }
    }
    s
}

#[derive(Debug, Clone)]
pub enum Shape {
    Rect { w: f64, h: f64 },
    Circle { d: f64 },
}

/// The fixed mechanical parts of the LED controller.
pub struct FixedSet {
    pub usb: usize,
    pub module: usize,
    pub holes: Vec<usize>,
    /// Plug units of rail A and rail B, each plug with its attached parts.
    pub rail_a: Vec<Unit>,
    pub rail_b: Vec<Unit>,
}

/// Board origin (top left of the bounding box), as in the original file.
pub const ORIGIN: Point = pt(100.0, 100.0);

/// Distance from the module origin to its far (antenna) end, from the original board: the module
/// sits flush with the right edge at x = 200 with its origin at x = 187.2.
const MODULE_EDGE: f64 = 12.8;
/// Half width of the module (antenna end).
const MODULE_HALF: f64 = 9.0;
/// USB-C origin from the edge (0.8 mm overhang of the receptacle as in the original).
const USB_EDGE: f64 = 2.9;
/// Plug origin from the edge (original: y = 108.05 with the edge at y = 100).
const PLUG_EDGE: f64 = 8.05;
/// Mounting hole centre from the edges.
const HOLE_INSET: f64 = 4.0;
/// Rotations of the parts on the right / top edge in the original.
const MODULE_ROT_RIGHT: f64 = -90.0;
const USB_ROT_LEFT: f64 = -90.0;
const PLUG_ROT_TOP: f64 = 180.0;

pub struct Arrangement {
    pub outline: Outline,
    pub units: Vec<Unit>,
    /// Box into which the original placement of the movable parts is mapped as a start.
    pub core: R,
}

fn set_pose(u: &mut Unit, pos: Point, rot: f64) {
    u.pos = pos;
    u.rot = rot;
}

fn check_fixed(d: &Design, units: &[Unit], gap: f64) -> Result<()> {
    for i in 0..units.len() {
        for j in i + 1..units.len() {
            let s = units_separation(d, &units[i], &units[j]);
            if s < gap - 1e-6 {
                bail!(
                    "fixed parts {} and {} collide (separation {:.2} mm); the outline is too small",
                    units[i].name,
                    units[j].name,
                    s
                );
            }
        }
    }
    Ok(())
}

/// Width of a plug unit along the edge (x extent with the top edge rotation).
fn plug_width(d: &Design, u: &Unit) -> (f64, f64) {
    let mut t = u.clone();
    set_pose(&mut t, pt(0.0, 0.0), PLUG_ROT_TOP);
    let mut b = R::empty();
    for q in t.quads(d) {
        b = b.union(&q.bbox());
    }
    (b.x0, b.x1)
}

pub fn arrange(d: &Design, fx: &FixedSet, shape: &Shape, gap: f64) -> Result<Arrangement> {
    // fixed single parts: the unit rotation is the part rotation
    let single = |i: usize| {
        let mut u = Unit::single(d, i);
        u.members[0].rel_rot = 0.0;
        u.rot = d.parts[i].rot;
        u
    };
    let mut usb = single(fx.usb);
    let mut module = single(fx.module);
    let mut holes: Vec<Unit> = fx.holes.iter().map(|&i| single(i)).collect();
    let mut rail_a = fx.rail_a.clone();
    let mut rail_b = fx.rail_b.clone();
    let (px0, px1) = plug_width(d, &rail_a[0]);
    let pw = px1 - px0;
    match *shape {
        Shape::Rect { w, h } => {
            let o = ORIGIN;
            let outline = Outline::Rect {
                x0: o.x,
                y0: o.y,
                x1: o.x + w,
                y1: o.y + h,
            };
            set_pose(&mut module, o + pt(w - MODULE_EDGE, h / 2.0), MODULE_ROT_RIGHT);
            // the USB-C sits in the upper part of the left edge as in the original (y = 20 of 60)
            set_pose(&mut usb, o + pt(USB_EDGE, (h / 3.0).max(12.0)), USB_ROT_LEFT);
            // right holes in the corners if the antenna keepout (48 mm along the edge) leaves room
            let right_x = if h / 2.0 - 24.0 >= HOLE_INSET + 3.45 + gap {
                w - HOLE_INSET
            } else {
                w - 12.0
            };
            let hp = [
                pt(HOLE_INSET, HOLE_INSET),
                pt(right_x, HOLE_INSET),
                pt(HOLE_INSET, h - HOLE_INSET),
                pt(right_x, h - HOLE_INSET),
            ];
            for (u, p) in holes.iter_mut().zip(hp) {
                set_pose(u, o + p, 0.0);
            }
            // plug rows between the holes
            let lo = HOLE_INSET + 3.5 + gap;
            let hi = right_x - 3.5 - gap;
            let n = rail_a.len().max(rail_b.len()) as f64;
            let pitch = ((hi - lo - pw) / (n - 1.0)).min(18.5);
            if pitch < pw + gap {
                bail!(
                    "plug rows do not fit: {:.1} mm for {} plugs of {:.1} mm",
                    hi - lo,
                    n,
                    pw
                );
            }
            let start = (lo + hi) / 2.0 - pitch * (n - 1.0) / 2.0 - (px0 + px1) / 2.0;
            for (i, u) in rail_a.iter_mut().enumerate() {
                set_pose(u, o + pt(start + pitch * i as f64, PLUG_EDGE), PLUG_ROT_TOP);
            }
            // the bottom row is turned by 180 degrees: its extents along the edge are mirrored
            let start_b = start + (px0 + px1);
            for (i, u) in rail_b.iter_mut().enumerate() {
                set_pose(
                    u,
                    o + pt(start_b + pitch * i as f64, h - PLUG_EDGE),
                    PLUG_ROT_TOP - 180.0,
                );
            }
            let mut units = vec![usb, module];
            units.extend(holes);
            units.extend(rail_a);
            units.extend(rail_b);
            check_fixed(d, &units, gap)?;
            Ok(Arrangement {
                outline,
                units,
                core: outline.bbox(),
            })
        }
        Shape::Circle { d: dia } => {
            let r = dia / 2.0;
            let c = ORIGIN + pt(r, r);
            let outline = Outline::Circle { c, r };
            // polar placement: angle phi counter-clockwise from +x on screen (y down)
            let at = |phi: f64, rad: f64| c + pt(phi.to_radians().cos(), -phi.to_radians().sin()) * rad;
            let sag = |half: f64| r - (r * r - half * half).max(0.0).sqrt();
            // module on the right (0 deg), its antenna corners on the circle
            set_pose(
                &mut module,
                at(0.0, r - MODULE_EDGE - sag(MODULE_HALF)),
                MODULE_ROT_RIGHT,
            );
            set_pose(&mut usb, at(180.0, r - USB_EDGE), USB_ROT_LEFT);
            // plugs radially, rail A on the upper arc, rail B on the lower arc. The plug body is
            // not centred on its origin (pin 1): the middle of its front edge goes on the radius,
            // with the overhang of the original (front 0.45 mm over the edge) less the sagitta.
            let mut t = rail_a[0].clone();
            set_pose(&mut t, pt(0.0, 0.0), 0.0);
            let m0 = &t.members[0];
            let (mp, mr) = t.member_pose(m0);
            let pb = d.parts[m0.part]
                .court
                .iter()
                .fold(R::empty(), |a, r| a.union(&r.transformed_bbox(mp, mr)));
            // at rotation 0 the plug faces +y (bottom edge in the rectangle)
            let front_mid = pt((pb.x0 + pb.x1) / 2.0, pb.y1);
            let overhang = pb.y1 - PLUG_EDGE;
            let fr = r + overhang - sag((pb.x1 - pb.x0) / 2.0);
            let place_rail = |rail: &mut Vec<Unit>, centre: f64, pitch: f64, up: bool| {
                let n = rail.len() as f64;
                for (i, u) in rail.iter_mut().enumerate() {
                    let k = i as f64 - (n - 1.0) / 2.0;
                    // left to right: on the top arc the angle decreases, on the bottom it increases
                    let phi = if up { centre - k * pitch } else { centre + k * pitch };
                    let rot = phi + 90.0 - 360.0;
                    set_pose(u, at(phi, fr) - kicad_rot(front_mid, rot), rot);
                }
            };
            // smallest pitch at which neighbouring plugs (with PTC and cap) keep the gap
            let mut pitch = 10.0;
            loop {
                place_rail(&mut rail_a, 90.0, pitch, true);
                let ok = (0..rail_a.len() - 1).all(|i| units_separation(d, &rail_a[i], &rail_a[i + 1]) >= gap + 0.2);
                if ok {
                    break;
                }
                pitch += 0.25;
                if pitch > 60.0 {
                    bail!("plugs do not fit on the circle");
                }
            }
            // shift the rails away from the module if needed (toward the USB-C side)
            let mut centre = 90.0;
            loop {
                place_rail(&mut rail_a, centre, pitch, true);
                place_rail(&mut rail_b, 360.0 - centre, pitch, false);
                let free = |u: &Unit| {
                    units_separation(d, u, &module) >= gap + 0.5 && units_separation(d, u, &usb) >= gap + 0.5
                };
                if rail_a.iter().all(free) && rail_b.iter().all(free) {
                    break;
                }
                centre += 0.5;
                if centre > 130.0 {
                    bail!("plugs, module and USB-C do not fit on the circle of {dia} mm");
                }
            }
            // holes: one in each of the four arcs between the rails and the module / USB-C
            let mut placed: Vec<Unit> = vec![usb.clone(), module.clone()];
            placed.extend(rail_a.iter().cloned());
            placed.extend(rail_b.iter().cloned());
            let first_a = centre + 1.5 * pitch;
            let last_a = centre - 1.5 * pitch;
            let arcs = [
                (first_a, 180.0),
                (0.0, last_a),
                (180.0, 360.0 - first_a),
                (360.0 - last_a, 360.0),
            ];
            for (u, (a0, a1)) in holes.iter_mut().zip(arcs) {
                let mid = (a0 + a1) / 2.0;
                let mut found = false;
                let mut rad = r - HOLE_INSET - 0.5;
                'outer: while rad > r * 0.35 {
                    for step in 0..40 {
                        // alternate around the middle of the arc
                        let off = ((step + 1) / 2) as f64 * if step % 2 == 0 { 1.0 } else { -1.0 };
                        let phi = mid + off;
                        if phi <= a0.min(a1) || phi >= a0.max(a1) {
                            continue;
                        }
                        set_pose(u, at(phi, rad), 0.0);
                        if placed.iter().all(|p| units_separation(d, u, p) >= gap + 0.5) {
                            found = true;
                            break 'outer;
                        }
                    }
                    rad -= 0.5;
                }
                if !found {
                    bail!("no room for mounting hole {}", u.name);
                }
                placed.push(u.clone());
            }
            check_fixed(d, &placed, gap)?;
            // the original 100 x 60 placement (5:3) mapped into the inscribed rectangle
            let a = (3.0f64 / 5.0).atan();
            let (hw, hh) = (r * a.cos() * 0.98, r * a.sin() * 0.98);
            Ok(Arrangement {
                outline,
                units: placed,
                core: R {
                    x0: c.x - hw,
                    y0: c.y - hh,
                    x1: c.x + hw,
                    y1: c.y + hh,
                },
            })
        }
    }
}
