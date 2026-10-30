// Jacobi rounds of the track points, the same forces as the point loop in relax.rs, in f32
// around the middle of the board. `main` writes the move of every point, `apply` adds it. The
// piece grids are from the start of the batch: `drift` grows the queries to make up for that.

struct Params {
    n_points: u32,
    n_groups: u32,
    drift: f32,
    tension: f32,
    margin: f32,
    max_step: f32,
    edge_clr: f32,
    max_clr: f32,
    max_vr: f32,
    n_outline: u32,
    cell_x: f32,
    cell_y: f32,
    cell: f32,
    cell_nx: u32,
    cell_ny: u32,
    via_x: f32,
    via_y: f32,
    via_cell: f32,
    via_nx: u32,
    via_ny: u32,
}

struct Track {
    width: f32,
    neck_w: f32,
    half_w: f32,
    clr: f32,
    net_clr: f32,
    net: u32,
    group: u32,
    neck_lo: u32,
    neck_hi: u32,
    layer: u32,
}

struct Piece {
    // the piece runs from pts[start] to pts[start + 1]
    start: u32,
    half_w: f32,
    net: u32,
    group: u32,
}

struct Tier {
    origin: vec2f,
    cell: f32,
    nx: u32,
    ny: u32,
    start_base: u32,
    piece_base: u32,
    max_extent: f32,
    max_clr: f32,
    pad: u32,
}

struct Obstacle {
    min: vec2f,
    max: vec2f,
    clr: f32,
    own_net: u32,
    floor: u32,
    lo: u32,
    hi: u32,
    pad: u32,
}

struct Via {
    c: vec2f,
    r: f32,
    net: u32,
    clr: f32,
    pad: u32,
}

@group(0) @binding(0) var<uniform> pr: Params;
@group(0) @binding(1) var<storage, read_write> pts: array<vec2f>;
@group(0) @binding(2) var<storage, read> pt_track: array<u32>;
@group(0) @binding(3) var<storage, read> tracks: array<Track>;
@group(0) @binding(4) var<storage, read> necks: array<vec4f>;
@group(0) @binding(5) var<storage, read> pieces: array<Piece>;
@group(0) @binding(6) var<storage, read> tiers: array<Tier>;
@group(0) @binding(7) var<storage, read> tier_start: array<u32>;
// per layer the base into cell_start, then cell_start holds positions into cell_ids
@group(0) @binding(8) var<storage, read> cell_start: array<u32>;
@group(0) @binding(9) var<storage, read> cell_ids: array<u32>;
@group(0) @binding(10) var<storage, read> obstacles: array<Obstacle>;
@group(0) @binding(11) var<storage, read> edges: array<vec4f>;
@group(0) @binding(12) var<storage, read> near_edge: array<u32>;
@group(0) @binding(13) var<storage, read> outline: array<vec2u>;
@group(0) @binding(14) var<storage, read> vias: array<Via>;
@group(0) @binding(15) var<storage, read> via_grid: array<u32>;
@group(0) @binding(17) var<storage, read> groups: array<f32>;
@group(0) @binding(18) var<storage, read_write> moves: array<vec2f>;

fn clamp_cell(v: f32, o: f32, cell: f32, n: u32) -> u32 {
    return min(u32(max(floor((v - o) / cell), 0.0)), n - 1u);
}

fn point_cell(p: vec2f) -> u32 {
    return clamp_cell(p.y, pr.cell_y, pr.cell, pr.cell_ny) * pr.cell_nx
        + clamp_cell(p.x, pr.cell_x, pr.cell, pr.cell_nx);
}

// Distance to the boundary of the polygon in edges[lo..hi], the nearest point on it, and whether
// p is inside (even-odd).
struct Near {
    d: f32,
    q: vec2f,
    inside: bool,
}

fn nearest(p: vec2f, lo: u32, hi: u32) -> Near {
    var best = 1e30;
    var q = p;
    var inside = false;
    for (var i = lo; i < hi; i++) {
        let e = edges[i];
        let a = e.xy;
        let ab = e.zw - a;
        let l2 = dot(ab, ab);
        var t = 0.0;
        if l2 > 0.0 {
            t = clamp(dot(p - a, ab) / l2, 0.0, 1.0);
        }
        let c = a + ab * t;
        let dq = p - c;
        let d2 = dot(dq, dq);
        if d2 < best {
            best = d2;
            q = c;
        }
        if (e.y > p.y) != (e.w > p.y) {
            if p.x < e.x + (p.y - e.y) * (e.z - e.x) / (e.w - e.y) {
                inside = !inside;
            }
        }
    }
    return Near(sqrt(best), q, inside);
}

fn safe_norm(v: vec2f) -> vec2f {
    let l = length(v);
    if l > 0.0 {
        return v / l;
    }
    return v;
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) id: vec3u) {
    let i = id.x;
    if i >= pr.n_points {
        return;
    }
    let tag = pt_track[i];
    moves[i] = vec2f(0.0, 0.0);
    // the ends of a track stay
    if (tag >> 31u) != 0u {
        return;
    }
    let p = pts[i];
    let cell = point_cell(p);
    let t = tracks[tag];
    var w = t.width;
    for (var k = t.neck_lo; k < t.neck_hi; k++) {
        let r = necks[k];
        if p.x >= r.x && p.x <= r.z && p.y >= r.y && p.y <= r.w {
            w = t.neck_w;
            break;
        }
    }
    let half = w * 0.5;
    var mv = ((pts[i - 1u] + pts[i + 1u]) * 0.5 - p) * pr.tension;
    var push = vec2f(0.0, 0.0);

    // static obstacles
    let lo = cell_start[t.layer] + cell;
    let id_lo = cell_start[lo];
    let id_hi = cell_start[lo + 1u];
    for (var k = id_lo; k < id_hi; k++) {
        let o = obstacles[cell_ids[k]];
        if o.own_net == t.net {
            continue;
        }
        var need = o.clr;
        if o.floor != 0u {
            need = max(need, t.clr);
        }
        need += half + pr.margin;
        let dx = max(max(o.min.x - p.x, 0.0), p.x - o.max.x);
        let dy = max(max(o.min.y - p.y, 0.0), p.y - o.max.y);
        if dx * dx + dy * dy >= need * need {
            continue;
        }
        let n = nearest(p, o.lo, o.hi);
        var d = n.d;
        if n.inside {
            d = -d;
        }
        if d < need {
            if n.inside {
                push += safe_norm(n.q - p) * (need - d);
            } else {
                push += safe_norm(p - n.q) * (need - d);
            }
        }
    }

    // board edge, only near it
    if near_edge[cell] != 0u {
        var count = 0u;
        for (var k = 0u; k < pr.n_outline; k++) {
            let r = outline[k];
            if nearest(p, r.x, r.y).inside {
                count++;
            }
        }
        let inside_board = pr.n_outline == 0u || (count & 1u) == 1u;
        for (var k = 0u; k < pr.n_outline; k++) {
            let r = outline[k];
            let n = nearest(p, r.x, r.y);
            let need = pr.edge_clr + half + pr.margin;
            if n.d < need || !inside_board {
                if inside_board {
                    push += safe_norm(p - n.q) * (need - n.d);
                } else {
                    push += safe_norm(n.q - p) * (need + n.d);
                }
            }
        }
    }

    // vias of other nets
    let reach = half + pr.max_clr + pr.max_vr + pr.margin + 0.01;
    let vx0 = clamp_cell(p.x - reach, pr.via_x, pr.via_cell, pr.via_nx);
    let vx1 = clamp_cell(p.x + reach, pr.via_x, pr.via_cell, pr.via_nx);
    let vy0 = clamp_cell(p.y - reach, pr.via_y, pr.via_cell, pr.via_ny);
    let vy1 = clamp_cell(p.y + reach, pr.via_y, pr.via_cell, pr.via_ny);
    for (var y = vy0; y <= vy1; y++) {
        let row = y * pr.via_nx;
        let a = via_grid[row + vx0];
        let b = via_grid[row + vx1 + 1u];
        for (var k = a; k < b; k++) {
            let v = vias[via_grid[pr.via_nx * pr.via_ny + 1u + k]];
            if v.net == t.net {
                continue;
            }
            let need = v.r + max(v.clr, t.clr) + half + pr.margin;
            let dv = p - v.c;
            let d2 = dot(dv, dv);
            if d2 < need * need {
                let d = sqrt(d2);
                push += safe_norm(dv) * ((need - d) * 0.5);
            }
        }
    }

    // other tracks, they push back from their side
    let base = t.half_w + pr.margin;
    for (var ti = t.layer * 2u; ti < t.layer * 2u + 2u; ti++) {
        let g = tiers[ti];
        let radius = base + max(t.net_clr, g.max_clr) + g.max_extent + pr.drift;
        let x0 = clamp_cell(p.x - radius, g.origin.x, g.cell, g.nx);
        let x1 = clamp_cell(p.x + radius, g.origin.x, g.cell, g.nx);
        let y0 = clamp_cell(p.y - radius, g.origin.y, g.cell, g.ny);
        let y1 = clamp_cell(p.y + radius, g.origin.y, g.cell, g.ny);
        for (var y = y0; y <= y1; y++) {
            let row = g.start_base + y * g.nx;
            let a = tier_start[row + x0] + g.piece_base;
            let b = tier_start[row + x1 + 1u] + g.piece_base;
            for (var k = a; k < b; k++) {
                let o = pieces[k];
                if o.net == t.net {
                    continue;
                }
                let oa = pts[o.start];
                let ab = pts[o.start + 1u] - oa;
                let l2 = dot(ab, ab);
                var s = 0.0;
                if l2 > 0.0 {
                    s = clamp(dot(p - oa, ab) / l2, 0.0, 1.0);
                }
                let dq = p - (oa + ab * s);
                let d2 = dot(dq, dq);
                let need = half + o.half_w + groups[t.group * pr.n_groups + o.group] + pr.margin;
                if d2 < need * need {
                    let d = sqrt(d2);
                    if d > 1e-6 {
                        push += dq / d * ((need - d) * 0.5);
                    } else {
                        push += safe_norm(vec2f(-ab.y, ab.x)) * ((need - d) * 0.5);
                    }
                }
            }
        }
    }

    // constraints win over tension
    if dot(push, push) > 0.0 {
        let pn = safe_norm(push);
        let into = dot(mv, pn);
        if into < 0.0 {
            mv -= pn * into;
        }
        mv += push;
    }
    let l = length(mv);
    if l > pr.max_step {
        mv *= pr.max_step / l;
    }
    moves[i] = mv;
}

@compute @workgroup_size(64)
fn apply(@builtin(global_invocation_id) id: vec3u) {
    if id.x < pr.n_points {
        pts[id.x] += moves[id.x];
    }
}
