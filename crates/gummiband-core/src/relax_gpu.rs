//! Experimental: the point loop of the relaxation as a compute shader. The way to the GPU and
//! back costs far more than a round, so the GPU runs a batch of rounds at a time, up to the next
//! resampling. Meanwhile the vias move on the CPU, against the tracks of the batch start. The
//! shader sees f32 coordinates around the middle of the board and returns how far each point
//! went, which the caller adds to its f64 points, so a point that does not move stays exactly
//! where it is.

use std::collections::HashMap;
use std::sync::OnceLock;
use std::time::Instant;

use bytemuck::{Pod, Zeroable};
use rayon::prelude::*;
use wgpu::util::DeviceExt;

use super::{CellLists, LayerPieces, NearObstacle, Piece, RelaxConfig};
use crate::board::Board;
use crate::geom::{Point, Polygon};
use crate::index::PointGrid;
use crate::prof::{Slot, add, time};
use crate::realize::Geometry;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
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

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct TrackG {
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

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct PieceG {
    start: u32,
    half_w: f32,
    net: u32,
    group: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct TierG {
    origin: [f32; 2],
    cell: f32,
    nx: u32,
    ny: u32,
    start_base: u32,
    piece_base: u32,
    max_extent: f32,
    max_clr: f32,
    pad: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct ObstacleG {
    min: [f32; 2],
    max: [f32; 2],
    clr: f32,
    own_net: u32,
    floor: u32,
    lo: u32,
    hi: u32,
    pad: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct ViaG {
    c: [f32; 2],
    r: f32,
    net: u32,
    clr: f32,
    pad: u32,
}

/// The device and the pipeline, once per process.
struct Gpu {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::ComputePipeline,
    apply: wgpu::ComputePipeline,
}

fn gpu() -> Option<&'static Gpu> {
    static GPU: OnceLock<Option<Gpu>> = OnceLock::new();
    GPU.get_or_init(|| {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::METAL,
            ..wgpu::InstanceDescriptor::new_without_display_handle()
        });
        let adapter = pollster::block_on(instance.request_adapter(&Default::default())).ok()?;
        eprintln!("gpu: {}", adapter.get_info().name);
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            required_limits: adapter.limits(),
            ..Default::default()
        }))
        .ok()?;
        // the bindings are sized by the code above and every loop ends
        let module = unsafe {
            device.create_shader_module_trusted(
                wgpu::include_wgsl!("relax_gpu.wgsl"),
                wgpu::ShaderRuntimeChecks::unchecked(),
            )
        };
        let pipeline_of = |entry| {
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: None,
                layout: None,
                module: &module,
                entry_point: Some(entry),
                compilation_options: Default::default(),
                cache: None,
            })
        };
        let (pipeline, apply) = (pipeline_of("main"), pipeline_of("apply"));
        Some(Gpu {
            device,
            queue,
            pipeline,
            apply,
        })
    })
    .as_ref()
}

/// How far a piece may wander from its grid cell during a batch before it is overlooked, in mm.
const DRIFT: f64 = 0.15;

/// Bindings that change per round, by their binding number.
const DYNAMIC: [u32; 7] = [1, 2, 5, 6, 7, 14, 15];

pub(super) struct GpuRelax {
    g: &'static Gpu,
    origin: Point,
    params: Params,
    params_buf: wgpu::Buffer,
    statics: Vec<(u32, wgpu::Buffer)>,
    dynamic: Vec<Option<wgpu::Buffer>>,
    moves: Option<(wgpu::Buffer, wgpu::Buffer)>,
    bind: Option<(wgpu::BindGroup, wgpu::BindGroup)>,
    net_group: Vec<u32>,
    capacity: u64,
    running: Option<wgpu::SubmissionIndex>,
    // reused per batch
    pts: Vec<[f32; 2]>,
    tags: Vec<u32>,
    pieces: Vec<PieceG>,
    tiers: Vec<TierG>,
    tier_start: Vec<u32>,
    via_list: Vec<ViaG>,
    via_cells: Vec<u32>,
    piece_pt: Vec<u32>,
}

/// What stays the same during one relaxation.
pub(super) struct Setup<'a> {
    pub board: &'a Board,
    pub geo: &'a Geometry,
    pub cfg: &'a RelaxConfig,
    pub origin: Point,
    pub cells: &'a CellLists,
    pub layer_cells: &'a [CellLists],
    pub near_obs: &'a [NearObstacle],
    pub obs_polys: &'a [Polygon],
    pub near_edge: &'a [bool],
    pub net_clr: &'a [f64],
    pub half_w: &'a [f64],
    pub clr: &'a [f64],
    pub max_clr: f64,
    pub max_vr: f64,
    pub via_grid: &'a PointGrid,
}

/// What the GPU needs at the start of a batch.
pub(super) struct Round<'a> {
    pub tracks: &'a [Vec<Point>],
    pub grids: &'a [LayerPieces],
    pub pieces: &'a [Piece],
    pub vias: &'a [(Point, f64, u32, f64)],
    pub via_grid: &'a PointGrid,
    pub tension: f64,
    /// Rounds to run.
    pub rounds: u32,
}

/// Nets of one group have the same clearance to every other net: one group per class
/// clearance, and one for every net with custom rules.
fn clearance_groups(board: &Board) -> (Vec<u32>, Vec<f32>) {
    let mut special: Vec<u32> = board.pair_clearance.keys().flat_map(|&(a, b)| [a, b]).collect();
    special.sort_unstable();
    special.dedup();
    let mut by_clr: HashMap<u64, u32> = HashMap::new();
    let mut reps: Vec<u32> = Vec::new();
    let net_group: Vec<u32> = (0..board.nets.len() as u32)
        .map(|n| {
            let key = if special.binary_search(&n).is_ok() {
                u64::MAX - n as u64
            } else {
                board.class_of(n).clearance.to_bits()
            };
            *by_clr.entry(key).or_insert_with(|| {
                reps.push(n);
                reps.len() as u32 - 1
            })
        })
        .collect();
    let mut matrix = Vec::with_capacity(reps.len() * reps.len());
    for &a in &reps {
        for &b in &reps {
            matrix.push(board.track_clearance(a, b) as f32);
        }
    }
    (net_group, matrix)
}

fn f2(p: Point, o: Point) -> [f32; 2] {
    [(p.x - o.x) as f32, (p.y - o.y) as f32]
}

impl GpuRelax {
    pub fn new(s: Setup<'_>) -> Option<GpuRelax> {
        let g = gpu()?;
        let o = s.origin;
        let (net_group, groups) = clearance_groups(s.board);
        let mut necks: Vec<[f32; 4]> = Vec::new();
        let tracks: Vec<TrackG> = s
            .geo
            .tracks
            .iter()
            .enumerate()
            .map(|(i, t)| {
                let neck_lo = necks.len() as u32;
                for r in &t.necks {
                    let (a, b) = (f2(r.min, o), f2(r.max, o));
                    necks.push([a[0], a[1], b[0], b[1]]);
                }
                TrackG {
                    width: t.width as f32,
                    neck_w: t.neck_w as f32,
                    half_w: s.half_w[i] as f32,
                    clr: s.clr[t.net as usize] as f32,
                    net_clr: s.net_clr[t.net as usize] as f32,
                    net: t.net,
                    group: net_group[t.net as usize],
                    neck_lo,
                    neck_hi: necks.len() as u32,
                    layer: t.layer as u32,
                }
            })
            .collect();
        let mut edges: Vec<[f32; 4]> = Vec::new();
        let mut add_poly = |p: &Polygon| {
            let lo = edges.len() as u32;
            for (a, b) in p.edges() {
                let (a, b) = (f2(a, o), f2(b, o));
                edges.push([a[0], a[1], b[0], b[1]]);
            }
            (lo, edges.len() as u32)
        };
        let obstacles: Vec<ObstacleG> = s
            .near_obs
            .iter()
            .zip(s.obs_polys)
            .map(|(n, p)| {
                let (lo, hi) = add_poly(p);
                ObstacleG {
                    min: f2(n.bbox.min, o),
                    max: f2(n.bbox.max, o),
                    clr: n.clr as f32,
                    own_net: n.own_net,
                    floor: n.floor as u32,
                    lo,
                    hi,
                    pad: 0,
                }
            })
            .collect();
        let outline: Vec<[u32; 2]> = s
            .board
            .outline
            .iter()
            .map(|p| {
                let (lo, hi) = add_poly(p);
                [lo, hi]
            })
            .collect();
        // per layer the base of its starts, then the starts as positions in one id list
        let n_layers = s.layer_cells.len();
        let mut cell_start: Vec<u32> = vec![0; n_layers];
        let mut cell_ids: Vec<u32> = Vec::new();
        for (l, c) in s.layer_cells.iter().enumerate() {
            cell_start[l] = cell_start.len() as u32;
            let base = cell_ids.len() as u32;
            cell_start.extend(c.start.iter().map(|x| x + base));
            cell_ids.extend_from_slice(&c.ids);
        }
        let near_edge: Vec<u32> = s.near_edge.iter().map(|&b| b as u32).collect();
        let storage = |data: &[u8]| {
            let empty = [0u8; 16];
            g.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: None,
                contents: if data.is_empty() { &empty } else { data },
                usage: wgpu::BufferUsages::STORAGE,
            })
        };
        let statics = vec![
            (3, storage(bytemuck::cast_slice(&tracks))),
            (4, storage(bytemuck::cast_slice(&necks))),
            (8, storage(bytemuck::cast_slice(&cell_start))),
            (9, storage(bytemuck::cast_slice(&cell_ids))),
            (10, storage(bytemuck::cast_slice(&obstacles))),
            (11, storage(bytemuck::cast_slice(&edges))),
            (12, storage(bytemuck::cast_slice(&near_edge))),
            (13, storage(bytemuck::cast_slice(&outline))),
            (17, storage(bytemuck::cast_slice(&groups))),
        ];
        let (via_origin, via_cell, via_nx, via_ny) = s.via_grid.layout();
        let params = Params {
            n_points: 0,
            n_groups: (groups.len() as f64).sqrt().round() as u32,
            drift: DRIFT as f32,
            tension: 0.0,
            margin: s.cfg.margin as f32,
            max_step: s.cfg.max_step as f32,
            edge_clr: s.board.rules.edge_clearance as f32,
            max_clr: s.max_clr as f32,
            max_vr: s.max_vr as f32,
            n_outline: outline.len() as u32,
            cell_x: (s.cells.origin.x - o.x) as f32,
            cell_y: (s.cells.origin.y - o.y) as f32,
            cell: s.cells.cell as f32,
            cell_nx: s.cells.nx as u32,
            cell_ny: s.cells.ny as u32,
            via_x: (via_origin.x - o.x) as f32,
            via_y: (via_origin.y - o.y) as f32,
            via_cell: via_cell as f32,
            via_nx: via_nx as u32,
            via_ny: via_ny as u32,
        };
        let params_buf = g.device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: std::mem::size_of::<Params>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        Some(GpuRelax {
            g,
            origin: o,
            params,
            params_buf,
            statics,
            dynamic: vec![None; DYNAMIC.len()],
            moves: None,
            bind: None,
            net_group,
            capacity: 0,
            running: None,
            pts: Vec::new(),
            tags: Vec::new(),
            pieces: Vec::new(),
            tiers: Vec::new(),
            tier_start: Vec::new(),
            via_list: Vec::new(),
            via_cells: Vec::new(),
            piece_pt: Vec::new(),
        })
    }

    fn gather(&mut self, r: &Round<'_>, geo: &Geometry) {
        let o = self.origin;
        self.pts.clear();
        self.tags.clear();
        self.piece_pt.clear();
        for (ti, p) in r.tracks.iter().enumerate() {
            let n = p.len();
            let first = self.pts.len() as u32;
            self.piece_pt.extend(first..first + n as u32 - 1);
            for (i, q) in p.iter().enumerate() {
                self.pts.push(f2(*q, o));
                let end = if i == 0 || i + 1 == n { 1u32 << 31 } else { 0 };
                self.tags.push(ti as u32 | end);
            }
        }
        self.tiers.clear();
        self.tier_start.clear();
        self.pieces.clear();
        let (mut piece_base, net_group, piece_pt) = (0u32, &self.net_group, &self.piece_pt);
        for l in r.grids {
            for tier in [&l.fine, &l.far] {
                let (origin, cell, nx, ny) = tier.grid.layout();
                self.tiers.push(TierG {
                    origin: f2(origin, o),
                    cell: cell as f32,
                    nx: nx as u32,
                    ny: ny as u32,
                    start_base: self.tier_start.len() as u32,
                    piece_base,
                    max_extent: tier.max_extent as f32,
                    max_clr: tier.max_clr as f32,
                    pad: 0,
                });
                self.tier_start.extend_from_slice(tier.grid.starts());
                piece_base += tier.near.len() as u32;
                self.pieces.par_extend(tier.near.par_iter().map(|n| {
                    let pc = &r.pieces[n.id as usize];
                    let t = &geo.tracks[pc.track as usize];
                    PieceG {
                        start: piece_pt[n.id as usize],
                        half_w: (t.piece_width(pc.a, pc.b) * 0.5) as f32,
                        net: t.net,
                        group: net_group[t.net as usize],
                    }
                }));
            }
        }
        self.via_list.clear();
        self.via_list.extend(r.vias.iter().map(|v| ViaG {
            c: f2(v.0, o),
            r: v.1 as f32,
            net: v.2,
            clr: v.3 as f32,
            pad: 0,
        }));
        self.via_cells.clear();
        self.via_cells.extend_from_slice(r.via_grid.starts());
        self.via_cells.extend_from_slice(r.via_grid.ids());
    }

    /// Start a batch of rounds; `finish` returns its result.
    pub fn start(&mut self, r: &Round<'_>, geo: &Geometry) {
        time(Slot::GpuGather, || self.gather(r, geo));
        let t_up = Instant::now();
        let g = self.g;
        let datas: [&[u8]; 7] = [
            bytemuck::cast_slice(&self.pts),
            bytemuck::cast_slice(&self.tags),
            bytemuck::cast_slice(&self.pieces),
            bytemuck::cast_slice(&self.tiers),
            bytemuck::cast_slice(&self.tier_start),
            bytemuck::cast_slice(&self.via_list),
            bytemuck::cast_slice(&self.via_cells),
        ];
        for (slot, data) in self.dynamic.iter_mut().zip(datas) {
            let need = (data.len() as u64).max(16);
            if slot.as_ref().is_none_or(|b| b.size() < need) {
                *slot = Some(g.device.create_buffer(&wgpu::BufferDescriptor {
                    label: None,
                    size: (need * 3 / 2).next_multiple_of(16),
                    usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
                    mapped_at_creation: false,
                }));
                self.bind = None;
            }
            if !data.is_empty() {
                g.queue.write_buffer(slot.as_ref().unwrap(), 0, data);
            }
        }
        let n = self.pts.len();
        let bytes = (n as u64 * 8).max(16);
        if bytes > self.capacity {
            self.capacity = (bytes * 3 / 2).next_multiple_of(16);
            let buffer = |usage| {
                g.device.create_buffer(&wgpu::BufferDescriptor {
                    label: None,
                    size: self.capacity,
                    usage,
                    mapped_at_creation: false,
                })
            };
            self.moves = Some((
                buffer(wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC),
                buffer(wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST),
            ));
            self.bind = None;
        }
        self.params.n_points = n as u32;
        self.params.tension = r.tension as f32;
        g.queue
            .write_buffer(&self.params_buf, 0, bytemuck::bytes_of(&self.params));
        let (moves, staging) = self.moves.as_ref().unwrap();
        if self.bind.is_none() {
            fn entry(binding: u32, buf: &wgpu::Buffer) -> wgpu::BindGroupEntry<'_> {
                wgpu::BindGroupEntry {
                    binding,
                    resource: buf.as_entire_binding(),
                }
            }
            let mut entries = vec![entry(0, &self.params_buf), entry(18, moves)];
            for (b, buf) in &self.statics {
                entries.push(entry(*b, buf));
            }
            for (b, buf) in DYNAMIC.iter().zip(&self.dynamic) {
                entries.push(entry(*b, buf.as_ref().unwrap()));
            }
            let bound = |pipeline: &wgpu::ComputePipeline, entries: &[wgpu::BindGroupEntry]| {
                g.device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: None,
                    layout: &pipeline.get_bind_group_layout(0),
                    entries,
                })
            };
            let few = [
                entry(0, &self.params_buf),
                entry(1, self.dynamic[0].as_ref().unwrap()),
                entry(18, moves),
            ];
            self.bind = Some((bound(&g.pipeline, &entries), bound(&g.apply, &few)));
        }
        let mut enc = g.device.create_command_encoder(&Default::default());
        let groups = (n as u32).div_ceil(64);
        let (bind, bind_apply) = self.bind.as_ref().unwrap();
        for _ in 0..r.rounds {
            let mut pass = enc.begin_compute_pass(&Default::default());
            pass.set_pipeline(&g.pipeline);
            pass.set_bind_group(0, bind, &[]);
            pass.dispatch_workgroups(groups, 1, 1);
            drop(pass);
            let mut pass = enc.begin_compute_pass(&Default::default());
            pass.set_pipeline(&g.apply);
            pass.set_bind_group(0, bind_apply, &[]);
            pass.dispatch_workgroups(groups, 1, 1);
        }
        enc.copy_buffer_to_buffer(self.dynamic[0].as_ref().unwrap(), 0, staging, 0, n as u64 * 8);
        add(Slot::GpuUpload, t_up.elapsed().as_nanos() as u64);
        self.running = Some(g.queue.submit([enc.finish()]));
    }

    /// Wait for the batch: how far each point went, in order.
    pub fn finish(&mut self) -> Vec<[f32; 2]> {
        let t_wait = Instant::now();
        let g = self.g;
        let n = self.pts.len();
        let sub = self.running.take().expect("no batch running");
        let (_, staging) = self.moves.as_ref().unwrap();
        let slice = staging.slice(..n as u64 * 8);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        g.device
            .poll(wgpu::PollType::Wait {
                submission_index: Some(sub),
                timeout: None,
            })
            .expect("gpu batch failed");
        let view = slice.get_mapped_range().expect("mapped");
        let out = bytemuck::cast_slice::<u8, [f32; 2]>(&view)
            .iter()
            .zip(&self.pts)
            .map(|(a, b)| [a[0] - b[0], a[1] - b[1]])
            .collect();
        drop(view);
        staging.unmap();
        add(Slot::GpuWait, t_wait.elapsed().as_nanos() as u64);
        out
    }
}
