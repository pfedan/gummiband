//! What a watched run shows its viewer ([`crate::watch`]): all its copper as a whole view, and
//! each connection it lays or gives up as a step. [`Router::route_one`] calls both.

use super::{Goal, Router};
use crate::geom::Point;
use crate::watch::{Dot, Line, Stage, UNTAGGED, View, Watch};

/// What the viewer of a watched run has, for the changes of the next step.
#[derive(Clone, Default)]
pub(super) struct Shown {
    /// The segments of each connection.
    segs: Vec<Vec<u32>>,
    /// The via sites, sorted.
    sites: Vec<u32>,
}

impl<'a> Router<'a> {
    /// The line of an open connection for a viewer: from its pin to the nearest pin it may end
    /// on.
    fn open_line(&self, ci: usize) -> (Point, Point, u32) {
        let pos = |p: usize| self.board.pins[p].pos;
        let c = &self.conns[ci];
        let from = pos(c.from);
        let to = match &c.goal {
            Goal::Pins(t) => t
                .iter()
                .map(|&p| pos(p))
                .min_by(|a, b| a.dist(from).total_cmp(&b.dist(from))),
            _ => None,
        };
        (from, to.unwrap_or(from), ci as u32)
    }

    /// Shows all the copper of this run as the topology gives it now, and what is still open.
    pub(super) fn view_all(&self, w: &Watch, show: &dyn Fn(&View)) {
        let (geo, segs, sites) = if self.soft_on {
            self.soft_tagged()
        } else {
            self.tagged_geometry()
        };
        let mut conn_of = vec![UNTAGGED; self.segs.len()];
        for (ci, ids) in self.conn_segs.iter().enumerate() {
            for &sid in ids {
                conn_of[sid as usize] = ci as u32;
            }
        }
        let mut v = View::empty(Stage::Laying, w.phase());
        // the vias of the view are those with a drill, in the order of the geometry
        let drilled: Vec<u32> = geo
            .vias
            .iter()
            .zip(&sites)
            .filter(|(g, _)| g.drill > 0.0)
            .map(|(_, &s)| s)
            .collect();
        v.add(
            &geo,
            |i| conn_of.get(segs[i] as usize).copied().unwrap_or(UNTAGGED),
            |_| UNTAGGED,
        );
        for (d, &s) in v.dots.iter_mut().zip(&drilled) {
            d.site = s;
        }
        for (ci, ids) in self.conn_segs.iter().enumerate() {
            if ids.is_empty() {
                v.open.push(self.open_line(ci));
            } else {
                v.step.0 += 1;
            }
        }
        v.step.1 = self.conns.len();
        show(&v);
    }

    /// Shows what changed since the last step: the connections that got other segments or lost
    /// theirs, with their copper as it lies now, and the vias that came and went. The copper
    /// of the other connections stays as the viewer has it, though the new neighbour may push
    /// it aside: the next whole view puts that right.
    pub(super) fn view_step(&mut self, w: &Watch, explored: &[(Point, u8)]) {
        let Some(mut shown) = self.shown.take() else { return };
        shown.segs.resize(self.conns.len(), Vec::new());
        let changed: Vec<u32> = (0..self.conns.len())
            .filter(|&ci| shown.segs[ci] != self.conn_segs[ci])
            .map(|ci| ci as u32)
            .collect();
        for &ci in &changed {
            shown.segs[ci as usize].clone_from(&self.conn_segs[ci as usize]);
        }
        let mut sites: Vec<u32> = self.vias.keys().copied().collect();
        sites.sort_unstable();
        let gone_sites: Vec<u32> = shown
            .sites
            .iter()
            .copied()
            .filter(|s| sites.binary_search(s).is_err())
            .collect();
        let new_sites: Vec<u32> = sites
            .iter()
            .copied()
            .filter(|s| shown.sites.binary_search(s).is_err())
            .collect();
        shown.sites = sites;
        self.shown = Some(shown);
        let me = &*self;
        w.attempt(|show| {
            let mut tracks: Vec<(u32, crate::realize::Track)> = Vec::new();
            for &ci in &changed {
                for &sid in &me.conn_segs[ci as usize] {
                    tracks.extend(me.seg_tracks(sid).into_iter().map(|t| (ci, t)));
                }
            }
            let mut v = View::empty(Stage::Step, w.phase());
            v.lines = tracks
                .iter()
                .map(|(ci, t)| Line {
                    pts: &t.pts,
                    width: t.width,
                    layer: t.layer,
                    net: t.net,
                    conn: *ci,
                })
                .collect();
            v.dots = new_sites
                .iter()
                .filter_map(|&site| {
                    let via = me.placed_via(site, &me.vias[&site])?;
                    (via.drill > 0.0).then_some(Dot {
                        at: via.at,
                        diameter: via.diameter,
                        drill: via.drill,
                        net: via.net,
                        half: via.half,
                        site,
                    })
                })
                .collect();
            v.open = changed
                .iter()
                .filter(|&&ci| me.conn_segs[ci as usize].is_empty())
                .map(|&ci| me.open_line(ci as usize))
                .collect();
            v.gone = &changed;
            v.gone_sites = &gone_sites;
            v.explored = explored;
            v.step = (me.conn_segs.iter().filter(|s| !s.is_empty()).count(), me.conns.len());
            show(&v);
        });
    }

    /// A copy that tries what the run may not keep: it shows the watch nothing. (A copy made to
    /// go back to is a plain `clone`, and the run is watched on after it is put back.)
    pub(crate) fn trial(&self) -> Router<'a> {
        let mut r = self.clone();
        r.cfg.watch = None;
        r
    }
}
