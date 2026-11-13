//! Experimental: the copper zones of a board file as areas with the settings of their fill, for
//! the fill model of `gummiband_core::fill`.

use std::collections::HashMap;

use gummiband_core::board::{LayerId, LayerSet};
use gummiband_core::fill::{Area, Connect};
use gummiband_core::geom::{Point, Polygon, pt};

use crate::read::KicadBoard;
use crate::sexpr::Node;

/// What a board file says about its areas.
#[derive(Debug, Clone, Default)]
pub struct Areas {
    /// One area per zone and layer.
    pub areas: Vec<Area>,
    /// The fill KiCad wrote for each area (empty: not filled), same order as `areas`.
    pub filled: Vec<Vec<Polygon>>,
    /// Pads that say themselves how they join a zone, by pin.
    pub pad_connect: HashMap<usize, Connect>,
    /// Rule areas that forbid copper fill.
    pub no_pour: Vec<(LayerSet, Polygon)>,
}

fn pts_of(n: &Node) -> Vec<Point> {
    n.child("pts")
        .map(|p| {
            p.children("xy")
                .filter_map(|xy| {
                    let a = xy.args();
                    Some(pt(a.first()?.as_f64()?, a.get(1)?.as_f64()?))
                })
                .collect()
        })
        .unwrap_or_default()
}

fn layers_of(kb: &KicadBoard, z: &Node) -> Vec<(LayerId, String)> {
    let mut names: Vec<String> = Vec::new();
    for key in ["layers", "layer"] {
        if let Some(l) = z.child(key) {
            for x in l.args().iter().filter_map(|x| x.as_str()) {
                match x {
                    "*.Cu" | "F&B.Cu" => names.extend(kb.layer_names.iter().cloned()),
                    _ => names.push(x.to_string()),
                }
            }
        }
    }
    names
        .into_iter()
        .filter_map(|n| Some((kb.board.layer_id(&n)?, n)))
        .collect()
}

/// KiCad's `zone_connect` of a pad or footprint.
fn connect_of(code: f64) -> Option<Connect> {
    match code as i64 {
        0 => Some(Connect::None),
        1 => Some(Connect::Thermal),
        2 => Some(Connect::Solid),
        3 => Some(Connect::ThermalTht),
        _ => None,
    }
}

/// The copper zones and the fill rule areas at the root of the board file, and the pads that
/// set their own zone connection.
pub fn areas(kb: &KicadBoard) -> Areas {
    let b = &kb.board;
    let by_number: HashMap<i64, &str> = kb
        .net_numbers
        .as_ref()
        .map(|m| m.iter().map(|(k, v)| (*v, k.as_str())).collect())
        .unwrap_or_default();
    let mut out = Areas::default();
    for z in kb.root.children("zone") {
        let rings: Vec<Polygon> = z
            .children("polygon")
            .map(pts_of)
            .filter(|p| p.len() >= 3)
            .map(Polygon::new)
            .collect();
        if rings.is_empty() {
            continue;
        }
        let layers = layers_of(kb, z);
        if let Some(k) = z.child("keepout") {
            if k.child_str("copperpour") == Some("not_allowed") {
                let mut set = LayerSet::default();
                for (l, _) in &layers {
                    set.insert(*l);
                }
                out.no_pour.extend(rings.into_iter().map(|r| (set, r)));
            }
            continue;
        }
        let name = z.child_str("net_name").map(str::to_string).or_else(|| {
            let a = z.child("net")?.args();
            let first = a.first()?;
            match (first.as_f64(), first.as_str()) {
                (Some(num), _) if !by_number.is_empty() => by_number.get(&(num as i64)).map(|s| s.to_string()),
                (_, Some(s)) => Some(s.to_string()),
                _ => None,
            }
        });
        let Some(net) = name.as_deref().filter(|n| !n.is_empty()).and_then(|n| b.net_id(n)) else {
            continue;
        };
        let pads = z.child("connect_pads");
        let connect = match pads.and_then(|p| p.args().first()).and_then(|x| x.as_str()) {
            Some("yes") => Connect::Solid,
            Some("no") => Connect::None,
            Some("thru_hole_only") => Connect::ThermalTht,
            _ => Connect::Thermal,
        };
        let fill = z.child("fill");
        let island_min = match fill.and_then(|f| f.child_f64("island_removal_mode")).unwrap_or(0.0) as i64 {
            1 => 0.0,
            2 => fill.and_then(|f| f.child_f64("island_area_min")).unwrap_or(0.0),
            _ => f64::INFINITY,
        };
        for (layer, lname) in layers {
            out.areas.push(Area {
                net,
                layer,
                name: z.child_str("name").unwrap_or("").to_string(),
                rings: rings.clone(),
                priority: z.child_f64("priority").unwrap_or(0.0) as i64,
                tie: z.child_str("uuid").unwrap_or("").to_string(),
                clearance: pads.and_then(|p| p.child_f64("clearance")).unwrap_or(0.0),
                min_width: z.child_f64("min_thickness").unwrap_or(0.25),
                connect,
                thermal_gap: fill.and_then(|f| f.child_f64("thermal_gap")).unwrap_or(0.5),
                spoke_width: fill.and_then(|f| f.child_f64("thermal_bridge_width")).unwrap_or(0.5),
                island_min,
            });
            out.filled.push(
                z.children("filled_polygon")
                    .filter(|f| f.child_str("layer") == Some(lname.as_str()))
                    .map(pts_of)
                    .filter(|p| p.len() >= 3)
                    .map(Polygon::new)
                    .collect(),
            );
        }
    }
    // pads (or their footprints) with a zone connection of their own
    let mut by_name: HashMap<&str, Vec<usize>> = HashMap::new();
    for (i, p) in b.pins.iter().enumerate() {
        by_name.entry(p.name.as_str()).or_default().push(i);
    }
    for fp in kb
        .root
        .args()
        .iter()
        .filter(|n| matches!(n.head(), Some("footprint" | "module")))
    {
        let reference = fp
            .children("property")
            .find(|p| p.args().first().and_then(|x| x.as_str()) == Some("Reference"))
            .and_then(|p| p.args().get(1)?.as_str())
            .unwrap_or("?");
        let of_part = fp.child_f64("zone_connect").and_then(connect_of);
        for pad in fp.children("pad") {
            let Some(c) = pad.child_f64("zone_connect").and_then(connect_of).or(of_part) else {
                continue;
            };
            let number = pad.args().first().and_then(|x| x.as_str()).unwrap_or("");
            for &pin in by_name
                .get(format!("{reference}.{number}").as_str())
                .into_iter()
                .flatten()
            {
                out.pad_connect.insert(pin, c);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::read::{ReadOptions, parse_board};

    #[test]
    fn zones_are_read_with_the_settings_of_their_fill() {
        let text = r#"(kicad_pcb (version 20240108) (generator "test")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "") (net 1 "GND")
  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (stroke (width 0.1)))
  (footprint "R" (layer "F.Cu") (at 5 5) (property "Reference" "R1")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "GND") (zone_connect 2))
    (pad "2" smd rect (at 2 0) (size 1 1) (layers "F.Cu") (net 1 "GND")))
  (zone (net 1) (net_name "GND") (layers "F.Cu" "B.Cu") (uuid "aa") (name "pour") (priority 2)
    (connect_pads thru_hole_only (clearance 0.3)) (min_thickness 0.2)
    (fill yes (thermal_gap 0.4) (thermal_bridge_width 0.35) (island_removal_mode 2) (island_area_min 3))
    (polygon (pts (xy 0 0) (xy 20 0) (xy 20 20) (xy 0 20)))
    (filled_polygon (layer "B.Cu") (pts (xy 1 1) (xy 19 1) (xy 19 19) (xy 1 19))))
  (zone (layers "F.Cu") (uuid "bb") (keepout (tracks allowed) (vias allowed) (copperpour not_allowed))
    (polygon (pts (xy 10 10) (xy 12 10) (xy 12 12) (xy 10 12))))
)"#;
        let kb = parse_board(text.to_string(), None, &ReadOptions::default()).unwrap();
        let a = areas(&kb);
        assert_eq!(a.areas.len(), 2);
        let f = &a.areas[0];
        assert_eq!((f.layer, f.priority, f.tie.as_str()), (0, 2, "aa"));
        assert_eq!(
            (f.clearance, f.min_width, f.thermal_gap, f.spoke_width),
            (0.3, 0.2, 0.4, 0.35)
        );
        assert_eq!((f.connect, f.island_min), (Connect::ThermalTht, 3.0));
        assert!(a.filled[0].is_empty());
        assert_eq!(a.filled[1].len(), 1);
        assert_eq!(a.no_pour.len(), 1);
        let pin = kb.board.pins.iter().position(|p| p.name == "R1.1").unwrap();
        assert_eq!(a.pad_connect.get(&pin), Some(&Connect::Solid));
        assert_eq!(a.pad_connect.len(), 1);
    }
}
