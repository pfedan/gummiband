//! Placement of KiCad boards: `auto` places a board from its netlist with the locked and
//! mechanical parts fixed, `variant` and `planes` make the LED controller outline variants of the
//! `gummiband-place` program. The annealer and the footprint model are shared.

pub mod anneal;
pub mod auto;
pub mod model;
pub mod planes;
pub mod variant;
