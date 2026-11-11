//! The pipelines behind the front ends of Gummiband. The CLI (`gummiband`) and the MCP server
//! (`gummiband-mcp`) map their arguments to the options here, call, and present the result: an
//! option, its default and what it does exist once.

pub mod export;
pub mod nets;
pub mod old;
pub mod options;
pub mod route;
pub mod svg;

pub use nets::{AppliedNetOptions, NetOptions, apply_net_options, parse_net_options, read_net_options};
pub use options::{Choice, Kind, OPTIONS, OptionDoc, option_doc};
pub use route::{
    Alternative, Angles, Angles45, Cleared, Event, Frames, Hooks, Pairs, Placement, Preset, PresetValues, Room,
    RouteOptions, RouteOutcome, Routed, Scope, Trial, Tuned, Written, clear_corridors, copy_project_files, defaults,
    load_board, route_board, route_file,
};
