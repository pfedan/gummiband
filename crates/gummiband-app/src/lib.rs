//! The pipelines behind the front ends of Gummiband. The CLI (`gummiband`) and the MCP server
//! (`gummiband-mcp`) map their arguments to the options here, call, and present the result: an
//! option, its default and what it does exist once.

pub mod areas;
pub mod nets;
pub mod options;
pub mod route;
pub mod svg;

pub use areas::{AREA_OPTIONS, AreaOptions, AreaReport, area_option_doc, area_report};
pub use nets::{AppliedNetOptions, NetOptions, apply_net_options, parse_net_options, read_net_options};
pub use options::{Kind, OPTIONS, OptionDoc, option_doc};
pub use route::{
    Alternative, Angles, Angles45, Areas, Cleared, Event, Frames, Hooks, Placement, Room, RouteOptions, RouteOutcome,
    Routed, Trial, Tuned, Written, clear_corridors, copy_project_files, defaults, load_board, parse_pour, route_board,
    route_file,
};
