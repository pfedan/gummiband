//! The pipelines behind the front ends of Gummiband. The CLI (`gummiband`) and the MCP server
//! (`gummiband-mcp`) map their arguments to the options here, call, and present the result: an
//! option, its default and what it does exist once.

pub mod nets;
pub mod route;

pub use nets::{AppliedNetOptions, NetOptions, apply_net_options, parse_net_options, read_net_options};
pub use route::{
    Alternative, Angles, Angles45, Cleared, Event, Hooks, Placement, Room, RouteOptions, RouteOutcome, Routed, Trial,
    Tuned, Written, clear_corridors, copy_project_files, defaults, load_board, route_board, route_file,
};
