//! The pipelines behind the front ends of Gummiband. The CLI (`gummiband`) and the MCP server
//! (`gummiband-mcp`) map their arguments to the options here, call, and present the result: an
//! option, its default and what it does exist once.

pub mod route;

pub use route::{
    Angles, Angles45, Cleared, Event, Hooks, Placement, Room, RouteOptions, RouteOutcome, Routed, Trial, Tuned,
    clear_corridors, defaults, load_board, route_board, route_file,
};
