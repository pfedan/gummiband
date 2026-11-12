//! The table of the route options as JSON, for a front end that is not written in Rust. The
//! KiCad plugin keeps a copy next to its sources (`plugin/gb_plugin/route_options.json`) and
//! takes the words of the choices, their sentences and the presets from it, so the dialog
//! offers what the CLI and the MCP server offer. A test of this crate fails when the copy and
//! the table differ, and names the command that writes the copy anew ([`WRITE_COPY`]).

use serde_json::{Value, json};

use crate::{Kind, OPTIONS, Preset};

/// Where the plugin keeps its copy, from the root of the workspace.
pub const PLUGIN_COPY: &str = "plugin/gb_plugin/route_options.json";

/// The command that writes the plugin's copy anew, from the root of the workspace.
pub const WRITE_COPY: &str = "cargo run -p gummiband-app --example plugin_table > plugin/gb_plugin/route_options.json";

/// The table and the presets as JSON text: `options` with name, kind, default in words, help
/// and the words of a choice, and `presets` with what each sets.
pub fn options_json() -> String {
    let kind = |k: Kind| match k {
        Kind::Switch => "switch",
        Kind::OnOff => "on_off",
        Kind::Number => "number",
        Kind::Choice => "choice",
        Kind::Text => "text",
        Kind::List => "list",
        Kind::Path => "path",
    };
    let options: Vec<Value> = OPTIONS
        .iter()
        .map(|o| {
            let choices: Vec<Value> = o
                .choices
                .iter()
                .map(|c| json!({"value": c.value, "help": c.help}))
                .collect();
            json!({
                "name": o.name,
                "kind": kind(o.kind),
                "default": o.default,
                "help": o.help,
                "choices": choices,
            })
        })
        .collect();
    let presets: Vec<Value> = Preset::ALL
        .iter()
        .map(|p| {
            let v = p.values();
            json!({
                "name": p.word(),
                "sets": {
                    "runs": v.runs,
                    "generations": v.generations,
                    "geometry_generations": v.geometry_generations,
                    "open_end_seconds": v.open_end_seconds,
                },
            })
        })
        .collect();
    let note = format!(
        "Written from gummiband_app::OPTIONS and gummiband_app::Preset. Do not edit, write it again: {WRITE_COPY}"
    );
    let mut text = serde_json::to_string_pretty(&json!({
        "note": note,
        "options": options,
        "presets": presets,
    }))
    .expect("JSON");
    text.push('\n');
    text
}
