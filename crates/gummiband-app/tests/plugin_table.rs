//! The KiCad plugin's copy of the option table is the table.

use std::path::Path;

use gummiband_app::export::{PLUGIN_COPY, WRITE_COPY, options_json};

/// The plugin reads the words of the choices, their sentences and the presets from a JSON file
/// next to its sources. It has to be what `gummiband_app::OPTIONS` says today.
#[test]
fn the_plugin_s_copy_of_the_table_is_the_table() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").join(PLUGIN_COPY);
    let copy = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    assert!(
        copy == options_json(),
        "{PLUGIN_COPY} is not what the table prints, write it again:\n{WRITE_COPY}"
    );
    // what the plugin relies on: the choices it shows and the four values of a preset
    let json: serde_json::Value = serde_json::from_str(&copy).unwrap();
    let option = |name: &str| {
        json["options"]
            .as_array()
            .unwrap()
            .iter()
            .find(|o| o["name"] == name)
            .unwrap_or_else(|| panic!("no {name}"))
            .clone()
    };
    for name in ["scope", "pairs", "preset"] {
        assert_eq!(option(name)["kind"], "choice");
        assert!(!option(name)["choices"].as_array().unwrap().is_empty());
    }
    let presets = json["presets"].as_array().unwrap();
    assert_eq!(presets.len(), option("preset")["choices"].as_array().unwrap().len());
    for p in presets {
        for key in ["runs", "generations", "geometry_generations", "open_end_seconds"] {
            assert!(p["sets"][key].is_number(), "{} {key}", p["name"]);
        }
    }
}
