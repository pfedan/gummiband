//! Prints the table of the route options as JSON, the copy the KiCad plugin keeps:
//! `cargo run -p gummiband-app --example plugin_table > plugin/gb_plugin/route_options.json`

fn main() {
    print!("{}", gummiband_app::export::options_json());
}
