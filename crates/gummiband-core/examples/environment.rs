//! Prints docs/ENVIRONMENT.md from the table of `gummiband_core::knobs`:
//! `cargo run -p gummiband-core --example environment > docs/ENVIRONMENT.md`

fn main() {
    print!("{}", gummiband_core::knobs::markdown());
}
