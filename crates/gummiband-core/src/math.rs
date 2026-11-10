//! The transcendental functions of the workspace, in pure Rust.
//!
//! `f64::sin`, `cos`, `atan2`, `exp`, `ln`, `powf`, `hypot` and their kin call the maths library
//! of the system, and their last bits differ between glibc versions, musl and macOS. A routed
//! board is the end of a long chain of decisions, so one different last bit gives another file.
//! The functions here come from the `libm` crate, a port of musl's maths library to Rust that is
//! compiled into the program: the same input gives the same bits on every machine and with every
//! C library. They use only the basic IEEE 754 operations (`+ - * /`, `sqrt`, `fma`, rounding to
//! an integer), which every platform must round correctly, and Rust never contracts `a * b + c`
//! into an `fma` on its own.
//!
//! All code of the workspace calls these functions, and a test below fails when a source file
//! calls a method of `f64` or `f32` that goes to the system library. What stays with the standard
//! library is exact by definition: `sqrt`, `abs`, `floor`, `ceil`, `round`, `trunc`,
//! `rem_euclid`, `mul_add`, `min`, `max`, `to_degrees` and `to_radians` (one multiplication).
//! `powi` is not on that list: its documentation allows a different precision per platform, so
//! write the product out (`x * x`).
//!
//! The shader of the experimental GPU relaxation (`relax_gpu.rs`) has its own maths on the
//! graphics card and is outside this rule.

#[inline]
pub fn sin(x: f64) -> f64 {
    libm::sin(x)
}

#[inline]
pub fn cos(x: f64) -> f64 {
    libm::cos(x)
}

#[inline]
pub fn tan(x: f64) -> f64 {
    libm::tan(x)
}

/// (sin x, cos x), the same bits as [`sin`] and [`cos`].
#[inline]
pub fn sin_cos(x: f64) -> (f64, f64) {
    (libm::sin(x), libm::cos(x))
}

#[inline]
pub fn asin(x: f64) -> f64 {
    libm::asin(x)
}

#[inline]
pub fn acos(x: f64) -> f64 {
    libm::acos(x)
}

#[inline]
pub fn atan(x: f64) -> f64 {
    libm::atan(x)
}

/// The angle of the vector (x, y), as `y.atan2(x)`.
#[inline]
pub fn atan2(y: f64, x: f64) -> f64 {
    libm::atan2(y, x)
}

#[inline]
pub fn sinh(x: f64) -> f64 {
    libm::sinh(x)
}

#[inline]
pub fn cosh(x: f64) -> f64 {
    libm::cosh(x)
}

#[inline]
pub fn tanh(x: f64) -> f64 {
    libm::tanh(x)
}

#[inline]
pub fn exp(x: f64) -> f64 {
    libm::exp(x)
}

#[inline]
pub fn exp2(x: f64) -> f64 {
    libm::exp2(x)
}

/// e^x - 1, exact near 0.
#[inline]
pub fn exp_m1(x: f64) -> f64 {
    libm::expm1(x)
}

/// The natural logarithm.
#[inline]
pub fn ln(x: f64) -> f64 {
    libm::log(x)
}

/// ln(1 + x), exact near 0.
#[inline]
pub fn ln_1p(x: f64) -> f64 {
    libm::log1p(x)
}

#[inline]
pub fn log2(x: f64) -> f64 {
    libm::log2(x)
}

#[inline]
pub fn log10(x: f64) -> f64 {
    libm::log10(x)
}

/// x to the power y, as `x.powf(y)`.
#[inline]
pub fn powf(x: f64, y: f64) -> f64 {
    libm::pow(x, y)
}

/// sqrt(x^2 + y^2) without overflow in between.
#[inline]
pub fn hypot(x: f64, y: f64) -> f64 {
    libm::hypot(x, y)
}

#[inline]
pub fn cbrt(x: f64) -> f64 {
    libm::cbrt(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The methods of `f64` and `f32` that end in the maths library of the system (or, for
    /// `powi`, in a product whose precision the standard library leaves open).
    const SYSTEM: &[&str] = &[
        "sin", "cos", "tan", "sin_cos", "asin", "acos", "atan", "atan2", "sinh", "cosh", "tanh", "asinh", "acosh",
        "atanh", "exp", "exp2", "exp_m1", "ln", "ln_1p", "log", "log2", "log10", "powf", "powi", "hypot", "cbrt",
        "gamma", "ln_gamma",
    ];

    /// The Rust sources of the workspace: (path, text).
    fn sources() -> Vec<(std::path::PathBuf, String)> {
        fn walk(dir: &std::path::Path, out: &mut Vec<(std::path::PathBuf, String)>) {
            for e in std::fs::read_dir(dir).unwrap().flatten() {
                let p = e.path();
                if p.is_dir() {
                    if p.file_name().is_none_or(|n| n != "target") {
                        walk(&p, out);
                    }
                } else if p.extension().is_some_and(|x| x == "rs") {
                    let text = std::fs::read_to_string(&p).unwrap();
                    out.push((p, text));
                }
            }
        }
        let crates = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap();
        let mut out = Vec::new();
        walk(crates, &mut out);
        assert!(out.len() > 20);
        out
    }

    /// The text with comments, string literals and character literals blanked out; the line
    /// breaks stay, so a position still gives its line.
    fn code_only(text: &str) -> String {
        let s: Vec<char> = text.chars().collect();
        let mut out = String::with_capacity(text.len());
        let blank = |out: &mut String, from: usize, to: usize| {
            out.extend(s[from..to].iter().map(|&c| if c == '\n' { '\n' } else { ' ' }));
        };
        let mut i = 0;
        while i < s.len() {
            let rest = &s[i..];
            // `r` opens a raw string unless it ends a name; `br"..."` is a raw string too
            let ident_before = i > 0
                && (s[i - 1].is_alphanumeric() || s[i - 1] == '_')
                && !(s[i - 1] == 'b' && (i < 2 || !(s[i - 2].is_alphanumeric() || s[i - 2] == '_')));
            let end = if rest.starts_with(&['/', '/']) {
                i + rest.iter().position(|&c| c == '\n').unwrap_or(rest.len())
            } else if rest.starts_with(&['/', '*']) {
                // block comments nest
                let (mut depth, mut j) = (1, i + 2);
                while j < s.len() && depth > 0 {
                    if s[j..].starts_with(&['/', '*']) {
                        depth += 1;
                        j += 2;
                    } else if s[j..].starts_with(&['*', '/']) {
                        depth -= 1;
                        j += 2;
                    } else {
                        j += 1;
                    }
                }
                j
            } else if s[i] == '"' {
                let mut j = i + 1;
                while j < s.len() && s[j] != '"' {
                    j += if s[j] == '\\' { 2 } else { 1 };
                }
                (j + 1).min(s.len())
            } else if s[i] == 'r' && !ident_before && {
                let hashes = rest[1..].iter().take_while(|&&c| c == '#').count();
                rest.get(1 + hashes) == Some(&'"')
            } {
                // a raw string: r"...", r#"..."#, ...
                let hashes = rest[1..].iter().take_while(|&&c| c == '#').count();
                let mut j = i + 2 + hashes;
                while j < s.len()
                    && !(s[j] == '"' && s[j + 1..].iter().take(hashes).filter(|&&c| c == '#').count() == hashes)
                {
                    j += 1;
                }
                (j + 1 + hashes).min(s.len())
            } else if s[i] == '\'' && rest.len() > 2 && rest[1] == '\\' {
                // an escaped character literal: '\n', '\'', '\u{1F600}'
                i + 3 + rest[3..].iter().position(|&c| c == '\'').map_or(0, |k| k + 1)
            } else if s[i] == '\'' && rest.len() > 2 && rest[2] == '\'' {
                i + 3
            } else {
                // code; a lone ' is a lifetime
                out.push(s[i]);
                i += 1;
                continue;
            };
            blank(&mut out, i, end);
            i = end;
        }
        out
    }

    /// The calls of system maths in a source text: (line, what was found).
    fn system_calls(text: &str) -> Vec<(usize, String)> {
        let code = code_only(text);
        let mut found = Vec::new();
        for name in SYSTEM {
            for (needle, method) in [
                (format!(".{name}"), true),
                (format!("f64::{name}"), false),
                (format!("f32::{name}"), false),
            ] {
                for (i, _) in code.match_indices(&needle) {
                    let after = code[i + needle.len()..].chars().next();
                    // `.sin(`, also over a line break before the dot; `f64::sin` with or without
                    // a call, as in `.map(f64::sin)`; not `.sin_cos` for `sin` or `.cost`
                    let hit = if method {
                        code[i + needle.len()..].trim_start().starts_with('(')
                    } else {
                        let before = code[..i].chars().next_back();
                        after.is_none_or(|c| !c.is_alphanumeric() && c != '_')
                            && before.is_none_or(|c| !c.is_alphanumeric() && c != '_')
                    };
                    if hit {
                        found.push((code[..i].matches('\n').count() + 1, needle.clone()));
                    }
                }
            }
        }
        found.sort();
        found
    }

    #[test]
    fn only_this_module_calls_transcendental_functions() {
        let mut bad = Vec::new();
        for (path, text) in sources() {
            let p = path.to_string_lossy().replace('\\', "/");
            if p.ends_with("gummiband-core/src/math.rs") {
                continue;
            }
            for (line, what) in system_calls(&text) {
                bad.push(format!("{p}:{line}: {what}"));
            }
        }
        assert!(
            bad.is_empty(),
            "these calls go to the maths library of the system, whose results differ between machines; \
             use gummiband_core::math (and x * x for powi):\n{}",
            bad.join("\n")
        );
    }

    #[test]
    fn the_scan_finds_calls_and_skips_comments_and_strings() {
        let lines = |t: &str| system_calls(t);
        let hit = |l: usize, w: &str| (l, w.to_string());
        assert_eq!(lines("let a = x.sin();"), vec![hit(1, ".sin")]);
        assert_eq!(lines("let a = (b - c)\n    .atan2(d);"), vec![hit(2, ".atan2")]);
        assert_eq!(lines("let (s, c) = a.sin_cos();"), vec![hit(1, ".sin_cos")]);
        assert_eq!(lines("v.iter().copied().map(f64::exp).sum()"), vec![hit(1, "f64::exp")]);
        assert_eq!(
            lines("f32::hypot(a, b) + x.powi(2)"),
            vec![hit(1, ".powi"), hit(1, "f32::hypot")]
        );
        assert_eq!(
            lines("x.ln() /* y.ln() */ + z.ln_1p()"),
            vec![hit(1, ".ln"), hit(1, ".ln_1p")]
        );
        // comments, texts, the functions of this module, other names and exact operations
        for clean in [
            "// the old code called x.sin() here",
            "/// Returns `x.cos()`.\nfn f() {}",
            "/* a.tan() /* nested b.exp() */ c.ln() */",
            "let s = \"x.sin() \\\" y.cos()\";",
            "let s = r#\"x.sin() \" y.cos()\"#; let c = '\"'; let d = '\\''; // z.exp()",
            "fn f<'a>(x: &'a f64) -> f64 { math::sin(*x) + math::atan2(1.0, 2.0) }",
            "let c = self.cost(a) + plan.expand(b) + t.logs(c) + x.sqrt().abs().mul_add(a, b);",
            "let a = x.to_radians().rem_euclid(TAU).floor(); let sin = 1.0; let b = cfg.sin;",
            "let f = my_f64::sin; let g = math::cos;",
        ] {
            assert_eq!(lines(clean), vec![], "{clean}");
        }
    }

    /// A stream of test arguments in [lo, hi), the same on every machine.
    fn args(lo: f64, hi: f64) -> impl Iterator<Item = f64> {
        let mut state = 0x9E37_79B9_7F4A_7C15u64;
        (0..20_000).map(move |_| {
            // splitmix64
            state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = state;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^= z >> 31;
            lo + (hi - lo) * ((z >> 11) as f64 / (1u64 << 53) as f64)
        })
    }

    /// FNV-1a over the bits of the values.
    fn digest(values: impl Iterator<Item = f64>) -> u64 {
        values.fold(0xCBF2_9CE4_8422_2325u64, |h, v| {
            v.to_bits()
                .to_le_bytes()
                .iter()
                .fold(h, |h, &b| (h ^ b as u64).wrapping_mul(0x0000_0100_0000_01B3))
        })
    }

    /// The bits of every function over 20 000 arguments, as measured on x86_64 Linux. The same
    /// numbers on another processor or operating system are the point of this module; a new
    /// version of the `libm` crate that changes one changes the routed files too (then the
    /// reference hashes of `bench/hashes.sh` need an update, see `bench/README.md`).
    #[test]
    fn the_bits_are_the_same_everywhere() {
        let two = |f: fn(f64, f64) -> f64, lo: f64, hi: f64| {
            digest(args(lo, hi).zip(args(lo, hi).skip(7)).map(|(a, b)| f(a, b)))
        };
        let got = [
            ("sin", digest(args(-10.0, 10.0).map(sin))),
            ("sin far", digest(args(-1e6, 1e6).map(sin))),
            ("cos", digest(args(-10.0, 10.0).map(cos))),
            ("cos far", digest(args(-1e6, 1e6).map(cos))),
            ("tan", digest(args(-10.0, 10.0).map(tan))),
            (
                "sin_cos",
                digest(args(-10.0, 10.0).map(|x| sin_cos(x).0 + 2.0 * sin_cos(x).1)),
            ),
            ("asin", digest(args(-1.0, 1.0).map(asin))),
            ("acos", digest(args(-1.0, 1.0).map(acos))),
            ("atan", digest(args(-50.0, 50.0).map(atan))),
            ("atan2", two(atan2, -100.0, 100.0)),
            ("sinh", digest(args(-20.0, 20.0).map(sinh))),
            ("cosh", digest(args(-20.0, 20.0).map(cosh))),
            ("tanh", digest(args(-20.0, 20.0).map(tanh))),
            ("exp", digest(args(-50.0, 50.0).map(exp))),
            ("exp2", digest(args(-50.0, 50.0).map(exp2))),
            ("exp_m1", digest(args(-2.0, 2.0).map(exp_m1))),
            ("ln", digest(args(1e-9, 1e6).map(ln))),
            ("ln small", digest(args(0.0, 2.0).map(ln))),
            ("ln_1p", digest(args(-0.9, 2.0).map(ln_1p))),
            ("log2", digest(args(1e-9, 1e6).map(log2))),
            ("log10", digest(args(1e-9, 1e6).map(log10))),
            ("powf", two(powf, 0.0, 30.0)),
            ("hypot", two(hypot, -300.0, 300.0)),
            ("cbrt", digest(args(-1e6, 1e6).map(cbrt))),
        ];
        let want: [(&str, u64); 24] = WANT;
        for ((name, g), (wname, w)) in got.iter().zip(want.iter()) {
            assert_eq!(name, wname);
            assert_eq!(g, w, "{name}: 0x{g:016X}");
        }
    }

    const WANT: [(&str, u64); 24] = [
        ("sin", 0x76AE3C5BED9BBC95),
        ("sin far", 0xF68D959F6919E319),
        ("cos", 0x2C47521C0927FFF4),
        ("cos far", 0x90F9F77E9FBF9B49),
        ("tan", 0x616F5AFD9C6AC3A1),
        ("sin_cos", 0x20AC0AE36D026331),
        ("asin", 0xDA7B286DC7C886CF),
        ("acos", 0xE9B53217A9AC108D),
        ("atan", 0xD682094A8EA7F2BD),
        ("atan2", 0x309540478EB546E4),
        ("sinh", 0xE1C083C5E1C554BD),
        ("cosh", 0x1A0E71867F48F5D4),
        ("tanh", 0x2D7630736D292DF5),
        ("exp", 0xDD673CD22E90A352),
        ("exp2", 0xB1CB4A999C86B61D),
        ("exp_m1", 0x498F2B1BFC02A537),
        ("ln", 0x2B7517F69CB300AE),
        ("ln small", 0x5E04854891608D64),
        ("ln_1p", 0xD4D508A8A88432CD),
        ("log2", 0x71D39B4374374F06),
        ("log10", 0x6D0D313AE8E00CD2),
        ("powf", 0x599FAECCDC38E926),
        ("hypot", 0xF021D54595967E1B),
        ("cbrt", 0xE1BBBD7D30A799A2),
    ];
}
