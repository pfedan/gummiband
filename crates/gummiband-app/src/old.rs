//! Spellings of earlier versions that the CLI and the MCP server still take. Each is mapped onto
//! the option of today, and the front end says in one line what to write instead
//! ([`Respelled::cli`], [`Respelled::mcp`]). The old spellings are in no help text and in no
//! schema. The result of a route is the same in both spellings; the tests here pin every mapping.

use crate::Pairs;

/// What a caller gave for an option.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Given {
    /// A switch that is on (CLI: the flag, MCP: true).
    On,
    /// A switch that was switched off (MCP: false).
    Off,
    /// A word of a choice.
    Word(&'static str),
    Number(f64),
}

/// An old spelling a caller used, and what to write now.
#[derive(Debug, Clone, PartialEq)]
pub struct Respelled {
    /// The options as they were given.
    pub old: Vec<(&'static str, Given)>,
    /// The same in the spelling of today.
    pub now: Vec<(&'static str, Given)>,
}

impl Respelled {
    /// The line for a user of the CLI: `--pairs is now --pairs band`.
    pub fn cli(&self) -> String {
        let side = |list: &[(&str, Given)]| {
            list.iter()
                .map(|(name, given)| {
                    let flag = name.replace('_', "-");
                    match given {
                        Given::On => format!("--{flag}"),
                        Given::Off => format!("--{flag} false"),
                        Given::Word(w) => format!("--{flag} {w}"),
                        Given::Number(n) => format!("--{flag} {n}"),
                    }
                })
                .collect::<Vec<_>>()
                .join(" ")
        };
        format!("{} is now {}", side(&self.old), side(&self.now))
    }

    /// The line for a caller of the MCP tool: `pairs: true is now pairs: "band"`.
    pub fn mcp(&self) -> String {
        let side = |list: &[(&str, Given)]| {
            list.iter()
                .map(|(name, given)| match given {
                    Given::On => format!("{name}: true"),
                    Given::Off => format!("{name}: false"),
                    Given::Word(w) => format!("{name}: \"{w}\""),
                    Given::Number(n) => format!("{name}: {n}"),
                })
                .collect::<Vec<_>>()
                .join(", ")
        };
        format!("{} is now {}", side(&self.old), side(&self.now))
    }
}

/// `pairs` when it was not given as a word. It was a switch for the wide track (`switch`: the
/// CLI's `--pairs` without a word, the MCP's true or false), and the pull was on whenever
/// `pair_pull` was above 0. Returns the mode of today, and what to write unless nothing old was
/// given. A weight next to the wide track stays as it is: it pulls the pairs that get none.
pub fn pairs(switch: Option<bool>, pair_pull: Option<f64>) -> (Pairs, Option<Respelled>) {
    let mode = match (switch, pair_pull) {
        (Some(true), _) => Pairs::Band,
        (_, Some(w)) if w > 0.0 => Pairs::Pull,
        _ => Pairs::Off,
    };
    let weight = pair_pull.map(|w| ("pair_pull", Given::Number(w)));
    let mut old: Vec<(&str, Given)> = Vec::new();
    let mut now = vec![("pairs", Given::Word(mode.word()))];
    match switch {
        Some(on) => old.push(("pairs", if on { Given::On } else { Given::Off })),
        None if pair_pull.is_none() => return (mode, None),
        None => {}
    }
    // the weight is said again where it still counts: with the pull, and next to the wide track
    if mode != Pairs::Band || switch.is_none() {
        old.extend(weight);
    }
    if mode == Pairs::Pull {
        now.extend(weight);
    }
    (mode, Some(Respelled { old, now }))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every old spelling of the pair options, the mode it means and the line a user reads.
    #[test]
    fn old_pair_spellings() {
        let cases: [(Option<bool>, Option<f64>, Pairs, &str, &str); 7] = [
            (None, None, Pairs::Off, "", ""),
            (
                Some(true),
                None,
                Pairs::Band,
                "--pairs is now --pairs band",
                "pairs: true is now pairs: \"band\"",
            ),
            (
                Some(true),
                Some(1.5),
                Pairs::Band,
                "--pairs is now --pairs band",
                "pairs: true is now pairs: \"band\"",
            ),
            (
                None,
                Some(2.0),
                Pairs::Pull,
                "--pair-pull 2 is now --pairs pull --pair-pull 2",
                "pair_pull: 2 is now pairs: \"pull\", pair_pull: 2",
            ),
            (
                None,
                Some(0.0),
                Pairs::Off,
                "--pair-pull 0 is now --pairs off",
                "pair_pull: 0 is now pairs: \"off\"",
            ),
            (
                Some(false),
                None,
                Pairs::Off,
                "--pairs false is now --pairs off",
                "pairs: false is now pairs: \"off\"",
            ),
            (
                Some(false),
                Some(1.0),
                Pairs::Pull,
                "--pairs false --pair-pull 1 is now --pairs pull --pair-pull 1",
                "pairs: false, pair_pull: 1 is now pairs: \"pull\", pair_pull: 1",
            ),
        ];
        for (switch, weight, mode, cli, mcp) in cases {
            let (got, note) = pairs(switch, weight);
            assert_eq!(got, mode, "{switch:?} {weight:?}");
            assert_eq!(note.as_ref().map(Respelled::cli).unwrap_or_default(), cli);
            assert_eq!(note.as_ref().map(Respelled::mcp).unwrap_or_default(), mcp);
        }
    }
}
