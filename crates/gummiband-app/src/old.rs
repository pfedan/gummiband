//! Spellings of earlier versions that the CLI and the MCP server still take. Each is mapped onto
//! the option of today, and the front end says in one line what to write instead
//! ([`Respelled::cli`], [`Respelled::mcp`]). The old spellings are in no help text and in no
//! schema. The result of a route is the same in both spellings; the tests here pin every mapping.

use crate::{Pairs, Scope};

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

/// The switches that `scope` replaces: `keep_locked`, `keep_all` and `reroute`, each as it was
/// given (None: left out). `default` is the scope of the front end when nothing says otherwise:
/// fresh, or what a session loaded the board with.
///
/// What they did, and so what they are now: `keep_all` kept all copper, whatever `keep_locked`
/// said, and laid what was open (finish), with `reroute` it took the unlocked copper of the nets
/// out first (reroute); `keep_locked` alone kept the locked copper. `reroute` without `keep_all`
/// was refused and still is.
pub fn scope(
    keep_locked: Option<bool>,
    keep_all: Option<bool>,
    reroute: Option<bool>,
    default: Scope,
) -> Result<(Scope, Option<Respelled>), String> {
    let given = [
        ("keep_locked", keep_locked),
        ("keep_all", keep_all),
        ("reroute", reroute),
    ];
    if given.iter().all(|(_, g)| g.is_none()) {
        return Ok((default, None));
    }
    let scope = match (keep_all == Some(true), reroute == Some(true)) {
        (true, true) => Scope::Reroute,
        (true, false) => Scope::Finish,
        (false, true) => {
            return Err(
                "reroute without keep_all has no meaning: write scope reroute with nets, which keeps all other copper"
                    .to_string(),
            );
        }
        (false, false) => match keep_locked {
            Some(true) => Scope::KeepLocked,
            Some(false) => Scope::Fresh,
            None => default,
        },
    };
    let old = given
        .iter()
        .filter_map(|(name, g)| g.map(|on| (*name, if on { Given::On } else { Given::Off })))
        .collect();
    let now = vec![("scope", Given::Word(scope.word()))];
    Ok((scope, Some(Respelled { old, now })))
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

    /// Every combination of the three switches that `scope` replaces, as the CLI could give
    /// them (a flag is there or not): the scope it means, or that it had no meaning.
    #[test]
    fn old_scope_switches() {
        use Scope::*;
        // keep_locked, keep_all, reroute
        let cases: [(bool, bool, bool, Option<Scope>, &str); 8] = [
            (false, false, false, Some(Fresh), ""),
            (
                true,
                false,
                false,
                Some(KeepLocked),
                "--keep-locked is now --scope keep_locked",
            ),
            (false, true, false, Some(Finish), "--keep-all is now --scope finish"),
            (
                true,
                true,
                false,
                Some(Finish),
                "--keep-locked --keep-all is now --scope finish",
            ),
            (
                false,
                true,
                true,
                Some(Reroute),
                "--keep-all --reroute is now --scope reroute",
            ),
            (
                true,
                true,
                true,
                Some(Reroute),
                "--keep-locked --keep-all --reroute is now --scope reroute",
            ),
            (false, false, true, None, ""),
            (true, false, true, None, ""),
        ];
        for (keep_locked, keep_all, reroute, want, cli) in cases {
            let got = scope(
                keep_locked.then_some(true),
                keep_all.then_some(true),
                reroute.then_some(true),
                Fresh,
            );
            match want {
                Some(want) => {
                    let (got, note) = got.unwrap();
                    assert_eq!(got, want, "{keep_locked} {keep_all} {reroute}");
                    assert_eq!(note.map(|n| n.cli()).unwrap_or_default(), cli);
                }
                None => assert!(got.unwrap_err().starts_with("reroute without keep_all has no meaning")),
            }
        }
    }

    /// The MCP tool could say false too, and its session has a scope of its own for a board
    /// that was loaded with locked copper.
    #[test]
    fn old_scope_switches_of_a_session() {
        use Scope::*;
        // nothing given: the scope of the session
        assert_eq!(scope(None, None, None, KeepLocked), Ok((KeepLocked, None)));
        let said = |keep_locked, keep_all, reroute, default| {
            let (scope, note) = scope(keep_locked, keep_all, reroute, default).unwrap();
            (scope, note.unwrap().mcp())
        };
        // false takes the locked copper of the session out again
        assert_eq!(
            said(Some(false), None, None, KeepLocked),
            (Fresh, "keep_locked: false is now scope: \"fresh\"".to_string())
        );
        assert_eq!(
            said(None, Some(true), Some(false), KeepLocked),
            (
                Finish,
                "keep_all: true, reroute: false is now scope: \"finish\"".to_string()
            )
        );
        // a switch that was switched off decides nothing: the session's scope stays
        assert_eq!(
            said(None, Some(false), None, KeepLocked),
            (KeepLocked, "keep_all: false is now scope: \"keep_locked\"".to_string())
        );
    }
}
