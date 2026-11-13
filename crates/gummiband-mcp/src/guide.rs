//! The `guide` tool: the server's own documentation, built in, so that an agent can look up how
//! to use a feature or the whole workflow while it works. The topics are the sections of
//! docs/MCP.md and two chapters of the user guide.

const MCP_MD: &str = include_str!("../../../docs/MCP.md");
const CONCEPTS_MD: &str = include_str!("../../../docs/guide/concepts.md");
const OPTIONS_MD: &str = include_str!("../../../docs/guide/options.md");

/// Topic shown when none is asked for.
pub const DEFAULT_TOPIC: &str = "workflow";

/// Sections of MCP.md that are for the person setting up the server, not for the agent.
const SKIP: &[&str] = &[
    "build",
    "register_the_server",
    "claude_code",
    "claude_desktop",
    "try_it_without_a_client",
];

pub struct Topic {
    pub key: String,
    pub title: String,
    pub text: String,
}

/// Topic key of a heading: a single tool name as it is, otherwise the words before a colon in
/// snake case without a leading "the" (`Good practice: power_plan, ...` -> `good_practice`).
fn key_of(title: &str) -> String {
    let t = title.trim().trim_matches('`');
    if !t.is_empty() && t.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return t.to_ascii_lowercase();
    }
    let head = t.split(':').next().unwrap_or(t);
    let words: Vec<String> = head
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| !w.is_empty())
        .map(str::to_ascii_lowercase)
        .collect();
    let words = match words.first().map(String::as_str) {
        Some("the") if words.len() > 1 => &words[1..],
        _ => &words[..],
    };
    words.join("_")
}

/// Headings of a Markdown text outside code fences: (line index, level, title).
fn headings(lines: &[&str]) -> Vec<(usize, usize, String)> {
    let mut out = Vec::new();
    let mut fence = false;
    for (i, l) in lines.iter().enumerate() {
        if l.starts_with("```") {
            fence = !fence;
            continue;
        }
        if fence {
            continue;
        }
        let level = l.chars().take_while(|&c| c == '#').count();
        if (1..=4).contains(&level) && l[level..].starts_with(' ') {
            out.push((i, level, l[level..].trim().to_string()));
        }
    }
    out
}

/// All topics, in the order of the documents.
pub fn topics() -> Vec<Topic> {
    let lines: Vec<&str> = MCP_MD.lines().collect();
    let hs = headings(&lines);
    let mut out = Vec::new();
    for (k, (start, level, title)) in hs.iter().enumerate() {
        if *level < 2 {
            continue;
        }
        let key = key_of(title);
        if SKIP.contains(&key.as_str()) {
            continue;
        }
        // a section runs to the next heading of its level or above; "Tools" only to the next
        // heading, so that it is the overview table and not the whole reference
        let end = hs[k + 1..]
            .iter()
            .find(|(_, l, _)| key == "tools" || *l <= *level)
            .map_or(lines.len(), |h| h.0);
        let text = lines[*start..end].join("\n").trim_end().to_string();
        let key = if key == "working_with_the_tools" {
            DEFAULT_TOPIC.to_string()
        } else {
            key
        };
        out.push(Topic {
            key,
            title: title.clone(),
            text,
        });
    }
    for (key, md) in [("concepts", CONCEPTS_MD), ("options", OPTIONS_MD)] {
        let title = md
            .lines()
            .next()
            .unwrap_or(key)
            .trim_start_matches('#')
            .trim()
            .to_string();
        out.push(Topic {
            key: key.into(),
            title,
            text: md.trim_end().to_string(),
        });
    }
    out
}

/// The list of topics as Markdown.
pub fn index(topics: &[Topic]) -> String {
    let mut s =
        String::from("## Topics of the guide tool\n\nCall guide with one of these topics (default: workflow).\n\n");
    for t in topics {
        s.push_str(&format!("- `{}`: {}\n", t.key, t.title));
    }
    s
}

/// Text for `topic` (None: the workflow and the list of topics). Returns the topic key that was
/// answered (`index` for an unknown topic) and the Markdown.
pub fn lookup(topic: Option<&str>) -> (String, String, Vec<String>) {
    let all = topics();
    let keys: Vec<String> = all.iter().map(|t| t.key.clone()).collect();
    let want = topic.map(key_of).unwrap_or_else(|| DEFAULT_TOPIC.to_string());
    match all.iter().find(|t| t.key == want) {
        Some(t) if topic.is_none() => (t.key.clone(), format!("{}\n\n{}", t.text, index(&all)), keys),
        Some(t) => (t.key.clone(), t.text.clone(), keys),
        None if want == "index" || want == "topics" => ("index".into(), index(&all), keys),
        None => (
            "index".into(),
            format!("No topic {:?}.\n\n{}", topic.unwrap_or_default(), index(&all)),
            keys,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys() {
        assert_eq!(key_of("route"), "route");
        assert_eq!(
            key_of("Good practice: power_plan, power_copper, current_check, reference_check, ground_pour, area_report"),
            "good_practice"
        );
        assert_eq!(key_of("The prompt good_practice"), "prompt_good_practice");
        assert_eq!(
            key_of("Manufacturer limits: JLCPCB as an example"),
            "manufacturer_limits"
        );
        assert_eq!(
            key_of("Working with the tools: by hand, automatic, both"),
            "working_with_the_tools"
        );
    }

    #[test]
    fn every_tool_and_the_workflow_have_a_topic() {
        let all = topics();
        let keys: Vec<&str> = all.iter().map(|t| t.key.as_str()).collect();
        for k in [
            "workflow",
            "tools",
            "create_board",
            "load_board",
            "list_nets",
            "board_patterns",
            "part_info",
            "move_part",
            "place_parts",
            "track_corridors",
            "add_zone",
            "set_layer_type",
            "add_locked_copper",
            "lock",
            "set_net_options",
            "route",
            "angles_45",
            "power_plan",
            "power_copper",
            "current_check",
            "reference_check",
            "ground_pour",
            "area_report",
            "good_practice",
            "drc",
            "render",
            "guide",
            "open_points",
            "concepts",
            "options",
        ] {
            assert!(keys.contains(&k), "no topic {k}: {keys:?}");
        }
        for k in SKIP {
            assert!(!keys.contains(k), "{k} is for the person setting up the server");
        }
        let mut sorted = keys.clone();
        sorted.sort();
        sorted.dedup();
        assert_eq!(sorted.len(), keys.len(), "topic keys must be unique: {keys:?}");
    }

    #[test]
    fn sections_end_at_the_next_heading_of_their_level() {
        let (k, route, _) = lookup(Some("route"));
        assert_eq!(k, "route");
        assert!(route.starts_with("### route"));
        assert!(!route.contains("### Choosing opt-in options"));
        let (_, gp, _) = lookup(Some("good_practice"));
        assert!(gp.contains("#### power_plan") && gp.contains("#### ground_pour"));
        assert!(gp.contains("#### area_report"));
        assert!(!gp.contains("### drc"));
        let (_, tools, _) = lookup(Some("tools"));
        assert!(tools.contains("| `route` |") && !tools.contains("### create_board"));
        let (_, wf, _) = lookup(None);
        assert!(wf.contains("not a one-stop shop") && wf.contains("### Fine-tuning"));
        assert!(wf.contains("## Topics of the guide tool"));
        let (k, idx, _) = lookup(Some("no such thing"));
        assert_eq!(k, "index");
        assert!(idx.contains("`place_parts`"));
        // a heading inside a code block is no section
        assert!(
            !lookup(None)
                .2
                .iter()
                .any(|k| k == "binary_target_release_gummiband_mcp")
        );
    }
}
