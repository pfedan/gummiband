//! End to end over stdio: the binary as an MCP client would start it, one JSON-RPC message
//! per line, each answer awaited before the next request.

use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};

use serde_json::{Value, json};

struct Server {
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
    next: u64,
}

impl Server {
    fn start() -> Server {
        let mut child = Command::new(env!("CARGO_BIN_EXE_gummiband-mcp"))
            .env("RAYON_NUM_THREADS", "1")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .expect("start gummiband-mcp");
        let stdin = child.stdin.take().unwrap();
        let stdout = BufReader::new(child.stdout.take().unwrap());
        Server {
            child,
            stdin,
            stdout,
            next: 0,
        }
    }

    fn send(&mut self, msg: &Value) {
        let line = serde_json::to_string(msg).unwrap();
        assert!(!line.contains('\n'));
        writeln!(self.stdin, "{line}").unwrap();
        self.stdin.flush().unwrap();
    }

    fn request(&mut self, method: &str, params: Value) -> Value {
        self.next += 1;
        let id = self.next;
        self.send(&json!({"jsonrpc": "2.0", "id": id, "method": method, "params": params}));
        loop {
            let mut line = String::new();
            assert!(self.stdout.read_line(&mut line).unwrap() > 0, "server closed stdout");
            let msg: Value = serde_json::from_str(&line).expect("one JSON-RPC message per line");
            assert_eq!(msg["jsonrpc"], "2.0");
            if msg["id"] == json!(id) {
                assert!(msg.get("error").is_none(), "{method}: {msg}");
                return msg["result"].clone();
            }
        }
    }

    /// Call a tool; returns the structured content, checks the text content mirrors it.
    fn call(&mut self, name: &str, args: Value) -> Value {
        let r = self.request("tools/call", json!({"name": name, "arguments": args}));
        assert_eq!(r["isError"], json!(false), "{name}: {r}");
        let content = r["content"].as_array().unwrap();
        assert!(
            content[0]["text"].as_str().is_some_and(|t| !t.is_empty()),
            "summary text"
        );
        let mirrored: Value = serde_json::from_str(content[1]["text"].as_str().unwrap()).unwrap();
        assert_eq!(mirrored, r["structuredContent"]);
        r["structuredContent"].clone()
    }

    fn call_err(&mut self, name: &str, args: Value) -> String {
        let r = self.request("tools/call", json!({"name": name, "arguments": args}));
        assert_eq!(r["isError"], json!(true), "{name}: {r}");
        r["content"][0]["text"].as_str().unwrap().to_string()
    }
}

impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

fn pad(num: u32, x: f64, net: &str) -> String {
    format!(
        "(pad \"{num}\" smd roundrect (at {x} 0) (size 0.8 0.95) (layers \"F.Cu\" \"F.Mask\" \"F.Paste\") \
         (roundrect_rratio 0.25) (net \"{net}\"))"
    )
}

fn resistor(refdes: &str, x: f64, y: f64, n1: &str, n2: &str) -> String {
    format!(
        "(footprint \"Test:R_0603\" (layer \"F.Cu\") (at {x} {y}) \
         (property \"Reference\" \"{refdes}\" (at 0 -1.13) (layer \"F.Fab\")) (attr smd) {} {})",
        pad(1, -0.825, n1),
        pad(2, 0.825, n2)
    )
}

/// Two resistors, nets A and B between them, 20 x 15 mm outline.
fn tiny_board() -> String {
    format!(
        "(kicad_pcb (version 20260206) (generator \"gummiband_test\") (generator_version \"10.0\")\n\
         (general (thickness 1.6)) (paper \"A4\")\n\
         (layers (0 \"F.Cu\" signal) (2 \"B.Cu\" signal) (1 \"F.Mask\" user) (3 \"B.Mask\" user) \
         (13 \"F.Paste\" user) (15 \"B.Paste\" user) (25 \"Edge.Cuts\" user) (35 \"F.Fab\" user))\n\
         (setup (pad_to_mask_clearance 0))\n{}\n{}\n\
         (gr_rect (start 100 100) (end 120 115) (stroke (width 0.1) (type default)) (fill no) (layer \"Edge.Cuts\"))\n)\n",
        resistor("R1", 105.0, 105.0, "A", "B"),
        resistor("R2", 113.0, 110.0, "A", "B"),
    )
}

#[test]
fn initialize_list_load_route() {
    let dir = std::env::temp_dir().join(format!("gummiband-mcp-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let dir = dir.canonicalize().unwrap();
    let pcb = dir.join("tiny.kicad_pcb");
    std::fs::write(&pcb, tiny_board()).unwrap();

    let mut s = Server::start();
    let init = s.request(
        "initialize",
        json!({"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "test", "version": "0"}}),
    );
    assert_eq!(init["protocolVersion"], "2025-06-18");
    assert_eq!(init["serverInfo"]["name"], "gummiband-mcp");
    assert!(init["capabilities"]["tools"].is_object());
    s.send(&json!({"jsonrpc": "2.0", "method": "notifications/initialized"}));

    let tools = s.request("tools/list", json!({}));
    let mut names: Vec<&str> = tools["tools"]
        .as_array()
        .unwrap()
        .iter()
        .map(|t| t["name"].as_str().unwrap())
        .collect();
    names.sort();
    assert_eq!(
        names,
        [
            "add_locked_copper",
            "add_zone",
            "angles_45",
            "board_patterns",
            "create_board",
            "current_check",
            "drc",
            "ground_pour",
            "guide",
            "list_nets",
            "load_board",
            "lock",
            "move_part",
            "part_info",
            "place_parts",
            "power_copper",
            "power_plan",
            "reference_check",
            "render",
            "route",
            "set_layer_type",
            "set_net_options",
            "track_corridors"
        ]
    );
    for t in tools["tools"].as_array().unwrap() {
        assert_eq!(t["inputSchema"]["type"], "object", "{t}");
        assert_eq!(t["outputSchema"]["type"], "object", "{t}");
    }

    let b = s.call("load_board", json!({"path": pcb.display().to_string()}));
    assert_eq!(b["board"], "b1");
    assert_eq!(b["pins"], 4);
    assert_eq!(b["routable_nets"], 2);
    assert_eq!(b["connections"], 2);
    assert_eq!(b["layers"].as_array().unwrap().len(), 2);
    assert!((b["outline"]["width_mm"].as_f64().unwrap() - 20.0).abs() < 0.2, "{b}");

    let nets = s.call("list_nets", json!({"board": "b1"}));
    assert_eq!(nets["count"], 2);

    let o = s.call("set_net_options", json!({"board": "b1", "net": "B", "skip": true}));
    assert_eq!(o["options"]["skip"], true);
    // an unknown net is a tool error the model can recover from, not a protocol error
    let bad = s.request(
        "tools/call",
        json!({"name": "set_net_options", "arguments": {"board": "b1", "net": "nope"}}),
    );
    assert_eq!(bad["isError"], true);

    let r = s.call("route", json!({"board": "b1", "runs": 1, "generations": 0}));
    assert_eq!(r["skipped_nets"], json!(["B"]));
    assert_eq!(r["connections"], 1);
    assert_eq!(r["routed"], 1, "{r}");
    let out = dir.join("tiny.routed.kicad_pcb");
    assert_eq!(r["output"], out.display().to_string());
    assert!(std::fs::read_to_string(&out).unwrap().contains("(segment"));
    // the knobs of the CLI the tool took in later: the open end with a time budget stops by
    // itself, the via pass can be switched off, the meander spacing is taken
    let t0 = std::time::Instant::now();
    let r = s.call(
        "route",
        json!({"board": "b1", "runs": 1, "generations": 0, "open_end_seconds": 1.0, "open_via_cost": 10.0,
            "via_pass": 0.0, "tune_spacing": 1.5}),
    );
    assert_eq!(r["routed"], 1, "{r}");
    assert!(
        t0.elapsed().as_secs() < 60,
        "the open end did not stop: {:?}",
        t0.elapsed()
    );

    // pre-routing: look at a part, lock copper in a working copy, route around it
    let part = s.call("part_info", json!({"board": "b1", "reference": "R1"}));
    assert_eq!(part["pads"].as_array().unwrap().len(), 2);
    let (x, y) = (
        part["pads"][0]["x"].as_f64().unwrap(),
        part["pads"][0]["y"].as_f64().unwrap(),
    );
    let net = part["pads"][0]["net"].as_str().unwrap().to_string();
    let locked = s.call(
        "add_locked_copper",
        json!({"board": "b1", "tracks": [{"net": net, "layer": "F.Cu", "points": [[x, y], [x, y + 2.0]]}]}),
    );
    assert_eq!(locked["items_added"], 1);
    let work = dir.join("tiny.prerouted.kicad_pcb");
    assert_eq!(locked["path"], work.display().to_string());
    let text = std::fs::read_to_string(&work).unwrap();
    assert!(text.contains("(locked yes)"), "{text}");
    let r = s.call(
        "route",
        json!({"board": locked["board"], "runs": 1, "generations": 0, "seed": 7}),
    );
    assert!(r["routed"].as_u64().unwrap() >= 1, "{r}");
    assert!(!r["nets"].as_array().unwrap().is_empty(), "{r}");

    // locked copper on a routed track of another net is a short: refused, nothing written
    let text = std::fs::read_to_string(dir.join("tiny.routed.kicad_pcb")).unwrap();
    let seg = &text[text.find("(segment").unwrap()..];
    let num = |key: &str| -> (f64, f64) {
        let at = &seg[seg.find(key).unwrap() + key.len()..];
        let v: Vec<f64> = at.split([' ', ')']).filter_map(|t| t.parse().ok()).take(2).collect();
        (v[0], v[1])
    };
    let ((ax, ay), (bx, by)) = (num("(start "), num("(end "));
    let routed = s.call("load_board", json!({"path": dir.join("tiny.routed.kicad_pcb")}));
    let short = s.request(
        "tools/call",
        json!({"name": "add_locked_copper", "arguments": {"board": routed["board"],
            "vias": [{"net": "B", "x": (ax + bx) / 2.0, "y": (ay + by) / 2.0}]}}),
    );
    assert_eq!(short["isError"], true, "{short}");
    assert!(short.to_string().contains("touches A"), "{short}");
    assert!(!dir.join("tiny.routed.prerouted.kicad_pcb").exists());

    // keep_all adds to a routed board: net B joins, the tracks of A stay
    let keep_out = dir.join("tiny.kept.kicad_pcb");
    let r = s.call(
        "route",
        json!({"board": routed["board"], "keep_all": true, "nets": ["B"], "runs": 1, "generations": 0,
               "output": keep_out}),
    );
    assert_eq!(r["routed"], 1, "{r}");
    let before = std::fs::read_to_string(dir.join("tiny.routed.kicad_pcb"))
        .unwrap()
        .matches("(segment")
        .count();
    let after = std::fs::read_to_string(&keep_out).unwrap().matches("(segment").count();
    assert!(after > before, "{before} {after}");

    // nothing left to route: the search has no connections to work on
    let kept = s.call("load_board", json!({"path": keep_out}));
    let done = dir.join("tiny.done.kicad_pcb");
    let r = s.call(
        "route",
        json!({"board": kept["board"], "keep_all": true, "nets": ["B"], "output": done}),
    );
    assert_eq!(r["connections"], 0, "{r}");
    assert_eq!(
        std::fs::read_to_string(&done).unwrap().matches("(segment").count(),
        after
    );

    // lock: parts and the copper of a net in a working copy, only the flag changes; unlock again
    let lk = s.call(
        "lock",
        json!({"board": kept["board"], "parts": ["R1", "R9"], "nets": ["B"]}),
    );
    assert_eq!(lk["locked"], true, "{lk}");
    assert_eq!(lk["parts"], json!(["R1"]), "{lk}");
    assert!(lk["tracks"].as_u64().unwrap() >= 1, "{lk}");
    assert!(lk["warnings"].to_string().contains("R9"), "{lk}");
    let lock_copy = dir.join("tiny.kept.prerouted.kicad_pcb");
    assert_eq!(lk["path"], lock_copy.display().to_string());
    let locked_text = std::fs::read_to_string(&lock_copy).unwrap();
    assert_eq!(locked_text.matches("(segment").count(), after);
    assert!(locked_text.matches("(locked yes)").count() >= 2, "{locked_text}");
    let un = s.call(
        "lock",
        json!({"board": lk["board"], "parts": ["R1"], "all_copper": true, "unlock": true}),
    );
    assert_eq!(un["locked"], false, "{un}");
    assert_eq!(un["tracks"], lk["tracks"], "{un}");
    assert!(!std::fs::read_to_string(&lock_copy).unwrap().contains("(locked yes)"));
    assert!(
        s.call_err("lock", json!({"board": lk["board"]}))
            .contains("nothing to lock")
    );

    // guide: the workflow by default, a tool's reference by name, the list for anything else
    let g = s.request("tools/call", json!({"name": "guide", "arguments": {}}));
    assert_eq!(g["isError"], json!(false), "{g}");
    let wf = g["content"][0]["text"].as_str().unwrap();
    assert!(wf.contains("not a one-stop shop") && wf.contains("`lock`"), "{wf}");
    assert_eq!(g["structuredContent"]["topic"], "workflow");
    let g = s.request("tools/call", json!({"name": "guide", "arguments": {"topic": "route"}}));
    assert!(
        g["content"][0]["text"].as_str().unwrap().starts_with("### route"),
        "{g}"
    );
    let g = s.request(
        "tools/call",
        json!({"name": "guide", "arguments": {"topic": "nonsense"}}),
    );
    assert_eq!(g["structuredContent"]["topic"], "index", "{g}");

    // reroute replaces the copper of the net, the other nets stay
    let again = dir.join("tiny.again.kicad_pcb");
    let r = s.call(
        "route",
        json!({"board": kept["board"], "keep_all": true, "reroute": true, "nets": ["B"], "runs": 1,
               "generations": 0, "output": again}),
    );
    assert_eq!(r["routed"], 1, "{r}");
    assert!(std::fs::read_to_string(&again).unwrap().contains("(segment"));
    assert!(
        s.call_err("route", json!({"board": kept["board"], "reroute": true}))
            .contains("reroute")
    );
    assert!(r["blocked"].as_array().unwrap().is_empty(), "{r}");
    // the other nets shift aside or give way by default; both can be switched off
    let still = dir.join("tiny.still.kicad_pcb");
    let r = s.call(
        "route",
        json!({"board": kept["board"], "keep_all": true, "reroute": true, "nets": ["B"], "runs": 1,
               "generations": 0, "shove": false, "give": 0, "output": still}),
    );
    assert_eq!(r["routed"], 1, "{r}");
    assert!(
        s.call_err("route", json!({"board": kept["board"], "shove": true}))
            .contains("keep_all")
    );

    // placement and zones go into the same working copy
    let moved = s.call(
        "move_part",
        json!({"board": locked["board"], "reference": "R2", "x": 113.0, "y": 111.0, "rotation": 90}),
    );
    let text = std::fs::read_to_string(&work).unwrap();
    assert!(text.contains("(at 113 111 90)"), "{text}");
    let zone = s.call(
        "add_zone",
        json!({"board": moved["board"], "net": "A", "layer": "B.Cu", "points": [[101, 101], [119, 101], [119, 114], [101, 114]]}),
    );
    assert_eq!(zone["items_added"], 1);
    assert!(std::fs::read_to_string(&work).unwrap().contains("(zone"));
    let placed = s.call(
        "place_parts",
        json!({"board": zone["board"], "only": ["R2"], "starts": 1}),
    );
    assert_eq!(placed["placed"].as_array().unwrap().len(), 1, "{placed}");
    assert_eq!(placed["placed"][0]["reference"], "R2", "{placed}");

    drop(s);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn layer_types_plan_a_plane() {
    let dir = std::env::temp_dir().join(format!("gummiband-mcp-layers-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let dir = dir.canonicalize().unwrap();
    let pcb = dir.join("four.kicad_pcb");
    let text = tiny_board().replace(
        "(2 \"B.Cu\" signal)",
        "(4 \"In1.Cu\" signal) (6 \"In2.Cu\" jumper) (2 \"B.Cu\" signal)",
    );
    std::fs::write(&pcb, text).unwrap();

    let mut s = Server::start();
    s.request(
        "initialize",
        json!({"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "test", "version": "0"}}),
    );
    s.send(&json!({"jsonrpc": "2.0", "method": "notifications/initialized"}));
    s.call("load_board", json!({"path": pcb.display().to_string()}));
    let r = s.call(
        "set_layer_type",
        json!({"board": "b1", "layers": [{"layer": "In1.Cu", "type": "power"}, {"layer": "In2.Cu", "type": "signal"}]}),
    );
    assert_eq!(r["changed"][0]["old_type"], "signal", "{r}");
    assert_eq!(r["changed"][1]["old_type"], "jumper", "{r}");
    let planes: Vec<bool> = r["layers"]
        .as_array()
        .unwrap()
        .iter()
        .map(|l| l["plane"].as_bool().unwrap())
        .collect();
    assert_eq!(planes, [false, true, false, false], "{r}");
    assert!(r["warnings"][0].as_str().unwrap().contains("no zone"), "{r}");
    let work = dir.join("four.prerouted.kicad_pcb");
    let text = std::fs::read_to_string(&work).unwrap();
    assert!(text.contains("(4 \"In1.Cu\" power) (6 \"In2.Cu\" signal)"), "{text}");
    // the zone on the power layer is the plane of its net
    let z = s.call(
        "add_zone",
        json!({"board": r["board"], "net": "A", "layer": "In1.Cu", "points": [[101, 101], [119, 101], [119, 114], [101, 114]]}),
    );
    let nets = s.call("list_nets", json!({"board": z["board"], "filter": "A"}));
    assert_eq!(nets["nets"][0]["plane"], true, "{nets}");
    assert!(
        s.call_err(
            "set_layer_type",
            json!({"board": "b1", "layers": [{"layer": "F.Cu", "type": "power"}]})
        )
        .contains("outer")
    );
    assert!(
        s.call_err(
            "set_layer_type",
            json!({"board": "b1", "layers": [{"layer": "In1.Cu", "type": "plane"}]})
        )
        .contains("layer type")
    );

    drop(s);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn good_practice_plan_check_and_pour() {
    let dir = std::env::temp_dir().join(format!("gummiband-mcp-practice-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let dir = dir.canonicalize().unwrap();
    let pcb = dir.join("tiny.kicad_pcb");
    std::fs::write(&pcb, tiny_board()).unwrap();

    let mut s = Server::start();
    s.request(
        "initialize",
        json!({"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "test", "version": "0"}}),
    );
    s.send(&json!({"jsonrpc": "2.0", "method": "notifications/initialized"}));

    // the workflow prompt
    let prompts = s.request("prompts/list", json!({}));
    assert_eq!(prompts["prompts"][0]["name"], "good_practice", "{prompts}");
    let p = s.request(
        "prompts/get",
        json!({"name": "good_practice", "arguments": {"path": "/x/board.kicad_pcb", "currents": "A 2 A"}}),
    );
    let text = p["messages"][0]["content"]["text"].as_str().unwrap();
    assert!(text.contains("/x/board.kicad_pcb") && text.contains("power_plan") && text.contains("A 2 A"));

    // plan: 2 A on net A needs more than the 0.2 mm default width (1 oz, no stackup)
    s.call("load_board", json!({"path": pcb.display().to_string()}));
    let plan = s.call(
        "power_plan",
        json!({"board": "b1", "currents": [{"net": "A", "amps": 2.0}], "apply": true}),
    );
    let w = plan["nets"][0]["track_width_mm"].as_f64().unwrap();
    assert!(w > 0.5 && w < 1.0, "{plan}");
    assert_eq!(plan["applied"][0]["net"], "A", "{plan}");
    assert!(
        plan["dru_rules"].as_str().unwrap().contains("A.NetName == 'A'"),
        "{plan}"
    );

    // the route takes the width
    let r = s.call("route", json!({"board": "b1", "runs": 1, "generations": 0}));
    assert_eq!(r["routed"], 2, "{r}");
    let a = r["nets"]
        .as_array()
        .unwrap()
        .iter()
        .find(|n| n["net"] == "A")
        .unwrap()
        .clone();
    assert!(a["min_width_mm"].as_f64().unwrap() > w - 0.01, "{a}");

    // check: the two pin net gets its cross-section measured automatically
    let out = dir.join("tiny.routed.kicad_pcb");
    let c = s.call(
        "current_check",
        json!({"path": out, "paths": [{"net": "A", "amps": 2.0}, {"net": "B", "amps": 2.0}]}),
    );
    assert_eq!(c["paths"][0]["verdict"], "ok", "{c}");
    assert!(c["paths"][0]["neck"]["width_mm"].as_f64().unwrap() >= w - 0.06, "{c}");
    // net B kept the 0.2 mm default: flagged with a position
    assert!(c["paths"][1]["verdict"].as_str().unwrap().starts_with("FLAG"), "{c}");
    assert!(c["paths"][1]["neck"]["x"].is_number(), "{c}");
    assert_eq!(c["flagged"], 1, "{c}");

    // pour: zones of B on both layers and stitching vias clear of net A
    let g = s.call(
        "ground_pour",
        json!({"path": out, "net": "B", "pitch": 4.0, "edge_pitch": 0}),
    );
    assert_eq!(g["zones_added"], json!(["F.Cu", "B.Cu"]), "{g}");
    assert!(g["vias_added"].as_u64().unwrap() > 0, "{g}");
    let text = std::fs::read_to_string(dir.join("tiny.routed.gnd.kicad_pcb")).unwrap();
    assert_eq!(text.matches("(zone ").count(), 2);
    assert!(text.contains("(locked yes)"));
}

#[test]
fn power_copper_pours_a_path_before_routing() {
    let dir = std::env::temp_dir().join(format!("gummiband-mcp-copper-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let dir = dir.canonicalize().unwrap();
    let pcb = dir.join("tiny.kicad_pcb");
    std::fs::write(&pcb, tiny_board()).unwrap();

    let mut s = Server::start();
    s.request(
        "initialize",
        json!({"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "test", "version": "0"}}),
    );
    s.send(&json!({"jsonrpc": "2.0", "method": "notifications/initialized"}));
    s.call("load_board", json!({"path": pcb.display().to_string()}));
    let c = s.call(
        "power_copper",
        json!({"board": "b1", "paths": [{"net": "A", "amps": 3.0, "from": ["R1.1"], "to": ["R2.1"]}]}),
    );
    let pour = &c["pours"][0];
    assert_eq!(pour["kind"], "pour", "{c}");
    assert_eq!(pour["layer"], "F.Cu", "{c}");
    // R1.2 of net B sits next to R1.1: the pour gives way to it and still joins both pads
    assert_eq!(pour["cut_by"], json!(["pad R1.2 (B)"]), "{c}");
    assert_eq!(c["warnings"], json!([]), "{c}");
    assert_eq!(c["check"]["paths"][0]["verdict"], "ok", "{c}");
    let work = dir.join("tiny.prerouted.kicad_pcb");
    let text = std::fs::read_to_string(&work).unwrap();
    assert!(
        text.contains("(name \"gb:power A\")") && text.contains("(connect_pads yes"),
        "{text}"
    );

    // the pour joins A already: only B is left to route, and it keeps out of the pour
    let r = s.call("route", json!({"board": c["board"], "runs": 1, "generations": 0}));
    assert_eq!(r["connections"], 1, "{r}");
    assert_eq!(r["routed"], 1, "{r}");
    assert_eq!(r["violations"], json!([]), "{r}");

    // a pour of A after routing: the small "gb:" pour on F.Cu is no reason to leave F.Cu out,
    // and the pads join solid with the spoke width asked for
    let g = s.call(
        "ground_pour",
        json!({"path": work, "net": "A", "pitch": 0, "edge_pitch": 0,
               "pad_connection": "thermal_tht", "thermal_spoke_width": 0.6}),
    );
    assert_eq!(g["zones_added"], json!(["F.Cu", "B.Cu"]), "{g}");
    let text = std::fs::read_to_string(dir.join("tiny.prerouted.gnd.kicad_pcb")).unwrap();
    assert_eq!(text.matches("(connect_pads thru_hole_only").count(), 2, "{text}");
    assert_eq!(text.matches("(thermal_bridge_width 0.6)").count(), 2, "{text}");
    let bad = s.call_err(
        "ground_pour",
        json!({"path": work, "net": "A", "pad_connection": "spokes"}),
    );
    assert!(bad.contains("thermal_tht"), "{bad}");
}

const NETLIST: &str = r#"(export (version "E")
  (components
    (comp (ref "R1") (value "10k") (footprint "GbRes:R_0603")
      (fields (field (name "LCSC") "C25804"))
      (sheetpath (names "/") (tstamps "/")) (tstamps "1111"))
    (comp (ref "J1") (value "USB") (footprint "GbConn:Shielded")
      (sheetpath (names "/") (tstamps "/")) (tstamps "2222")))
  (nets
    (net (code "1") (name "GND")
      (node (ref "R1") (pin "2") (pintype "passive"))
      (node (ref "J1") (pin "2") (pintype "power_in"))
      (node (ref "J1") (pin "S1") (pintype "passive")))
    (net (code "2") (name "VBUS")
      (node (ref "R1") (pin "1") (pintype "passive"))
      (node (ref "J1") (pin "1") (pintype "power_out")))))
"#;

const R0603: &str = r#"(footprint "R_0603" (version 20241229) (generator "test") (layer "F.Cu")
  (property "Reference" "REF**" (at 0 -1.43 0) (layer "F.SilkS") (uuid "11111111-1111-4111-8111-111111111111"))
  (property "Value" "R_0603" (at 0 1.43 0) (layer "F.Fab") (uuid "22222222-2222-4222-8222-222222222222"))
  (attr smd)
  (fp_rect (start -1.48 -0.73) (end 1.48 0.73) (stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd"))
  (pad "1" smd roundrect (at -0.7875 0) (size 0.875 0.95) (layers "F.Cu" "F.Mask" "F.Paste") (roundrect_rratio 0.25))
  (pad "2" smd roundrect (at 0.7875 0) (size 0.875 0.95) (layers "F.Cu" "F.Mask" "F.Paste") (roundrect_rratio 0.25)))
"#;

const SHIELDED: &str = r#"(footprint "Shielded" (version 20241229) (layer "F.Cu")
  (property "Reference" "REF**" (at 0 -3 0) (layer "F.SilkS"))
  (property "Value" "Shielded" (at 0 3 0) (layer "F.Fab"))
  (attr smd)
  (fp_rect (start -3.5 -2) (end 3.5 2) (stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd"))
  (pad "1" smd rect (at -1 0) (size 0.6 1) (layers "F.Cu" "F.Mask" "F.Paste"))
  (pad "2" smd rect (at 1 0) (size 0.6 1) (layers "F.Cu" "F.Mask" "F.Paste"))
  (pad "SH" thru_hole oval (at -2.5 0) (size 1 1.6) (drill 0.6) (layers "*.Cu" "*.Mask"))
  (pad "SH" thru_hole oval (at 2.5 0) (size 1 1.6) (drill 0.6) (layers "*.Cu" "*.Mask")))
"#;

#[test]
fn create_board_from_a_netlist() {
    let dir = std::env::temp_dir().join(format!("gummiband-mcp-create-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(dir.join("libs/GbRes.pretty")).unwrap();
    std::fs::create_dir_all(dir.join("other/GbConn.pretty")).unwrap();
    let dir = dir.canonicalize().unwrap();
    std::fs::write(dir.join("libs/GbRes.pretty/R_0603.kicad_mod"), R0603).unwrap();
    std::fs::write(dir.join("other/GbConn.pretty/Shielded.kicad_mod"), SHIELDED).unwrap();
    // one library through the project table, the other through footprint_dirs
    std::fs::write(
        dir.join("fp-lib-table"),
        "(fp_lib_table (version 7)\n  (lib (name \"GbRes\") (type \"KiCad\") (uri \"${KIPRJMOD}/libs/GbRes.pretty\") (options \"\") (descr \"\")))\n",
    )
    .unwrap();
    let net = dir.join("t.net");
    std::fs::write(&net, NETLIST).unwrap();
    let pcb = dir.join("t.kicad_pcb");

    let mut s = Server::start();
    s.request(
        "initialize",
        json!({"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "test", "version": "0"}}),
    );
    s.send(&json!({"jsonrpc": "2.0", "method": "notifications/initialized"}));

    let args = json!({
        "netlist": net.display().to_string(),
        "output": pcb.display().to_string(),
        "width": 30, "height": 25, "layers": 4, "plane_layers": ["In1.Cu", "In2.Cu"],
        "mounting_holes": {"diameter": 2.2, "inset": 3},
        "footprint_dirs": [dir.join("other").display().to_string()],
        "net_classes": [{"name": "Power", "track_width": 0.5, "patterns": ["VBUS", "GND"]}],
        "rules": {"min_clearance": 0.127},
    });
    let c = s.call("create_board", args.clone());
    assert_eq!(c["board"], "b1");
    assert_eq!(c["version"], 20241229);
    assert_eq!(c["footprints"], 6);
    assert_eq!(c["mounting_holes"], 4);
    assert_eq!(c["nets"], 2);
    assert_eq!(c["pads"], 6);
    assert_eq!(c["pads_with_net"], 4);
    assert_eq!(c["missing_footprints"], json!([]));
    assert_eq!(c["outside"], json!([]));
    assert_eq!(c["unmatched"][0]["reference"], "J1");
    assert_eq!(c["unmatched"][0]["pads"], json!(["SH"]));
    assert_eq!(c["unmatched"][0]["pins"], json!(["S1 (GND)"]));
    assert_eq!(c["unmatched"][0]["suggested_alias"], json!({"J1": {"S1": "SH"}}));
    let w = c["warnings"].as_array().unwrap();
    assert!(
        w.iter()
            .any(|w| w.as_str().unwrap().contains("pin_aliases {\"J1\": {\"S1\": \"SH\"}}")),
        "{w:?}"
    );
    assert!(c["libraries"]["GbRes"].as_str().unwrap().ends_with("libs/GbRes.pretty"));

    // the file exists now
    let e = s.call_err("create_board", args.clone());
    assert!(e.contains("overwrite"), "{e}");

    let mut args = args;
    args["overwrite"] = json!(true);
    args["pin_aliases"] = json!({"J1": {"S1": "SH"}});
    let c = s.call("create_board", args);
    assert_eq!(c["board"], "b1", "the same file keeps its id");
    assert_eq!(c["pads_with_net"], 6);
    assert_eq!(c["unmatched"], json!([]));
    assert_eq!(c["project_merged"], true);

    let b = s.call("load_board", json!({"path": pcb.display().to_string()}));
    assert_eq!(b["outline"]["width_mm"], 30.0);
    let planes: Vec<bool> = b["layers"]
        .as_array()
        .unwrap()
        .iter()
        .map(|l| l["plane"].as_bool().unwrap())
        .collect();
    assert_eq!(planes, [false, true, true, false]);
    let power = b["net_classes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|c| c["name"] == "Power")
        .unwrap()
        .clone();
    assert_eq!(power["track_width"], 0.5);
    let nets = s.call("list_nets", json!({"board": "b1", "filter": "GND"}));
    assert_eq!(nets["nets"][0]["pins"], 4, "R1.2, J1.2 and both SH pads");
    assert_eq!(nets["nets"][0]["class"], "Power");

    let pro: Value = serde_json::from_str(&std::fs::read_to_string(dir.join("t.kicad_pro")).unwrap()).unwrap();
    assert_eq!(pro["board"]["design_settings"]["rules"]["min_clearance"], 0.127);
    let text = std::fs::read_to_string(&pcb).unwrap();
    assert!(text.contains("(property \"LCSC\" \"C25804\""));
    assert!(text.contains("(path \"/1111\")"));
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn track_corridors_list_the_parts_in_the_way() {
    let dir = std::env::temp_dir().join(format!("gummiband-mcp-corridors-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let dir = dir.canonicalize().unwrap();
    let bench = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../bench/boards");
    for ext in ["kicad_pcb", "kicad_pro", "kicad_dru"] {
        std::fs::copy(bench.join(format!("parts_xhard.{ext}")), dir.join(format!("x.{ext}"))).unwrap();
    }
    let mut s = Server::start();
    s.request(
        "initialize",
        json!({"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "test", "version": "0"}}),
    );
    s.send(&json!({"jsonrpc": "2.0", "method": "notifications/initialized"}));
    let pcb = dir.join("x.kicad_pcb");
    let before = std::fs::read_to_string(&pcb).unwrap();
    s.call("load_board", json!({"path": pcb.display().to_string()}));

    let c = s.call("track_corridors", json!({"board": "b1"}));
    let all: Vec<&str> = c["in_the_way"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_str().unwrap())
        .collect();
    // the same parts as `gummiband route --clear-corridors` on this board
    assert_eq!(
        all,
        [
            "F10", "F11", "F12", "F14", "F2", "F32", "F33", "F37", "F38", "F39", "R13"
        ],
        "{c}"
    );
    let corridors = c["corridors"].as_array().unwrap();
    assert!(!corridors.is_empty());
    for g in corridors {
        assert!(g["nets"].as_array().unwrap().len() >= 2, "{g}");
        assert!(g["width_mm"].as_f64().unwrap() > 0.0, "{g}");
        assert!(g["length_mm"].as_f64().unwrap() > 0.0, "{g}");
    }
    // only listed, nothing moved
    assert_eq!(std::fs::read_to_string(&pcb).unwrap(), before);
    assert!(!dir.join("x.prerouted.kicad_pcb").exists());

    // a part named fixed is never in the way
    let c = s.call("track_corridors", json!({"board": "b1", "fixed": ["R13"]}));
    assert!(!c["in_the_way"].as_array().unwrap().iter().any(|v| v == "R13"), "{c}");

    // the agent moves some of them itself, with a short leash
    let placed = s.call(
        "place_parts",
        json!({"board": "b1", "only": ["F10", "F11"], "corridors": true, "leash": 1.5}),
    );
    let moved: Vec<&str> = placed["placed"]
        .as_array()
        .unwrap()
        .iter()
        .map(|p| p["reference"].as_str().unwrap())
        .collect();
    assert_eq!(moved, ["F10", "F11"], "{placed}");

    let err = s.call_err(
        "route",
        json!({"board": "b1", "clear_corridors": true, "keep_all": true}),
    );
    assert!(err.contains("not with keep_all"), "{err}");
    let _ = std::fs::remove_dir_all(&dir);
}
