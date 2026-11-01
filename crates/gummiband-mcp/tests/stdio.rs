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
            "current_check",
            "drc",
            "ground_pour",
            "list_nets",
            "load_board",
            "move_part",
            "part_info",
            "place_parts",
            "power_copper",
            "power_plan",
            "render",
            "route",
            "set_net_options"
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

    // keep_all adds to a routed board: net B joins, the tracks of A stay
    let routed = s.call("load_board", json!({"path": dir.join("tiny.routed.kicad_pcb")}));
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
}
