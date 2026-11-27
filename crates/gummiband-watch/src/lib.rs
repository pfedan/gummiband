//! Watch a route while it runs.
//!
//! A [`Feed`] collects what a viewer is shown: the board as a scene, and the views the search
//! hands to the [`Watch`] the feed gives out for it ([`gummiband_core::watch`]). A [`Server`]
//! offers a feed over HTTP on a port of this machine, with the viewer page (`viewer.html`, one
//! file, WebGL) at `/`. [`Feed::recording`] is the same page with everything the feed holds
//! inside it: a file that replays the route without a server.
//!
//! The feed keeps the whole route, so a browser that opens late sees it from the start, and
//! thins out old glimpses when it grows beyond [`FEED_BYTES`].

use std::io::{BufRead, BufReader, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use gummiband_core::board::{Board, ObstacleKind};
use gummiband_core::geom::{Point, Polygon, Shape};
use gummiband_core::watch::{Stage, UNTAGGED, View, Watch};
use serde_json::{Value, json};

/// The viewer page.
const VIEWER: &str = include_str!("viewer.html");

/// The line of the viewer page in whose place a recording puts its data.
const RECORDING_SLOT: &str = "<!--GB-RECORDING-->";

/// Where a front end serves the viewer when it is given no address.
pub const DEFAULT_ADDRESS: &str = "127.0.0.1:7878";

/// The address to listen on for what a user wrote: nothing is [`DEFAULT_ADDRESS`], a port alone
/// is that port for this machine only, anything else is taken as it is (`0.0.0.0:7878` lets
/// other machines look).
pub fn address(given: Option<&str>) -> String {
    match given.map(str::trim).filter(|a| !a.is_empty()) {
        None => DEFAULT_ADDRESS.to_string(),
        Some(a) => match a.parse::<u16>() {
            Ok(port) => format!("127.0.0.1:{port}"),
            Err(_) => a.to_string(),
        },
    }
}

/// What a feed keeps of one route before it thins out its glimpses.
pub const FEED_BYTES: usize = 192 << 20;

/// The glimpses of a watched run come at most this often.
pub const GLIMPSE_EVERY: Duration = Duration::from_millis(80);

/// A search shows at most this many of the places it looked at; of more, every n-th.
const EXPLORED_MAX: usize = 30_000;

/// The same for a step: a run has one per connection.
const EXPLORED_STEP: usize = 500;

/// How long a request of the viewer waits for something new before it gets an empty answer.
const POLL_WAIT: Duration = Duration::from_secs(20);

const KIND_SCENE: u32 = 1;
const KIND_FRAME: u32 = 2;

struct Item {
    seq: u32,
    kind: u32,
    /// Kept when the feed thins out: scenes, the best candidates after the phases and the
    /// whole views of a run.
    key: bool,
    data: Arc<Vec<u8>>,
}

struct State {
    /// Counts the routes: a viewer that sees a new number starts over.
    session: u32,
    title: String,
    items: Vec<Item>,
    seq: u32,
    bytes: usize,
    limit: usize,
    finished: bool,
    t0: Instant,
    last_scene: Option<Arc<Vec<u8>>>,
    /// A viewer asked for this route.
    viewed: bool,
    /// A viewer got the route to its end.
    delivered: bool,
}

impl State {
    fn push(&mut self, kind: u32, key: bool, data: Vec<u8>) -> Arc<Vec<u8>> {
        self.seq += 1;
        self.bytes += data.len();
        let data = Arc::new(data);
        self.items.push(Item {
            seq: self.seq,
            kind,
            key,
            data: data.clone(),
        });
        while self.bytes > self.limit && self.thin() {}
        data
    }

    /// Takes out every second step and relaxation round of the older half. False: there was
    /// none to take.
    fn thin(&mut self) -> bool {
        let half = self.items.len() / 2;
        let mut odd = false;
        let mut gone = 0;
        let mut i = 0;
        self.items.retain(|it| {
            i += 1;
            if i > half || it.key {
                return true;
            }
            odd = !odd;
            if odd {
                gone += it.data.len();
            }
            !odd
        });
        self.bytes -= gone;
        gone > 0
    }

    /// The answer to a viewer that has everything up to `after` of `session`.
    fn answer(&self, session: u32, after: u32) -> Vec<u8> {
        let after = if session == self.session { after } else { 0 };
        let title = self.title.as_bytes();
        let mut out = Vec::new();
        out.extend_from_slice(b"GBW1");
        out.extend_from_slice(&self.session.to_le_bytes());
        out.extend_from_slice(&(self.finished as u32).to_le_bytes());
        out.extend_from_slice(&(title.len() as u32).to_le_bytes());
        out.extend_from_slice(title);
        pad4(&mut out, b' ');
        for it in self.items.iter().filter(|it| it.seq > after) {
            out.extend_from_slice(&it.kind.to_le_bytes());
            out.extend_from_slice(&it.seq.to_le_bytes());
            out.extend_from_slice(&(it.data.len() as u32).to_le_bytes());
            out.extend_from_slice(&it.data);
        }
        out
    }
}

struct Shared {
    state: Mutex<State>,
    changed: Condvar,
}

/// Everything a viewer is shown of the routes of a front end, one after the other.
#[derive(Clone)]
pub struct Feed {
    shared: Arc<Shared>,
}

impl Default for Feed {
    fn default() -> Self {
        Feed::with_limit(FEED_BYTES)
    }
}

impl Feed {
    pub fn new() -> Feed {
        Feed::default()
    }

    /// A feed that thins out its glimpses beyond `limit` bytes.
    pub fn with_limit(limit: usize) -> Feed {
        Feed {
            shared: Arc::new(Shared {
                state: Mutex::new(State {
                    session: 0,
                    title: String::new(),
                    items: Vec::new(),
                    seq: 0,
                    bytes: 0,
                    limit,
                    finished: true,
                    t0: Instant::now(),
                    last_scene: None,
                    viewed: false,
                    delivered: false,
                }),
                changed: Condvar::new(),
            }),
        }
    }

    /// A new route begins: the viewers forget the last one. `title` is what they call it.
    pub fn begin(&self, title: &str) {
        let mut s = self.shared.state.lock().unwrap();
        s.session += 1;
        s.title = title.to_string();
        s.items.clear();
        s.seq = 0;
        s.bytes = 0;
        s.finished = false;
        s.t0 = Instant::now();
        s.last_scene = None;
        s.viewed = false;
        s.delivered = false;
        self.shared.changed.notify_all();
    }

    /// The route is over; nothing more comes until the next [`Feed::begin`].
    pub fn finish(&self) {
        self.shared.state.lock().unwrap().finished = true;
        self.shared.changed.notify_all();
    }

    /// Waits, at most `wait`, until a viewer that follows the route has got it to its end: for
    /// a front end that ends with its route, and would take the last views with it. Returns at
    /// once when nobody watches.
    pub fn deliver(&self, wait: Duration) {
        let s = self.shared.state.lock().unwrap();
        let _ = self
            .shared
            .changed
            .wait_timeout_while(s, wait, |s| s.viewed && !s.delivered)
            .unwrap();
    }

    /// The watch for a search on `board`. The board becomes the scene of the views that follow
    /// (if it is not the scene already). This is what `gummiband_app::Hooks::watch` asks for.
    pub fn watch(&self, board: &Board) -> Watch {
        let scene = scene(board);
        {
            let mut s = self.shared.state.lock().unwrap();
            if s.last_scene.as_deref() != Some(&scene) {
                s.last_scene = Some(s.push(KIND_SCENE, true, scene));
                self.shared.changed.notify_all();
            }
        }
        let shared = self.shared.clone();
        Watch::new(GLIMPSE_EVERY, move |v: &View| {
            let t0 = shared.state.lock().unwrap().t0;
            let data = frame(v, t0.elapsed());
            let mut s = shared.state.lock().unwrap();
            // the steps of a run change its whole views, so those stay when the feed thins out
            s.push(KIND_FRAME, matches!(v.stage, Stage::Best | Stage::Laying), data);
            shared.changed.notify_all();
        })
    }

    /// Number of scenes and of views the feed holds.
    pub fn counts(&self) -> (usize, usize) {
        let s = self.shared.state.lock().unwrap();
        let scenes = s.items.iter().filter(|it| it.kind == KIND_SCENE).count();
        (scenes, s.items.len() - scenes)
    }

    /// The viewer page with the route the feed holds inside it: a file that replays the route
    /// wherever it is opened.
    pub fn recording(&self) -> String {
        let data = self.shared.state.lock().unwrap().answer(0, 0);
        let tag = format!(
            "<script id=\"gb-recording\" type=\"application/octet-stream\">{}</script>",
            base64(&data)
        );
        VIEWER.replacen(RECORDING_SLOT, &tag, 1)
    }

    /// Waits until the feed has something a viewer at (`session`, `after`) lacks, or its route
    /// ended since the viewer saw it running, at most `wait`; then the answer for it.
    fn poll(&self, session: u32, after: u32, finished: bool, wait: Duration) -> Vec<u8> {
        let mut s = self.shared.state.lock().unwrap();
        s.viewed = true;
        let (mut s, _) = self
            .shared
            .changed
            .wait_timeout_while(s, wait, |s| {
                s.session == session && s.seq <= after && s.finished == finished
            })
            .unwrap();
        if s.finished && !s.delivered {
            s.delivered = true;
            self.shared.changed.notify_all();
        }
        s.answer(session, after)
    }
}

fn pad4(out: &mut Vec<u8>, with: u8) {
    while out.len() % 4 != 0 {
        out.push(with);
    }
}

fn base64(data: &[u8]) -> String {
    const ABC: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for c in data.chunks(3) {
        let n = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
        for (i, shift) in [18, 12, 6, 0].into_iter().enumerate() {
            out.push(if i <= c.len() {
                ABC[(n >> shift & 63) as usize] as char
            } else {
                '='
            });
        }
    }
    out
}

/// A tenth of a micrometre is all a viewer needs, and keeps the numbers short.
fn mm(x: f64) -> f64 {
    (x * 1e4).round() / 1e4
}

fn flat(p: &Polygon) -> Vec<f64> {
    p.pts.iter().flat_map(|q| [mm(q.x), mm(q.y)]).collect()
}

/// The board as the viewer draws it under the copper of the route, as JSON:
///
/// - `bbox`: `[x0, y0, x1, y1]` of the outline, mm
/// - `layers`: `[{name, plane}]`, the copper layers by id
/// - `nets`: the net names by id
/// - `outline`, `courtyards`: closed polygons, each `[x, y, x, y, ...]`
/// - `shapes`: the copper of the board, its rule areas and holes, each
///   `[kind, layers, net, form, numbers...]`: `kind` an index into `kinds`, `layers` a bit per
///   layer id, `net` -1 for none, `form` 0 a circle (`x, y, r`), 1 a line with round ends
///   (`ax, ay, bx, by, r`), 2 a polygon (`x, y, ...`)
/// - `planes`: `[layer, net, x, y, ...]`, the outlines of the planes
fn scene(board: &Board) -> Vec<u8> {
    const KINDS: [&str; 8] = ["pad", "track", "via", "keepout", "zone", "pour", "graphic", "hole"];
    let bb = board.bbox();
    let shapes: Vec<Value> = board
        .obstacles
        .iter()
        .map(|o| {
            let kind = match o.kind {
                ObstacleKind::Pad => 0,
                ObstacleKind::Track => 1,
                ObstacleKind::Via => 2,
                ObstacleKind::Keepout { .. } => 3,
                ObstacleKind::Zone => 4,
                ObstacleKind::Pour => 5,
                ObstacleKind::Graphic => 6,
                ObstacleKind::Hole => 7,
            };
            let mut v = vec![json!(kind), json!(o.layers.0), json!(o.net.map_or(-1, |n| n as i64))];
            let nums: Vec<f64> = match &o.shape {
                Shape::Circle { c, r } => {
                    v.push(json!(0));
                    vec![mm(c.x), mm(c.y), mm(*r)]
                }
                Shape::Capsule { a, b, r } => {
                    v.push(json!(1));
                    vec![mm(a.x), mm(a.y), mm(b.x), mm(b.y), mm(*r)]
                }
                Shape::Polygon(p) => {
                    v.push(json!(2));
                    flat(p)
                }
            };
            v.extend(nums.into_iter().map(|x| json!(x)));
            // the hole of a pad or a via, for the viewer to drill it
            if let (Some(h), Shape::Circle { .. }) = (o.hole, &o.shape) {
                v.push(json!(mm(h)));
            }
            Value::Array(v)
        })
        .collect();
    let mut courtyards: Vec<(&String, Vec<f64>)> = Vec::new();
    for (name, r) in &board.courtyards {
        match board.courtyard_shapes.get(name) {
            Some(polys) => courtyards.extend(polys.iter().map(|p| (name, flat(p)))),
            None => courtyards.push((
                name,
                [r.min.x, r.min.y, r.max.x, r.min.y, r.max.x, r.max.y, r.min.x, r.max.y]
                    .map(mm)
                    .to_vec(),
            )),
        }
    }
    // the maps of the board have no order of their own
    courtyards.sort_by(|a, b| a.0.cmp(b.0).then(a.1.len().cmp(&b.1.len())));
    let planes: Vec<Value> = board
        .planes
        .iter()
        .map(|p| {
            let mut v = vec![json!(p.layer), json!(p.net)];
            v.extend(flat(&p.outline).into_iter().map(|x| json!(x)));
            Value::Array(v)
        })
        .collect();
    let scene = json!({
        "bbox": [mm(bb.min.x), mm(bb.min.y), mm(bb.max.x), mm(bb.max.y)],
        "layers": board.layers.iter().map(|l| json!({"name": l.name, "plane": l.is_plane})).collect::<Vec<_>>(),
        "nets": board.nets.iter().map(|n| n.name.as_str()).collect::<Vec<_>>(),
        "kinds": KINDS,
        "outline": board.outline.iter().map(flat).collect::<Vec<_>>(),
        "courtyards": courtyards.iter().map(|c| json!({"ref": c.0, "pts": c.1})).collect::<Vec<_>>(),
        "shapes": shapes,
        "planes": planes,
    });
    let mut out = serde_json::to_vec(&scene).unwrap();
    pad4(&mut out, b' ');
    out
}

/// A view as the viewer gets it: the length of a JSON head, the head (padded with spaces to a
/// multiple of four bytes), then 32 bit floats, little endian:
///
/// - per track `points, layer, net, width, connection` and its points `x, y`
/// - per via `x, y, diameter, drill, half x, half y, net, site`
/// - per open connection `ax, ay, bx, by, connection`
/// - per place the search looked at `x, y, layer`
/// - per clearance violation `x, y`
///
/// A connection or a site is -1 where the copper has none. The head says how many of each
/// there are (`n`), with the number of floats of the tracks. A view of the stage `step` is a
/// change of the views before it: the copper and the open lines of the connections in `gone`
/// and the vias of the sites in `gone_sites` go, what the view holds comes.
fn frame(v: &View, t: Duration) -> Vec<u8> {
    let cap = if v.stage == Stage::Step {
        EXPLORED_STEP
    } else {
        EXPLORED_MAX
    };
    let stride = v.explored.len().div_ceil(cap).max(1);
    let explored: Vec<&(Point, u8)> = v.explored.iter().step_by(stride).collect();
    let line_floats: usize = v.lines.iter().map(|l| 5 + 2 * l.pts.len()).sum();
    let length: f64 = v
        .lines
        .iter()
        .map(|l| l.pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>())
        .sum();
    let mut head = json!({
        "t": t.as_millis() as u64,
        "stage": match v.stage {
            Stage::Laying => "laying",
            Stage::Step => "step",
            Stage::Relaxing => "relaxing",
            Stage::Best => "best",
        },
        "phase": v.phase,
        "step": [v.step.0, v.step.1],
        "routed": v.routed.map(|r| [r.0, r.1]),
        "violations": v.violations,
        "vias": v.dots.len(),
        "length": mm(length),
        "n": {
            "lines": v.lines.len(),
            "line_floats": line_floats,
            "dots": v.dots.len(),
            "open": v.open.len(),
            "explored": explored.len(),
            "spots": v.spots.len(),
        },
    });
    if v.stage == Stage::Step {
        head["gone"] = json!(v.gone);
        head["gone_sites"] = json!(v.gone_sites);
    }
    let mut head = serde_json::to_vec(&head).unwrap();
    pad4(&mut head, b' ');
    let floats = line_floats + 8 * v.dots.len() + 5 * v.open.len() + 3 * explored.len() + 2 * v.spots.len();
    let mut out = Vec::with_capacity(4 + head.len() + 4 * floats);
    out.extend_from_slice(&(head.len() as u32).to_le_bytes());
    out.extend_from_slice(&head);
    let mut put = |x: f64| out.extend_from_slice(&(x as f32).to_le_bytes());
    let tag = |t: u32| if t == UNTAGGED { -1.0 } else { t as f64 };
    for l in &v.lines {
        for x in [l.pts.len() as f64, l.layer as f64, l.net as f64, l.width, tag(l.conn)] {
            put(x);
        }
        for p in l.pts {
            put(p.x);
            put(p.y);
        }
    }
    for d in &v.dots {
        for x in [
            d.at.x,
            d.at.y,
            d.diameter,
            d.drill,
            d.half.x,
            d.half.y,
            d.net as f64,
            tag(d.site),
        ] {
            put(x);
        }
    }
    for (a, b, conn) in &v.open {
        for x in [a.x, a.y, b.x, b.y, tag(*conn)] {
            put(x);
        }
    }
    for (p, l) in explored {
        for x in [p.x, p.y, *l as f64] {
            put(x);
        }
    }
    for p in v.spots {
        put(p.x);
        put(p.y);
    }
    out
}

/// A feed on a port of this machine. The threads of the server end with the process.
pub struct Server {
    addr: SocketAddr,
}

impl Server {
    /// Listens on `addr` (`127.0.0.1:7878`; port 0 takes a free one) and serves `feed`.
    pub fn bind(addr: &str, feed: &Feed) -> std::io::Result<Server> {
        let listener = TcpListener::bind(addr)?;
        let addr = listener.local_addr()?;
        let feed = feed.clone();
        std::thread::Builder::new()
            .name("gummiband-watch".into())
            .spawn(move || {
                for stream in listener.incoming().flatten() {
                    let feed = feed.clone();
                    let _ = std::thread::Builder::new()
                        .name("gummiband-watch-conn".into())
                        .spawn(move || {
                            let _ = serve(stream, &feed);
                        });
                }
            })?;
        Ok(Server { addr })
    }

    pub fn addr(&self) -> SocketAddr {
        self.addr
    }

    /// Where a browser finds the viewer.
    pub fn url(&self) -> String {
        let host = if self.addr.ip().is_unspecified() {
            "localhost".to_string()
        } else {
            self.addr.ip().to_string()
        };
        format!("http://{host}:{}/", self.addr.port())
    }
}

/// Answers the requests of one connection until it closes.
fn serve(stream: TcpStream, feed: &Feed) -> std::io::Result<()> {
    stream.set_read_timeout(Some(Duration::from_secs(120)))?;
    stream.set_nodelay(true)?;
    let mut reader = BufReader::new(stream.try_clone()?);
    let mut stream = stream;
    loop {
        let mut request = String::new();
        if reader.read_line(&mut request)? == 0 {
            return Ok(());
        }
        // the viewer sends nothing but GET, so the head is all there is to read
        loop {
            let mut line = String::new();
            if reader.read_line(&mut line)? == 0 || line == "\r\n" || line == "\n" {
                break;
            }
        }
        let mut parts = request.split_whitespace();
        let (method, target) = (parts.next().unwrap_or(""), parts.next().unwrap_or(""));
        let (path, query) = target.split_once('?').unwrap_or((target, ""));
        let arg = |name: &str| -> u32 {
            query
                .split('&')
                .filter_map(|kv| kv.split_once('='))
                .find(|kv| kv.0 == name)
                .and_then(|kv| kv.1.parse().ok())
                .unwrap_or(0)
        };
        let (status, kind, body): (&str, &str, Vec<u8>) = match (method, path) {
            ("GET", "/") | ("GET", "/index.html") => ("200 OK", "text/html; charset=utf-8", VIEWER.as_bytes().to_vec()),
            ("GET", "/poll") => (
                "200 OK",
                "application/octet-stream",
                feed.poll(arg("s"), arg("after"), arg("f") != 0, POLL_WAIT),
            ),
            ("GET", _) => ("404 Not Found", "text/plain", b"not found\n".to_vec()),
            _ => ("405 Method Not Allowed", "text/plain", b"GET only\n".to_vec()),
        };
        write!(
            stream,
            "HTTP/1.1 {status}\r\nContent-Type: {kind}\r\nContent-Length: {}\r\nCache-Control: no-store\r\n\r\n",
            body.len()
        )?;
        stream.write_all(&body)?;
        stream.flush()?;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gummiband_core::realize::Geometry;

    fn views(feed: &Feed, board: &Board, glimpses: usize) {
        let geo = Geometry::default();
        let w = feed.watch(board);
        w.show(&View::of(Stage::Laying, "portfolio", &geo));
        for _ in 0..glimpses {
            w.show(&View::of(Stage::Step, "portfolio", &geo));
        }
        w.show(&View::of(Stage::Best, "portfolio", &geo));
    }

    /// The items of an answer: (kind, seq, length).
    fn items(answer: &[u8]) -> Vec<(u32, u32, usize)> {
        let u = |at: usize| u32::from_le_bytes(answer[at..at + 4].try_into().unwrap());
        assert_eq!(&answer[..4], b"GBW1");
        let mut at = (16 + u(12) as usize).next_multiple_of(4);
        let mut out = Vec::new();
        while at < answer.len() {
            out.push((u(at), u(at + 4), u(at + 8) as usize));
            at += 12 + u(at + 8) as usize;
            assert_eq!(at % 4, 0, "floats start on a multiple of four");
        }
        out
    }

    #[test]
    fn a_port_alone_is_this_machine_only() {
        assert_eq!(address(None), DEFAULT_ADDRESS);
        assert_eq!(address(Some(" ")), DEFAULT_ADDRESS);
        assert_eq!(address(Some("9000")), "127.0.0.1:9000");
        assert_eq!(address(Some("0.0.0.0:9000")), "0.0.0.0:9000");
    }

    #[test]
    fn base64_as_everyone_writes_it() {
        assert_eq!(base64(b""), "");
        assert_eq!(base64(b"f"), "Zg==");
        assert_eq!(base64(b"fo"), "Zm8=");
        assert_eq!(base64(b"foobar"), "Zm9vYmFy");
    }

    #[test]
    fn a_viewer_gets_what_it_lacks_and_starts_over_on_a_new_route() {
        let board = Board::default();
        let feed = Feed::new();
        feed.begin("first");
        views(&feed, &board, 2);
        let all = items(&feed.poll(0, 0, false, Duration::ZERO));
        assert_eq!(
            all.iter().map(|i| (i.0, i.1)).collect::<Vec<_>>(),
            [(1, 1), (2, 2), (2, 3), (2, 4), (2, 5)]
        );
        // the same board again is no new scene
        views(&feed, &board, 0);
        let more = items(&feed.poll(1, 5, false, Duration::ZERO));
        assert_eq!(more.iter().map(|i| (i.0, i.1)).collect::<Vec<_>>(), [(2, 6), (2, 7)]);
        assert!(items(&feed.poll(1, 7, false, Duration::ZERO)).is_empty());
        feed.begin("second");
        views(&feed, &board, 0);
        let fresh = feed.poll(1, 7, false, Duration::ZERO);
        assert_eq!(u32::from_le_bytes(fresh[4..8].try_into().unwrap()), 2);
        assert_eq!(items(&fresh).len(), 3);
    }

    #[test]
    fn a_full_feed_thins_out_old_steps_and_keeps_the_rest() {
        let board = Board::default();
        let one = frame(
            &View::of(Stage::Step, "portfolio", &Geometry::default()),
            Duration::ZERO,
        )
        .len();
        let feed = Feed::with_limit(scene(&board).len() + 12 * one);
        feed.begin("full");
        views(&feed, &board, 40);
        let left = items(&feed.poll(0, 0, false, Duration::ZERO));
        assert!(left.len() < 20, "{} items left", left.len());
        // the scene and the whole view the steps change are still there
        assert_eq!(
            left[..2].iter().map(|i| (i.0, i.1)).collect::<Vec<_>>(),
            [(KIND_SCENE, 1), (KIND_FRAME, 2)]
        );
        // the last one is the best after the phase, and the newest steps are all there
        assert!(left.windows(2).all(|w| w[0].1 < w[1].1));
        assert_eq!(
            left[left.len() - 3..].iter().map(|i| i.1).collect::<Vec<_>>(),
            [41, 42, 43]
        );
    }

    #[test]
    fn the_end_of_a_route_wakes_a_waiting_viewer() {
        let feed = Feed::new();
        feed.begin("ends");
        let f = feed.clone();
        let waiting = std::thread::spawn(move || f.poll(1, 0, false, Duration::from_secs(30)));
        std::thread::sleep(Duration::from_millis(50));
        feed.finish();
        let answer = waiting.join().unwrap();
        assert_eq!(u32::from_le_bytes(answer[8..12].try_into().unwrap()), 1);
    }

    #[test]
    fn a_front_end_that_ends_waits_for_its_viewer_only() {
        let feed = Feed::new();
        feed.begin("unwatched");
        feed.finish();
        let t = Instant::now();
        feed.deliver(Duration::from_secs(30));
        assert!(t.elapsed() < Duration::from_secs(5), "nobody watches: no wait");
        feed.begin("watched");
        feed.poll(2, 0, false, Duration::ZERO);
        feed.finish();
        let f = feed.clone();
        let viewer = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            f.poll(2, 0, false, Duration::ZERO)
        });
        feed.deliver(Duration::from_secs(30));
        assert!(feed.shared.state.lock().unwrap().delivered);
        viewer.join().unwrap();
    }

    #[test]
    fn the_server_hands_out_the_page_and_the_feed() {
        use std::io::Read;
        let feed = Feed::new();
        feed.begin("served");
        views(&feed, &Board::default(), 1);
        let server = Server::bind("127.0.0.1:0", &feed).unwrap();
        let get = |path: &str| -> Vec<u8> {
            let mut s = TcpStream::connect(server.addr()).unwrap();
            write!(s, "GET {path} HTTP/1.1\r\nHost: x\r\nConnection: close\r\n\r\n").unwrap();
            s.shutdown(std::net::Shutdown::Write).unwrap();
            let mut all = Vec::new();
            s.read_to_end(&mut all).unwrap();
            let split = all.windows(4).position(|w| w == b"\r\n\r\n").unwrap();
            assert!(
                all.starts_with(b"HTTP/1.1 200 OK"),
                "{}",
                String::from_utf8_lossy(&all[..split])
            );
            all[split + 4..].to_vec()
        };
        assert_eq!(get("/"), VIEWER.as_bytes());
        assert_eq!(items(&get("/poll?s=0&after=0")).len(), 4);
        assert!(server.url().starts_with("http://127.0.0.1:"));
    }

    #[test]
    fn a_recording_is_the_page_with_the_feed_in_it() {
        assert_eq!(VIEWER.matches(RECORDING_SLOT).count(), 1);
        let feed = Feed::new();
        feed.begin("recorded");
        views(&feed, &Board::default(), 1);
        let page = feed.recording();
        assert!(page.contains(&base64(&feed.poll(0, 0, false, Duration::ZERO))));
        assert!(!page.contains(RECORDING_SLOT));
    }
}
