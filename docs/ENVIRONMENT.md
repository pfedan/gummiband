# Environment variables

<!-- Printed from the table in crates/gummiband-core/src/knobs.rs, do not edit by hand:
     cargo run -p gummiband-core --example environment > docs/ENVIRONMENT.md
     A test of gummiband-core fails when the two differ. -->

The router reads the `GB_*` variables below. They are for development and measurements, not a
stable interface: they are not part of the options of the CLI, the MCP server or the plugin, and
any of them may change or go with any commit. For normal work use the options
([command line](guide/cli.md), [MCP server](MCP.md)).

- They act in every program that routes: the `gummiband` CLI, the MCP server and, through the
  CLI it starts, the KiCad plugin. A variable set in the environment of an MCP client or of KiCad
  reaches the router unseen, so keep them out of such environments.
- With none of them set the router gives the same files byte for byte (`bench/hashes.sh`).
- A variable is read once per process, when it is first needed.
- Each keeps the way its value is read: *set to anything* means that also `0` or an empty value
  turns it on; *`0` turns it off* means that every other value, and no value, leaves it on.
- The table lives in `gummiband_core::knobs`, with the accessors the code uses; no other code
  reads a `GB_` variable.

## Logs

Output for the developer; the routed board stays the same.

| Variable | Value | Without it | What it does |
| --- | --- | --- | --- |
| `GB_LOG` | value | none | Comma separated topics of debug output on stderr, see the list of topics; `all` turns every topic on. |
| `GB_WHY` | value | unset | A pin name (`U1.3`): trace the search of the first open connection from this pin into `/tmp/gb-why.txt`; `--svg` draws it. |
| `GB_MUTANT_LOG` | value | unset | A file path: the open end writes one line of JSON per mutant (why it ended, what it changed). |
| `GB_NEAR` | value | unset | CLI `route`: `x,y[,radius]` in mm lists the tracks with a vertex near this point. |
| `GB_VIOL` | set to anything | off | CLI `route`: histograms of how much clearance the violations miss, and a sample of the severe ones. |

### Topics of `GB_LOG`

`GB_LOG=open,timing` turns on the topics it names, separated by commas; `all` is every topic. The output goes to stderr. A name that is no topic gets a warning. The variables of the last column did the same before 2026-11-10 and are gone.

| Topic | Output | Replaces |
| --- | --- | --- |
| `open` | The open end: one line per generation, side branches, merges, the rounds that make room. | `GB_OPEN_LOG` |
| `pair` | Differential pairs: breakouts, joints, and joins of the two tracks that found no short way. | `GB_PAIR_LOG` |
| `tune` | Length tuning: the rounds that make room for the meanders. | `GB_TUNE_LOG` |
| `neg` | Negotiated routing: conflicts and open connections round by round. | `GB_NEG_LOG` |
| `zip` | What closing up the pairs at the end (`pairs::zip`) did. | `GB_ZIP_LOG` |
| `plane` | The open end says when a new via would leave the plane of its net. | `GB_PLANE_LOG` |
| `octo` | `--angles 45`: the pieces that stay at their angle, and what is in their way. | `GB_OCTO_DEBUG` |
| `vias` | After a router run: via pairs and vias of the topology. | `GB_DEBUG_VIAS` |
| `corridors` | After a router run: corridors of the topology. | `GB_DEBUG_CORRIDOR` |
| `intersections` | After a router run: intersections in the topology. | `GB_DEBUG_X` |
| `timing` | Cumulative phase timers and work counters at the end of a route, and the split of each relaxation. | `GB_TIMING` |
| `validate` | After a router run: check the inner state of the router and print the problems. | `GB_VALIDATE` |
| `validate-each` | Check the inner state after every routed connection and print the one that broke it (slow). | `GB_VALIDATE_EACH` |
| `areas` | `areas fill`, in the last check: every track of a net left to its fill with its two ends and whether each is held, and the track ends that are taken back. |  |

## Switches

Each turns off a behaviour that is on by default (or, for the `GB_NO_*` names, a step of the run), to compare a run with and without it. DECISIONS.md promises the switches of the open end and of the freedoms a board leaves.

| Variable | Value | Without it | What it does |
| --- | --- | --- | --- |
| `GB_AUTO_NEGOTIATE` | `0` turns it off | on | A plain search that leaves at most 15 % of the connections open starts a negotiated search on its own. |
| `GB_TAPER` | `0` turns it off | on | The last step tapers a track at a spot that misses its clearance by a little. |
| `GB_EXITS` | `0` turns it off | on | A track leaves a pad in a tight row by a fixed straight piece on the middle line of the pad (`exit.rs`). |
| `GB_STITCH` | `0` turns it off | on | Layer changes of current paths get the vias that carry their current (`relax::stitch_vias`). |
| `GB_PIVOT_ARCS` | `0` turns it off | on | The finished copper gets its last form from its pivots (`relax::pivot_pass`): one arc around each pad corner, via or bundle a track bends around, with the radius of what lies between, and tangent lines. A track keeps the corners the shortcuts gave it where such an arc or line comes too close, and where the own check finds something new. |
| `GB_SPREAD` | `0` turns it off | on | Where the own check finds tracks too near to something and the passage has room for them, the tracks through it are laid again as one group (`spread::spread`): from one side outwards with the clearance and 2 um, lines and arcs, taken only if the check then finds nothing new. |
| `GB_ZIP` | `0` turns it off | on | Pairs the search pulled together close up to their gap at the end (`pairs::zip`). |
| `GB_GOAL_CHECK` | `0` turns it off | on | A search asks early whether its goal is shut in. The results are the same without, only slower. |
| `GB_BANDIT` | `0` turns it off | on | Open end, combination A: the via price of a move is drawn by what has won on this board (Thompson sampling). |
| `GB_AIM_COST` | number | 0.25 | Open end, combination A: this share of the moves aims at an expensive routed connection; `0` turns it off. |
| `GB_PRUNE` | `0` turns it off | on | Open end, combination A: a mutant that cannot beat its parent is dropped before its relaxation. |
| `GB_REF_COST` | number | 0.5 | Price per mm of a signal track over no copper of the plane next to its layer, and per 2 mm for a split of that plane (`0` turns it off). |
| `GB_OWN_VIA` | number | inf | A plane fan-out ends on a via of its own, not on the via or the fan-out track of another pin of its net (`inf`); a number lets it share a via at that factor of the via cost, `0` as before. |
| `GB_PLANE_HOLD` | `0` turns it off | on | Open end: a new via that reaches the plane of its net stays in it when it is moved. |
| `GB_NO_RELAX` | set to anything | off | The search returns its best topology without the geometry stage (for `examples/relax_bench.rs`). |
| `GB_NO_AREA_REPAIR` | set to anything | off | `areas fill`: what the last check finds open of the nets left to their fill is not laid again on the finished copper. |
| `GB_NO_AREA_FALLBACK` | set to anything | off | `areas fill`: a board whose route leaves a connection open is not routed a second time with every net as tracks. |
| `GB_NO_POLISH` | set to anything | off | Skips the polish relaxation of a winner that has clearance spots left. |
| `GB_NO_ARCS` | set to anything | off | Tracks are realised as polylines, without arcs around the corners. |
| `GB_NO_TUNE_ROOM` | set to anything | off | CLI `route`: length tuning does not make room for its meanders first. |

## Experiments and measurements

Seeds and limits for benchmarks, constants under test, and ideas that are off by default; `docs/MILESTONES.md` has the measurements. A number that does not parse counts as unset.

| Variable | Value | Without it | What it does |
| --- | --- | --- | --- |
| `GB_EXIT_INWARD` | `0` turns it off | on | Pads in a tight row get an exit under the part too, where there is room (off: outwards only). |
| `GB_EXIT_RAIL` | number | 0.15 | How far beyond its exit point a track stays on the middle line of its pad, in mm (0: it may turn at the exit point). |
| `GB_BAND` | set to anything | off | The band of a track takes the radius of the stack where the track lies next to the vertex it bends around, and where a wrap has no tangent, leaves its portal or its triangles, that wrap alone becomes a corner; today such a track loses all its arcs. |
| `GB_DISC` | set to anything | off | The band of a track is built from discs around every vertex of its triangles, each with the radius the tracks between it and the vertex ask for, pulled taut against them; no portal and no share of the slack. |
| `GB_EVO_SEED` | value | 0x5eed | Seed of the search when the caller gives none (`route --seed`). Search changes are judged over several seeds with it. |
| `GB_GPU` | set to anything | off | Relaxation rounds on the GPU; needs a binary built with the `gpu` feature. |
| `GB_OPEN_GENERATIONS` | value | no limit | Benchmarks: the open end stops by itself after this many generations. |
| `GB_OPEN_POP` | value | the size of the portfolio | Open end: mutants per generation. |
| `GB_LOCAL_IT` | number | 160 | Open end: relaxation rounds of a mutant. |
| `GB_TIGHTEN` | number | 24 | Open end: every this many generations the whole board is cleaned and relaxed, also what no move touched. |
| `GB_BRANCH_AFTER` | number | 24 | Open end: generations without a gain before a side branch starts. |
| `GB_BRANCH_LIFE` | number | 32 | Open end: generations a side branch gets. |
| `GB_BOLD_AIM` | number | 0.8 | Open end: share of the bold moves that aim at an open connection while there is one. |
| `GB_VIA_PRICE` | set to anything | off | Open end idea: the via price of a move reaches up to what a via is worth in the cost of the result. |
| `GB_MERGE` | set to anything | off | Open end idea: all mutants of a generation that win in places apart from each other count. |
| `GB_SPOT_FADE` | set to anything | off | Open end idea: a clearance spot that many moves failed to clear gets fewer moves. |
| `GB_WINDOW` | number | 0 | Open end idea: a mutant relaxes only the copper within this many mm of its new copper (0: the whole board). |
| `GB_AIR_IT` | number | 120 | Relaxation rounds of the steps that give copper air or make room (`--air`, room for tuning). |
| `GB_ROOM_ROUNDS` | number | 6 | Rounds of making room for meanders before length tuning. |
| `GB_GEO_SCREEN` | value | off | `k`: a geometry generation relaxes all candidates briefly and only the best k fully. |
| `GB_FINALISTS` | number | 24 | How many candidates of the search go on to the geometry stage. |
| `GB_FULL` | number | 6 | How many finalists get the full relaxation after the short one. |
| `GB_DIRS_ALL` | set to anything | off | Every run of the portfolio uses the preferred layer directions, not every third. |
| `GB_PLACE_ROUNDS` | number | 2 | Rounds in which parts give way at hot spots (`--move-parts`). |
| `GB_PLACE_AFTER_STOP` | set to anything | off | An open end that was stopped still gets the placement rounds. |
| `GB_NEG_ONLY` | set to anything | off | With `--negotiate`: only the negotiated search, without the plain one to compare with. |
| `GB_NEG_GROWTH` | number | 1.5 | Negotiated routing: factor by which the prices of overflow and crossing grow per round. |
| `GB_NEG_HISTORY` | number | 1.0 | Negotiated routing: how much a conflict that stays adds to the lasting price of its edge. |
| `GB_NEG_OVERFLOW` | number | 2.0 | Negotiated routing: starting price per mm of overflow of an edge. |
| `GB_NEG_CROSSING` | number | 1.5 | Negotiated routing: starting price per crossed track of another net. |
| `GB_NEG_TIGHT` | number | 3.0 | Negotiated routing: price of a crossing that leaves an edge no room at all. |
| `GB_NEG_RESERVE` | number | 0.05 | Negotiated routing: room in mm per gap on an edge from which on a crossing is free. |
| `GB_NEG_VIA` | number | 0 | Negotiated routing: a new via costs at least this many crossings at the price of the round (0: the plain via price). |
| `GB_NEG_VIA_CAP` | number | 3 | Negotiated routing: the most that GB_NEG_VIA raises the price of a via, as a multiple of the plain via price. |
| `GB_BACK_COST` | number | 0 | Extra cost per mm of track on the back layer of a board with more than one routing layer (0: off). |
| `GB_MAXEXP` | number | 400000 | Most expansions of one search for a connection. |
| `GB_RIPUP` | number | 3 | Rip-up passes of a router run. |
| `GB_ESCAPE` | number | 0 | Extra cost in mm for passing right next to a pad of another net that still waits for its route. |
| `GB_SEARCH_MARGIN` | number | 0.01 | Search: room a track or via asks for beyond its clearance, in mm. |
| `GB_AREA_ESCAPE` | number | 2 | `areas fill`: extra cost in mm for passing right next to a pad of a net left to the fill of its area, while the other nets are laid. |
| `GB_AREA_SLACK` | number | 0.035 | `areas fill`: added to every clearance of the fill when the model runs inside the search, in mm; more of it and the search counts on wider strips of fill only. |
| `GB_SHARE_KIND` | value | all kinds | `--patterns shares`: only the shares of this kind (`neighbours` or `decap`). |
| `GB_SKEW_WEIGHT` | number | 0.5 | Weight of a mm of skew need in the cost of a candidate. |
| `GB_REFINE` | number | 0.6 | Triangulation: long obstacle sides get a vertex every this many mm. |
| `GB_FEET` | number | 0 | Triangulation: a corner within this many mm of a side of other copper puts a vertex at its foot on that side (0: none). |
| `GB_ROUND_EXACT` | set to anything | off | Search: room next to round copper is measured from the circle, not from the corners of the polygon around it. |
| `GB_RELAX_IT` | number | 400 | Rounds of a full relaxation. |
| `GB_RELAX_SETTLE` | number | 0 | Relaxation: up to this many further rounds that only resolve clearances, until nothing moves. |
| `GB_SPACING` | number | 0.05 | Relaxation: distance of the points on a track in mm. |
| `GB_COARSE` | value | twice the spacing, at least 0.1 | Relaxation: distance of the points in mm while the bands still move a lot. |
| `GB_SHIFT` | value | unset | CLI `route`, debugging: `R8:0.3,0;C4:0,-0.2` routes the board with these parts shifted by mm. |
| `GB_CC_GAP` | value | the gap of `place` | CLI `route --clear-corridors`: gap between the parts placed again, in mm. |
| `GB_CC_HALO` | value | the halo of `place` | CLI `route --clear-corridors`: halo around the parts placed again, in mm. |
| `GB_CC_ESCAPE` | value | off | CLI `route --clear-corridors`: a number above 0 turns the escape halo on. |
| `GB_CC_SEMANTIC` | value | off | CLI `route --clear-corridors`: a number above 0 turns the semantic placement on. |
| `GB_CC_LEASH` | value | 1.5 | CLI `route --clear-corridors`: how far ICs and connectors in a band may move, in mm. |

## Other variables

Not the router's own, and read where they are needed:

| Variable | Read by | What for |
| --- | --- | --- |
| `RAYON_NUM_THREADS` | CLI, MCP server | Number of threads (CLI: the default of `--threads`; MCP server: 4 without it). The result does not depend on it. |
| `GUMMIBAND_KICAD_CLI`, `GUMMIBAND_RSVG_CONVERT` | MCP server | Paths of `kicad-cli` and `rsvg-convert`, before `PATH` and the usual install places. |
| `KICAD10_FOOTPRINT_DIR` ... `KICAD7_FOOTPRINT_DIR`, `KICAD_FOOTPRINT_DIR` | kicad crate | Footprint libraries, when a board is created from a netlist. |
| any `${NAME}` in an `fp-lib-table` | kicad crate | KiCad's path variables in library tables. |
| `HOME`, `APPDATA` | kicad crate | KiCad's global library table and a Flatpak install. |
