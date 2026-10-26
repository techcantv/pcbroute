# pcbroute development log

> These are the working notes from building pcbroute inside a larger project (a motor-driver board), kept as
> they were. Paths like `router/...`, `ml/...`, `bench/...` and `gen_pcb.py` refer to that project and are not part
> of this repository; the benchmark data isn't either. The router sources and every option described here are.

A parallel grid autorouter for KiCad boards, as a drop-in replacement for Freerouting's batch mode: it reads the
Specctra DSN that KiCad exports and writes a Specctra session (SES) that KiCad (or `gen_pcb.py`'s `read_ses`)
imports.

```bash
make -C router                                   # C++17, no dependencies; builds router/build/pcbroute
router/build/pcbroute -de board.dsn -do board.ses
router/build/pcbroute -de board.dsn --eval any_router.ses     # judge a session (open connections, violations, length, vias)
PCB_ROUTER=$PWD/router/build/pcbroute python3 hardware/kicad/gen_pcb.py   # use it in the bench pipeline
```

Freerouting's options (`-mp`, `--gui.*`, `--router.*`) are accepted and ignored. Run `pcbroute -h` for the rest.
The result is the same whatever `-mt` (the thread count) is; only the time changes.

`--events FILE` appends live router events as JSON lines (one flushed line each) for Product Studio's Router tab
(protocol v1: `docs/ROUTE_EVENTS.md` in Product Studio, not published, reader `npi.pcb.route_events`; `Board.route()` passes
the flag because `--help` lists it, and `ml/mlroute` passes it through). `begin` once the DSN is read; per
negotiation round a `pass` (`route`) and a `progress` (open connections, nets in conflict); a `net` (all its copper
now, in session units, plus its open connections) whenever a net's copper changed: as nets are committed during a
round (at most every 0.2 s, so a big board's first round fills in live) and in a sweep after each round. Legalizing
and the rescue are `pass` `ripup`, smoothing is `pass` `optimize`, then every net's final copper (exactly what the
session holds) and `end` (`ok: false` and `error` on a failure). Off unless given. With it the routes are identical
and the time is unchanged (9 bench boards, 3 runs each: 1.01x geo mean). A 138-net, 4-minute board writes ~4 MB.

## How it routes

- **Per-class clearance grids** (50 um pitch by default). For each net class being routed, every cell records
  which nets may put a track centre (and, separately, a via centre) there. That accounts for the class's width
  and clearance against every other net's copper, the keepouts and the board edge. Each clearance radius R gets the
  margin that keeps a straight move between two free cell centres legal: the sagitta R - sqrt(R^2 - h^2), h being
  half a diagonal step (2-7 um at 50 um for usual rules; `--wide-margin` restores the old h + 1 um on everything,
  which took 36 um off every channel).
- **Planes are pours.** By default (`--plane-mode reserve`) a DSN `plane` is its net's copper over its whole
  outline: that net joins it anywhere, and no other net may route there, so a GND plane on an inner layer takes the
  whole layer. `--plane-mode refill` treats it like a refilled KiCad zone: other nets route through with their
  clearance, the pour's net joins it where it is still copper, and a pour cut into islands is repaired (see
  "Inner pours" below). Its net can also reach it with a via on a layer it doesn't route on. Pours that must not be
  crossed come with explicit keepouts, as in `gen_pcb.py`'s `pass_dsn`.
- **Fine-pitch pins** (LQFP at 0.5 mm) leave no free cell centre inside the pad. Those pads get off-grid escape
  stubs: straight from the pad centre in 8 directions to the first free cell, checked against the exact geometry.
  Where the class width can't get out between the neighbouring pads (0.4 mm QFN pins under 0.2 / 0.2 mm rules) the
  stub **necks down**: narrower in steps of a tenth of the width, to `--neck-min` (100 um) but never below half the
  class width. `-v` counts the pads that needed it.
- **Negotiated congestion (PathFinder).** Every net is routed, overlaps are allowed at a price, and the price
  rises each round (`--pres-growth`, `--pres-max`). The cells where conflicts happen also get a growing history
  cost (`--hist-inc`), until nobody overlaps. Conflicting nets are repaired locally: only the stretch over another
  net is cut out, 2 mm each side and wider while the net stays stuck, and reconnected. What still conflicts at the
  end is legalized: everyone in conflict repairs around the others as hard obstacles, in parallel; a net that can't
  get through pushes its way across at a very high price and the nets it crosses repair in turn (rip-up and
  reroute, `--ripups` times per net). What still overlaps after that loses just the paths that overlap (open
  connections rather than violations); the rest of the net stays.
- **Rescue.** Each net still open then pushes through once more, the nets it crosses legalize around it with a
  fresh rip-up budget, and the board is kept only if fewer connections are open in all, else restored
  (`--rescue N` rounds, default 3; capped by `--rescue-time`, default half the routing time so far, at least 10 s).
  So it never makes a result worse.
- **Time limit.** `--time-limit S` makes a slow board finish with a legal, partial result instead of being killed:
  negotiation stops before a round would run past half of S, rip-ups stop at 80 %, the last forced pass stops at S
  and drops what still overlaps, and rescue and shove get only the time left. The session, `--dump-hist` and
  `--dump-nets` are written as usual, and stderr gets `pcbroute: time limit reached (<what was cut>)` before the
  summary line. One net's search is never interrupted, so a run can still go somewhat past S. Without the flag
  nothing changes.
- **One number per cell.** What the search reads is a cost map per class and layer: the step cost factor
  (1 + history) (1 + pressure x occupancy), infinite where a track centre can't go, negated where the cell is one
  net's own copper. It is kept in step with the occupancy as routes are stamped. A net's own footprint is never
  unstamped before its search: the search skips it through a bitmap of the window, so nets can be routed
  concurrently against the board exactly as committed.
- **Search.** A* with a per-layer heuristic: the cheapest walk on an empty grid with that layer's direction costs
  (F.Cu prefers x, B.Cu y) to the targets on that layer, from a chamfer distance map of the window, or to the
  nearest target on any layer plus a via. Every step costs at least that, so paths are shortest. A connection is
  first probed from the far piece towards the net's tree within a small budget (`--probe`): a piece walled in is
  found out cheaply and a short hop is done with; then the full search runs. A tree of a plane net has millions of
  states (every cell of the pour): it is kept as bits of the window, grown as the tree grows, and such nets are
  always searched from the pieces towards it.
- **Stamping.** A route's footprint on every class's grids is the disc of the clearance radius around each path
  cell (grown by half a cell for the diagonal steps), applied to horizontal runs of cells; vias likewise.
- **Parallel, and the same at any thread count.** Nets are committed one by one in a fixed order: the queue
  regrouped into maximal sets of nets whose boxes don't overlap, group after group, so that nets routed at the same
  time mostly don't interact. Each thread takes the next net in that order and routes it against the board as
  committed so far. Every search records the 16x16-cell blocks it read; a net is committed once every net before
  it is, if no commit since its search started touched a block it read, else it goes back with its old route,
  together with everything after it, and is routed again. So the result is exactly that of routing the nets one by
  one in that order, however many threads run, and the threads run ahead of the commit point on speculation.
  Negotiation remains chaotic in the sense that a small change to the board (or to a parameter) changes the
  routes; it is just no longer the thread count or timing that changes them.
- **Global routing first** (experimental, off by default: `--gcell 1500`): nets are routed on a coarse grid
  (1.5 mm gcells, capacity = tracks that fit straight through, negotiated congestion); each net's path widened by
  one gcell is the corridor its detailed search stays in, falling back to the full window if the corridor is a
  dead end.
- **GPU (optional).** `make -C router GPU=1 BD=build-gpu` also builds `src/gpu.cu` (CUDA, tested on an RTX 3060).
  A search that runs past `--gpu-pops` (300000) A* pops is finished on the GPU: the cost maps of its window (the
  net's own footprint left out, the learned prior's factor and corridor applied, where a via may land), solved
  exactly by repeated min-plus sweeps along rows, columns and diagonals plus layer changes, until nothing changes;
  the cheapest target is walked back on the CPU through the same costs (a path two vias of a kind too close, or that
  a concurrent commit changed under, goes back to the CPU search). So the GPU takes any search, in any round, with
  or without the prior, and tells "unreachable" exactly where the CPU would have burnt its budget (the walled-in
  pads of the input pass). `--gpu-slots` searches run at once; the others wait for a slot, so the result does not
  depend on timing (`--gpu-nowait` lets them carry on on the CPU instead). The blocks the GPU read count as read by
  the search for the commit check. `GPU_TIMES=1` prints each search's upload / sweep / total time, `GPU_PROF=1` the
  time per scan direction, `GPU_CHECK=1` verifies every "unreachable" verdict against the CPU search.
- **Straightening.** The grid staircases are redrawn with the fewest 0/45/90 degree segments that clear
  everything exactly.
- **Self-check.** The output is checked against the exact geometry (clearances by class, keepouts, board edge)
  before it is written. The `violations` count in the summary line is that check. Exit code 2 if it isn't 0.
- **Learned prior (optional).** `--ml-prior FILE` takes per-net maps of where the net's copper is likely to run
  (`src/mlprior.cpp`), made by `ml/mlroute` from RouteNet, a model trained on human-routed open-source boards. A step
  costs up to `1 + --ml-weight` times more where the map is below `--ml-sat` (no extra cost above it, which keeps
  the A* heuristic tight), and `--ml-corridor T` keeps the search where the map is at least `T` until that's a dead
  end. Off by default. `--dump-board F` writes the parsed board as JSON (what the model sees). See
  [ml/README.md](ml/README.md).
- **Learned prior in process (optional).** `make -C router ONNXRT=1 BD=build-onnx` (with `GPU=1` too:
  `BD=build-gpu-onnx`) adds `src/mlinfer.cpp`: `--ml-model ml/models/routenet.onnx` rasterizes the board, runs
  RouteNet through ONNX Runtime (CUDA when it loads, else the CPU; `--ml-device`) and builds the prior itself,
  without Python: the same maps as `ml/mlroute` (inputs bit for bit, maps to float rounding) and the same routes.
  The model loads while the DSN is read and predicts while the grids are built. `--ml-prior-out F` keeps the prior
  (without `-do` it stops there). The default build has no ONNX Runtime dependency and refuses `--ml-model`.
  `--ml-serve SOCK --ml-model …` keeps the model resident (CUDA warm) on a unix socket and `--ml-server SOCK` (or
  `PCBROUTE_ML_SERVER`) sends the batches there: the same maps byte for byte, ~1 s less per board, in process
  whenever no server answers (`--ml-server-spawn` then starts one). See ml/README.md.
- **Learned order (optional).** `--order FILE` routes the nets in the order the file lists them (one name per
  line, hardest first, optionally a TAB and a difficulty in 0-1), the unlisted ones after in the default order
  (biggest box first); `route_waves`' box-disjoint grouping then follows that order. `--budget-scale S` gives a
  net of difficulty d `1 + (S-1) d` times its search budget and window from the first round. `ml/mlorder` writes
  the file from the difficulty model (ml/difficulty.py) and runs pcbroute with it. Off by default; without these
  options the output is byte-identical to the build before them.

- **Net types (optional).** `--nt-dump F` classifies the nets (ground, power, differential pair, high-speed, crystal,
  analog, signal) and the `--nt-*` options apply routing guidelines per type; `nt_metrics.py` measures them. Off by
  default (byte-identical without them); see "Net types and routing guidelines" below.

`-v` prints the rounds (commits, rejected speculative routes, nets and cells in conflict, pops, time), the
slowest nets and the time per phase; `-vv` adds every search's size.

## Judging and benchmarking routers

`pcbroute -de board.dsn --eval board.ses` reads any router's session and reports, by the board's own rules:
nets to route, connections, **open** connections (pieces of a net its copper plus the session leave apart),
clearance / keepout / edge **violations** (the same exact check as the self-check), track length and vias. Exit
code 2 if anything is open or violated. It is the one yardstick `benchmark.py` applies to every router:

```bash
python3 router/benchmark.py --configs new,v1,freerouting,fastroute --boards 'router/ml/data/bench/*.dsn' --out bench/run.json
python3 router/benchmark.py --report bench/run.json --ref v1
```

Configs: `new` (`build/pcbroute`), `v1` (`build-v1/pcbroute`, the build before the 2026-10-14 rework), `new8` /
`v18` (8 threads), `freerouting` (2.4.1, `-mp 1`, 8 threads, no optimizer: `gen_pcb.py`'s settings), `fastroute`
(the Rust port of Freerouting, `~/.local/bin/fastroute`), and any command line with `--cmd 'name=cmd {dsn} {ses}
{threads}'`. Boards: the 34 held-out open-source boards of the learned prior's test set (`ml/data/bench`, their
tracks removed, outer pours stripped as `gen_pcb.py` does) and the four stepdrive passes (`bench/stepdrive`, from
`gen_pcb.py --work DIR --stop-after logic`). `bench/run_final.sh` is the run behind the tables below.

### Results (2026-10-14, Ryzen 7 2700, 16 threads unless noted)

The 34 held-out boards (`bench/final_boards.txt` has the per-board rows; one pcbroute job of another project was
using 3 cores throughout, for every router alike):

| | pcbroute (this) | pcbroute, 8 threads | pcbroute before the rework | Freerouting 2.4.1 | fastroute |
|---|---|---|---|---|---|
| wall time, all boards | **611 s** | 589 s | 2144 s | 2711 s (2 timeouts at 900 s) | 756 s |
| time per board vs before, geometric mean | 0.60 | 0.56 | 1 | 2.41 | 0.42 |
| open connections | 194 | 194 | 182 | 521 | 548 |
| clearance violations (`--eval`) | 0 | 0 | 0 | 68 | 169 |
| boards routed clean | 23 | 23 | 20 | 8 | 6 |
| vias | 1968 | 1968 | 2447 | 1237 | 815 |
| track length | 45.9 m | 45.9 m | 47.6 m | 38.5 m | 42.9 m |

Of the 24 boards that took the old build 2 s or more, the new one is faster on 19 (the six biggest: 461 -> 71 s,
355 -> 82 s, 233 -> 68 s, 214 -> 22 s, 206 -> 47 s, 187 -> 24 s) and slower on 5 (worst 34 -> 62 s). Open
connections move by a few either way from one build to the next: negotiation is chaotic, and the boards with
dozens open have pads no track of the class width can reach (both builds, and Freerouting, leave them). Freerouting
and fastroute are judged with `-mp 1` (one routing pass, no optimizer), as `gen_pcb.py` runs them; their shorter
track length and fewer vias go with the many connections they leave open. fastroute's violations are its own
output (it routes closer to pads than the DSN clearance allows).

The stepdrive passes (`bench/final_stepdrive.txt`; the logic pass gets 3 passes in Freerouting / fastroute as in
`gen_pcb.py`):

| pass | pcbroute (this) | before the rework | Freerouting | fastroute |
|---|---|---|---|---|
| gate | 2.5 s, 0 open | 1.9 s, 0 open | 41 s, 0 open | 4.3 s, 0 open |
| input | 82 s, 3 open | 35 s, 2 open | 236 s, 6 open | 6.3 s, 3 open |
| power | 4.9 s, 0 open | 3.1 s, 0 open | 358 s, 12 open | 32 s, 8 open |
| logic | **93 s, 0 open** | 210 s, 2 open | 1038 s, 6 open | 181 s, 15 open, 3 violations |

The input pass (wide power tracks, pours) is the one case the rework made slower: most of its searches end by
running out of budget or by exhausting a walled region, which the old search did with fewer pops.

What the time went into, before: the tree of a plane net (every cell of the pour, ~2 M states) re-scanned for every
connection; three memory loads per neighbour; a heuristic that ignored the direction costs; capsules stamped per
staircase segment; and, at 16 threads, a parallel scheme whose results and times depended on the thread count (the
same board routed in 2 s at 1 thread and 10 s at 16). Found on the way: two A* traps, floating-point sums of equal
costs that reopened closed states by thousandths of a micron (9 M re-expansions in one search), and a coarse
heuristic map read nearest-neighbour that is inconsistent by a whole coarse step; and the straightener accepting a
track within 1 um of a keepout that the checker rejects. The parameter sweeps (`bench/tune*.json`, run with the
deterministic router so that pops, rounds and open counts are exact) found no better pressure or history schedule
than the defaults; `--hweight 1.1-1.2` is 20-30 % faster on most boards but 2-4x slower on the most congested one,
so it is left optional.

### Completion work (2026-10-15)

Aimed at leaving nothing open. On the 30 benchmark boards (3 800 connections, `ml/bench/v2/*.json`), counting the
pads pcbroute couldn't enter at all ("unreachable", which the tables above left out) together with the open
connections:

| build | open | unreachable | unconnected in all | complete | route time |
|---|---|---|---|---|---|
| 2026-10-14 rework | 67 | 154 | 221 | 94.2 % | 507 s |
| + neck-down stubs, drop only the overlapping paths | 164 | 17 | 181 | 95.2 % | 772 s |
| + exact grid margin | 100 | 17 | 117 | 96.9 % | 645 s |
| + rescue (this) | **87** | **17** | **104** | **97.3 %** | 845 s |

Violations stayed 0 (self-check and `--eval`); an AddressSanitizer / UBSan build ran three neck-down-heavy boards
clean. The neck-down made 137 unreachable pads reachable, at first at the cost of many new open connections (a
whole net was dropped if any of it could not be made legal: +3.3V on OpenFC-Lite lost 43 connections for 3 bad
paths). The time went up on the boards whose fine-pitch pads now take part (kicanvas 17 -> 161 s, OpenFC-Lite 97 ->
270 s); the rescue costs up to half the routing time and only runs when something is open. Stepdrive: gate, power
and logic 0 open (logic 84 -> 57 s); the input pass stayed at 3 open, which turned out to be a fence bug in
`gen_pcb.py`, not a placement limit (see "The input pass's 3 open connections" below).

What is left on the benchmark is mostly the boards' rules, not the router: the three worst (OpenFC-Lite 56,
goldfish 13, Pi5 PCIe switch 5) were routed by their designers with 0.1-0.15 mm tracks while their DSN says 0.2 / 0.2
mm (0.4 mm QFN pins 0.2 mm apart), and the 17 unreachable pads sit inside wire keepouts the footprints define (a
USB-C connector, an inductor) or are pads no stub of half the class width gets out of.

### GPU build (2026-10-15)

`build-gpu/pcbroute` with the rewritten `src/gpu.cu` (RTX 3060, 16 CPU threads). The stepdrive passes, route time
and open connections (`--eval`, 0 violations throughout; GPU results are byte-identical from run to run):

| pass | CPU build | GPU build | on the GPU |
|---|---|---|---|
| gate | 1.6 s, 0 open | 1.4 s, 0 open | nothing past the threshold |
| power | 3.1 s, 0 open | 3.0 s, 0 open | nothing past the threshold |
| input (DSN from before the fence fix below) | 104 s, 3 open | **78 s, 3 open** | 111 searches finished, 161 found unreachable (the CPU burnt 3 M pops on each) |
| logic | 50 s, 0 open | **33-39 s, 0 open** | 154 searches finished (17 sweeps, ~80 ms each), 11 back to the CPU |
| logic, `--gpu-pops 100000` | | 36 s, 0 open | 210 finished |
| logic with the learned prior (`ml/mlroute`) | 112 s, 0 open | 97-101 s, 1 open | 380 finished, 28 unreachable |

The GPU's "unreachable" verdicts are what the input pass gains from: its wide power tracks have pads walled in, and
the CPU search can only find that out by exhausting its budget. With the learned prior the heuristic is looser, so
2.5 times as many searches cross the threshold and go to the GPU; the prior still costs twice the time of plain
pcbroute (see `ml/README.md`). The GPU build makes more vias on the logic pass (182 vs 167) and takes more rounds
(23 vs 13): its paths are as cheap as the CPU's but tie-break differently, and the negotiation is chaotic.

The 30 held-out boards (`ml/bench/v2/gpu_t16.json`, one run each at 16 threads, `ml/bench.py --configs
base,baseg,w2,w2g`; `w2` = `ml/mlroute --ml-weight 2`, the learned prior; `g` = the GPU build):

| | pcbroute CPU | pcbroute GPU | prior, CPU | prior, GPU |
|---|---|---|---|---|
| route time, all boards | 753 s | **609 s** | 1 062 s | 856 s |
| time per board vs the CPU build, geometric mean | 1 | 0.98 (faster on 13 of 30) | 1.49 | 1.18 |
| open connections (+ 17 unreachable pads, the same for all) | 87 | 92 | 93 | **83** |
| boards routed clean | 22 | 23 | 22 | 24 |
| vias | 2 100 | 2 124 | 2 246 | 2 280 |

The GPU pays on the boards that take long: OpenFC-Lite 242 -> 171 s, OtterCam 58 -> 35 s, Pi5 PCIe switch 57 -> 30 s,
bakeneko 29 -> 22 s; the small boards never cross the threshold and lose nothing. With the prior it pays more
(0.80 per board, faster on 21 of 30: bakeneko 91 -> 42 s, teensy-eurorack 70 -> 30 s, OtterCam 117 -> 67 s), since
the prior's looser heuristic is what sends searches past the threshold. Completion moves by a few connections either
way per board between any two of the four (OpenFC-Lite: 50, 58, 51, 51 open), the chaos of negotiation rather than
a property of the build; the prior with the GPU came out best on this run (83 open, 24 clean boards), the prior
alone worst, and the plain GPU build is the fastest by a fifth.

### The input pass's 3 open connections (2026-10-15)

Routed alone (`--only VB`), VB still left one connection open: its rail-switch strip at x = 85 mm was walled in on
F.Cu with In1 free underneath (`--only VB --dump 86 48 12` showed the track grid of In1 empty and the via grid blocked
everywhere). The cause was the pass fence in `gen_pcb.py`'s `pass_dsn()`: "Input stays off B.Cu in the power
domain" was written as a plain DSN `(keepout ...)` on B.Cu, which forbids wires and vias on that layer, and a through
via is on every layer, so no Input via could exist anywhere in the power domain: the class's In1 layer was
unreachable, and the three phases had to share F.Cu alone. The DRC rule `write_dru()` makes of the same fence only
disallows tracks. A fence on some layers is now a `wire_keepout` (on every layer: still a `keepout`). The input
pass then routes clean: 36 of 36 connections in 3 s (8 vias), from 104 s with 3 open, and KiCad's DRC with the
domain rules passes the result. Freerouting reads `wire_keepout` too; its result on the pass is unchanged (6 open).
`bench/stepdrive/*.dsn` were regenerated from the fixed generator (the old set is in `bench/stepdrive/prefence/`);
the tables above used the old set. On the new set, 16 threads: input 3.4 s (CPU) / 2.9 s (GPU), logic 49 s (CPU) /
39 s (GPU), gate 1.4 s, power 1.8 s, every pass 0 open and 0 violations.

### Net types and routing guidelines (2026-10-19)

An experiment: classify the nets by type and apply the usual PCB routing guidelines, each as an option that is off
by default (without them the output is byte-identical: checked on 4 bench boards against the build before).

**Classification** (`src/nettypes.cpp`, `--nt-dump F` writes it as JSON, alone it just classifies and exits). From the
net name without its sheet path (for KiCad's `Net-(U2-USB_DP)` the pin name), the net class name, and the footprints
of the net's pins (re-read from the DSN placement): **ground** (`GND*`, `*GND`, `VSS`), **power** (`+3V3`, `3.3V`,
`5V_SYS`, `VCC*`, `VBUS`, `BATT`, a class named like `Power`; not `EN_VBAT`, `FB_5V2`, `DOUT_5V`), **differential pair**
(`X_P`/`X_N`, `XP`/`XM`, `X+`/`X-`, `CANH`/`CANL`, both nets present; a pair is **diff** when the names say USB, Ethernet,
PCIe, HDMI, DSI, LVDS, TX/RX, RS-485 or a USB connector is on it, else it keeps its other type, e.g. **analog** for
`INA_INP`/`INA_INN`), **crystal** (a pin on a `Crystal`/`Oscillator`/`Resonator` footprint, or `XIN`/`XOUT`/`XTAL*`/
`OSC_IN`), **high-speed** (`*CLK*` but `SWCLK`, `SCK`, `QSPI*`, `SDIO`, `DDR`, `DQ*`, `RMII`, `ULPI`), **analog**
(`AIN*`, `ADC_IA` but not `ADC_CS`, `VREF`, `SENSE`, `SPK`, an `Audio` sheet), else **signal**. Checked by eye on all 34
bench boards and the stepdrive passes; on the bench's nets with two pads or more: 1677 signal, 125 diff, 99 power,
41 high-speed, 27 analog, 15 crystal, 10 ground. Known misses: an op-amp's `VS+`/`VS-` come out as a signal pair,
`QWIIC_SCK` (I2C) as high-speed.

**Guidelines** (`--nt-*`, CPU searches only: a search under the cost changes below doesn't go to the GPU):

| option | what it does |
|---|---|
| `--nt-hs-first` | diff, high-speed and crystal nets are routed first (then the default order) |
| `--nt-hs-via F --nt-hs-len K` | those nets pay F times the via cost, and give way to congestion K times less: a step's cost factor c becomes 1 + (c - 1) / K, so the others detour instead |
| `--nt-pair W` | a pair's second net is routed right after the first and its steps cost 1 + W away from the first's track (within two track pitches, `--nt-pair-dist`); it waits for the first to be committed, so the result stays the same at any thread count |
| `--nt-plane-via UM` | ground and power nets that have a plane pay UM for a via instead of 1500: each pad drops a via to the pour next to it rather than tracking to another |
| `--nt-power-first` | supplies routed before plain signals |
| `--nt-power-width UM` | supplies without a plane get a class of their own, UM wide (if wider than theirs) |
| `--nt-xtal-zone UM --nt-xtal-via F` | other nets' vias within UM of a crystal net's pads cost F times more |
| `--nt-all` | all of them: hs-via 2, hs-len 2, pair 1, plane-via 200, power-width 400, xtal-zone 1000 |

**Which bench DSNs keep planes.** `make_dsn.py` / `gen_pcb.py` strip outer-layer pours (an outer pour in a DSN makes
pcbroute mark its whole outline as that net's, so other nets can't use that layer there). Of the 34 held-out boards 4
keep inner planes (7828613a: GND, 6V0_SERVO; 803592c0: GND, +3V3; 93dc5feb: GND x3, +5V, +3.3V, +1.1V; e67b446e: GND,
+3V3, +24V); the other 30 have none, so the plane-via guideline only acts on those 4. The stepdrive passes keep inner
GND / N planes and outer pours (P on B.Cu, the phase and shunt nets' local F.Cu pours), but every plane supply there is
already fanned out by `gen_pcb.py` before the pass, so pcbroute routes none of them: the plane-via option changes
nothing on stepdrive. (The first version applied it to any net with a pour, which made the local pours of VA / VB /
VC take more vias on the input pass; it now applies to ground and power only.)

**Results.** `bench/nettypes/` (run.sh / run_half.sh, `bench_all.json`, `metrics_all.json`): the 34 held-out boards and
the stepdrive passes, 8 threads, one run per config (routes are deterministic, so open counts, vias and lengths are
exact; the times are not: the machine ran other sessions' jobs and up to three of these runners at once). Totals from
`benchmark.py`, guideline metrics from `nt_metrics.py` (fast = diff, high-speed and crystal nets; pairs: the 25 pairs
with both nets routed in every config; pad-plane: each SMD pad of a plane supply that the run routes, distance to the
nearest via or through-hole pad of its net):

| config | open | better / worse boards | vias | length m | fast-net vias | fast-net length m | pair skew mm | pair coupled | pad-plane mm (pads > 1 mm) | supply width um | time vs base |
|---|---|---|---|---|---|---|---|---|---|---|---|
| base | 100 | | 2106 | 47.50 | 272 | 5.12 | 3.49 | 0.35 | 2.36 (443) | 271 | 1 |
| hs-first | 99 | 3 / 3 | 2096 | 47.54 | 264 | 5.17 | 3.25 | 0.40 | 2.42 (441) | 270 | 0.99 |
| **hs-via 2, hs-len 2** | **98** | 4 / 2 | **2026** | 47.17 | **143** | **4.93** | 2.57 | 0.46 | 2.26 (439) | 270 | 1.05 |
| pair 1 | 102 | 3 / 2 | 2091 | 47.45 | 261 | 5.14 | 2.51 | **0.61** | 2.46 (448) | 272 | 0.97 |
| plane-via 200 | 107 | 1 / 2 | 2221 | 47.43 | 265 | 5.06 | 3.82 | 0.34 | **1.60 (361)** | 271 | 1.03 |
| power-first | 102 | 5 / 3 | 2088 | 47.70 | 249 | 5.24 | 2.93 | 0.40 | 2.22 (434) | 274 | 1.01 |
| power-width 400 | 114 | 4 / 7 | 2064 | 47.25 | 258 | 5.07 | 4.36 | 0.29 | 2.33 (443) | **428** | 1.11 |
| xtal-zone 1000 | 108 | 1 / 2 | 2078 | 47.26 | 263 | 5.08 | 3.47 | 0.35 | 2.38 (450) | 271 | 0.97 |
| all | 127 | 3 / 8 | 2183 | 47.35 | 152 | 4.94 | 2.58 | 0.66 | 1.55 (340) | 429 | 1.21 |

Violations 0 everywhere. Stepdrive (4 passes): every config 0 open, 0 violations; the passes have no fast nets, so
the hs and pair options change nothing; power-first and power-width change the logic pass (179 and 196 vias for 178,
length +1-1.5 %), `--nt-all` makes 282 vias for 250 and 4.78 m of track for 4.68 m.

What it says:
- **No guideline improves completion.** Open connections move by a few either way per board, the chaos of
  negotiation (93dc5feb alone swings from 51 to 68 open across configs that hardly touch it); hs-first, hs-cost, pair,
  power-first, plane-via and xtal-zone are all within that noise of base (98-108). Two cost connections for real:
  **power-width 400** (+14 open, 7 boards worse, 4 better: wider supplies don't get into fine-pitch pads and take
  room) and **`--nt-all`** (+27 open, 8 worse, 3 better; it includes the power width). Like the learned ordering
  before, routing the "important" nets first (hs-first, power-first) changes nothing measurable.
- **The cost guidelines do what they say, mostly for free.** `--nt-hs-via 2 --nt-hs-len 2` takes the vias on fast nets
  from 272 to 143 (-47 %) and their length 4 % down, with 80 fewer vias in all and completion unchanged (98 open): the
  one option worth turning on when fast nets matter. `--nt-pair 1` lifts the coupled share of a pair (one net's track
  within 1 mm of the other's on the same layer) from 0.35 to 0.61, and skew from 3.5 to 2.5 mm, at no cost; skew is
  noisy (the hs-cost run lowers it as much) and nothing here matches lengths. `--nt-plane-via 200` brings the plane
  supplies' SMD pads from 2.36 to 1.60 mm of their nearest via on average (443 to 361 pads farther than 1 mm) for
  115 more vias (+5 %), completion within noise.
- **Crystal zone: nothing to fix.** Base already leaves only 2 other-net vias within 1 mm of crystal pads on the whole
  bench (crystal nets are tiny and the boards are sparse there); the zone takes it to 1.

So, as with the learned ordering: rule-based ordering does not pay. The per-type costs are guideline knobs, not
completion gains: hs-cost (vias off fast nets) and pair (coupling) are free; plane-via costs 5 % vias; power width
costs completion and stays off.

### Net types again, on the boards' own rules (2026-10-19, second run)

The first run above used `data/bench`, where every board had lost its `.kicad_pro` and routed with KiCad's
default 0.2 / 0.2 mm netclass. This run uses the designers' rules (`data/bench_rules`, see "Last-mile shove"), with
two more fixes to the bench found on the way:

- **Pour-only rule areas were exported as full keepouts.** KiCad's Specctra export writes every rule area that is
  not a wire or via keepout as a `(keepout)` on every layer, so an area that only forbids pours, pads or footprints
  walled off copper the designer routed through (OtterCam's BGA, tiny-engineer, OpenFC-Lite, the Pi5 PCIe switch:
  16 areas on 10 of the 34 boards). `ml/tools/make_dsn.py` now drops them before the export; `data/bench_rules_ko`
  is `bench_rules` exported that way (the other 24 boards are unchanged). On 9 of the 10 boards plain pcbroute goes
  from 13 open connections to 2 (tiny-engineer 7 -> 0, OtterCam 2 -> 0 and 91 -> 6 s). Credit: the PlaceNet rows
  session spotted it in its route renders.
- **An inner pour takes its whole layer.** Not fixed here. Contrary to "Planes are pours" above, `build_static()`
  marks every pour cell as the pour net's own, so no other net routes on a layer under a pour. On 3 of the 4 bench
  boards with inner planes the designer routed signals there (iris-brainstem 508 mm on In2, OpenFC-Lite 343 mm on
  In2 and 52 on In3, kicanvas 183 mm on In2); pcbroute routes nothing on any inner layer of them. A separate session
  is adding `--plane-mode refill`. Until then the plane-via guideline is measured on boards that lost their inner
  routing layers.

Setup: the 34 boards (24 from `bench_rules`, the 10 fixed ones from `bench_rules_ko`), base, each `--nt-*` option
alone, and two combinations, each with and without `--shove 3`. 8 threads, one run each, deterministic: opens, vias,
lengths and the guideline metrics are exact. Times are noisy: three runners shared the machine, and the base runs
went first under a different load, so the ratios below are only rough. Results in
`bench/nettypes2/fixed/` (`bench_all.json`, `metrics_all.json`). Combination `combo` = hs-via 2, hs-len 2, pair 1,
plane-via 200, xtal-zone 1000; `combo2` = the same without the hs costs.

| config | open | open, `--shove 3` | better / worse boards | vias | length m | fast-net vias | fast-net length m | pair skew mm | pair coupled | pad-plane mm (pads > 1 mm) | other vias near crystals | time vs base |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| base | 34 | 28 | | 2214 | 47.73 | 255 | 5.29 | 3.36 | 0.47 | 2.19 (410) | 9 | 1 |
| hs-first | 49 | 39 | 2 / 7 | 2199 | 47.27 | 256 | 5.13 | 2.15 | 0.47 | 2.28 (410) | 9 | 0.90 |
| hs-via 2, hs-len 2 | 42 | 38 | 2 / 3 | 2207 | 47.50 | **154** | 5.09 | 3.04 | 0.57 | 2.08 (404) | 7 | 0.95 |
| pair 1 | 39 | 33 | 1 / 2 | 2197 | 47.56 | 236 | 5.05 | **2.06** | **0.67** | 2.20 (408) | 10 | 0.92 |
| plane-via 200 | 37 | 29 | 0 / 2 | 2389 | 47.74 | 251 | 5.26 | 3.43 | 0.50 | **1.37 (295)** | 9 | 0.91 |
| power-first | 44 | 38 | 1 / 4 | 2203 | 47.21 | 240 | 5.00 | 3.28 | 0.50 | 2.01 (404) | 10 | 0.93 |
| power-width 400 | 46 | 43 | 1 / 9 | 2240 | 47.65 | 253 | 5.19 | 2.09 | 0.47 | 2.17 (409) | 6 | 0.97 |
| xtal-zone 1000 | **32** | **25** | 3 / 2 | 2210 | 47.63 | 251 | 5.28 | 3.35 | 0.47 | 2.14 (413) | **3** | 0.86 |
| combo | 44 | 37 | 2 / 6 | 2325 | 47.31 | 155 | 4.92 | 1.71 | 0.72 | 1.49 (303) | 3 | 1.16 |
| **combo2** | **34** | **29** | 3 / 3 | 2351 | 47.32 | 239 | 5.05 | 1.80 | 0.68 | 1.39 (296) | 8 | 1.13 |

Violations 0 in every run. "better / worse" counts boards against base without shove. The guideline metrics are the
same with `--shove 3` to within a few vias, since the shove only touches the last open connections. Pair skew and
coupling are over the 56 pairs routed in every config. Stepdrive was not rerun: its DSNs come from `gen_pcb.py`, not
from the bench export, and every pass routes with 0 open in every config of the first run, so there is nothing for
the shove to do.

Which first-run conclusions change:
- **"No guideline improves completion"** holds. **"The orderings are neutral"** does not: with fewer connections left
  open, routing the fast nets first costs 15 (11 with the shove; tsuraraGB alone +9), and power first costs 10. Both
  are now clearly harmful.
- **"hs-cost and pair are free"** no longer holds. hs-cost still takes 40 % of the vias off fast nets (255 -> 154),
  but it costs 8 open (10 with the shove), and pair attraction costs 5. Almost all of it is on the two boards
  dense with fast nets: da70dce4, 34 diff nets (Tx / Rx pairs and REFCLK) (+6 and +3), and kicanvas (+4, +3). Making fast
  nets stiffer pushes the others into detours the board doesn't have room for.
- **plane-via** is as before. Pad-to-plane distance drops from 2.19 to 1.37 mm and the pads farther than 1 mm from
  410 to 295, for 175 more vias (+8 %). Completion moves by +3 (+1 with the shove), on the plane boards only.
- **power-width** stays harmful: +12, 9 boards worse.
- **"Crystal zone: nothing to fix"** was an artefact of the false keepouts. On the fixed bench, base puts 9 other-net
  vias within 1 mm of crystal pads, and the zone takes that to 3 with no completion cost (32 open, 25 with the
  shove: the best of all configs, within noise of base).
- **Best combination: `combo2`** (`--nt-pair 1 --nt-plane-via 200 --nt-xtal-zone 1000`). It meets the pair, plane
  and crystal guidelines together (coupling 0.68, skew 1.8 mm, pad-to-plane 1.39 mm) at base's completion (34 open,
  29 with the shove), for 6 % more vias and about 13 % more time. Adding the hs costs (`combo`) buys the 40 % fewer
  fast-net vias for 10 more open connections. Worth it only where the fast nets matter more than finishing the
  route; `--nt-all` (which also widens power) is not.
- **The shove helps every config by about the same amount** (3 to 10 fewer open), and doesn't touch the guideline
  metrics: the two compose.

### Last-mile shove (2026-10-19, experimental: `--shove D`, off by default)

A push-and-shove pass for the connections still open at the very end, after the rescue (`src/shove.inc`, included
into the router; hooks in `route_job`). Without the option the output is byte-identical to the build before it, and
with it the result is the same at any thread count (`-mt 1 / 3 / 8` give the same session: the pass is
single-threaded, in job order).

**What the open connections were.** On the small bench boards every connection left open (27c160 /A14, iris
SPK_N, tsuraraGB /RGB0 and /RGB1) had been through the rescue: "pushed through, 0 open, 10-14 cells over others",
then the cascade failed. 10-14 cells is one trace's footprint across: the rescue's path *crosses* a trace (the
cheapest way through when every overlapped cell costs the same), and no amount of pushing on that layer undoes a
crossing. So the shove's path is searched differently.

**How it works.**
- The open connection is routed with another net's *clearance* crossable at a price (soft search: a cell costs
  (1 + history)(1 + 1e4 x occupancy)), but not its *copper*: the cells where this track would overlap another
  net's track or via cost `--shove-cross` (1000) times more (a core mask built over the window). So the path
  squeezes in next to the others wherever it can, and crosses a trace only where nothing else gets through. This
  path is then frozen.
- Each net it now conflicts with is pushed: its stretch in the way is cut out (`--shove-cut`, 1 mm each side) and
  rerouted inside a small window (`--shove-reach`, 1.5 mm around the cut) with everything else as a hard
  obstacle. That search hugs the frozen copper, so the trace moves aside by the least that clears it: the grid
  version of offsetting the segment and walking the via (a literal offset is one of the paths it can find).
- A net the path crosses is cut the same way, and its local reroute has to get past the frozen path: it dives to
  another layer through two vias under it and comes back (the layer hop). No special code: the hard repair can
  use vias, and the frozen path walls off only its own layer.
- A pushed net that can't get around that way takes the soft route in the same window, is frozen, and pushes the
  nets it now overlaps, down to D levels (at most 64 pushes per attempt).
- If a net could not be pushed, the attempt is retried with the path kept off that net (frozen from the start), up
  to 4 tries.
- The attempt is kept only if the connection is closed, nothing overlaps on the grids (every net near the
  windows), no pushed net lost a connection, and the exact check (`check_routes`, the `--eval` rules) passes on all
  copper near the windows. Otherwise every net it touched is put back. Then the usual straightening and final
  self-check.

Three things the first versions got wrong, all found on these cases:
1. A failed hard repair leaves its cut pieces dangling, and `tidy_paths` trims them back to the pads. The soft
   retry then had nothing left inside its window, so it always failed. It now restarts from the net as it was.
2. The grids are not symmetric. A track's footprint on the via grid is a capsule, a via's on the track grid a
   disc, so a pushed track can stay off a via's footprint and still put that via in conflict. The frozen mask
   therefore also covers the cells that would put a frozen net in conflict, and it applies to hard searches too.
   Conflicts are followed both ways (who covers my copper, whose copper I cover).
3. For the same reason a pushed net's *kept* copper can make a frozen via conflict without being in conflict
   itself. Those cells are cut too (`ShoveCtx::cut`, a hook in `route_job`'s cut).

**Results.** The 34 held-out boards (`ml/bench.py`, 8 threads; the 26 boards with nothing open are untouched:
the pass does nothing when nothing is open). Opens exclude the 17 unreachable pads. 0 violations throughout,
self-check and `--eval`. The machine was shared with two other sessions' benchmarks, so route times vary by
±20 % between identical runs.

| board (layers) | base (rescue) | + shove v1 | + retries, no crossing | crossing 10x | 100x | **1000x (default)** | no crossing, `--rescue 0` |
|---|---|---|---|---|---|---|---|
| OtterCam (2) | 2 | 2 | 2 | 2 | 2 | 2 | 3 |
| goldfish (2) | 13 | 11 | 10 | 11 | 10 | **10** | 10 |
| 27c160 (2) | 1 | 1 | 1 | 0 | 0 | **0** | 1 |
| iris-brainstem (4) | 1 | 1 | 1 | 1 | 1 | 1 | 1 |
| tsuraraGB (2) | 2 | 1 | 1 | 1 | 0 | **0** | 1 |
| OpenFC-Lite (6) | 50 | 47 | 47 | 47 | 47 | **46** | 50 |
| Pi5 PCIe switch (2) | 5 | 4 | 4 | 3 | 3 | **3** | 4 |
| kicanvas (4) | 13 | 13 | 13 | 13 | 13 | 13 | 14 |
| **open connections** | **87** | 80 | 79 | 78 | 76 | **75** | 84 |
| route time, these 8 boards | 1081 s | 1091 s | 964 s | 885 s | 874 s | 869 s | 680 s |
| vias, these 8 | 878 | 890 | 891 | 893 | 901 | 905 | 894 |

The pass itself is cheap: 1-3 s on the small boards, 25 s on OpenFC-Lite (64 attempts, 3 kept; the rescue
before it took 144 s for 9). The time differences between the columns are mostly noise from the shared machine.
In the last column the shove replaces the rescue: 30 % faster on these boards, but 5 connections worse, because
the rescue's whole-net reroutes still win where a different topology is needed (OtterCam, OpenFC-Lite, kicanvas).
The two passes are complementary, so `--shove 3` after the rescue is the setting to use. Letting the path cross
(with the crossed trace hopping layers) gained 4 more connections; the higher the price, the more it squeezes first
and crosses only as a last resort, which was best. All crossing runs: 0 violations by `--eval`, the same session at
1 and 8 threads.

Before and after (`router/crop.py`: board JSON from `--dump-board` plus a session; highlighted net outlined):

![tsuraraGB /RGB0](doc/shove_tsuraraGB_RGB0.png)
tsuraraGB /RGB0: walled in at its connector pin. The F.Cu bus under it steps down one pitch and a B.Cu vertical
moves over; 7 nets pushed, 3 levels deep.

![goldfish /PF6](doc/shove_goldfish_PF6.png)
goldfish /PF6: the F.Cu traces above move up to make room for its via.

![27c160 /A14](doc/shove_27c160_A14.png)
27c160 /A14, between two DIP rows on a 2-layer board, needed a crossing: the B.Cu trace /A10 dives to F.Cu through
two vias under /A14's new path and comes back; 3 nets moved.

![goldfish /PB5](doc/shove_goldfish_PB5.png)
goldfish /PB5: a B.Cu run under the QFN moves up and two F.Cu traces shift, 3 nets pushed.

**What is left.** 75 open, of which OpenFC-Lite 46, goldfish 10 and the Pi5 switch 3 are boards whose designers
routed narrower tracks than their DSN's rules (see above). iris SPK_N sits in a QFN fan-out where every try fails on
a push with no way across. The bench's layer counts are the boards' own (5 of the 8 boards with opens are 2-layer,
where a crossed trace has only one layer to dive to); layers typed `power` are routed like the others. `-v` prints
each attempt and push (window, around / across, why it failed).

### The bench with each board's own rules (2026-10-19)

The completion work above blamed the worst boards on their rules ("routed by their designers with 0.1-0.15 mm tracks
while their DSN says 0.2 / 0.2 mm"). The DSN was wrong, not the designers: `ml/tools/fetch_repos.sh` sparse-cloned only
the `*.kicad_pcb` files, and since KiCad 6 the netclasses (track width, clearance, via) live in the `.kicad_pro`. So
all 34 held-out DSNs carried KiCad's default netclass, 0.2 / 0.2 mm. Fixed:

- `fetch_repos.sh` also keeps `*.kicad_pro` and `*.kicad_dru`; `make_dsn.py` warns when a KiCad 6+ board has no
  project beside it.
- `ml/tools/make_bench_rules.sh` fetches the 34 bench boards' projects (`data/bench_rules/projects/`; the boards
  themselves are byte-identical upstream) and writes `data/bench_rules/*.dsn`. `data/bench` is left as it was (the
  tables above and other experiments use it); `ml/bench.py --bench data/bench_rules` routes the new set.
- 23 of the 34 repos have a project. The rules it brings go both ways: OpenFC-Lite 0.09 / 0.09 mm, goldfish and
  kiisu 0.15 / 0.15, stillpoint 0.15 / 0.12, HIDman 0.25 / 0.152; but tsuraraGB, treble, megadesk 0.25 mm and power
  classes up to 1.5 mm (tiny-engineer).
- `make_dsn.py --drawn-widths` (used by the script): a net drawn narrower than its netclass gets the narrowest width
  drawn on it, never below the board's minimum track width (no minimum without a project). The netclass width is only
  KiCad's default for new tracks; DRC enforces the clearance and the minimum. The PCIe debifurcator's class says
  0.6 mm and its 46 PCIe / JTAG nets are drawn at 0.3 (0.3 is its minimum); the Pi5 switch, which has no project,
  has nets drawn at 0.1-0.15 mm. Clearances stay the netclass's (KiCad's default where there is no project).

22 DSNs change. Every board, 8 threads (`ml/bench.py`; the other 12 boards keep their earlier results):

| | old bench | old bench, `--shove 3` | **own rules** | **own rules, `--shove 3`** |
|---|---|---|---|---|
| open connections | 87 | 75 | **35** | **30** |
| unreachable pads | 17 | 17 | 15 | 15 |
| boards routed clean | 19 | 21 | 19 | 20 |
| violations (self-check) | 0 | 0 | 0 | 0 |
| route time, all boards | 1451 s | 1236 s | 924 s | 916 s |
| vias | 2100 | 2127 | 2220 | 2236 |

| board | old bench (shove) | own rules (shove) | rules |
|---|---|---|---|
| OpenFC-Lite | 50 (46), 500 s | **3 (3), 58 s** | 0.09 / 0.09 mm |
| goldfish | 13 (10) | 4 (3) | 0.15 / 0.15, Power 0.25 |
| Pi5 PCIe switch | 5 (3) | 2 (1) | no project: drawn 0.1-0.15 |
| iris-brainstem | 1 (1) | 0 (0) | 0.2 / 0.15, rails 0.6-0.8 |
| 27c160 | 1 (0) | 0 (0) | no project (KiCad 5 rules in the board), 13 nets drawn narrower |
| tsuraraGB | 2 (0) | **10 (8)** | 0.25 / 0.2, Bold 0.5: stricter than the default |
| amiga2000 gfxcard | 0 (0) | 1 (0) | 6 nets drawn narrower |
| PCIe debifurcator | 0 | 0 (2 pads unreachable) | 0.6 / 0.18 class, PCIe nets drawn at 0.3 |
| OtterCam | 2 (2) | 2 (2) | 0.2 / 0.2 (= default) |
| kicanvas | 13 (13) | 13 (13) | no project, nothing drawn narrower |

So most of what was left open was the rules, as suspected, but the rules were ours. tsuraraGB is the honest
counter-example: its designer routed it at 0.25 mm and pcbroute leaves 8 open there. Use `data/bench_rules` from now on;
`data/bench` stays for comparison with the tables above.

**Default bench: `data/bench_rules_ko`** (until 2026-10-20; now `data/bench_rules_ko3`, see "Baseline on
bench_rules_ko3" below). The same boards and rules, exported after `make_dsn.py` stopped writing rule
areas that forbid neither tracks nor vias (they only keep pours, pads or footprints out) as keepouts on every layer:
16 areas on 10 boards change, and plain pcbroute leaves 2 open on those 10 instead of 13. `ml/bench.py` and
`benchmark.py` route it by default; `tools/make_bench_rules.sh [OUT]` rebuilds it (projects stay in
`data/bench_rules/projects/`).

### Footprint keepouts (2026-10-19)

KiCad's DSN export puts a keepout on every copper layer for each bare (NPTH) hole, and a footprint's own rule areas,
inside the footprint's `(image ...)`. `read_dsn` read only the structure's keepouts, so routing **and `--eval`**
ignored them: tracks and vias crossed mounting holes, switch posts and antenna areas while the self-check said 0
violations. They are now placed with their component (rotation; a back-side part mirrors them to the other layers).
`--no-image-keepouts` restores the old reading for A/B runs: with it the parsed board is identical to the previous
build's on all 34 bench boards, and boards without footprint keepouts route byte-identically.
`tests/test_image_keepouts.py` covers placement, rotation, mirroring, the switch and a route around a hole.

23 of the 34 bench boards and 2 080 of the 3 916 training boards have some. The sessions already on disk, scored
again by the fixed `--eval`:

| sessions | violations before | now | sessions with new violations |
|---|---|---|---|
| `bench/ses`, pcbroute `new` (default rules) | 0 | 360 | 13 of 34 |
| `bench/ses`, Freerouting | 68 | 69 | 1 of 32 |
| `ml/bench/v2`, `base` / `w2g` | 0 / 0 | 340 / 382 | 12 / 13 of 34 |
| `bench/nettypes2` (own rules), `base` | 0 | 373 | 12 of 34 |
| `ml/data/routed` (route_all training data) | 1 594 | 31 836 | 1 017 of 3 796 |

Freerouting already obeyed them, so the comparison with it was unfair to Freerouting. Most of the new violations sit
on keyboards (switch posts and pegs: treble 162, bakeneko-65 126) and the Pi5 PCIe switch (33).

KiCad's DRC agrees. Five boards routed with the old and the fixed build, the sessions imported into the original board
with its project (tracks and copper pours removed), `kicad-cli pcb drc` errors:

| board | old build | fixed build | open (old / fixed) |
|---|---|---|---|
| treble | 72 (hole clearance 70, hole to hole 2) | 0 | 0 / 0 |
| d4b12703 | 11 (items not allowed: a rule area) | 0 | 0 / 0 |
| 232b8b30 | 23 (hole clearance 17, hole to hole 6) | 0 | 10 / 9 |
| bakeneko-65 | 177 (hole clearance 171) | 18: footprints overlapping in the design itself | 0 / 0 |
| Pi5 PCIe switch | 306 (hole clearance 93) | 259: 199 track width (no project), 16 vias inside NPTH pads' own 1.7 mm clearance, J12's own pads | 2 / 1 |

Left over then: a pad's own clearance. KiCad's export writes a hole as the drill plus 0.25 mm all round (the
default hole clearance), whatever the pad or the `.kicad_dru` ask, and pcbroute keeps no clearance from a keepout.

### Hole keepouts with KiCad's hole clearance (2026-10-19)

`make_dsn.py` now grows each image keepout circle that is a hole (an NPTH pad, an unnumbered PTH like the HDMI
shell pegs, a pad on no copper layer) to what KiCad's DRC enforces: the board's hole clearance, or an unconditional /
NPTH `hole_clearance` rule of the `.kicad_dru`, from the hole's edge, and the pad's own clearance (`PAD.GetOwnClearance`:
its override, the footprint's) from its copper. A circle is matched to its pads through every placement of its image,
only to pads whose drill it covers: treble's `CutoutSwitch` markers (B.Adhesive pads, no drill, written as 0.5 mm
circles) sit on the switch holes and must not take their clearance. Every keepout circle of the 34 bench boards
matches a pad. Regenerated into `data/bench_rules_ko2`, only keepout circles change, on two boards:

| board | circles | change |
|---|---|---|
| Pi5 PCIe switch | 10 | the CM4 mounting holes' own 1.7 mm clearance: 3.2 mm circle to 6.1 mm |
| trackball-lt6c | 8 | its `.kicad_dru` asks 0.254 mm from NPTH holes: 1.640 to 1.648 mm |

KiCad DRC, routed with `-mt 8`, sessions imported as before (tracks and copper pours removed, the project beside):

| board | errors (`bench_rules_ko`) | errors (`bench_rules_ko2`) | of which from the design |
|---|---|---|---|
| Pi5 PCIe switch | 259: hole clearance 46 (42 routed tracks and vias at the 1.7 mm holes) | 218: hole clearance 4 | all but 2: 199 track width (no project), J12's own PTH / NPTH pads 4, CSa / Rga pads 1, outline, courtyards, J2 annular ring and edge |
| treble | 0 | same DSN | |
| bakeneko-65 | 18 | same DSN | all: switch footprints overlapping (MX pads on the next switch's NPTH holes, 12 hole clearance + 6 mask bridge) |
| 232b8b30 | 0 (9 open) | same DSN | |

No routed track or via is left inside a hole's clearance. The 2 Pi5 errors from the routing that are not about holes:
a solder mask bridge where rx2+ crosses a B.Mask aperture polygon (the DSN doesn't carry mask openings; 1 with
`bench_rules_ko` too).

`data/bench_rules_ko2` replaces `data/bench_rules_ko` (old set kept as `data/bench_rules_ko_v1`) once the running
baseline on `bench_rules_ko` is done: only Pi5 and lt6c are affected.

### Inner pours (2026-10-19, `--plane-mode refill`, off by default)

The bullet above used to claim that pours were "refilled around other nets' copper". They weren't:
`build_static` stamped every cell of a plane as its net's own, so no other net could route on the layer. On the 4
bench boards with inner pours, pcbroute routed nothing on any inner layer. The designers ran signals there, counted
in pcbnew as the tracks of nets other than the pour's on the pour layers: iris-brainstem In2 508 mm, OpenFC-Lite In2
343 + In3 52 mm, kicanvas In2 183 mm, stillpoint none.

`--plane-mode refill` (`reserve` is the default and the old behaviour):

- **Routing.** A pour is no obstacle. Its cells are still its net's terminals wherever no other copper stands, but
  only 0.5 mm inside the outline: KiCad fills zones with filleted corners (0.5 mm radius on kicanvas) and joins a
  track to a zone by the track's end point. Where a smaller pour of another net overlaps it on the same layer, the
  smaller one wins. (Both are guesses for pours without zone rules. With them, the margin is half the zone's
  minimum width, plus its fillets, and the higher priority wins: see "Zone rules in the DSN".)
- **Islands.** `pour_islands` (router.cpp) refills a pour on a grid of `-s` cells. It keeps the cells inside the
  plane that are at least the clearance plus half of `--pour-min-width` (default 250 um, KiCad's zone minimum
  width) from other nets' copper, from smaller pours of other nets, from keepouts and from the plane's own outline.
  That set is the pour eroded by half its minimum width, so its 4-connected pieces are the real fill's islands,
  with necks narrower than the minimum width cut, as KiCad cuts them. The net's copper joins an island where it
  comes within half the minimum width of it. `--pour-clearance UM` raises the pour's clearance above the classes'
  (a KiCad zone has its own: 0.5 mm on kicanvas). The DSN's zone rules replace both options, and refine the
  islands further (below).
- **`--eval --plane-mode refill`** judges by those islands. The connections to make are counted from the net's
  copper without its pours (iris: 192 instead of 166), because what a pour joins depends on the routes. A pour joins
  nothing by itself, only through its islands refilled around everything else, and an island that touches none of
  its net's copper is no piece. Reserve-mode `--eval` is unchanged.
- **Repair.** The router assumes a pour joins its net anywhere, but routes, and smoothing after them, can cut it.
  After smoothing, `repair_pours` finds the pour nets that the islands leave more open than the router counted,
  and routes them again (up to 3 rounds). In that pass every route so far is fixed copper, their pours are
  dropped, and their copper that touches a common island is merged into one group. Repair tracks aren't smoothed.
  The run then prints `refilled pours: N of M connections open as --eval counts them`, the self-check by the
  islands. The summary line's `unrouted` includes what the repair left open. Its `connections` stays the router's
  count, which still takes a pour as joined.

Where refill puts its signals on those layers (tracks of nets other than the pours', mm) against the designers:

| board | refill | designer |
|---|---|---|
| iris-brainstem | In1 (GND) 134, In2 (6V0_SERVO) 229 | In2 508 |
| stillpoint | In1 (GND) 43, In2 (+3V3 / GND) 56 | none |
| OpenFC-Lite | In1 (GND) 53, In2 (GND) 58, In3 (supplies) 50, In4 (GND) 74 | In2 343, In3 52 |
| kicanvas | In1 (GND) 421, In2 (GND / supplies) 265 | In2 183 |

The designers leave the GND layer next to the signal layer whole (iris In1, kicanvas In1, OpenFC In1 / In4) and run
their signals across the supply or second GND layer. Refill has no such preference: it slices the reference planes.
`--pour-gnd-cost` (below) adds that preference.

**A/B** (`-mt 6`, one run each, eval by `--plane-mode refill`; the 4 pour boards and 5 without, the machine shared
with a full bench run, so times are only rough). "Open" is reserve-eval / refill-eval:

| board | mode | open | violations | vias | length mm | inner-layer mm, all nets (designer, pkl) | route s |
|---|---|---|---|---|---|---|---|
| iris-brainstem | reserve | 0 / 0 | 0 | 99 | 889 | 0 (490) | 27.5 |
| | refill | 0 / 0 | 0 | 90 | 768 | 363 | 9.4 |
| stillpoint | reserve | 0 / 0 | 0 | 114 | 611 | 0 (0) | 12.7 |
| | refill | 0 / 0 | 0 | 111 | 615 | 99 | 7.2 |
| kicanvas | reserve | 13 / 13 | 0 | 240 | 2320 | 0 (572) | 275.0 |
| | refill | 14 / 14 | 0 | 237 | 2225 | 892 | 203.8 |
| OpenFC-Lite | reserve | 3 / 3 | 0 | 214 | 946 | 0 (371) | 26.4 |
| | refill | 1 / 1 | 0 | 199 | 803 | 240 | 15.6 |
| HIDman, PicoWiFiModem, Polaron, treble, PCIe debifurcator | both | 0, 0, 0, 0, 2 | 0 | identical | identical | (debifurcator 1082, no pours) | noise |

- On the boards without pours both modes write byte-identical sessions. So does reserve against the c45082d build,
  on all 9 boards.
- Refill is shorter (-14 % on iris, OpenFC), uses fewer vias and routes faster. It leaves fewer open on OpenFC
  (3 to 1). Kicanvas's 13 vs 14 is the router's usual variance on its congested signal nets: the 14th is a GND pad
  of the RP2040 that the router itself reports unrouted, not an island.
- The repair pass ran once each on OpenFC and kicanvas (1 net, under a second). On kicanvas it closed the cut. On
  OpenFC it left the one open that the router had already counted (+3.3V, which KiCad also reports). Two earlier
  versions were worse. Repairing before smoothing left kicanvas one open caused by islands. Without the overlap rule,
  OpenFC's +3.3V on In3 really had 7 open (KiCad) while `--eval` said 1.

**KiCad DRC.** Each session was imported into the original board with its project. Tracks were removed and every
zone kept and refilled by KiCad (`ZONE_FILLER`), then `kicad-cli pcb drc` (errors / unconnected items):

| board | designer | reserve | refill | unconnected items on pour nets (reserve / refill) |
|---|---|---|---|---|
| iris-brainstem | 0 / 0 | 1 clearance / 23 | **0 / 22** | 0 / 0 |
| OpenFC-Lite | 5 / 0 | 200 (199 hole clearance) / 16 | 200 (the same) / **13** | +3.3V 1 / +3.3V 1 |
| kicanvas (no project) | 198 / 0 | 129 (58 track width, 69 mask bridge) / 30 | 127 / 32 | +3V3 1 / +3V3 1, GND 1 |
| stillpoint | 0 / 0 | 6 / 0 | 12 / 0 | 0 / 0 |

- The other unconnected items are nets the DSN leaves out (outer-pour nets lose their pins in `make_dsn.py`) and
  connections pcbroute reports open itself. The same goes for kicanvas's +3V3 (an unreachable pad) and GND (above).
- Stillpoint's extra 6 errors are `items_not_allowed` from a custom rule in its `.kicad_dru` (`radar_ground_only`:
  an area for GND only). The DSN doesn't carry it. Reserve already broke it with 5 vias; refill also puts inner
  tracks there.
- KiCad agrees with the island check: before the overlap rule, `--eval` counted OpenFC's +3.3V 1 open where KiCad
  found 7, and with it both say 7 for that session.

**Gaps the DSN leaves.** KiCad's export drops zone priority, the zone's own clearance, its minimum width and its
fillet. `make_dsn.py` now carries those (see "Zone rules in the DSN" below). Still missing: thermal reliefs (spoke
count and width), the connect-pads mode, and custom DRC rules (stillpoint's `radar_ground_only`).

- **Plane quality.** Refill lets signals slice reference planes, the designers' GND layers included (above):
  designers keep the GND layer next to the signal layers whole. `--pour-gnd-cost` (below) keeps them off; there is
  no cost for cutting a pour into narrow pieces.

**Keeping the reference planes whole** (`--pour-gnd-cost F`, off by default; refill only). A track centre over a
ground pour (a net `nettypes.cpp` types as ground) of a reference layer costs F times as much per step. The reference
layers are, from each outer layer, the nearest inner layer with a ground pour: iris In1, stillpoint In1 and In2,
OpenFC In1 and In4, kicanvas In1 and In2. Where a smaller pour of another net wins the overlap (kicanvas's +3V3 and
+24V on In2) there is no ground fill and no extra cost. `--pour-gnd-all` puts the cost on every ground pour (OpenFC's
In2 too). Without the option the sessions are byte-identical to bacb10a on the 4 boards.

A/B (`-mt 6`, one run each, `--eval --plane-mode refill`; inner mm: tracks of nets that have no pour on that layer,
the designer's counted the same way from the `.kicad_pcb`):

| board | mode | open | vias | length mm | route s | inner mm per layer (reference layers in bold) |
|---|---|---|---|---|---|---|
| iris-brainstem | designer | | | | | **In1 0**, In2 508 |
| | refill | 0 | 90 | 768 | 9.3 | **In1 134**, In2 229 |
| | x3 | 0 | 92 | 767 | 9.5 | **In1 5**, In2 336 |
| | x10 | 0 | 91 | 773 | 10.2 | **In1 0**, In2 350 |
| stillpoint | designer | | | | | **In1 0**, **In2 0** |
| | refill | 0 | 111 | 615 | 7.3 | **In1 43**, **In2 56** |
| | x3 | 0 | 110 | 614 | 7.0 | **In1 0**, **In2 78** (over +3V3) |
| | x10 | 0 | 112 | 612 | 7.3 | **In1 0**, **In2 75** (over +3V3) |
| OpenFC-Lite | designer | | | | | **In1 0**, In2 343, In3 52, **In4 0** |
| | refill | 1 | 199 | 803 | 15.1 | **In1 53**, In2 59, In3 50, **In4 74** |
| | x3 | 1 | 202 | 835 | 15.4 | **In1 0**, In2 97, In3 116, **In4 0** |
| | x10 | 2 | 192 | 831 | 24.8 | **In1 0**, In2 117, In3 97, **In4 0** |
| | x3 all | 1 | 204 | 850 | 14.5 | **In1 0**, In2 0, In3 185, **In4 0** |
| kicanvas | designer | | | | | **In1 0**, **In2 183** |
| | refill | 14 | 237 | 2225 | 192.1 | **In1 539**, **In2 313** |
| | x3 | 14 | 241 | 2213 | 294.1 | **In1 9**, **In2 56** (over +24V) |
| | x10 | 12 | 237 | 2223 | 173.2 | **In1 0**, **In2 39** (over +24V) |

- The cost does what the designers do: the GND layer next to each outer layer stays whole at x10 on all 4 boards,
  for under 1 % of length on iris, stillpoint and kicanvas (OpenFC +3.5 %, as In1 / In4 carried 127 mm). The
  signals go to the supply layer (iris In2, stillpoint's +3V3 on In2, kicanvas's +24V) or OpenFC's second GND layer.
- Opens: unchanged on iris and stillpoint, kicanvas 14 -> 12, OpenFC 1 -> 2 at x10 (1 at x3). Both within the
  router's run-to-run variance on these boards (kicanvas's signal nets; OpenFC's +3.3V pour); not separated here.
- Kicanvas's designer still runs 183 mm across its GND-mostly In2. With In2 the reference of B.Cu, x10 leaves only
  39 mm there, all over the +24V pour; the outer layers take the rest.
- The "inner mm" here leaves out a net's tracks only on layers where it has a pour, so kicanvas's +3V3 tracks on In1
  count (539 / 313 here, 421 / 265 in the table above).

`tests/test_plane_refill.py` covers reserve against refill, the eval of a cut pour, the repair, byte-identity
without planes, and `--pour-gnd-cost` moving a track off the reference plane. Switching the default to refill wants the full 34-board bench first (only the 4 pour boards can
change).

### Zone rules in the DSN (2026-10-19)

`make_dsn.py` writes what KiCad's Specctra export drops from each pour into the DSN's structure, after the planes:

```
(pcbroute_zones
  (zone GND In1.Cu (priority 0) (clearance 500) (min_width 254) (fillet 2000))
  (zone +3V3 In2.Cu (priority 1) (clearance 500) (min_width 254) (fillet 500)) ...)
```

There is one `(zone)` per `(plane)`, in the export's order (zones in board order, then each zone's copper layers).
Values are in um. `make_dsn.py` checks each one against its plane's net and layer. pcbroute reads the scope and
ignores it, with a warning, if it doesn't match. Freerouting 2.4.1 skips scopes it doesn't know (checked: it
routed stillpoint). Only the 4 inner-pour DSNs of the bench change.

With these rules, `--plane-mode refill` (reserve mode doesn't read them, and is byte-identical on the 4 boards):

- **Priority** decides an overlap of pours of different nets (`pour_wins`). The smaller pour wins only on a tie,
  which KiCad breaks by zone id. The routing terminals give way to the winner by the clearance plus half the
  minimum width.
- **The winner's fill, not its outline, is knocked out.** KiCad fills in priority order, and a lower zone takes
  whatever the higher one's island removal leaves. On OpenFC, a track cuts +1.1V's fill and +3.3V fills in behind
  it, inside +1.1V's outline. `pour_islands` therefore refills the winning pour first, keeps only the islands its
  net's copper touches, and knocks those out with the clearance. It recurses up to 6 levels, then falls back to
  the outline.
- **Per-zone clearance** (the larger of it and the classes'), **minimum width**, and **fillet**. A fillet cuts
  each convex outline corner as KiCad's `Fillet` does, limited to half of each edge beside it, both in the islands
  and in the routing terminals. `--pour-clearance` and `--pour-min-width` now apply only to pours without rules.
- **Pours of one net that touch on a layer** are refilled together (`pour_group`), each by its own rules. Their
  union is then cut into islands, as KiCad joins their fills. Kicanvas has two overlapping +3V3 pours on In2.
- **The erosion is half a cell diagonal short of half the minimum width.** A neck exactly the minimum width wide
  then keeps its cells. Kicanvas's 0.5 mm zone clearance leaves necks like that, and KiCad keeps them. The cost:
  a neck up to a cell diagonal (71 um at `-s 50`) narrower than the minimum width also counts as joined.

**KiCad DRC against `--eval`.** Each session was imported into the original board and every zone refilled by
KiCad (`drc.py` as above). The table shows the unconnected items on pour nets (KiCad) and the open connections
(`--eval --plane-mode refill`). "Guess" means routed and judged without zone rules, from data/bench_rules_ko2.
"Rules" means routed with them. With rules, `--eval` agrees with KiCad on every pour net of all 16 sessions
(these 8, an earlier run of each, and the 1112a0b kicanvas session: GND 1, +3V3 1). Without them, it missed two
of kicanvas's three:

| board | session | KiCad | `--eval`, zone rules | `--eval`, guessed |
|---|---|---|---|---|
| iris-brainstem | guess / rules | 0 / 0 | 0 / 0 | 0 / 0 |
| stillpoint | guess / rules | 0 / 0 | 0 / 0 | 0 / 0 |
| OpenFC-Lite | guess / rules | +3.3V 1 / +3.3V 1 | the same | the same |
| kicanvas | guess | GND 1, +3V3 2 | GND 1, +3V3 2 | GND 1, +3V3 0 |
| kicanvas | rules | +3V3 1 | +3V3 1 | +3V3 1 |

Before the fill knockout, the zone-rules `--eval` gave OpenFC's guess session +3.3V 2, against KiCad's 1. Before
`pour_group` and the shorter erosion, it gave kicanvas's 1112a0b session +3V3 4, against KiCad's 1.

**A/B** (`-mt 8`, one run each, the machine shared with the baseline bench). Open and violations are judged by
`--eval` with the zone rules, "self" is the router's own `refilled pours` check, and lengths are in mm:

| board | mode | open | violations | vias | length | inner layers | self | route s |
|---|---|---|---|---|---|---|---|---|
| iris-brainstem | reserve | 0 | 0 | 99 | 889 | 0 | | 17.7 |
| | refill, guessed | 0 | 0 | 90 | 768 | 363 | 0 | 10.2 |
| | refill, rules | 0 | 0 | 93 | 764 | 361 | 0 | 9.9 |
| stillpoint | reserve | 0 | 0 | 114 | 611 | 0 | | 7.1 |
| | refill, guessed | 0 | 0 | 111 | 615 | 99 | 0 | 7.2 |
| | refill, rules | 0 | 0 | 110 | 612 | 106 | 0 | 8.1 |
| kicanvas | reserve | 13 | 0 | 240 | 2320 | 0 | | 185.9 |
| | refill, guessed | 16 | 0 | 237 | 2222 | 892 | 14 | 194.5 |
| | refill, rules | 13 | 0 | 239 | 2210 | 829 | 13 | 211.6 |
| OpenFC-Lite | reserve | 3 | 0 | 214 | 946 | 0 | | 23.9 |
| | refill, guessed | 1 | 0 | 199 | 798 | 238 | 1 | 14.8 |
| | refill, rules | 1 | 0 | 195 | 810 | 253 | 1 | 14.2 |

- On kicanvas, the guesses (0.2 mm class clearance against the zones' 0.5 mm) let the router believe in pour
  connections that KiCad's fill doesn't make. Its self-check said 14 open where KiCad finds 16. With the rules,
  the self-check, `--eval` and KiCad all say 13, which is reserve's count, with 829 mm on the inner layers.
- OpenFC and iris route the same with or without rules (their zones' clearances are within 20 um of the classes').
  Stillpoint differs only by noise.
- Refill with rules still leaves kicanvas's +3V3 1 open (an unreachable pad, as before) and OpenFC's +3.3V 1.

`tests/test_plane_refill.py` (class `ZoneRules`) covers priority against area, a zone clearance cutting a neck,
mismatched rules, touching same-net pours, and a winner knocked out only by its fill.

### Baseline on bench_rules_ko (2026-10-20)

The reference from now on, replacing every table above: `ml/data/bench_rules_ko` (34 boards, each board's own rules,
rule areas exported right), 16 threads, Ryzen 7 2700. Everything scored by the one yardstick,
`build/pcbroute -de board.dsn --eval board.ses`. `benchmark.py` now records the load average before and after each
run; runs made while other sessions pushed it to 8-22 were repeated with it under 8 (the first base passes took
850-888 s at load 15-22, 665 s at under 7). Results in `bench/base_ko/` (`*_rerun.json` are the timed passes;
`run.sh`, `rerun*.sh` the command lines), made with the builds of `c45082d` (footprint keepouts obeyed by routing
and by `--eval`).

**Patched the same day for the new board set.** The zone-rule and hole-keepout merges (`866ff51`) re-exported
6 boards: trackball-lt6c (5eec2d82) and Pi5 PCIe switch (f7d30ef4) for hole-keepout clearance, and the inner-pour
boards iris-brainstem (7828613a), stillpoint (803592c0), OpenFC-Lite (93dc5feb) and kicanvas (e67b446e) for zone
rules (the old set is `ml/data/bench_rules_ko_v1`). Those 6 were re-run with every config at `866ff51` builds and
spliced into the table below (`bench/patch_ko/`; `splice.py` makes the table, `*_rerun.json` are quiet re-timings,
load under 7). The other 28 route byte-identically at `866ff51` (3 checked). Only the Pi5 switch routes
differently: plain pcbroute and the GPU build 1 -> 0 open, the prior 1 -> 2, Freerouting 51 + 6 viol. -> 57 + 2 viol.;
`--shove 3` and the order stay at 0 with slightly different copper. trackball and the four inner-pour boards route
exactly as before under every config, so the zone rules change nothing pcbroute or Freerouting does on them. Before
the patch: 33 open plain, 30 with `--shove 3`, 555 Freerouting (67 viol.), 35 prior, 30 order.

| | pcbroute | `--shove 3` | GPU build | Freerouting 2.4.1 (`-mp 1`) | prior (`w2gi`) | learned order (`ord`) | `--shove 3` + order |
|---|---|---|---|---|---|---|---|
| runs | 3 | 3 | 3 | 3 | 1 | 1 | 1 |
| open connections | **32** | 30 | 32 | 561 | 36 | 30 | **26** |
| violations (`--eval`) | 0 | 0 | 0 | 63 (amiga2000 61, Pi5 switch 2) | 0 | 0 | 0 |
| boards clean | 28 | 29 | 28 | 7 | 27 | 29 | 29 |
| wall time per pass | 667 s | 689 s | 666 s | 2067 s | 1157 s | 918 s | 993 s |
| time per board vs pcbroute, geo mean | 1 | 1.00 | 1.01 | 3.67 | 2.15 | 1.71 | 1.74 |
| track length | 47.97 m | 48.04 m | 47.97 m | 42.13 m | 48.32 m | 47.68 m | 47.81 m |
| vias | 2301 | 2310 | 2301 | 1578 | 2457 | 2230 | 2236 |

(Patched runs: the 6 re-exported boards got one pass per config plus a timing pass; the times are the 28 boards'
timed passes plus the 6 boards' quiet passes.)

- **Repeatability.** Every config gave the same open count, violations, length and vias on every board in all its
  runs; only times move (±1 % between passes on a quiet machine). So a single pass is enough for completion, and
  the `w2gi` / `ord` columns are exact for this build and thread count; their times are single passes.
- **Open includes unreachable pads.** A verbose base pass (`bench/base_ko/verbose/`) reports 7 terminals with no
  way in (OpenFC-Lite 2, PCIe debifurcator 2, kicanvas 2, and one on 803592c0 that still ends connected), so at least
  6 of the 32 open cannot be routed at the DSN's widths. bench_rules had 15.
- **The GPU build does nothing here.** Its sessions are byte-identical to the CPU build's and its time the same: with
  each board's own rules no search reaches `--gpu-pops` (300 000 pops; checked with `-v` on kicanvas, the slowest
  board). On the old bench it paid on OpenFC-Lite (242 -> 171 s) because 0.2 mm tracks at a 0.09 mm board made its
  searches hopeless; now OpenFC-Lite routes in 25 s.
- **`--shove 3`** closes 2 (amiga2000 1 -> 0, goldfish 4 -> 3) for 3 % more time; the Pi5 switch, which it used to
  close too, now routes in plain pcbroute. tsuraraGB stays at 9 (it went 10 -> 8 on bench_rules).

Per board, open connections where any config leaves some (pcbroute's times on the right; `-` = 0 open; * = re-exported):

| board (layers) | pcbroute | shove | Freerouting | prior | order | shove + order | pcbroute s | Freerouting s |
|---|---|---|---|---|---|---|---|---|
| kicanvas (4) * | 13 | 13 | 37 | 13 | 11 | **10** | 178 | 793 |
| tsuraraGB (2) | 9 | 9 | 59 | **14** | 9 | **8** | 110 | 25 |
| goldfish (2) | 4 | 3 | 20 | 1 | 2 | 2 | 11 | 27 |
| OpenFC-Lite (6) * | 3 | 3 | 12 | 3 | 1 | 1 | 25 | 422 |
| PCIe debifurcator (4) | 2 | 2 | 42 | 2 | **7** | **5** | 42 | 71 |
| amiga2000 gfxcard (2) | 1 | - | 46 + 61 viol. | - | - | - | 48 | 99 |
| Pi5 PCIe switch (2) * | - | - | 57 + 2 viol. | 2 | - | - | 40 | 74 |
| tiny-engineer (2) | - | - | 3 | 1 | - | - | 4 | 8 |
| treble (2) | - | - | 84 | - | - | - | 10 | 188 |
| 18 other boards | - | - | 1-33 each | - | - | - | | |

**What changes against the tables above.**
- **pcbroute vs itself.** 87 open on `data/bench` (KiCad's default netclass), 35 / 30 with `--shove 3` on
  `bench_rules` at 8 threads, now 32 / 30 on `bench_rules_ko` at 16 threads, and this time 0 violations by an
  `--eval` that sees the footprint keepouts (the earlier sessions were 0 by a blind `--eval`, 340-373 by the fixed one).
- **pcbroute vs Freerouting.** On the old bench 194 vs 521 open, 2.4 times faster per board, with Freerouting
  judged against keepouts pcbroute ignored. Both obeying the same keepouts and each board's rules: **32 vs 561 open,
  0 vs 63 violations, 28 vs 7 boards clean, 3.7 times faster per board** (3.1 in total). Freerouting is run as
  `gen_pcb.py` runs it (one pass, no optimizer), so its shorter track and fewer vias go with the connections it leaves.
  It is faster only on tsuraraGB, teensy-eurorack, bakeneko-65, AKOR_01, a keyboard (ad555446) and dc775ffa, all
  boards where it leaves 7-59 open.
- **The prior still does not pay**, now more clearly: 36 open vs 32 (it was 83 vs 87, the best of four configs), and
  1.73 times the time (it was 1.14). It wins goldfish (4 -> 1) and amiga2000 (1 -> 0) but loses tsuraraGB (9 -> 14),
  tiny-engineer (0 -> 1) and the Pi5 switch (0 -> 2). Measured in process (`build-gpu-onnx --ml-model`, weight 2, sat
  0.3, no corridor) against the GPU build, which here is the CPU build. (`build-gpu-onnx` had been rebuilt at
  `ae3fa14`, which adds `--ml-serve` and leaves the router and the in-process prior unchanged; no model server was used.)
- **The learned order is no longer neutral on completion**: 30 open vs 32 (it was 87 vs 87), as good as
  `--shove 3`, better on 4 boards (OpenFC-Lite 3 -> 1, goldfish 4 -> 2, kicanvas 13 -> 11, amiga2000 1 -> 0)
  and worse on one (PCIe debifurcator 2 -> 7). It costs 38 % more time in total (1.71 per board: `mlorder` starts
  Python and the model, ~1.5 s, which dominates the small boards; kicanvas 178 -> 375 s). Order and shove act on
  different boards (only amiga2000 overlaps), hence the next run.

### `--shove 3` + learned order (2026-10-20)

`ml/mlorder ... --budget-scale 1 --shove 3` (mlorder passes `--shove 3` through to pcbroute) on all 34 boards at
`866ff51`, two passes (the timed one at load 3.7-7.5, the first at 5-7.6: 993 s and 1022 s, same copper):
**26 open, 0 violations, 29 boards clean, 993 s per pass, 1.74 per board**, against 30 / 689 s / 1.00 for
`--shove 3` alone and 30 / 918 s / 1.71 for the order alone (32 / 667 s plain).

| board | pcbroute | shove | order | shove + order |
|---|---|---|---|---|
| kicanvas | 13 | 13 | 11 | **10** |
| tsuraraGB | 9 | 9 | 9 | **8** |
| goldfish | 4 | **3** | **2** | 2 |
| OpenFC-Lite | 3 | 3 | **1** | 1 |
| PCIe debifurcator | 2 | 2 | 7 | 5 |
| amiga2000 gfxcard | 1 | **0** | **0** | 0 |
| total | 32 | 30 | 30 | **26** |

- **The two add up, and a little more.** The order's gains are all kept (OpenFC-Lite, goldfish, kicanvas,
  amiga2000), and shove with the order closes a connection that neither does alone on kicanvas (11 -> 10) and on
  tsuraraGB (9 -> 8, the board nothing had moved on bench_rules_ko). Shove's own goldfish gain (4 -> 3) is inside
  the order's (2).
- **The debifurcator is the one loss left**: 5 open against 2 plain, better than the order alone (7). Still 29
  boards clean, the same as either alone: the gains are on boards that stay open (kicanvas, tsuraraGB).
- **Time is the order's.** 8 % more than the order alone, 44 % more than `--shove 3` alone, per board 1.74 vs 1.71.
  Shove adds little on most boards. The debifurcator (80 -> 101 s against the order) and tsuraraGB (76 -> 82 s) cost
  more, and kicanvas goes 375 -> 418 s (178 s plain).
- Track 47.81 m and 2236 vias, between the order's 47.68 m / 2230 and shove's 48.04 m / 2310.

### Baseline on bench_rules_ko3 (2026-10-20)

**The default bench from now on: `ml/data/bench_rules_ko3`**, the same 34 boards from `tools/make_bench_rules.sh`
after two `make_dsn.py` export fixes (`tests/test_dsn_export.py`):

- **Inner planes of outer + inner pours.** A zone on F.Cu and inner layers was deleted whole, so its inner planes
  vanished. It now keeps its inner layers, as an inner-only pour. On the bench only the PCIe debifurcator (GND on
  F/In1/In2/B) changes: it gets `(plane GND In1.Cu)` and `(plane GND In2.Cu)`. 107 of the 634 multi-layer training
  boards in `data/routed` were exported without those planes.
- **The board's copper-to-edge clearance.** The DSN now carries `(pcbroute_rules (edge_clearance UM))` and pcbroute
  takes it as the `--edge` default, for routing and `--eval` (`--edge` still overrides it). Its source: the
  `.kicad_pro`'s `min_copper_edge_clearance`, or the last unconditional `edge_clearance` rule of the `.kicad_dru`
  (KiCad 6+); a KiCad 5.99 board's setup; a KiCad 5.1 board's Default netclass clearance (5.1 has no edge setting).
  A KiCad 6+ board without its project gets none (kicanvas, Pi5 switch: 500 as before). 32 boards get one, from 0
  (HIDman) and 25 (tsuraraGB, OtterCam, EL-210-MOD) to 500; 23 are below the old 500.

Otherwise the DSNs are identical to `bench_rules_ko` (kept as it is). One pass each (every config repeats exactly),
16 threads, build at `21aebcf`, load 4-5; results in `bench/ko3/` (`run.sh`, `compare.py`).

| | pcbroute | `--shove 3` | learned order | `--shove 3` + order |
|---|---|---|---|---|
| open connections (bench_rules_ko) | 32 | 30 | 30 | 26 |
| **open connections (bench_rules_ko3)** | **38** | **35** | **37** | **31** |
| violations (`--eval`) | 0 | 0 | 0 | 0 |
| boards clean | 28 | 29 | 28 | 28 |
| wall time per pass | 754 s | 778 s | 975 s | 1110 s |
| track length | 47.41 m | 47.52 m | 47.64 m | 47.83 m |
| vias | 2239 | 2254 | 2223 | 2249 |

Per board, where any config leaves some open (bench_rules_ko -> bench_rules_ko3; edge in um):

| board (edge) | pcbroute | shove | order | shove + order |
|---|---|---|---|---|
| kicanvas (-) | 13 | 13 | 11 | 10 |
| tsuraraGB (25) | 9 -> 14 | 9 -> 14 | 9 -> 13 | 8 -> 13 |
| goldfish (200) | 4 | 3 | 2 -> 7 | 2 -> 4 |
| PCIe debifurcator (250, planes) | 2 -> 4 | 2 -> 3 | 7 -> 3 | 5 -> 2 |
| OpenFC-Lite (200) | 3 -> 2 | 3 -> 2 | 1 -> 2 | 1 |
| amiga2000 gfxcard (80) | 1 | - | - | - |
| bakeneko-65 (200) | - | - | 0 -> 1 | 0 -> 1 |
| total | 32 -> 38 | 30 -> 35 | 30 -> 37 | 26 -> 31 |

- **The numbers are worse because the bench is more honest.** Each board is now judged at its own edge rule, and
  the debifurcator's signals must stay off its GND planes, on F/B as the designer routed them.
- **tsuraraGB is noise, not the edge.** 5 of the 6 extra open plain. On the `bench_rules_ko` DSN, `--edge`
  500 / 400 / 300 / 200 / 100 / 25 leaves 9 / 13 / 10 / 14 / 14 / 14 open: no trend, a congested 2-layer board
  whose negotiation reshuffles on any change. A single pass there can't tell a better router from a worse one;
  read its row apart.
- **The debifurcator's 2 "no way in" GND pads are gone** (edge fingers 0.3 mm from the key notch, unreachable at
  0.5 mm). With its planes it leaves 4 plain, but the order now helps it (7 -> 3, 5 -> 2 with shove), where on
  `bench_rules_ko` it was the order's one loss.
- **OpenFC-Lite's 2 "no way in" pads stay** at its 0.2 mm edge: their cause is something else.
- **The learned order loses its edge on goldfish** (2 -> 7) and bakeneko-65 (0 -> 1): it was trained on DSNs
  exported at 0.5 mm with no outer + inner planes (likely, not tested).
- Freerouting, the GPU build and the prior were not re-run on bench_rules_ko3.

## Known gaps

Neck-down only on escape stubs out of fine-pitch pads: elsewhere pcbroute uses the class width, so a pour boxed
in for a 1.2 mm track stays open (and `patch_route` closes it). Hole-to-hole spacing between a net's
own vias assumes the drill is half the via diameter (DSN padstacks don't carry drills). Back-side components are
read but haven't been tested. Pads get escape stubs only when no free cell centre is inside them.
