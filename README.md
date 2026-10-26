# pcbroute

A fast, parallel grid autorouter for KiCad boards. It reads the Specctra DSN that KiCad exports and writes a Specctra
session (SES) that KiCad imports, so it can stand in for Freerouting's batch mode.

This is the router from the video *"I Autorouted 3,916 Open-Source Circuit Boards"*: 3,916 boards from 681
open-source KiCad projects, traces removed and routed again from scratch. 77 % of the boards came out fully routed, and
92.9 % of all ~470,000 connections were routed.

- **Negotiated congestion (PathFinder)** on per-net-class clearance grids (50 µm by default), then rip-up and reroute,
  then a rescue pass that is kept only if it leaves fewer connections open.
- **Same result at any thread count.** Nets are routed in parallel on speculation and committed in a fixed order, so
  `-mt 1` and `-mt 16` give byte-identical sessions.
- **Fine-pitch escapes** off the grid, necking down where the class width can't get out between pads.
- **Planes**: reserved as their net's pour (default), or treated like refilled KiCad zones (`--plane-mode refill`).
- **Straightened output**: grid staircases redrawn as the fewest 0/45/90° segments.
- **Self-checked**: the result is checked against the exact geometry (clearances by class, keepouts, board edge)
  before it is written. Exit code 2 if anything is violated.
- **Optional CUDA back end**: long searches are finished on the GPU.
- **Optional learned prior**: a small CNN (RouteNet, `models/routenet.onnx`) suggests where each net's copper should
  run.
- **`--eval`**: scores any router's session against the board's own rules: open connections, violations, length, vias.

This repository is the router as it ran in the video, plus two bug fixes:

- **GPU builds now use the GPU.** A shadowed variable kept every search on the CPU, so the GPU build in the video
  routed exactly like the CPU build. Fixed, with a test (`tests/test_gpu_gate.py`).
- **`--plane-mode refill` counts open connections correctly.** The pour repair counted dead connections more than once,
  so some boards reported more open connections than they had connections. The routes themselves are unchanged.

The video's settings were:

```bash
build/pcbroute -de board.dsn -do board.ses --plane-mode refill --layer-dir yx --time-limit 240
```

## Build

C++17, no dependencies:

```bash
make                  # -> build/pcbroute
```

`-march=native` is the default. To build a binary for other machines, override the flags:
`make CXXFLAGS="-O3 -std=c++17"`.

GPU build (needs `nvcc`; `CUDA_ARCH` defaults to `sm_86`, the RTX 30 series):

```bash
make GPU=1 BD=build-gpu CUDA_ARCH=sm_86
```

## Use with KiCad

1. In the PCB editor: **File → Export → Specctra DSN…** → `board.dsn`
2. Route:
   ```bash
   build/pcbroute -de board.dsn -do board.ses
   ```
3. Back in KiCad: **File → Import → Specctra Session…** → `board.ses`

There is also a scripted export, `tools/make_dsn.py` (it runs with KiCad's Python and `pcbnew`). It strips the existing
tracks and vias, drops outer-layer pours, and adds the board's edge clearance and zone rules to the DSN:

```bash
python3 -I tools/make_dsn.py board.kicad_pcb board.dsn
```

## Common options

```bash
build/pcbroute -de board.dsn -do board.ses -mt 8            # threads (default: all cores)
build/pcbroute -de board.dsn -do board.ses --time-limit 240  # stop with a legal partial result instead of running on
build/pcbroute -de board.dsn -do board.ses --plane-mode refill   # let other nets route through inner pours
build/pcbroute -de board.dsn -do board.ses -v                # per-round progress and timing
build/pcbroute -de board.dsn --eval other_router.ses         # judge any session
build/pcbroute -h                                            # everything else
```

Freerouting's command-line options (`-mp`, `--gui.*`, `--router.*`) are accepted and ignored, so tools that call
Freerouting can call pcbroute instead. One summary line goes to stderr:

```
pcbroute: 38 nets, 53 connections, 13 unrouted, 13 rounds, 0 violations, setup 0.05 s, route 23.40 s, total 23.47 s
```

`--events FILE` streams live progress as JSON lines (rounds, per-net copper as it is committed). That is how the video's
timelapse was drawn.

## Learned prior (optional)

`make ONNXRT=1 BD=build-onnx` (add `GPU=1` for CUDA) builds in ONNX Runtime so that pcbroute can run RouteNet
itself:

```bash
python3 -m venv venv && venv/bin/pip install onnxruntime-gpu==1.31.0
make ONNXRT=1 BD=build-onnx
build-onnx/pcbroute -de board.dsn -do board.ses --ml-model models/routenet.onnx --ml-device cpu
```

The Makefile takes the library from `./venv` (`ORT_SITE` / `ORT_LIB` override it) and the headers from
`third_party/`. The model was trained on human-routed open-source boards. It is an experiment: on the benchmark it
changed completion by a few connections either way and cost time on the CPU. Plain pcbroute is the default for that
reason.

`--order FILE` routes nets in a given order (one net name per line, hardest first). The video's training run used it
to collect data for a net-difficulty model.

## Tests

```bash
python3 -I tests/test_plane_refill.py
python3 -I tests/test_image_keepouts.py
python3 -I tests/test_dsn_export.py                       # parts need KiCad's pcbnew module
PCBROUTE_GPU=build-gpu/pcbroute python3 -I tests/test_gpu_gate.py   # skipped without a GPU build
```

## How it works

[docs/DEVLOG.md](docs/DEVLOG.md) is the full development log. It covers the algorithms, every option, the benchmarks
against Freerouting, and what did and didn't work.

## Known gaps

Neck-down applies only on escape stubs out of fine-pitch pads. Hole-to-hole spacing between a net's own vias assumes the
drill is half the via diameter (DSN padstacks don't carry drills). Back-side components are read but not well tested.
Negotiation is chaotic: a small change to the board or a parameter can move a few connections either way.

## License

MIT, see [LICENSE](LICENSE). The ONNX Runtime headers in `third_party/` are © Microsoft, MIT-licensed
(see `third_party/onnxruntime-1.31.0/LICENSE`).
