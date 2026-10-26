// Grid router: negotiated congestion (PathFinder) over per-class clearance grids, nets routed in parallel batches.
#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "dsn.h"

struct MlPrior;

struct Options {
    double step = 50;            // grid pitch, um
    double window = 8000;        // search window margin around a net's terminals, um
    double via_cost = 1500;      // a via costs as much as this much track, um
    size_t max_pops = 3000000;   // search budget per connection and round (doubles while a net stays stuck)
    int full_every = 0;          // a net still in conflict after this many rounds is rerouted whole (0: never)
    size_t probe_pops = 100000;  // pops of the probe from the pieces towards the tree before the full search from the tree (0: none); unbounded when the tree is huge or the pieces many
    double pres_growth = 1.6;    // the pressure on overlaps grows by this factor each round...
    double pres_max = 0;         // ...up to this (0: no limit)
    double hist_inc = 0.5;       // history cost added to a cell per round it is in conflict
    double wrong_way = 1.25;     // track cost factor against a layer's preferred direction (F.Cu: x, B.Cu: y); 1: off
    bool f_cu_y = false;         // --layer-dir yx: F.Cu prefers y (up-down), B.Cu x (left-right)
    double hweight = 1.0;        // A* heuristic weight (1: shortest paths; more: faster searches)
    double edge_clearance = 500; // copper to board edge, um
    bool wide_margin = false;    // the old grid margin (half a diagonal step on every clearance) instead of the exact sagitta
    // DSN planes (pours): false (reserve) keeps a pour's whole area for its net, no other net routes there; true
    // (refill) lets other nets through with their clearance, like a refilled KiCad zone, and a pour then joins its
    // net's copper only through its refilled islands (--eval and the self-check)
    bool plane_refill = false;
    double pour_min_width = 250; // --plane-mode refill: a pour's narrowest neck (KiCad's zone minimum width), um
    double pour_clearance = 0;   // ...and its clearance to other nets' copper when larger than theirs (a KiCad zone's own), um
    // ...and the step cost factor of a track over a ground pour (0: off): over the reference planes, the ground pours
    // of the inner layer with one nearest each outer layer, or over every ground pour (pour_gnd_all)
    double pour_gnd_cost = 0;
    bool pour_gnd_all = false;
    double neck_min = 100;       // narrowest neck-down into a fine-pitch pad the class width can't reach, um (never below half the class width; 0: half the class width)
    int threads = 0;             // 0: hardware concurrency, capped by the memory the search windows may take
    double mem_gb = 0;           // that memory budget in GB (0: 45 % of what is free when routing starts)
    int iterations = 40;         // negotiation rounds before legalizing
    int patience = 6;            // ...or after this many rounds without a 10 % improvement
    int ripups = 3;              // when legalizing, times a net may push through others
    int rescue = 3;              // rounds of the rescue pass for nets still open (0: off)
    double rescue_seconds = 0;   // ...and its time limit (0: half the routing time before it, at least 10 s)
    double deadline = 0;         // --time-limit: steady-clock time (as now() counts it) the routing should be done by (0: none)
    int shove = 0;               // last-mile shove for connections still open at the end: levels of nets pushed aside (0: off)
    double shove_cut = 1000;     // ...a pushed net is cut this far (um) each side of where it is in the way, and rerouted...
    double shove_reach = 1500;   // ...within this margin (um) around the cut
    double shove_cross = 1000;   // ...a soft path may cross another net's track at this many times the price of squeezing into its clearance; the crossed net then hops layers locally (0: never)
    int shove_rounds = 3;        // passes over the nets still open (stops when a pass gains nothing)
    double gcell = 0;            // global routing cell, um (0: no global routing, search the whole window)
    int gmargin = 1;             // global-route corridor: cells added around the global path
    size_t gpu_pops = 300000;    // GPU builds: a search that runs past this many pops is finished on the GPU (0: never)
    int gpu_slots = 4;           // searches solved on the GPU at once (each with its own buffers and stream)
    double gpu_mem_gb = 4;       // GPU memory one search may take (bigger windows stay on the CPU)
    bool gpu_wait = true;        // wait for a free GPU slot (the result then doesn't depend on timing); false: carry on on the CPU meanwhile
    std::string ml_prior;        // learned routing prior (mlroute.py): per-net maps of where the net's copper is likely to run
    std::function<const MlPrior&()> ml_prior_get;  // ...or the prior predicted in process (--ml-model, src/mlinfer.cpp), waited for at the end of the setup
    double ml_weight = 1.0;      // a step costs up to 1 + this where the prior says the net won't run (0: no cost bias)
    double ml_sat = 0.3;         // a prior at or above this is full confidence: no extra cost (keeps the A* heuristic tight)
    double ml_corridor = 0;      // search only where the prior is at least this (0: off); falls back to the window if stuck
    std::string ml_congestion;   // predicted congestion map (one map named "*", MLP1 file): seeds the history costs
    double ml_cong_weight = 1.0; // ...a fully predicted hotspot starts with this much history (1 = like two rounds in conflict)
    std::string order;           // net order file (ml/mlorder): "name<TAB>difficulty" per line, hardest first; the jobs are routed in that order
    double budget_scale = 1.0;   // a net of difficulty d (0-1, from the order file) gets 1 + (this - 1) d times the search budget and window from round 1 (1: off)
    // net types (nettypes.h) and the routing guidelines they drive; all off by default (output byte-identical)
    std::string nt_dsn;          // the DSN to classify the nets from (set by main when any --nt-* option is on)
    bool nt_hs_first = false;    // route differential, high-speed and crystal nets first...
    bool nt_power_first = false; // ...then power and ground, before plain signals
    double nt_hs_via = 1;        // via cost factor for differential, high-speed and crystal nets
    double nt_hs_len = 1;        // ...and how much less they give way to congestion: the step cost factor c becomes 1 + (c - 1) / this
    double nt_pair = 0;          // differential pairs: the second net's steps cost 1 + this away from the first net's track (0: off)
    double nt_pair_dist = 0;     // ...within this of it (um; 0: two track pitches of the class)
    double nt_plane_via = -1;    // via cost (um) of a ground or power net that has a plane: a via down to the pour next to the pad (-1: off)
    double nt_power_width = 0;   // track width (um) of power and ground nets without a plane, if wider than their class (0: off)
    double nt_xtal_zone = 0;     // other nets' vias within this of a crystal net's pads (um)...
    double nt_xtal_via = 4;      // ...cost this many times more (0 zone: off)
    bool disjoint_batches = true;
    int spec = 2;                // nets routed on speculation beyond a wave's non-overlapping group (see Router::run)
    bool strict = false;         // (kept for the command line; nets are always committed in order now)
    bool legalize_par = true;    // legalize in parallel passes (false: one net at a time)
    int verbose = 1;
    std::string only;            // route just this net (debugging)
    std::string dump_hist;       // write the final history (congestion) map, pooled to 0.25 mm, to this file (training data)
    std::string dump_nets;       // write per-net statistics (rounds in conflict, pops, time, length, vias, open) as JSON to this file
    double dump_x = 0, dump_y = 0, dump_r = 0;  // print the static grid around this point, mm (debugging)
};

struct Seg {
    int layer;
    P a, b;
    double width;
};
struct ViaOut {
    P at;
    int type;
};
struct NetResult {
    int net;
    std::vector<Seg> segs;
    std::vector<ViaOut> vias;
    int open = 0;  // connections still missing
};

struct RouteStats {
    int nets = 0, connections = 0, unrouted = 0, iterations = 0, violations = 0;
    double seconds_setup = 0, seconds_route = 0;
    std::string limited;  // what --time-limit cut short (empty: nothing)
    std::vector<int> dead;  // per net, the connections to terminals no route can reach (in unrouted)
    std::vector<int> conns;  // per net, its connections (in connections)
};

std::vector<NetResult> route_board(const Board& B, const Options& o, RouteStats& stats);
int repair_pours(const Board& B, const Options& o, std::vector<NetResult>& R, RouteStats& stats);  // --plane-mode refill
int smooth_routes(const Board& B, const Options& o, std::vector<NetResult>& R);
int check_routes(const Board& B, const Options& o, const std::vector<NetResult>& R, int verbose);
void dump_board(const Board& B, const std::string& path);  // the parsed board as JSON (for mlroute.py)
int eval_session(Board B, const Options& o, const std::string& ses, int verbose);  // judge any router's session
// --plane-mode refill: the islands of plane (a pour), with the pours of its net it touches on its layer (pour_group),
// refilled around `other` (copper of other nets): per island, the indices into `own` (its net's copper) of the items
// touching it (src/router.cpp)
std::vector<const Item*> pour_group(const Board& B, const Item& plane);
std::vector<std::vector<int>> pour_islands(const Board& B, const Options& o, const Item& plane, const std::vector<const Item*>& own,
                                           const std::vector<const Item*>& other);
// connections still open with the copper `extra` (per net) added to the board's, as --eval counts them, and per net
// (src/eval.cpp)
int open_connections(const Board& B, const Options& o, const std::vector<std::vector<Item>>& extra, int verbose, int* nets = nullptr,
                     int* connections = nullptr, std::vector<int>* per_net = nullptr);
std::vector<std::vector<Item>> result_items(const Board& B, const std::vector<NetResult>& R);  // a route's copper as items
void write_ses(const Board& B, const std::vector<NetResult>& R, const std::string& path);
