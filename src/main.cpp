// pcbroute: a parallel grid autorouter for Specctra DSN files (KiCad export), writing a Specctra session (SES).
// Command line compatible with Freerouting's batch mode: pcbroute -de board.dsn -do board.ses [-mt threads]
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "dsn.h"
#include "events.h"
#include "mlinfer.h"
#include "nettypes.h"
#include "router.h"

static void usage() {
    std::fprintf(stderr,
                 "usage: pcbroute -de IN.dsn -do OUT.ses [options]\n"
                 "  -mt N            threads (default: all cores, fewer when the search windows would not fit --mem)\n"
                 "  --mem GB         memory the threads' search windows may take (default: 45 %% of what is free)\n"
                 "  -s UM            grid pitch in micrometres (default 50)\n"
                 "  -it N            negotiation rounds before legalizing (default 40)\n"
                 "  --patience N     stop negotiating after N rounds without a 10 %% improvement (default 6)\n"
                 "  --ripups N       times a net may push through others when legalizing (default 3)\n"
                 "  --rescue N       rounds of the rescue pass: a net still open pushes through, the others reroute, kept only\n"
                 "                   if fewer connections end up open (default 3; 0: off); --rescue-time S caps it (default: half\n"
                 "                   the routing time before it, at least 10 s)\n"
                 "  --time-limit S   finish within about S seconds from the start: negotiation stops early (at half the\n"
                 "                   time), rip-ups at 80 %%, what still overlaps at S is dropped; the routes stay legal and the\n"
                 "                   session, history and net dumps are written as usual (default: no limit)\n"
                 "  --shove D        last-mile shove (off by default): a connection still open at the end squeezes into the\n"
                 "                   others' clearance, the nets in its way are pushed aside locally, pushing others in turn, up to D\n"
                 "                   levels deep (3 is a good value); kept only if it closes the connection and stays legal.\n"
                 "                   --shove-cut UM (default 1000) and --shove-reach UM (1500) size the local repair\n"
                 "                   --shove-cross F: a path may cross a track at F times the price of a squeeze, the crossed\n"
                 "                   net then hops layers locally (default 1000; 0: never cross)\n"
                 "  --via-cost UM    cost of a via in micrometres of track (default 1500)\n"
                 "  --probe N        pops of the probe from the far pieces before the search from the net's tree (default 100000)\n"
                 "  --pres-growth F  growth of the overlap pressure per round (default 1.6), --pres-max P its cap (0: none)\n"
                 "  --hist-inc H     history cost added per conflicting round (default 0.5)\n"
                 "  --full-every N   reroute a net whole after N rounds in conflict (default 0: repair locally only)\n"
                 "  --wrong-way F    track cost factor against a layer's preferred direction, F.Cu x / B.Cu y (default 1.25; 1 = off)\n"
                 "  --layer-dir D    preferred directions of F.Cu and B.Cu: xy (default: F.Cu left-right, B.Cu up-down) or yx\n"
                 "  --edge UM        copper to board edge clearance (default: the DSN's (pcbroute_rules (edge_clearance UM)), else 500)\n"
                 "  --neck-min UM    narrowest neck-down track into a fine-pitch pad the class width can't reach (default 100,\n"
                 "                   never below half the class width)\n"
                 "  --wide-margin    keep half a diagonal grid step on every clearance (the old builds; tighter: the exact sagitta)\n"
                 "  --plane-mode M   DSN planes (pours): reserve (default: the whole pour is its net's, no other net routes there)\n"
                 "                   or refill (other nets route through with their clearance, like a refilled KiCad zone; the\n"
                 "                   pour then connects its net only through its islands that touch the net's copper)\n"
                 "                   (where two overlap, the higher zone priority wins, from the DSN's (pcbroute_zones) that\n"
                 "                   make_dsn.py writes, else the smaller pour; those also give each zone's own clearance,\n"
                 "                   minimum width and corner fillet)\n"
                 "  --pour-min-width UM  refill, pours without zone rules: KiCad's zone minimum width (default 250)\n"
                 "  --pour-clearance UM  refill, pours without zone rules: their clearance to other nets when larger than theirs\n"
                 "  --pour-gnd-cost F    refill: a track over a reference plane costs F times as much (off; the ground pours of\n"
                 "                   the inner layer with one nearest each outer layer, which designers keep whole)\n"
                 "  --pour-gnd-all   refill: ...over every ground pour instead\n"
                 "  --window UM      search window margin around a net (default 8000)\n"

                 "  --gpu-pops N     GPU builds: a search past N pops is finished on the GPU (default 300000; 0: never)\n"
                 "  --gpu-slots N    searches solved on the GPU at once (default 4); --gpu-mem GB per search (default 4)\n"
                 "  --gpu-nowait     carry on on the CPU while every GPU slot is busy (faster, but the result then depends on timing)\n"
                 "  --no-check       skip the exact clearance check of the result\n"
                 "  --no-image-keepouts  ignore the keepouts inside footprint images (bare holes, rule areas), as\n"
                 "                   pcbroute did before 2026-10-19 (for A/B runs)\n"
                 "  --no-smooth      keep the raw grid paths (no straightening)\n"
                 "  --ml-prior FILE  learned routing prior from mlroute.py (per-net maps)\n"
                 "  --ml-model FILE  predict the prior in process with RouteNet (models/routenet.onnx;\n"
                 "                   builds with make ONNXRT=1): the prior mlroute.py would write, without Python\n"
                 "  --ml-server SOCK run the --ml-model batches on the resident model at the unix socket SOCK (pcbroute --ml-serve;\n"
                 "                   env PCBROUTE_ML_SERVER), in process when none answers for the same model and options;\n"
                 "                   --ml-server-spawn then starts one, detached, for the next runs\n"
                 "  --ml-serve SOCK  with --ml-model: keep the model loaded and serve it on SOCK until --ml-idle S seconds pass\n"
                 "                   without a request (default 1800); the GPU memory is given back after --ml-trim S idle (30)\n"
                 "  --ml-device D    auto (CUDA when it loads, else the CPU), cuda or cpu (default auto)\n"
                 "  --ml-prior-out F write the predicted prior to F (an MLP1 file for --ml-prior); without -do, stop there\n"
                 "  --ml-union-share S, --ml-grow PX  as mlroute.py's --union-share (0.5) and --grow (2)\n"
                 "  --ml-weight W    cost factor up to 1+W off the prior's predicted path (default 1)\n"
                 "  --ml-sat S       a prior of S or more (0-1) costs nothing extra (default 0.3)\n"
                 "  --ml-corridor T  search only where the prior is at least T (0-1, default 0: off)\n"
                 "  --ml-congestion F  predicted congestion map (MLP1 file, map \"*\"): seeds the history costs with it,\n"
                 "                   --ml-cong-weight W per fully predicted hotspot (default 1)\n"
                 "  --order FILE     route the nets in this order (ml/mlorder: one net name per line, hardest first, optionally\n"
                 "                   TAB and a difficulty 0-1); nets not listed come after, in the default order\n"
                 "  --budget-scale S a net of difficulty d gets 1 + (S-1) d times the search budget and window from round 1 (default 1: off)\n"
                 "Net types (from names, classes, footprints) and routing guidelines, all off by default:\n"
                 "  --nt-dump F      write the net classification as JSON to F (alone: and exit)\n"
                 "  --nt-hs-first    route differential, high-speed and crystal nets first; --nt-power-first: then power and ground\n"
                 "  --nt-hs-via F    via cost factor for those fast nets (default 1); --nt-hs-len K: they give way to congestion K times less\n"
                 "  --nt-pair W      a pair's second net costs 1+W per step away from the first's track (--nt-pair-dist UM, default 2 pitches)\n"
                 "  --nt-plane-via UM  via cost of ground / power nets with a plane: a via to the pour next to the pad (default -1: off)\n"
                 "  --nt-power-width UM  width of power and ground nets without a plane, when wider than their class\n"
                 "  --nt-xtal-zone UM  other nets' vias within UM of a crystal net's pads cost --nt-xtal-via F times more (default 4)\n"
                 "  --nt-all         the set above with --nt-hs-via 2 --nt-hs-len 2 --nt-pair 1 --nt-plane-via 200 --nt-power-width 400\n"
                 "                   --nt-xtal-zone 1000 (later options override)\n"
                 "  --dump-board F   write the parsed board as JSON to F and exit (for mlroute.py)\n"
                 "  --dump-hist F    write the final history (congestion) map to F, pooled to 0.25 mm (ml/tools/route_all.py)\n"
                 "  --dump-nets F    write per-net statistics (rounds in conflict, pops, time, length, vias, open) as JSON to F\n"
                 "  --events FILE    write live routing events (JSON lines: begin, pass, net copper, progress, end) to FILE,\n"
                 "                   for Product Studio's Router tab (protocol v1, its docs/ROUTE_EVENTS.md)\n"
                 "  --eval SES       judge a session file (from any router) against IN.dsn: open connections, violations, length, vias\n"
                 "  -q               quiet\n"
                 "Freerouting options (-mp, --gui.*, --router.*) are accepted and ignored.\n");
}

int main(int argc, char** argv) {
    std::string in, out;
    Options o;
    bool check = true, smooth = true, image_keepouts = true;
    std::string dump, eval, events, ntdump;
    bool nt = false, edge_set = false;
    MlModelOptions mo;
    std::string prior_out;
    std::string serve;
    double serve_idle = 1800, serve_trim = 30;
    double time_limit = 0;
    if (const char* e = std::getenv("PCBROUTE_ML_SERVER")) mo.server = e;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) { usage(); std::exit(1); }
            return argv[++i];
        };
        if (a == "-de") in = next();
        else if (a == "-do") out = next();
        else if (a == "-mt") o.threads = std::atoi(next().c_str());
        else if (a == "--mem") o.mem_gb = std::atof(next().c_str());
        else if (a == "-s") o.step = std::atof(next().c_str());
        else if (a == "-it") o.iterations = std::atoi(next().c_str());
        else if (a == "--patience") o.patience = std::atoi(next().c_str());
        else if (a == "--ripups") o.ripups = std::atoi(next().c_str());
        else if (a == "--rescue") o.rescue = std::atoi(next().c_str());
        else if (a == "--rescue-time") o.rescue_seconds = std::atof(next().c_str());
        else if (a == "--time-limit") time_limit = std::atof(next().c_str());
        else if (a == "--shove") o.shove = std::atoi(next().c_str());
        else if (a == "--shove-cut") o.shove_cut = std::atof(next().c_str());
        else if (a == "--shove-reach") o.shove_reach = std::atof(next().c_str());
        else if (a == "--shove-rounds") o.shove_rounds = std::atoi(next().c_str());
        else if (a == "--shove-cross") o.shove_cross = std::atof(next().c_str());
        else if (a == "--via-cost") o.via_cost = std::atof(next().c_str());
        else if (a == "--wrong-way") o.wrong_way = std::atof(next().c_str());
        else if (a == "--layer-dir") {
            std::string d = next();
            if (d != "xy" && d != "yx") { std::fprintf(stderr, "--layer-dir: xy or yx\n"); return 1; }
            o.f_cu_y = d == "yx";
        }
        else if (a == "--edge") { o.edge_clearance = std::atof(next().c_str()); edge_set = true; }
        else if (a == "--neck-min") o.neck_min = std::atof(next().c_str());
        else if (a == "--wide-margin") o.wide_margin = true;
        else if (a == "--plane-mode") {
            std::string m = next();
            if (m != "reserve" && m != "refill") { std::fprintf(stderr, "--plane-mode: reserve or refill\n"); return 1; }
            o.plane_refill = m == "refill";
        }
        else if (a == "--pour-min-width") o.pour_min_width = std::atof(next().c_str());
        else if (a == "--pour-clearance") o.pour_clearance = std::atof(next().c_str());
        else if (a == "--pour-gnd-cost") { o.pour_gnd_cost = std::atof(next().c_str()); nt = true; }  // ground nets by type
        else if (a == "--pour-gnd-all") o.pour_gnd_all = true;
        else if (a == "--window") o.window = std::atof(next().c_str());
        else if (a == "--hweight") o.hweight = std::atof(next().c_str());
        else if (a == "--max-pops") o.max_pops = size_t(std::atof(next().c_str()));
        else if (a == "--full-every") o.full_every = std::atoi(next().c_str());
        else if (a == "--probe") o.probe_pops = size_t(std::atof(next().c_str()));
        else if (a == "--pres-growth") o.pres_growth = std::atof(next().c_str());
        else if (a == "--pres-max") o.pres_max = std::atof(next().c_str());
        else if (a == "--hist-inc") o.hist_inc = std::atof(next().c_str());
        else if (a == "--gpu-pops") o.gpu_pops = size_t(std::atof(next().c_str()));
        else if (a == "--gpu-slots") o.gpu_slots = std::atoi(next().c_str());
        else if (a == "--gpu-mem") o.gpu_mem_gb = std::atof(next().c_str());
        else if (a == "--gpu-nowait") o.gpu_wait = false;
        else if (a == "--gcell") o.gcell = std::atof(next().c_str());
        else if (a == "--gmargin") o.gmargin = std::atoi(next().c_str());
        else if (a == "--ml-prior") o.ml_prior = next();
        else if (a == "--ml-model") mo.model = next();
        else if (a == "--ml-device") mo.device = next();
        else if (a == "--ml-server") mo.server = next();
        else if (a == "--ml-server-spawn") mo.spawn_server = true;
        else if (a == "--ml-serve") serve = next();
        else if (a == "--ml-idle") serve_idle = std::atof(next().c_str());
        else if (a == "--ml-trim") serve_trim = std::atof(next().c_str());
        else if (a == "--ml-prior-out") prior_out = next();
        else if (a == "--ml-union-share") mo.union_share = std::atof(next().c_str());
        else if (a == "--ml-grow") mo.grow = std::atoi(next().c_str());
        else if (a == "--ml-dump-inputs") mo.dump_inputs = next();
        else if (a == "--ml-weight") o.ml_weight = std::atof(next().c_str());
        else if (a == "--ml-sat") o.ml_sat = std::atof(next().c_str());
        else if (a == "--ml-corridor") o.ml_corridor = std::atof(next().c_str());
        else if (a == "--ml-congestion") o.ml_congestion = next();
        else if (a == "--ml-cong-weight") o.ml_cong_weight = std::atof(next().c_str());
        else if (a == "--order") o.order = next();
        else if (a == "--budget-scale") o.budget_scale = std::atof(next().c_str());
        else if (a == "--dump-board") dump = next();
        else if (a == "--nt-dump") { ntdump = next(); nt = true; }
        else if (a == "--nt-hs-first") o.nt_hs_first = nt = true;
        else if (a == "--nt-power-first") o.nt_power_first = nt = true;
        else if (a == "--nt-hs-via") { o.nt_hs_via = std::atof(next().c_str()); nt = true; }
        else if (a == "--nt-hs-len") { o.nt_hs_len = std::max(1.0, std::atof(next().c_str())); nt = true; }
        else if (a == "--nt-pair") { o.nt_pair = std::atof(next().c_str()); nt = true; }
        else if (a == "--nt-pair-dist") { o.nt_pair_dist = std::atof(next().c_str()); nt = true; }
        else if (a == "--nt-plane-via") { o.nt_plane_via = std::atof(next().c_str()); nt = true; }
        else if (a == "--nt-power-width") { o.nt_power_width = std::atof(next().c_str()); nt = true; }
        else if (a == "--nt-xtal-zone") { o.nt_xtal_zone = std::atof(next().c_str()); nt = true; }
        else if (a == "--nt-xtal-via") { o.nt_xtal_via = std::atof(next().c_str()); nt = true; }
        else if (a == "--nt-all") {
            o.nt_hs_first = o.nt_power_first = nt = true;
            o.nt_hs_via = 2; o.nt_hs_len = 2; o.nt_pair = 1; o.nt_plane_via = 200; o.nt_power_width = 400; o.nt_xtal_zone = 1000;
        }
        else if (a == "--dump-hist") o.dump_hist = next();
        else if (a == "--dump-nets") o.dump_nets = next();
        else if (a == "--eval") eval = next();
        else if (a == "--events") events = next();
        else if (a == "--free-batches") o.disjoint_batches = false;
        else if (a == "--strict") o.strict = true;
        else if (a == "--spec") o.spec = std::atoi(next().c_str());
        else if (a == "--legalize-seq") o.legalize_par = false;
        else if (a == "--no-check") check = false;
        else if (a == "--no-image-keepouts") image_keepouts = false;
        else if (a == "--no-smooth") smooth = false;
        else if (a == "-q") o.verbose = 0;
        else if (a == "-v") o.verbose = 2;
        else if (a == "-vv") o.verbose = 3;
        else if (a == "--only") o.only = next();
        else if (a == "--dump") { o.dump_x = std::atof(next().c_str()); o.dump_y = std::atof(next().c_str()); o.dump_r = std::atof(next().c_str()); }
        else if (a == "-mp") next();
        else if (a.rfind("--gui", 0) == 0 || a.rfind("--router", 0) == 0) {}
        else if (a == "-h" || a == "--help") { usage(); return 0; }
        else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); usage(); return 1; }
    }
    if (!serve.empty()) {
        if (mo.model.empty()) { std::fprintf(stderr, "--ml-serve needs --ml-model\n"); return 1; }
        mo.verbose = o.verbose;
        try {
            return ml_serve(mo, serve, serve_idle, serve_trim);
        } catch (std::exception& e) {
            std::fprintf(stderr, "pcbroute --ml-serve: %s\n", e.what());
            return 1;
        }
    }
    if (in.empty() || (out.empty() && dump.empty() && eval.empty() && ntdump.empty() && prior_out.empty() && mo.dump_inputs.empty())) {
        usage();
        return 1;
    }
    if (nt) o.nt_dsn = in;
    try {
        auto t0 = std::chrono::steady_clock::now();
        if (time_limit > 0) o.deadline = std::chrono::duration<double>(t0.time_since_epoch()).count() + time_limit;
        if (!mo.model.empty() && !o.ml_prior.empty()) throw std::runtime_error("--ml-model and --ml-prior exclude each other");
        mo.verbose = o.verbose;
        std::unique_ptr<MlPredictor> pred;
        if (!mo.model.empty() || !mo.dump_inputs.empty()) pred = std::make_unique<MlPredictor>(mo);  // the model loads while the DSN is read
        Board B = read_dsn(in, image_keepouts);
        if (!edge_set && B.edge_clearance >= 0) o.edge_clearance = B.edge_clearance;  // the board's rule, unless --edge
        double t_read = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (o.verbose)
            std::fprintf(stderr, "read %s: %zu layers, %zu nets, %zu copper items, %zu keepouts in %.2f s\n", in.c_str(),
                         B.layers.size(), B.nets.size(), B.items.size(), B.keepouts.size(), t_read);
        if (!dump.empty()) { dump_board(B, dump); return 0; }
        if (!ntdump.empty()) {
            dump_net_types(B, classify_nets(B, in), ntdump);
            if (out.empty() && eval.empty()) return 0;
        }
        if (!eval.empty()) return eval_session(B, o, eval, o.verbose);
        if (pred) {
            pred->start(B);  // runs while the router sets up its grids; the router waits for it when it loads the prior
            if (out.empty()) {
                if (!prior_out.empty()) write_ml_prior(pred->get(), prior_out);
                else pred->get();
                return 0;
            }
            if (!mo.model.empty())
                o.ml_prior_get = [&]() -> const MlPrior& {
                    const MlPrior& P = pred->get();
                    if (!prior_out.empty()) write_ml_prior(P, prior_out);
                    return P;
                };
            else pred->get();
        }
        if (!events.empty()) {
            if (!g_events.open(events)) throw std::runtime_error("cannot write " + events);
            g_events.begin(B);
        }
        RouteStats S;
        auto R = route_board(B, o, S);
        if (smooth) {
            g_events.pass(g_events.passes + 1, "optimize");
            smooth_routes(B, o, R);
        }
        repair_pours(B, o, R, S);  // --plane-mode refill: pours the routes cut into islands
        if (check) S.violations = check_routes(B, o, R, o.verbose);
        if (check && o.plane_refill && std::any_of(B.items.begin(), B.items.end(), [](const Item& it) { return it.kind == Kind::Plane; })) {
            // the router takes a pour as joined wherever it is; the refill may have cut it into islands
            int conn = 0, open = open_connections(B, o, result_items(B, R), o.verbose > 1, nullptr, &conn);
            std::fprintf(stderr, "refilled pours: %d of %d connections open as --eval counts them\n", open, conn);
        }
        write_ses(B, R, out);
        if (!S.limited.empty()) std::fprintf(stderr, "pcbroute: time limit reached (%s)\n", S.limited.c_str());
        g_events.final_nets(B, R);
        g_events.end(S.connections - S.unrouted, S.unrouted, true, "", check ? S.violations : -1);
        double total = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::fprintf(stderr,
                     "pcbroute: %d nets, %d connections, %d unrouted, %d rounds, %s violations, "
                     "setup %.2f s, route %.2f s, total %.2f s\n",
                     S.nets, S.connections, S.unrouted, S.iterations,
                     check ? std::to_string(S.violations).c_str() : "unchecked", S.seconds_setup, S.seconds_route, total);
        return S.violations ? 2 : 0;
    } catch (std::exception& e) {
        std::fprintf(stderr, "pcbroute: %s\n", e.what());
        g_events.end(0, 0, false, e.what());
        return 1;
    }
}
