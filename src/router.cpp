#include "router.h"

#include "events.h"
#include "mlprior.h"
#include "nettypes.h"
#ifdef PCB_GPU
#include "gpu.h"
#endif

#include <algorithm>
#include <array>
#include <map>
#include <climits>
#include <fstream>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <malloc.h>
#include <deque>
#include <functional>
#include <numeric>
#include <queue>
#include <set>
#include <stdexcept>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <unordered_set>

namespace {

constexpr uint16_t FREE = 0, BLOCK = 0xFFFF;
constexpr float TIE = 1.0f;  // f = TIE g + h: among equal f, prefer the deeper state
constexpr float INF = 1e30f;
const double SQ2 = std::sqrt(2.0);

double now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Grid {
    double x0 = 0, y0 = 0, s = 50;
    int nx = 0, ny = 0;
    size_t n() const { return size_t(nx) * ny; }
    P center(int i, int j) const { return {x0 + (i + 0.5) * s, y0 + (j + 0.5) * s}; }
    P center(uint32_t c) const { return center(int(c % nx), int(c / nx)); }
};

// f(cell) for every cell whose centre is within R of the shape's copper (strictly), or inside it.
template <class F> void raster(const Grid& g, const Shape& sh, double R, F&& f) {
    if (sh.poly.size() >= 3) {
        Box b = sh.box();
        int j0 = std::max(0, int(std::floor((b.y0 - g.y0) / g.s - 0.5)));
        int j1 = std::min(g.ny - 1, int(std::ceil((b.y1 - g.y0) / g.s - 0.5)));
        std::vector<double> xs;
        for (int j = j0; j <= j1; ++j) {
            double y = g.y0 + (j + 0.5) * g.s;
            xs.clear();
            for (size_t a = 0, c = sh.poly.size() - 1; a < sh.poly.size(); c = a++) {
                const P &p = sh.poly[a], &q = sh.poly[c];
                if ((p.y > y) != (q.y > y)) xs.push_back(p.x + (y - p.y) * (q.x - p.x) / (q.y - p.y));
            }
            std::sort(xs.begin(), xs.end());
            for (size_t k = 0; k + 1 < xs.size(); k += 2) {
                int i0 = std::max(0, int(std::ceil((xs[k] - g.x0) / g.s - 0.5)));
                int i1 = std::min(g.nx - 1, int(std::floor((xs[k + 1] - g.x0) / g.s - 0.5)));
                for (int i = i0; i <= i1; ++i) f(uint32_t(j) * g.nx + i);
            }
        }
    }
    double rr = sh.r + R;
    if (rr <= 0) return;
    for (auto& [a, b] : sh.segs) {
        int i0 = std::max(0, int(std::floor((std::min(a.x, b.x) - rr - g.x0) / g.s - 0.5)));
        int i1 = std::min(g.nx - 1, int(std::ceil((std::max(a.x, b.x) + rr - g.x0) / g.s - 0.5)));
        int j0 = std::max(0, int(std::floor((std::min(a.y, b.y) - rr - g.y0) / g.s - 0.5)));
        int j1 = std::min(g.ny - 1, int(std::ceil((std::max(a.y, b.y) + rr - g.y0) / g.s - 0.5)));
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i)
                if (seg_point_dist(a, b, g.center(i, j)) < rr) f(uint32_t(j) * g.nx + i);
    }
}

// --plane-mode refill: what a pour's KiCad zone settings decide, from the DSN's zone rules (make_dsn.py) when it
// has them, else guessed. pour_wins: whether pour z of another net takes the overlap with pour p on their layer,
// by zone priority (KiCad breaks a tie by the zones' ids, which the DSN lacks) or else the smaller pour (designers
// give the local pour the higher priority).
bool pour_wins(const Board& B, const Item& z, const Item& p) {
    if (z.zone >= 0 && p.zone >= 0 && B.zones[z.zone].priority != B.zones[p.zone].priority)
        return B.zones[z.zone].priority > B.zones[p.zone].priority;
    return poly_area(z.shape.poly) < poly_area(p.shape.poly);
}
double pour_clearance(const Board& B, const Options& o, const Item& p) { return p.zone >= 0 ? B.zones[p.zone].clearance : o.pour_clearance; }
double pour_min_width(const Board& B, const Options& o, const Item& p) {
    return p.zone >= 0 && B.zones[p.zone].min_width > 0 ? B.zones[p.zone].min_width : o.pour_min_width;
}

// f(cell) for the cells a fillet of radius r at each convex corner of the polygon cuts off, and those within m of
// the arc: per corner, the cells of the wedge between the arc's centre and its tangent points (the side of the
// corner) farther than r - m from the centre. A fillet is limited to half of each edge beside it, as KiCad's.
template <class F> void fillet_cuts(const Grid& g, const std::vector<P>& poly, double r, double m, F&& f) {
    const size_t n = poly.size();
    if (n < 3 || r <= 0) return;
    double area2 = 0;
    for (size_t i = 0, j = n - 1; i < n; j = i++) area2 += cross(poly[j], poly[i]);
    for (size_t i = 0; i < n; ++i) {
        P v = poly[i], a = poly[(i + n - 1) % n], c = poly[(i + 1) % n];
        double la = norm(a - v), lc = norm(c - v);
        if (la <= 0 || lc <= 0 || cross(v - a, c - v) * area2 <= 0) continue;  // a reflex or degenerate corner
        P u1 = (a - v) * (1 / la), u2 = (c - v) * (1 / lc);
        double th = std::acos(std::clamp(dot(u1, u2), -1.0, 1.0));  // interior angle
        if (th < 1e-3 || th > M_PI - 1e-3) continue;
        double t = std::min(r / std::tan(th / 2), std::min(la, lc) / 2), re = t * std::tan(th / 2);
        P bis = u1 + u2;
        bis = bis * (1 / norm(bis));
        P cen = v + bis * (re / std::sin(th / 2)), t1 = v + u1 * t, t2 = v + u2 * t;
        const double sgn = cross(t1 - cen, t2 - cen), keep = std::max(0.0, re - m);
        Box bx;
        bx.add(v); bx.add(t1); bx.add(t2);
        int i0 = std::max(0, int(std::floor((bx.x0 - g.x0) / g.s - 0.5))), i1 = std::min(g.nx - 1, int(std::ceil((bx.x1 - g.x0) / g.s - 0.5)));
        int j0 = std::max(0, int(std::floor((bx.y0 - g.y0) / g.s - 0.5))), j1 = std::min(g.ny - 1, int(std::ceil((bx.y1 - g.y0) / g.s - 0.5)));
        for (int jj = j0; jj <= j1; ++jj)
            for (int ii = i0; ii <= i1; ++ii) {
                P q = g.center(ii, jj) - cen;
                if (cross(t1 - cen, q) * sgn >= 0 && cross(q, t2 - cen) * sgn >= 0 && norm(q) > keep) f(uint32_t(jj) * g.nx + ii);
            }
    }
}

struct Span {
    int j, i0, i1;
};

// Rows of cells whose centre is within r of segment ab (strictly): one span per row (a capsule is convex).
std::atomic<long> n_gpu{0}, n_gpu_us{0}, n_gpu_sweeps{0}, n_gpu_fail{0}, n_gpu_none{0};
std::atomic<long> n_gpu_why[4];  // why a GPU solution was not taken: 0 solve failed, 1 chain broke, 2 via spacing along the path, 3 via near an old one
std::atomic<long> n_ml_fallback{0};  // searches that left the learned corridor
std::atomic<long> n_caps{0}, n_stamp{0}, n_rows{0}, n_pops{0}, n_push{0}, n_search{0};
std::atomic<long> us_dmap{0}, us_init{0}, us_loop{0};  // inside the searches (profiling)
double t_stamp = 0, t_conf = 0, t_setupsearch = 0;  // phase timers (profiling)
inline void capsule_spans(const Grid& g, P a, P b, double r, std::vector<Span>& out) {
    ++n_caps;
    if (r <= 0) return;
    P d = b - a;
    double L2 = dot(d, d), L = std::sqrt(L2);
    int j0 = std::max(0, int(std::floor((std::min(a.y, b.y) - r - g.y0) / g.s - 0.5)));
    int j1 = std::min(g.ny - 1, int(std::ceil((std::max(a.y, b.y) + r - g.y0) / g.s - 0.5)));
    for (int j = j0; j <= j1; ++j) {
        double y = g.y0 + (j + 0.5) * g.s, lo = 1e300, hi = -1e300;
        for (P c : {a, b}) {
            double e = y - c.y;
            if (std::fabs(e) < r) { double w = std::sqrt(r * r - e * e); lo = std::min(lo, c.x - w); hi = std::max(hi, c.x + w); }
        }
        if (L2 > 0) {
            double e = y - a.y, ml = -1e300, mh = 1e300;
            // perpendicular distance |(x - a.x) dy - e dx| / L < r
            if (d.y != 0) {
                double u = a.x + (e * d.x - r * L) / d.y, v = a.x + (e * d.x + r * L) / d.y;
                ml = std::max(ml, std::min(u, v)); mh = std::min(mh, std::max(u, v));
            } else if (std::fabs(e) >= r) mh = -1e300;
            // projection (x - a.x) dx + e dy in [0, L2]
            if (d.x != 0) {
                double u = a.x - e * d.y / d.x, v = a.x + (L2 - e * d.y) / d.x;
                ml = std::max(ml, std::min(u, v)); mh = std::min(mh, std::max(u, v));
            } else if (e * d.y < 0 || e * d.y > L2) mh = -1e300;
            if (ml < mh) { lo = std::min(lo, ml); hi = std::max(hi, mh); }
        }
        if (lo >= hi) continue;
        int i0 = std::max(0, int(std::floor((lo - g.x0) / g.s - 0.5)) + 1);
        int i1 = std::min(g.nx - 1, int(std::ceil((hi - g.x0) / g.s - 0.5)) - 1);
        if (i0 <= i1) out.push_back({j, i0, i1});
    }
}

// Sort and merge spans so that every cell appears once.
inline void merge_spans(std::vector<Span>& v) {
    std::sort(v.begin(), v.end(), [](const Span& a, const Span& b) { return a.j < b.j || (a.j == b.j && a.i0 < b.i0); });
    size_t w = 0;
    for (size_t k = 0; k < v.size(); ++k) {
        if (w > 0 && v[w - 1].j == v[k].j && v[k].i0 <= v[w - 1].i1 + 1) v[w - 1].i1 = std::max(v[w - 1].i1, v[k].i1);
        else v[w++] = v[k];
    }
    v.resize(w);
}

inline void merge(uint16_t& v, uint16_t n) { v = (v == FREE || v == n) ? n : BLOCK; }
// The occupancy and cost grids are read by the searches while the commits write them (Router::run): plain loads
// and stores, but declared as such. A search that read a cell a commit changed is rejected by the block check,
// so a value seen half-way can't reach the output.
template <class T> inline T ld(const T& x) { T r; __atomic_load(&x, &r, __ATOMIC_RELAXED); return r; }
template <class T> inline void st(T& x, T v) { __atomic_store(&x, &v, __ATOMIC_RELAXED); }
inline bool ok(uint16_t v, uint16_t n) { return v == FREE || v == n; }

struct RClass {          // one net class being routed in this run, with its clearance grids
    int cls;
    double hw, clr, vr;  // half width, clearance, via radius
    int via = -1;        // via type (-1: no vias)
    std::vector<char> lay;
    std::vector<std::vector<uint16_t>> st;   // per layer: static obstacles for a track centre
    std::vector<uint16_t> vst;               // static obstacles for a via centre
    std::vector<std::vector<uint8_t>> oc, ov;  // per layer: other nets' routes, for a track / via centre
    // What the search reads, one number per cell: the step cost factor (1 + history) (1 + pressure x occupancy);
    // INF where no track centre may go (static copper, or any occupancy when legalizing); negated where the cell
    // is one net's own copper, which only that net may enter (checked against st then). Kept in step with
    // st / oc / hist by rebuild_costs() and stamp(). vcst: the same for a via centre, summed over the layers.
    std::vector<std::vector<float>> cst;
    std::vector<float> vcst;
};

struct Kern {  // a disc of cells: for each row offset, the half width in cells
    std::vector<std::pair<int, int>> rows;
};

// An off-grid stub joining a grid state to its pad (cell centre first), of half width hw: the class's, or less (a
// neck-down) where a fine-pitch pad can't be reached at full width.
struct Stub {
    uint32_t s;
    std::vector<P> pts;
    double hw;
};

struct Comp {
    std::vector<uint32_t> states;            // layer * N + cell
    std::vector<Stub> attach;                // states joined to their pad by an off-grid stub
    Box box;
};

struct Job {
    int net, rc;
    std::vector<Comp> comps;
    Box box;
    std::vector<std::vector<uint32_t>> paths;   // routed paths, as states
    std::vector<uint32_t> keep;                 // junction states (vertices to keep)
    std::vector<Stub> stubs;
    int unrouted = 0, dead = 0;  // dead: terminals no route can reach
    std::vector<std::vector<Span>> stamps;      // per occupancy grid id: rows of cells stamped
    int conflicts = 0;
    double seconds = 0;  // total search time
    std::vector<uint8_t> touch, touchv;  // blocks of cells the last search read (track grids per layer; via grids)
    std::vector<uint8_t> guide;  // global-route corridor: layer * gnx * gny + gcell, empty if none
    const NetPrior* ml = nullptr; // learned prior of this net, if any
    int stuck = 0;       // rounds in a row this job has been in conflict
    int conflict_rounds = 0;  // rounds it was in conflict in all (training data)
    size_t pops = 0;
    int prio = -1;       // position in the --order file (-1: not listed; listed nets go first)
    float difficulty = 0;  // predicted difficulty 0-1 from the --order file: scales the budget and window (--budget-scale)
    uint64_t ev_sig = 0;   // --events: signature of the copper last shown
    int pair = -1;         // --nt-pair: the job of the first net of this net's differential pair (this is the second)
    bool pair_ok = true;   // ...whose track this search may follow (false while that net is routed after this one)
    bool ev_shown = false;
};

struct HeapItem {
    float f, g;
    uint32_t s;
    bool operator<(const HeapItem& o) const { return f > o.f; }
};

struct Node {               // 8 bytes per state of the search window (a whole 6-layer board at 25 um has 80 M)
    float g;
    uint16_t ep;                // g, par and rec are valid where ep == epoch
    uint8_t par;                // move into the state: 0-7 a step (di/dj index), 8+l a via from layer l, 255 a start
    uint8_t rec;                // steps along the path since its last via (255: none or far): no second via too close
};

struct Work {               // one thread's A* scratch, 9 bytes per state of the search window
    std::vector<Node> nd;
    std::vector<uint64_t> tbit; // target states
    std::vector<uint64_t> own, ownv;  // the net's own footprint (track / via grids)
    std::vector<uint64_t> treeb;      // the states joined so far (a plane makes millions: kept as bits, grown as the tree grows)
    std::vector<uint8_t> tcmask;      // ...and the coarse heuristic cells they cover
    std::vector<uint32_t> tcz, gcz;
    std::vector<uint8_t> touch, touchv;  // blocks read by the search (see Router::run)
    std::vector<uint32_t> popep;         // debugging (-vv): the epoch a state was last expanded in
    std::vector<uint32_t> tlist;
    std::unordered_map<uint32_t, int> tcomp;
    uint16_t epoch = 0;
    std::vector<HeapItem> heap;
    std::vector<float> dmap;    // heuristic: per layer, a coarse map of the distance to that layer's targets
    std::vector<float> hmin;    // ...and to the nearest target on any layer, plus a via
};

struct Router {
    const Board& B;
    const Options& o;
    Grid G;
    int NL;
    double m;  // grid margin of the old builds (--wide-margin): half a diagonal step, whatever the distance
    // Grid margin for an inflation radius R: what keeps a straight move between two free cell centres at least R
    // from the obstacle. A cell is free when its centre is at least R from it; the nearest point of the obstacle
    // to any point of a move is then at least R from both ends of the move, so at least sqrt(R^2 - h^2) from the
    // move, h being half the longest move (a diagonal step). The same holds between two grid paths (the closest
    // points of two segments that don't cross include an end of one of them) and against an off-grid stub. So
    // the margin is the sagitta R - sqrt(R^2 - h^2): 2-4 um for usual rules at 50 um, where the old margin was h.
    double mg(double R) const {
        const double h = G.s * SQ2 / 2;
        if (o.wide_margin || R <= 2 * h) return m;
        return R - std::sqrt(R * R - h * h) + 1;
    }
    std::vector<RClass> rcs;
    std::vector<std::vector<float>> hist;
    std::vector<Job> jobs;
    int dead_total = 0;
    std::vector<int> dead_by_net;  // per net, for RouteStats::dead
    int necked = 0;  // pads reached only by a neck-down stub
    MlPrior prior;                 // learned routing prior (--ml-prior)
    std::vector<NetInfo> nt;       // net types (--nt-*; empty when no such option is on)
    // --pour-gnd-cost: per layer the step cost factor of a track centre (empty: none on the layer)
    std::vector<std::vector<float>> gnd_fac;
    std::vector<uint8_t> xzone;    // --nt-xtal-zone: cells near a crystal net's pads
    std::vector<int> pcol, prow;   // fine column / row -> prior pixel column / row
    std::vector<int> play;         // router layer -> prior layer, -1 if the prior has no map for it

    // exact geometry of the static copper and keepouts, bucketed, for the off-grid stubs
    struct SItem { int net, layer; const Shape* sh; Box box; double clr; bool keepout; };
    std::vector<SItem> sitems;
    Box sbox;
    int sbx = 0, sby = 0;
    const double SBK = 1000;
    std::vector<std::vector<int>> sbucket;
    Shape edge;
    double maxclr = 0;

    void build_index() {
        for (auto& it : B.items)
            if (it.kind != Kind::Plane) sitems.push_back({it.net, it.layer, &it.shape, it.shape.box(), clr_of(it.net), false});
        for (auto& k : B.keepouts)
            if (k.wires) sitems.push_back({-1, k.layer, &k.shape, k.shape.box(), 0, true});
        for (auto& c : B.classes) maxclr = std::max(maxclr, c.clearance);
        sbox = Box{G.x0, G.y0, G.x0 + G.nx * G.s, G.y0 + G.ny * G.s};
        sbx = int((sbox.x1 - sbox.x0) / SBK) + 1;
        sby = int((sbox.y1 - sbox.y0) / SBK) + 1;
        sbucket.assign(size_t(sbx) * sby, {});
        for (size_t k = 0; k < sitems.size(); ++k) {
            const Box& b = sitems[k].box;
            int i0 = std::clamp(int((b.x0 - sbox.x0) / SBK), 0, sbx - 1), i1 = std::clamp(int((b.x1 - sbox.x0) / SBK), 0, sbx - 1);
            int j0 = std::clamp(int((b.y0 - sbox.y0) / SBK), 0, sby - 1), j1 = std::clamp(int((b.y1 - sbox.y0) / SBK), 0, sby - 1);
            for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) sbucket[size_t(j) * sbx + i].push_back(int(k));
        }
        edge = B.outline;
        edge.poly.clear();
    }

    // A straight track of half width hw from a to b on layer l clears the static copper of other nets exactly.
    bool seg_clear(int l, P a, P b, double hw, double c, int net) const {
        Shape cap = Shape::path({a, b}, hw);
        Box bb = cap.box();
        if (B.has_outline && (!point_in_poly(B.outline.poly, a) || !point_in_poly(B.outline.poly, b) ||
                              shape_dist(cap, edge) < o.edge_clearance))
            return false;
        double g = std::max(c, maxclr);
        int i0 = std::clamp(int((bb.x0 - g - sbox.x0) / SBK), 0, sbx - 1), i1 = std::clamp(int((bb.x1 + g - sbox.x0) / SBK), 0, sbx - 1);
        int j0 = std::clamp(int((bb.y0 - g - sbox.y0) / SBK), 0, sby - 1), j1 = std::clamp(int((bb.y1 + g - sbox.y0) / SBK), 0, sby - 1);
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i)
                for (int k : sbucket[size_t(j) * sbx + i]) {
                    const SItem& it = sitems[k];
                    if (it.layer >= 0 && it.layer != l) continue;
                    if (it.keepout) {
                        if (it.box.overlaps(bb, 1) && shape_dist(cap, *it.sh) < 1) return false;  // as check_routes judges it
                        continue;
                    }
                    if (it.net == net && net >= 0) continue;
                    double need = std::max(c, it.clr) + CLR_SAFETY;
                    if (it.box.overlaps(bb, need) && shape_dist(cap, *it.sh) < need) return false;
                }
        return true;
    }

    Router(const Board& b, const Options& opt) : B(b), o(opt) { NL = int(B.layers.size()); }

    // netless copper (a pin of a net another pass routes) follows the board-wide rule, like Freerouting
    double clr_of(int net) const { return B.classes[net < 0 ? 0 : B.nets[net].cls].clearance; }

    // ---- cost maps: what the search reads (see RClass::cst)
    float pres_now = 0;
    bool hard_now = false;
    // ...leaving out `own` of the occupancy (a net's own stamp, which its search ignores)
    float cost_x(const RClass& R, int l, uint32_t c, int own) const { return cost_xp(R, l, c, own, pres_now, hard_now); }
    float cost_xp(const RClass& R, int l, uint32_t c, int own, float pres, bool hard) const {
        uint16_t s = R.st[l][c];
        if (s == BLOCK) return INF;
        int occ = int(ld(R.oc[l][c])) - own;
        if (hard && occ > 0) return INF;
        float v = (1.f + hist[l][c]) * (1.f + pres * std::max(occ, 0));
        if (!gnd_fac.empty() && !gnd_fac[l].empty()) v *= gnd_fac[l][c];
        return s == FREE ? v : -v;
    }
    float cost_of(const RClass& R, int l, uint32_t c) const { return cost_x(R, l, c, 0); }
    float vcost_x(const RClass& R, uint32_t c, int own) const { return vcost_xp(R, c, own, pres_now, hard_now); }
    float vcost_xp(const RClass& R, uint32_t c, int own, float pres, bool hard) const {
        uint16_t s = R.vst[c];
        if (s == BLOCK) return INF;
        int occ = -own;
        float hs = 0;
        for (int l = 0; l < NL; ++l) { occ += ld(R.ov[l][c]); hs += hist[l][c]; }
        if (hard && occ > 0) return INF;
        float v = (1.f + hs / NL) * (1.f + pres * std::max(occ, 0));
        return s == FREE ? v : -v;
    }
    float vcost_of(const RClass& R, uint32_t c) const { return vcost_x(R, c, 0); }
    // blocks of cells, the unit in which a search's reads and a stamp's changes are compared (Router::run)
    static constexpr int BS = 16;
    int bw = 0, bh = 0;
    size_t BN = 0;
    void rebuild_costs(float pres, bool hard) {
        pres_now = pres;
        hard_now = hard;
        std::vector<std::thread> th;
        for (auto& R : rcs) {
            if (R.cst.empty()) { R.cst.assign(NL, std::vector<float>(G.n(), INF)); R.vcst.assign(G.n(), INF); }
            for (int l = 0; l < NL; ++l)
                th.emplace_back([&, l] { for (uint32_t c = 0; c < G.n(); ++c) R.cst[l][c] = cost_of(R, l, c); });
            th.emplace_back([&] { for (uint32_t c = 0; c < G.n(); ++c) R.vcst[c] = vcost_of(R, c); });
        }
        for (auto& t : th) t.join();
    }

    // ---- stamp kernels: the footprint of a track cell / via of class rc on the grids of class k, as discs.
    // A disc around every cell of a path covers the capsule around the track between consecutive cells when its
    // radius is grown to sqrt(r^2 + s^2 / 2) (the midpoint of a diagonal step is s / sqrt 2 from both cells).
    std::vector<std::vector<Kern>> kern;  // [rc][k * 4 + {track on oc, track on ov, via on oc, via on ov}]
    Kern make_kern(double r, bool capsule) const {
        double rk = capsule ? std::sqrt(r * r + G.s * G.s / 2) : r;
        Kern k;
        if (rk <= 0) return k;
        int R = int(std::ceil(rk / G.s));
        for (int dj = -R; dj <= R; ++dj) {
            double dy = dj * G.s;
            if (dy * dy >= rk * rk) continue;
            int w = int(std::ceil(std::sqrt(rk * rk - dy * dy) / G.s)) - 1;  // cells strictly within rk
            if (w >= 0) k.rows.push_back({dj, w});
        }
        return k;
    }
    void build_kernels() {
        kern.assign(rcs.size(), {});
        for (size_t r = 0; r < rcs.size(); ++r)
            for (size_t k = 0; k < rcs.size(); ++k) {
                const RClass &R = rcs[r], &K = rcs[k];
                double c = std::max(R.clr, K.clr);
                for (double rk : {R.hw + K.hw + c, R.hw + K.vr + c}) kern[r].push_back(make_kern(rk + mg(rk), true));
                for (double rk : {R.vr + K.hw + c, R.vr + K.vr + c}) kern[r].push_back(make_kern(rk + mg(rk), false));
            }
    }

    void setup() {
        Box bb;
        if (B.has_outline) bb = B.outline.box();
        else for (auto& it : B.items) bb.add(it.shape.box());
        G.s = o.step;
        G.x0 = bb.x0;
        G.y0 = bb.y0;
        G.nx = int(std::ceil((bb.x1 - bb.x0) / G.s));
        G.ny = int(std::ceil((bb.y1 - bb.y0) / G.s));
        m = G.s * SQ2 / 2 + 1;
        bw = (G.nx + BS - 1) / BS;
        bh = (G.ny + BS - 1) / BS;
        BN = size_t(bw) * bh;
        hist.assign(NL, std::vector<float>(G.n(), 0.f));
        build_index();

        // the nets to route: those whose copper is in more than one piece, grouped by class (grids are only
        // built for the classes that have something to route)
        if (!o.nt_dsn.empty()) nt = classify_nets(B, o.nt_dsn);
        gnd_setup();
        // --nt-power-width: supply nets without a plane get a class of their own, as wide as asked
        auto wide = [&](size_t n) {
            return !nt.empty() && o.nt_power_width > B.classes[B.nets[n].cls].width && nt_supply(nt[n].type) && !nt[n].plane;
        };
        std::vector<int> cls_rc(B.classes.size() * 2, -1);
        std::vector<char> need(B.nets.size(), 0);
        for (size_t n = 0; n < B.nets.size(); ++n) {
            const Net& N = B.nets[n];
            if (!N.has_pins || (!o.only.empty() && N.name != o.only)) continue;
            std::vector<int> comp;
            if (components(int(n), comp) < 2) continue;
            need[n] = 1;
            const size_t key = N.cls * 2 + wide(n);
            if (cls_rc[key] < 0) {
                const NetClass& C = B.classes[N.cls];
                RClass rc;
                rc.cls = N.cls;
                rc.hw = (key & 1 ? o.nt_power_width : C.width) / 2;
                rc.clr = C.clearance;
                rc.via = C.vias.empty() ? -1 : C.vias[0];
                rc.vr = rc.via >= 0 ? B.vias[rc.via].diameter / 2 : 0;
                rc.lay.assign(NL, 0);
                for (int l = 0; l < NL; ++l) rc.lay[l] = C.layers[l];
                cls_rc[key] = int(rcs.size());
                rcs.push_back(std::move(rc));
            }
        }
        std::vector<std::thread> th;
        for (auto& rc : rcs) th.emplace_back([&] { build_static(rc); });
        for (auto& t : th) t.join();
        for (size_t n = 0; n < B.nets.size(); ++n)
            if (need[n]) make_job(int(n), cls_rc[B.nets[n].cls * 2 + wide(n)]);
        build_kernels();
        load_prior();
        load_order();
        global_route();
        nt_setup();
    }

    // ---- --plane-mode refill --pour-gnd-cost: designers keep the ground plane next to a signal layer whole and run
    // their inner signals over the supply or second ground layer. A track centre over a ground pour of the inner
    // layer with one nearest each outer layer (its reference plane; --pour-gnd-all: over any ground pour) costs the
    // factor more, except where a smaller pour of another net wins the overlap (no ground fill there).
    void gnd_setup() {
        if (!o.plane_refill || o.pour_gnd_cost <= 0 || nt.empty()) return;
        auto gnd = [&](const Item& it) { return it.kind == Kind::Plane && it.net >= 0 && it.layer >= 0 && nt[it.net].type == NetType::Ground; };
        std::vector<char> has(NL, 0), ref(NL, 0);
        for (auto& it : B.items)
            if (gnd(it)) has[it.layer] = 1;
        for (int l = 1; l < NL - 1; ++l)
            if (has[l] && (o.pour_gnd_all || std::find(has.begin() + 1, has.begin() + l, 1) == has.begin() + l ||
                           std::find(has.begin() + l + 1, has.end() - 1, 1) == has.end() - 1))
                ref[l] = 1;
        gnd_fac.assign(NL, {});
        for (auto& it : B.items) {
            if (!gnd(it) || !ref[it.layer]) continue;
            auto& f = gnd_fac[it.layer];
            if (f.empty()) f.assign(G.n(), 1.f);
            raster(G, it.shape, 0, [&](uint32_t q) { f[q] = float(o.pour_gnd_cost); });
        }
        for (auto& z : B.items)  // where a smaller pour of another net wins, there is no ground fill
            if (z.kind == Kind::Plane && z.layer >= 0 && !gnd_fac[z.layer].empty() && !gnd(z))
                for (auto& it : B.items)
                    if (gnd(it) && it.layer == z.layer && poly_area(z.shape.poly) < poly_area(it.shape.poly) && z.shape.box().overlaps(it.shape.box())) {
                        raster(G, z.shape, 0, [&](uint32_t q) { gnd_fac[z.layer][q] = 1.f; });
                        break;
                    }
        if (o.verbose) {
            std::fprintf(stderr, "ground pour cost x%.2f on", o.pour_gnd_cost);
            for (int l = 0; l < NL; ++l)
                if (!gnd_fac[l].empty()) std::fprintf(stderr, " %s", B.layers[l].c_str());
            std::fprintf(stderr, "\n");
        }
    }

    // ---- net types (--nt-*): the crystal via zone, and what the options say of each net
    void nt_setup() {
        if (nt.empty()) return;
        if (o.verbose) {
            int cnt[7] = {0};
            for (auto& J : jobs) ++cnt[int(nt[J.net].type)];
            std::fprintf(stderr, "net types of the %zu nets to route:", jobs.size());
            for (int t = 0; t < 7; ++t) if (cnt[t]) std::fprintf(stderr, " %d %s", cnt[t], net_type_name(NetType(t)));
            std::fprintf(stderr, "\n");
        }
        if (o.nt_xtal_zone > 0) {
            xzone.assign(G.n(), 0);
            size_t k = 0;
            for (auto& it : B.items)
                if (it.kind == Kind::Pad && it.net >= 0 && nt[it.net].type == NetType::Crystal)
                    raster(G, it.shape, o.nt_xtal_zone, [&](uint32_t q) { k += !xzone[q]; xzone[q] = 1; });
            if (o.verbose) std::fprintf(stderr, "crystal via zone: %zu cells\n", k);
        }
    }
    double job_via_cost(const Job& J) const {
        if (nt.empty()) return o.via_cost;
        const NetInfo& I = nt[J.net];
        if (o.nt_plane_via >= 0 && I.plane && nt_supply(I.type)) return o.nt_plane_via;  // a local pour of a signal is no reference plane
        return nt_fast(I.type) ? o.via_cost * o.nt_hs_via : o.via_cost;
    }
    // Order (--nt-hs-first, --nt-power-first): fast nets, then supplies, then the rest, each in the order they had;
    // with --nt-pair a differential pair's second net right after its first, which it then follows.
    void nt_order() {
        if (nt.empty()) return;
        auto rank = [&](const Job& J) {
            NetType t = nt[J.net].type;
            return o.nt_hs_first && nt_fast(t) ? 0 : o.nt_power_first && nt_supply(t) ? 1 : 2;
        };
        if (o.nt_hs_first || o.nt_power_first)
            std::stable_sort(jobs.begin(), jobs.end(), [&](const Job& a, const Job& b) { return rank(a) < rank(b); });
        if (o.nt_pair <= 0) return;
        std::vector<int> job_of(B.nets.size(), -1);
        for (size_t j = 0; j < jobs.size(); ++j) job_of[jobs[j].net] = int(j);
        auto pairable = [&](int net) { NetType t = nt[net].type; return t == NetType::Diff || t == NetType::Analog; };
        std::vector<Job> out;
        std::vector<char> placed(jobs.size(), 0);
        int pairs = 0;
        for (size_t j = 0; j < jobs.size(); ++j) {
            if (placed[j]) continue;
            placed[j] = 1;
            int pn = nt[jobs[j].net].pair, pj = pn >= 0 ? job_of[pn] : -1;
            out.push_back(std::move(jobs[j]));
            if (pj >= 0 && !placed[pj] && pairable(out.back().net) && pairable(pn)) {
                placed[pj] = 1;
                out.push_back(std::move(jobs[pj]));
                out.back().pair = int(out.size()) - 2;
                ++pairs;
            }
        }
        jobs = std::move(out);
        if (o.verbose) std::fprintf(stderr, "differential pairs routed together: %d\n", pairs);
    }

    // ---- learned order (ml/mlorder): one net name per line, hardest first, optionally TAB and a difficulty in 0-1.
    // Listed nets are routed in that order before the others (which keep the default order); the difficulty scales
    // a net's search budget and window from the first round (--budget-scale).
    void load_order() {
        if (o.order.empty()) return;
        std::ifstream f(o.order);
        if (!f) throw std::runtime_error("cannot open " + o.order);
        std::unordered_map<std::string, std::pair<int, float>> pos;
        std::string line;
        int k = 0;
        while (std::getline(f, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty() || line[0] == '#') continue;
            size_t tab = line.find('\t');
            std::string name = line.substr(0, tab);
            float d = tab == std::string::npos ? 0.f : float(std::clamp(std::atof(line.c_str() + tab + 1), 0.0, 1.0));
            if (!pos.count(name)) pos[name] = {k++, d};
        }
        int matched = 0;
        for (auto& J : jobs) {
            auto it = pos.find(B.nets[J.net].name);
            if (it != pos.end()) { J.prio = it->second.first; J.difficulty = it->second.second; ++matched; }
        }
        if (o.verbose)
            std::fprintf(stderr, "order: %d nets listed in %s, %d of %zu jobs matched, budget scale %.2f\n", k, o.order.c_str(), matched, jobs.size(), o.budget_scale);
    }

    // ---- predicted congestion (mlroute.py --congestion): the history costs start from it instead of 0, so the
    // first rounds already avoid the places where the negotiation would have pushed the nets apart
    void load_congestion() {
        if (o.ml_congestion.empty()) return;
        MlPrior C = read_ml_prior(o.ml_congestion);
        auto it = C.nets.find("*");
        if (it == C.nets.end()) throw std::runtime_error("congestion map: no map named \"*\" in " + o.ml_congestion);
        const NetPrior& M = it->second;
        size_t seeded = 0;
        for (int l = 0; l < NL; ++l) {
            int pl = -1;
            for (size_t k = 0; k < C.layers.size(); ++k) if (C.layers[k] == B.layers[l]) pl = int(k);
            if (pl < 0) continue;
            for (int j = 0; j < G.ny; ++j) {
                int y = int(std::floor((G.y0 + (j + 0.5) * G.s - C.y0) / C.pitch)) - M.j0;
                if (y < 0 || y >= M.h) continue;
                for (int i = 0; i < G.nx; ++i) {
                    int x = int(std::floor((G.x0 + (i + 0.5) * G.s - C.x0) / C.pitch)) - M.i0;
                    if (x < 0 || x >= M.w) continue;
                    uint8_t v = M.data[(size_t(pl) * M.h + y) * M.w + x];
                    if (v) { hist[l][size_t(j) * G.nx + i] = float(o.ml_cong_weight) * v / 255.f; ++seeded; }
                }
            }
        }
        if (o.verbose) std::fprintf(stderr, "congestion map: %zu cells seeded with history (weight %.2f)\n", seeded, o.ml_cong_weight);
    }

    // ---- learned prior: maps from mlroute.py, matched to the jobs by net name and to the layers by name
    void load_prior() {
        load_congestion();
        if (o.ml_prior_get) prior = o.ml_prior_get();
        else if (o.ml_prior.empty()) return;
        else prior = read_ml_prior(o.ml_prior);
        pcol.resize(G.nx);
        prow.resize(G.ny);
        for (int i = 0; i < G.nx; ++i) pcol[i] = int(std::floor((G.x0 + (i + 0.5) * G.s - prior.x0) / prior.pitch));
        for (int j = 0; j < G.ny; ++j) prow[j] = int(std::floor((G.y0 + (j + 0.5) * G.s - prior.y0) / prior.pitch));
        play.assign(NL, -1);
        for (int l = 0; l < NL; ++l)
            for (size_t k = 0; k < prior.layers.size(); ++k)
                if (prior.layers[k] == B.layers[l]) play[l] = int(k);
        int matched = 0;
        for (auto& J : jobs) {
            auto it = prior.nets.find(B.nets[J.net].name);
            if (it != prior.nets.end()) { J.ml = &it->second; ++matched; }
        }
        if (o.verbose)
            std::fprintf(stderr, "ML prior: %zu maps at %.0f um, %d of %zu nets matched, weight %.2f, corridor %.2f\n", prior.nets.size(),
                         prior.pitch, matched, jobs.size(), o.ml_weight, o.ml_corridor);
    }
    // the prior of net map mp at fine cell (ci, cj) on router layer l, 0-255 (255 where the prior has no say)
    uint8_t prior_at(const NetPrior* mp, int l, int ci, int cj) const {
        int pl = play[l];
        if (pl < 0) return 255;
        int x = pcol[ci] - mp->i0, y = prow[cj] - mp->j0;
        if (x < 0 || y < 0 || x >= mp->w || y >= mp->h) return 0;
        return mp->data[(size_t(pl) * mp->h + y) * mp->w + x];
    }

    // ---- global routing: the coarse first stage, as in chip routers. Nets are routed on a grid of gcells with
    // capacities taken from the free fine cells, negotiating congestion; each net's path, widened a little, becomes
    // the corridor its detailed search stays in (it falls back to the whole window if the corridor is a dead end).
    int GS = 0, gnx = 0, gny = 0;
    std::vector<uint16_t> colnode, rownode;
    size_t GN() const { return size_t(gnx) * gny; }
    uint32_t gnode(int i, int j) const { return uint32_t(rownode[j]) * gnx + colnode[i]; }

    void global_route() {
        if (o.gcell <= 0 || jobs.empty()) return;
        double t0 = now();
        GS = std::max(4, int(o.gcell / G.s));
        gnx = (G.nx + GS - 1) / GS;
        gny = (G.ny + GS - 1) / GS;
        colnode.resize(G.nx);
        rownode.resize(G.ny);
        for (int i = 0; i < G.nx; ++i) colnode[i] = uint16_t(i / GS);
        for (int j = 0; j < G.ny; ++j) rownode[j] = uint16_t(j / GS);
        const size_t n = GN();
        // capacity in tracks: the parallel tracks that can run straight through a gcell, from the rows (columns) of
        // fine cells that are free along the gcell's whole width, over the track pitch; the better direction counts
        std::vector<std::vector<std::vector<float>>> cap(rcs.size());
        for (size_t k = 0; k < rcs.size(); ++k) {
            const RClass& R = rcs[k];
            double pitch = std::max(1.0, std::ceil((2 * R.hw + R.clr) / G.s));
            cap[k].assign(NL, std::vector<float>(n, 0.f));
            for (int l = 0; l < NL; ++l) {
                if (!R.lay[l]) continue;
                std::vector<int> rows(n, 0), cols(n, 0);
                const auto& st = R.st[l];
                for (int j = 0; j < G.ny; ++j) {  // rows: longest free run inside each gcell
                    int run = 0, best = 0;
                    for (int i = 0; i < G.nx; ++i) {
                        if (i % GS == 0) run = best = 0;
                        run = st[size_t(j) * G.nx + i] == FREE ? run + 1 : 0;
                        best = std::max(best, run);
                        if ((i % GS == GS - 1 || i == G.nx - 1) && best >= 0.9 * std::min(GS, G.nx - i / GS * GS)) ++rows[gnode(i, j)];
                    }
                }
                for (int i = 0; i < G.nx; ++i) {
                    int run = 0, best = 0;
                    for (int j = 0; j < G.ny; ++j) {
                        if (j % GS == 0) run = best = 0;
                        run = st[size_t(j) * G.nx + i] == FREE ? run + 1 : 0;
                        best = std::max(best, run);
                        if ((j % GS == GS - 1 || j == G.ny - 1) && best >= 0.9 * std::min(GS, G.ny - j / GS * GS)) ++cols[gnode(i, j)];
                    }
                }
                for (size_t g = 0; g < n; ++g) cap[k][l][g] = float(std::max(rows[g], cols[g]) / pitch);
            }
        }
        std::vector<float> use(NL * n, 0.f), gh(NL * n, 0.f), fixed(NL * n, 0.f);  // fixed: nets ending in the gcell
        std::vector<std::vector<uint32_t>> terms(jobs.size());  // each net's terminal gcell-layers
        for (size_t ji = 0; ji < jobs.size(); ++ji) {
            for (auto& C : jobs[ji].comps)
                for (uint32_t s : C.states) {
                    uint32_t cell = uint32_t(s % G.n());
                    terms[ji].push_back(uint32_t(s / G.n()) * uint32_t(n) + gnode(int(cell % G.nx), int(cell / G.nx)));
                }
            std::sort(terms[ji].begin(), terms[ji].end());
            terms[ji].erase(std::unique(terms[ji].begin(), terms[ji].end()), terms[ji].end());
            for (uint32_t v : terms[ji]) fixed[v] += 1;
        }
        // tracks through a gcell beyond what ends there; ends are unavoidable, so they don't count as congestion
        auto excess = [&](uint32_t v, int rc) { return use[v] - fixed[v] - std::max(cap[rc][int(v / n)][v % n], 0.5f) + (use[v] > fixed[v] ? 0.f : -1e9f); };
        std::vector<std::vector<uint32_t>> nodes(jobs.size());  // nodes of each net's global route
        std::vector<float> dist(NL * n);
        std::vector<int32_t> par(NL * n);
        std::vector<uint32_t> seen(NL * n, 0);
        uint32_t ep = 0;
        auto route_net = [&](size_t ji, float pres) {
            Job& J = jobs[ji];
            const RClass& R = rcs[J.rc];
            const auto& cp = cap[J.rc];
            std::vector<std::vector<uint32_t>> term(J.comps.size());
            for (size_t c = 0; c < J.comps.size(); ++c) {
                for (uint32_t s : J.comps[c].states) {
                    uint32_t cell = uint32_t(s % G.n());
                    term[c].push_back(uint32_t(s / G.n()) * uint32_t(n) + gnode(int(cell % G.nx), int(cell / G.nx)));
                }
                std::sort(term[c].begin(), term[c].end());
                term[c].erase(std::unique(term[c].begin(), term[c].end()), term[c].end());
            }
            std::vector<uint32_t> got = term[0];
            std::vector<int> left;
            for (size_t c = 1; c < term.size(); ++c) left.push_back(int(c));
            const float step = float(GS * G.s), w = float(o.wrong_way);
            auto into = [&](uint32_t v) {  // cost of a track through a gcell
                int l = int(v / n);
                uint32_t g = uint32_t(v % n);
                float c = cp[l][g], over = std::binary_search(terms[ji].begin(), terms[ji].end(), v) ? 0.f : std::max(0.f, use[v] - fixed[v] + 1 - std::max(c, 0.5f));
                return (1.f + 3.f * (1.f - std::min(c, 1.f))) * (1.f + gh[v]) * (1.f + pres * over);
            };
            using QI = std::pair<float, uint32_t>;
            while (!left.empty()) {
                ++ep;
                std::unordered_map<uint32_t, int> tgt;
                for (int c : left) for (uint32_t v : term[c]) tgt[v] = c;
                std::priority_queue<QI, std::vector<QI>, std::greater<QI>> pq;
                for (uint32_t v : got) { seen[v] = ep; dist[v] = 0; par[v] = -1; pq.push({0.f, v}); }
                int64_t hit = -1;
                while (!pq.empty()) {
                    auto [d, v] = pq.top();
                    pq.pop();
                    if (d > dist[v]) continue;
                    if (tgt.count(v)) { hit = v; break; }
                    int l = int(v / n);
                    uint32_t g = uint32_t(v % n);
                    int gi = int(g % gnx), gj = int(g / gnx);
                    auto push = [&](uint32_t u, float nd) {
                        if (seen[u] != ep || nd < dist[u]) { seen[u] = ep; dist[u] = nd; par[u] = int32_t(v); pq.push({nd, u}); }
                    };
                    const std::string& nm = B.layers[l];
                    int pref = nm == "F.Cu" ? 1 : nm == "B.Cu" ? 2 : 0;
                    if (pref && o.f_cu_y) pref = 3 - pref;
                    if (R.lay[l])
                        for (int dd = 0; dd < 4; ++dd) {
                            int ni = gi + (dd == 0) - (dd == 1), nj = gj + (dd == 2) - (dd == 3);
                            if (ni < 0 || nj < 0 || ni >= gnx || nj >= gny) continue;
                            uint32_t u = uint32_t(l) * uint32_t(n) + uint32_t(nj) * gnx + ni;
                            if (!R.lay[l] && !tgt.count(u)) continue;
                            float f = !pref ? 1.f : ((dd < 2) == (pref == 1)) ? 1.f : w;
                            push(u, d + step * f * into(u));
                        }
                    if (R.via >= 0)
                        for (int l2 = 0; l2 < NL; ++l2) {
                            if (l2 == l) continue;
                            uint32_t u = uint32_t(l2) * uint32_t(n) + g;
                            if (!R.lay[l2] && !tgt.count(u)) continue;
                            push(u, d + float(o.via_cost) * (1.f + gh[u]) * (1.f + pres * std::max(0.f, use[u] - fixed[u] - cp[l2][g])));
                        }
                }
                if (hit < 0) { left.erase(left.begin()); continue; }  // walled off at this scale: leave it to the detailed search
                int c = tgt[uint32_t(hit)];
                for (int64_t v = hit; v >= 0; v = par[v]) got.push_back(uint32_t(v));
                got.insert(got.end(), term[c].begin(), term[c].end());
                left.erase(std::find(left.begin(), left.end(), c));
            }
            std::sort(got.begin(), got.end());
            got.erase(std::unique(got.begin(), got.end()), got.end());
            nodes[ji] = std::move(got);
        };
        std::vector<size_t> order(jobs.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
            auto hp = [&](size_t k) { return jobs[k].box.x1 - jobs[k].box.x0 + jobs[k].box.y1 - jobs[k].box.y0; };
            return hp(a) < hp(b);
        });
        int over_nodes = 0, it = 0;
        for (; it < 12; ++it) {
            float pres = 0.5f * std::pow(1.8f, float(it));
            for (size_t ji : order) {
                if (it > 0) {
                    bool bad = false;
                    for (uint32_t v : nodes[ji]) bad |= excess(v, jobs[ji].rc) > 0;
                    if (!bad) continue;
                    for (uint32_t v : nodes[ji]) use[v] -= 1;
                }
                route_net(ji, pres);
                for (uint32_t v : nodes[ji]) use[v] += 1;
            }
            over_nodes = 0;
            std::vector<char> hot(NL * n, 0);
            for (size_t ji = 0; ji < jobs.size(); ++ji)
                for (uint32_t v : nodes[ji])
                    if (excess(v, jobs[ji].rc) > 0) hot[v] = 1;
            for (size_t v = 0; v < NL * n; ++v) if (hot[v]) { ++over_nodes; gh[v] += 0.5f; }
            if (!over_nodes) break;
        }
        const int M = o.gmargin;
        for (size_t ji = 0; ji < jobs.size(); ++ji) {
            Job& J = jobs[ji];
            J.guide.assign(NL * n, 0);
            for (uint32_t v : nodes[ji]) {
                int l = int(v / n), gi = int(v % n) % gnx, gj = int(v % n) / gnx;
                for (int dj = -M; dj <= M; ++dj)
                    for (int di = -M; di <= M; ++di) {
                        int ni = gi + di, nj = gj + dj;
                        if (ni >= 0 && nj >= 0 && ni < gnx && nj < gny) J.guide[size_t(l) * n + size_t(nj) * gnx + ni] = 1;
                    }
            }
        }
        if (o.verbose)
            std::fprintf(stderr, "global route: %dx%d gcells of %.1f mm, %d rounds, %d congested gcell-layers left, %.2f s\n", gnx, gny,
                         GS * G.s / 1000, it + 1, over_nodes, now() - t0);
    }

    void build_static(RClass& rc) {
        rc.st.assign(NL, std::vector<uint16_t>(G.n(), FREE));
        rc.vst.assign(G.n(), FREE);
        rc.oc.assign(NL, std::vector<uint8_t>(G.n(), 0));
        rc.ov.assign(NL, std::vector<uint8_t>(G.n(), 0));
        // outside the board, and the edge band
        if (B.has_outline) {
            std::vector<char> in(G.n(), 0);
            raster(G, Shape::polygon(B.outline.poly), 0, [&](uint32_t c) { in[c] = 1; });
            for (size_t c = 0; c < G.n(); ++c)
                if (!in[c]) { for (int l = 0; l < NL; ++l) rc.st[l][c] = BLOCK; rc.vst[c] = BLOCK; }
            Shape edge = B.outline;
            edge.poly.clear();
            raster(G, edge, rc.hw + o.edge_clearance + mg(rc.hw + o.edge_clearance), [&](uint32_t c) { for (int l = 0; l < NL; ++l) rc.st[l][c] = BLOCK; });
            raster(G, edge, rc.vr + o.edge_clearance + mg(rc.vr + o.edge_clearance), [&](uint32_t c) { rc.vst[c] = BLOCK; });
        }
        for (auto& it : B.items) {
            // a plane is a pour: reserved, only its own net sees it; refilled (--plane-mode refill), it isn't an
            // obstacle at all, and its cells are its net's terminals (make_job) wherever no other copper stands
            if (it.kind == Kind::Plane) {
                if (it.net >= 0 && it.layer >= 0 && !o.plane_refill)
                    raster(G, it.shape, 0, [&](uint32_t q) { if (rc.st[it.layer][q] == FREE) rc.st[it.layer][q] = uint16_t(it.net + 1); });
                continue;
            }
            uint16_t v = it.net < 0 ? BLOCK : uint16_t(it.net + 1);
            double c = std::max(rc.clr, clr_of(it.net)) + CLR_SAFETY;
            for (int l = 0; l < NL; ++l) {
                if (it.layer >= 0 && it.layer != l) continue;
                raster(G, it.shape, rc.hw + c + mg(rc.hw + c), [&](uint32_t q) { merge(rc.st[l][q], v); });
            }
            raster(G, it.shape, rc.vr + c + mg(rc.vr + c), [&](uint32_t q) { merge(rc.vst[q], v); });
            // no via in a pad, nor overlapping a drilled pad or via of any net (hole to hole spacing)
            const double rh = rc.vr + (it.smd ? 0 : 100);
            if (it.smd || it.kind == Kind::Via || it.kind == Kind::Pad) raster(G, it.shape, rh + mg(rh), [&](uint32_t q) { rc.vst[q] = BLOCK; });
        }
        for (auto& k : B.keepouts) {
            if (k.wires)
                for (int l = 0; l < NL; ++l)
                    if (k.layer < 0 || k.layer == l) raster(G, k.shape, rc.hw + mg(rc.hw), [&](uint32_t q) { rc.st[l][q] = BLOCK; });
            if (k.vias) raster(G, k.shape, rc.vr + mg(rc.vr), [&](uint32_t q) { rc.vst[q] = BLOCK; });
        }
    }

    // connected components of a net's existing copper: comp[k] for N.items[k]; returns their number
    int components(int net, std::vector<int>& comp) const {
        const Net& N = B.nets[net];
        size_t n = N.items.size();
        std::vector<int> up(n);
        std::iota(up.begin(), up.end(), 0);
        std::function<int(int)> find = [&](int a) { return up[a] == a ? a : up[a] = find(up[a]); };
        for (size_t a = 0; a < n; ++a)
            for (size_t b = a + 1; b < n; ++b) {
                const Item &A = B.items[N.items[a]], &Bi = B.items[N.items[b]];
                bool join = A.group == Bi.group;
                if (!join && (A.layer < 0 || Bi.layer < 0 || A.layer == Bi.layer) &&
                    A.shape.box().overlaps(Bi.shape.box(), 2))
                    join = shape_dist(A.shape, Bi.shape) <= 1;
                if (join) up[find(int(a))] = find(int(b));
            }
        std::vector<int> root_comp(n, -1);
        int k = 0;
        comp.assign(n, 0);
        for (size_t a = 0; a < n; ++a) {
            int r = find(int(a));
            if (root_comp[r] < 0) root_comp[r] = k++;
            comp[a] = root_comp[r];
        }
        return k;
    }

    void make_job(int net, int rc) {
        const Net& N = B.nets[net];
        RClass& R = rcs[rc];
        uint16_t me = uint16_t(net + 1);
        std::vector<int> comp;
        Job J;
        J.net = net;
        J.rc = rc;
        J.comps.resize(components(net, comp));
        for (size_t a = 0; a < N.items.size(); ++a) {
            Comp& C = J.comps[comp[a]];
            const Item& it = B.items[N.items[a]];
            C.box.add(it.shape.box());
            size_t before = C.states.size();
            for (int l = 0; l < NL; ++l) {
                if ((it.layer >= 0 && it.layer != l) || (!R.lay[l] && it.kind != Kind::Plane)) continue;
                // a via onto a pour of a layer this class doesn't route only counts well inside the pour's outline,
                // where its fill reliably reaches (tracks may land on the rim: tier 1 polygons leave only a rim free)
                // Refilled (--plane-mode refill), a track may end only 0.5 mm inside the outline as well: KiCad fills a
                // zone with filleted corners (0.5 mm radius here) and joins a track to it by its end point
                // With the zone's rules from the DSN (make_dsn.py), refilled: half its minimum width inside the
                // outline (where the fill reaches but for its filleted corners, which are cut from the terminals too)
                std::unordered_set<uint32_t> rim;
                if (it.kind == Kind::Plane && (!R.lay[l] || o.plane_refill)) {
                    Shape e = it.shape;
                    e.poly.clear();
                    const bool rules = o.plane_refill && it.zone >= 0;
                    const double m = rules ? pour_min_width(B, o, it) / 2 + G.s : 500;
                    raster(G, e, R.lay[l] ? m : R.vr + m, [&](uint32_t q) { rim.insert(q); });
                    if (rules) fillet_cuts(G, it.shape.poly, B.zones[it.zone].fillet, R.lay[l] ? m : R.vr + m, [&](uint32_t q) { rim.insert(q); });
                }
                // refilled, a pour gives way where another net's pour wins the overlap on its layer (pour_wins), and
                // with the zone rules, also within their clearance of it, where the fill is knocked out
                if (it.kind == Kind::Plane && o.plane_refill)
                    for (auto& z : B.items)
                        if (z.kind == Kind::Plane && z.net != it.net && z.layer == l && pour_wins(B, z, it) && z.shape.box().overlaps(it.shape.box())) {
                            const double c = it.zone >= 0 ? std::max({clr_of(it.net), clr_of(z.net), pour_clearance(B, o, it), pour_clearance(B, o, z)}) +
                                                                pour_min_width(B, o, it) / 2 : 0;
                            raster(G, z.shape, c, [&](uint32_t q) { rim.insert(q); });
                        }
                raster(G, it.shape, 0, [&](uint32_t q) {
                    if (ok(R.st[l][q], me) && !rim.count(q)) C.states.push_back(uint32_t(l) * G.n() + q);
                });
            }
            if (C.states.size() == before && it.kind == Kind::Pad) {
                // no free cell centre inside the pad (fine pitch): escape off-grid, straight out of the pad
                // centre in each of 8 directions to the first free cell, checked against the exact geometry; if
                // the class's width can't get out, neck down (a narrower stub) in steps to --neck-min
                const double hw_min = std::min(R.hw, std::max(o.neck_min / 2, R.hw / 2)), step = std::max(5.0, R.hw / 10);
                std::vector<double> hws{R.hw};
                for (double h = R.hw - step; h > hw_min + 1e-6; h -= step) hws.push_back(h);
                if (hw_min < R.hw) hws.push_back(hw_min);
                for (double hw : hws) {
                    if (C.states.size() > before) break;
                    for (int l = 0; l < NL; ++l) {
                        if ((it.layer >= 0 && it.layer != l) || !R.lay[l]) continue;
                        for (int d = 0; d < 8; ++d) {
                            P dir{std::cos(d * M_PI / 4), std::sin(d * M_PI / 4)};
                            bool got = false;
                            for (double t = 0; t <= 3000 && !got; t += G.s / 2) {
                                P e = it.center + dir * t;
                                if (t > 0 && !seg_clear(l, it.center, e, hw, R.clr, net)) break;
                                int ci = int(std::floor((e.x - G.x0) / G.s - 0.5)), cj = int(std::floor((e.y - G.y0) / G.s - 0.5));
                                for (int k = 0; k < 4 && !got; ++k) {
                                    int i = ci + (k & 1), j = cj + (k >> 1);
                                    if (i < 0 || j < 0 || i >= G.nx || j >= G.ny) continue;
                                    uint32_t q = uint32_t(j) * G.nx + i;
                                    if (!ok(R.st[l][q], me) || !seg_clear(l, e, G.center(i, j), hw, R.clr, net)) continue;
                                    uint32_t sq = uint32_t(l) * G.n() + q;
                                    std::vector<P> stub{G.center(i, j)};
                                    if (t > 0) stub.push_back(e);
                                    stub.push_back(it.center);
                                    C.states.push_back(sq);
                                    C.attach.push_back({sq, stub, hw});
                                    got = true;
                                }
                            }
                        }
                    }
                    if (hw < R.hw && C.states.size() > before) ++necked;
                }
            }
        }
        // terminals no route can reach
        for (size_t c = 0; c < J.comps.size() && J.comps.size() > 1;)
            if (J.comps[c].states.empty()) {
                ++J.dead;
                if (o.verbose) {
                    P p = J.comps[c].box.empty() ? P{} : P{(J.comps[c].box.x0 + J.comps[c].box.x1) / 2, (J.comps[c].box.y0 + J.comps[c].box.y1) / 2};
                    std::fprintf(stderr, "  %s: no way into the terminal at (%.2f, %.2f) mm\n", N.name.c_str(), p.x / 1000, -p.y / 1000);
                }
                J.comps.erase(J.comps.begin() + c);
            } else ++c;
        dead_total += J.dead;
        if (dead_by_net.size() <= size_t(J.net)) dead_by_net.resize(B.nets.size(), 0);
        dead_by_net[size_t(J.net)] += J.dead;
        std::sort(J.comps.begin(), J.comps.end(), [](const Comp& a, const Comp& b) { return a.states.size() > b.states.size(); });
        for (auto& C : J.comps) J.box.add(C.box);
        J.stamps.resize(rcs.size() * 2 * NL);
        if (J.comps.size() > 1) jobs.push_back(std::move(J));
    }

    // Trim dangling tails (a kept piece whose cut end the reconnection didn't use, or what a dropped path left
    // hanging): a path end must sit on a terminal or on another path. Then the junctions and stubs: the ends of the
    // paths.
    void tidy_paths(Job& J) const {
        {
            std::unordered_set<uint32_t> term;
            for (auto& C : J.comps) term.insert(C.states.begin(), C.states.end());
            std::unordered_map<uint32_t, int> cnt;
            for (auto& p : J.paths) for (uint32_t x : p) ++cnt[x];
            bool changed = true;
            while (changed) {
                changed = false;
                for (auto& p : J.paths) {
                    auto anchored = [&](uint32_t x) { return term.count(x) || cnt[x] > 1; };
                    while (p.size() >= 2 && !anchored(p.back())) { --cnt[p.back()]; p.pop_back(); changed = true; }
                    while (p.size() >= 2 && !anchored(p.front())) { --cnt[p.front()]; p.erase(p.begin()); changed = true; }
                    if (p.size() < 2 && !p.empty()) { --cnt[p[0]]; p.clear(); changed = true; }
                }
                J.paths.erase(std::remove_if(J.paths.begin(), J.paths.end(), [](auto& p) { return p.empty(); }), J.paths.end());
            }
        }
        J.keep.clear();
        std::unordered_set<uint32_t> used;
        for (auto& p : J.paths) { used.insert(p.front()); used.insert(p.back()); J.keep.push_back(p.front()); J.keep.push_back(p.back()); }
        std::vector<Stub> st;
        for (auto& a : J.stubs) if (used.count(a.s)) st.push_back(a);
        std::sort(st.begin(), st.end(), [](auto& a, auto& b) { return a.s < b.s; });
        st.erase(std::unique(st.begin(), st.end(), [](auto& a, auto& b) { return a.s == b.s; }), st.end());
        J.stubs = st;
    }

    // What still overlaps another net after legalizing: drop just the paths that run over other nets' copper (and
    // those ending on an escape stub that does), keep the rest of the net, and count the connections that leaves
    // open. Returns the paths dropped.
    int drop_conflicts(Job& J) {
        const size_t N = G.n();
        const RClass& R = rcs[J.rc];
        auto bad_cell = [&](const std::vector<uint32_t>& path, size_t k) {
            int l = int(path[k] / N);
            uint32_t c = uint32_t(path[k] % N);
            bool bad = R.oc[l][c] > 1;
            if (k + 1 < path.size() && path[k + 1] / N != path[k] / N)
                for (int l2 = 0; l2 < NL; ++l2) bad |= R.ov[l2][c] > 1;
            return bad;
        };
        std::unordered_set<uint32_t> bad_stub;
        for (auto& [s, pts, hw] : J.stubs) {
            int l = int(s / N);
            bool bad = false;
            for (size_t k = 1; k < pts.size() && !bad; ++k) {
                double len = norm(pts[k] - pts[k - 1]);
                for (double t = 0; t <= len && !bad; t += G.s / 2) {
                    P p = pts[k - 1] + (pts[k] - pts[k - 1]) * (len > 0 ? t / len : 0);
                    int i = int((p.x - G.x0) / G.s), j = int((p.y - G.y0) / G.s);
                    if (i >= 0 && j >= 0 && i < G.nx && j < G.ny && R.oc[l][uint32_t(j) * G.nx + i] > 1) bad = true;
                }
            }
            if (bad) bad_stub.insert(s);
        }
        size_t before = J.paths.size();
        J.paths.erase(std::remove_if(J.paths.begin(), J.paths.end(), [&](const std::vector<uint32_t>& p) {
            if (bad_stub.count(p.front()) || bad_stub.count(p.back())) return true;
            for (size_t k = 0; k < p.size(); ++k) if (bad_cell(p, k)) return true;
            return false;
        }), J.paths.end());
        int dropped = int(before - J.paths.size());
        tidy_paths(J);
        // the terminals the paths left still join: one open connection per group but one
        int nc = int(J.comps.size()), ne = nc + int(J.paths.size());
        std::vector<int> up(ne);
        std::iota(up.begin(), up.end(), 0);
        auto find = [&](int a) { while (up[a] != a) a = up[a] = up[up[a]]; return a; };
        std::unordered_map<uint32_t, int> owner;
        for (int e = 0; e < ne; ++e)
            for (uint32_t x : e < nc ? J.comps[e].states : J.paths[e - nc]) {
                auto [it, fresh] = owner.emplace(x, e);
                if (!fresh) up[find(e)] = find(it->second);
            }
        std::unordered_set<int> roots;
        for (int e = 0; e < nc; ++e) roots.insert(find(e));
        J.unrouted = int(roots.size()) - 1 + J.dead;
        restamp(J);
        return dropped;
    }

    // ---- A* ----
    // Route a job. If it already has paths (a reroute), keep them except around the stretches that run over other
    // nets' footprints (cut out with `REPAIR` cells to spare on each side) and only reconnect what that cut apart.
    bool route_job(Job& J, Work& W, float pres, bool hard) {
        const RClass& R = rcs[J.rc];
        // net types (--nt-*): this net's via cost, how much it gives way to congestion, the pair track it follows
        const double VC = job_via_cost(J);
        const float ikc = !nt.empty() && nt_fast(nt[J.net].type) ? float(1.0 / o.nt_hs_len) : 1.f;
        const Job* PJ = J.pair >= 0 && J.pair_ok && !jobs[J.pair].paths.empty() ? &jobs[J.pair] : nullptr;
        const float pw = 1.f + float(o.nt_pair);
        const bool xz = !xzone.empty() && nt[J.net].type != NetType::Crystal;
        std::vector<uint64_t> nearb;  // the pair track's surroundings, as bits of the window
        auto damp = [ikc](float c) { return ikc != 1.f && c > 1.f ? 1.f + (c - 1.f) * ikc : c; };
        uint16_t me = uint16_t(J.net + 1);
        const size_t N = G.n();
        // the cut widens while the net stays in conflict: 2, 4, then 8 mm each side
        const int REPAIR = sx && sx->repair >= 0 ? sx->repair : std::max(4, int(2000 / G.s)) << std::min(J.stuck / 2, 2);
        const bool soft = sx && sx->soft;  // shove: other nets' clearance can be squeezed into, at a price (see shove.inc)
        std::vector<std::vector<uint32_t>> pieces;
        if (W.touch.size() != BN * NL) { W.touch.assign(BN * NL, 0); W.touchv.assign(BN, 0); }
        for (auto& path : J.paths) {
            int f = -1, b = -1;
            for (size_t k = 0; k < path.size(); ++k) {
                int l = int(path[k] / N);
                uint32_t c = uint32_t(path[k] % N);
                W.touch[size_t(l) * BN + size_t(c / G.nx / BS) * bw + c % G.nx / BS] = 1;  // these reads count too (run)
                W.touchv[size_t(c / G.nx / BS) * bw + c % G.nx / BS] = 1;
                bool bad = ld(R.oc[l][c]) > 1;  // the net's own stamp counts 1
                if (k + 1 < path.size() && path[k + 1] / N != path[k] / N)
                    for (int l2 = 0; l2 < NL; ++l2) bad |= ld(R.ov[l2][c]) > 1;
                if (sx && sx->cut.count(path[k])) bad = true;  // shove: this cell puts a frozen net in conflict
                if (bad) { if (f < 0) f = int(k); b = int(k); }
            }
            if (f < 0) { pieces.push_back(std::move(path)); continue; }
            if (f - REPAIR > 0) pieces.emplace_back(path.begin(), path.begin() + (f - REPAIR));
            if (b + REPAIR + 1 < int(path.size())) pieces.emplace_back(path.begin() + (b + REPAIR + 1), path.end());
        }
        J.paths.clear(); J.keep.clear(); J.stubs.clear(); J.unrouted = J.dead;
        for (auto& C : J.comps) J.stubs.insert(J.stubs.end(), C.attach.begin(), C.attach.end());
        // group the terminals and the kept pieces into connected groups; pieces cut loose from every terminal go
        int nc = int(J.comps.size()), ne = nc + int(pieces.size());
        std::vector<int> up(ne);
        std::iota(up.begin(), up.end(), 0);
        auto find = [&](int a) { while (up[a] != a) a = up[a] = up[up[a]]; return a; };
        std::unordered_map<uint32_t, int> owner;
        for (int e = 0; e < ne; ++e) {
            const auto& st = e < nc ? J.comps[e].states : pieces[e - nc];
            for (uint32_t x : st) {
                auto [it, fresh] = owner.emplace(x, e);
                if (!fresh) up[find(e)] = find(it->second);
            }
        }
        struct Grp { std::vector<uint32_t> states; Box box; bool terminal = false; };
        std::vector<int> gid(ne, -1);
        std::vector<Grp> grp;
        for (int e = 0; e < ne; ++e) {
            int r = find(e);
            if (gid[r] < 0) { gid[r] = int(grp.size()); grp.emplace_back(); }
            Grp& g = grp[gid[r]];
            if (e < nc) {
                g.terminal = true;
                g.states.insert(g.states.end(), J.comps[e].states.begin(), J.comps[e].states.end());
                g.box.add(J.comps[e].box);
            } else
                for (uint32_t x : pieces[e - nc]) { g.states.push_back(x); g.box.add(G.center(uint32_t(x % N)), 0); }
        }
        for (int e = nc; e < ne; ++e)
            if (grp[gid[find(e)]].terminal) J.paths.push_back(std::move(pieces[e - nc]));
        // this net's vias so far: a new one keeps its drill (taken as half the via diameter) plus 0.25 mm from
        // them, KiCad's default hole to hole spacing
        std::vector<uint32_t> myvias;
        auto note_vias = [&](const std::vector<uint32_t>& p) {
            for (size_t k = 0; k + 1 < p.size(); ++k)
                if (p[k] / N != p[k + 1] / N) myvias.push_back(uint32_t(p[k] % N));
        };
        for (auto& p : J.paths) note_vias(p);
        const double via_gap2 = std::pow((R.vr + 250) / G.s, 2);
        const int via_gapc = int(std::ceil((R.vr + 250) / G.s));  // ...and along one path, counted in steps
        std::vector<uint32_t> tree = grp[gid[find(0)]].states;
        std::vector<int> left;
        for (size_t c = 0; c < grp.size(); ++c)
            if (int(c) != gid[find(0)] && grp[c].terminal) left.push_back(int(c));
        bool gd = !J.guide.empty() && !sx;  // stay in the global-route corridor
        // learned prior: steps where the net isn't expected cost up to 1 + ml_weight times more; with a corridor,
        // the search stays where the prior is at least ml_corridor (targets always count) until that's a dead end
        const NetPrior* mp = J.ml && (o.ml_weight > 0 || o.ml_corridor > 0) && !sx ? J.ml : nullptr;
        bool mc = mp && o.ml_corridor > 0;
        const uint8_t mthr = uint8_t(std::clamp(std::ceil(o.ml_corridor * 255), 0.0, 255.0));
        float mlf[256];
        const float sat = float(std::clamp(o.ml_sat, 0.01, 1.0));
        for (int v = 0; v < 256; ++v) mlf[v] = mp ? 1.f + float(o.ml_weight) * (1.f - std::min(1.f, v / 255.f / sat)) : 1.f;
        const size_t GNn = GN();
        // what this search reads, by block, so that a wave can tell whether another net's change could have
        // altered its result (see run)
        if (W.touch.size() != BN * NL) { W.touch.assign(BN * NL, 0); W.touchv.assign(BN, 0); }
        auto blk = [&](int l, int gi, int gj) { return size_t(l) * BN + size_t(gj / BS) * bw + gi / BS; };
        const bool anyown = std::any_of(J.stamps.begin(), J.stamps.end(), [](auto& v) { return !v.empty(); });
        const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1}, dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
        const float len[8] = {float(G.s), float(G.s), float(G.s), float(G.s), float(G.s * SQ2), float(G.s * SQ2),
                              float(G.s * SQ2), float(G.s * SQ2)};
        // per layer, cost factor of each move: F.Cu prefers x, B.Cu prefers y, or the other way with --layer-dir yx
        // (diagonals pay half the surcharge)
        float dirf[8][8];
        for (int ll = 0; ll < 8; ++ll) {
            int pref = ll < NL ? (B.layers[ll] == "F.Cu" ? 1 : B.layers[ll] == "B.Cu" ? 2 : 0) : 0;
            if (pref && o.f_cu_y) pref = 3 - pref;
            float w = float(o.wrong_way), wd = 1.f + (w - 1.f) / 2;
            for (int d = 0; d < 8; ++d)
                dirf[ll][d] = !pref ? 1.f : d >= 4 ? wd : ((d < 2) == (pref == 1)) ? 1.f : w;
        }
        // the search window: the net's box plus the margin, then 4 times the margin, then everything. Set up once
        // per stage: its arrays, the net's own footprint and the tree as bits (a plane makes a tree of millions of
        // states; it is grown as the tree grows, and the searches run from the pieces towards it)
        int stage = 0, cur_stage = -1;
        int wi0 = 0, wj0 = 0, wi1 = 0, wj1 = 0, ww = 0, wh = 0, CS = 1, cw = 0, ch = 0;
        size_t WN = 0, WS = 0, WB = 0, CN = 0, tree_in = 0;
        bool anyownv = false, bigtree = false;
        std::vector<uint32_t> tst;
        auto loc = [&](uint32_t s) -> int64_t {
            int l = int(s / N);
            uint32_t c = uint32_t(s % N);
            int i = int(c % G.nx) - wi0, j = int(c / G.nx) - wj0;
            if (i < 0 || j < 0 || i >= ww || j >= wh) return -1;
            return (int64_t(l) * wh + j) * ww + i;
        };
        auto coarse = [&](uint32_t q) { return size_t(q / WN) * CN + size_t((q % WN) / ww / CS) * cw + (q % ww) / CS; };
        auto add_tree = [&](uint32_t s) {
            int64_t q = loc(s);
            if (q < 0) return;
            ++tree_in;
            W.treeb[q >> 6] |= uint64_t(1) << (q & 63);
            size_t k = coarse(uint32_t(q));
            if (!W.tcmask[k]) { W.tcmask[k] = 1; W.tcz.push_back(uint32_t(k)); }
            if (!bigtree) tst.push_back(uint32_t(q));
        };
        auto ownbit = [&](size_t q) { return anyown && ((W.own[q >> 6] >> (q & 63)) & 1); };
        // the cost of a cell for this net: the map, or recomputed without its own stamp
        auto cm_at = [&](int l, uint32_t c, size_t q) -> float {
            float v = ownbit(q) ? cost_x(R, l, c, 1) : ld(R.cst[l][c]);
            if (sx && sx->frozen(J.rc, l, c)) v = INF;
            else if (soft && !(v < INF)) { float m = sx->over(l, c); v = m < INF ? cost_xp(R, l, c, ownbit(q) ? 1 : 0, sx->pres * m, false) : INF; }
            return v;
        };
        auto vm_at = [&](uint32_t c, int i, int j) -> float {
            float vm = ld(R.vcst[c]);
            int own = 0;
            if (anyownv) {
                for (int l2 = 0; l2 < NL; ++l2) {
                    size_t q = (size_t(l2) * wh + j) * ww + i;
                    own += int((W.ownv[q >> 6] >> (q & 63)) & 1);
                }
                if (own) vm = vcost_x(R, c, own);
            }
            if (sx && sx->frozen(J.rc, NL, c)) vm = INF;
            else if (soft && !(vm < INF)) { float m = sx->over(NL, c); vm = m < INF ? vcost_xp(R, c, own, sx->pres * m, false) : INF; }
            return vm;
        };
        while (!left.empty()) {
            double tss = now();
            if (stage != cur_stage) {
                cur_stage = stage;
                Box win = J.box.grown(stage == 0 ? o.window : 4 * o.window);
                if (o.budget_scale != 1.0 && J.difficulty > 0) win = J.box.grown((stage == 0 ? o.window : 4 * o.window) * (1 + (o.budget_scale - 1) * J.difficulty));
                wi0 = std::max(0, int((win.x0 - G.x0) / G.s)); wj0 = std::max(0, int((win.y0 - G.y0) / G.s));
                wi1 = std::min(G.nx - 1, int((win.x1 - G.x0) / G.s)); wj1 = std::min(G.ny - 1, int((win.y1 - G.y0) / G.s));
                if (stage >= 2) { wi0 = wj0 = 0; wi1 = G.nx - 1; wj1 = G.ny - 1; }
                if (sx && sx->has_win) { wi0 = sx->wi0; wj0 = sx->wj0; wi1 = sx->wi1; wj1 = sx->wj1; }  // shove: a local window
                ww = wi1 - wi0 + 1; wh = wj1 - wj0 + 1;
                WN = size_t(ww) * wh; WS = WN * NL; WB = WS / 64 + 1;
                if (W.nd.size() < WS) {
                    if (o.verbose > 2) std::fprintf(stderr, "    window grows to %zu states (net %s, stage %d)\n", WS, B.nets[J.net].name.c_str(), stage);
                    W.nd.assign(WS, Node{0.f, 0, 0, 0}); W.tbit.assign(WB, 0); W.own.assign(WB, 0); W.ownv.assign(WB, 0);
                    W.epoch = 0;
                }
                // this net's own footprint (the search sees the board as if it weren't there): bits over the window
                anyownv = false;
                if (anyown) {
                    std::fill(W.own.begin(), W.own.begin() + WB, 0);
                    std::fill(W.ownv.begin(), W.ownv.begin() + WB, 0);
                    for (int l = 0; l < NL; ++l)
                        for (int kind = 0; kind < 2; ++kind)
                            for (auto& sp : J.stamps[(J.rc * 2 + kind) * NL + l]) {
                                if (sp.j < wj0 || sp.j > wj1) continue;
                                int a = std::max(sp.i0, wi0), b = std::min(sp.i1, wi1);
                                auto& bits = kind ? W.ownv : W.own;
                                for (int i = a; i <= b; ++i) {
                                    size_t q = (size_t(l) * wh + (sp.j - wj0)) * ww + (i - wi0);
                                    bits[q >> 6] |= uint64_t(1) << (q & 63);
                                }
                                anyownv |= kind && a <= b;
                            }
                }
                if (PJ) {  // within the pair distance of the first net's track, on its layer
                    nearb.assign(WB, 0);
                    const RClass& PR = rcs[PJ->rc];
                    const int rr = int(std::ceil((o.nt_pair_dist > 0 ? o.nt_pair_dist : 2 * (2 * std::max(R.hw, PR.hw) + std::max(R.clr, PR.clr))) / G.s));
                    for (auto& p : PJ->paths)
                        for (uint32_t st : p) {
                            int l = int(st / N), ci = int(st % N % G.nx) - wi0, cj = int(st % N / G.nx) - wj0;
                            for (int dj = -rr; dj <= rr; ++dj) {
                                int j = cj + dj;
                                if (j < 0 || j >= wh) continue;
                                int hw = int(std::sqrt(double(rr * rr - dj * dj)));
                                for (int i = std::max(0, ci - hw); i <= std::min(ww - 1, ci + hw); ++i) {
                                    size_t q = (size_t(l) * wh + j) * ww + i;
                                    nearb[q >> 6] |= uint64_t(1) << (q & 63);
                                }
                            }
                        }
                }
                // the heuristic's coarse grid (see search), and the tree
                CS = WN <= 160000 ? 1 : WN <= 640000 ? 2 : WN <= 2500000 ? 4 : 8;
                cw = (ww + CS - 1) / CS; ch = (wh + CS - 1) / CS;
                CN = size_t(cw) * ch;
                if (W.treeb.size() < WB) W.treeb.resize(WB);
                std::fill(W.treeb.begin(), W.treeb.begin() + WB, 0);
                W.tcmask.assign(CN * NL, 0);
                W.tcz.clear();
                tst.clear();
                tree_in = 0;
                bigtree = tree.size() > 20000;
                for (uint32_t s : tree) add_tree(s);
            }
            // the pieces left to join (their group), as window states
            W.tcomp.clear();
            std::vector<uint32_t> gst;
            for (int c : left)
                for (uint32_t s : grp[c].states)
                    if (int64_t q = loc(s); q >= 0) { gst.push_back(uint32_t(q)); W.tcomp[uint32_t(q)] = c; }
            W.gcz.clear();
            {
                std::vector<uint8_t> seen(CN * NL, 0);
                for (uint32_t q : gst) { size_t k = coarse(q); if (!seen[k]) { seen[k] = 1; W.gcz.push_back(uint32_t(k)); } }
            }
            // few pieces: the search from the tree gets an exact heuristic, the cheapest walk to the nearest piece's
            // box on its layer (the lowest direction costs of any layer, plus a via when the box is on another)
            struct TBox { int l, i0, j0, i1, j1; };
            std::vector<TBox> tboxes;
            if (left.size() <= 4) {
                std::vector<TBox> per(size_t(left.size()) * NL, TBox{-1, 1 << 30, 1 << 30, -1, -1});
                for (size_t k = 0; k < left.size(); ++k)
                    for (uint32_t s : grp[left[k]].states) {
                        int64_t q = loc(s);
                        if (q < 0) continue;
                        int l = int(q / WN), i = int(q % ww), j = int((q / ww) % wh);
                        TBox& b = per[k * NL + l];
                        b.l = l; b.i0 = std::min(b.i0, i); b.i1 = std::max(b.i1, i); b.j0 = std::min(b.j0, j); b.j1 = std::max(b.j1, j);
                    }
                for (auto& b : per) if (b.l >= 0) tboxes.push_back(b);
            }
            float wxm = INF, wym = INF, wdm = INF;
            for (int l = 0; l < NL && l < 8; ++l)
                if (R.lay[l]) { wxm = std::min(wxm, dirf[l][0]); wym = std::min(wym, dirf[l][2]); wdm = std::min(wdm, dirf[l][4] * float(SQ2)); }
            auto glob = [&](int64_t q) {
                int i = int(q % ww), j = int((q / ww) % wh), l = int(q / int64_t(WN));
                return uint32_t(l) * N + uint32_t(j + wj0) * G.nx + (i + wi0);
            };
#ifdef PCB_GPU
            const bool shove_win = bool(sx);   // shove mode, read here: inside the search, sx is the grid step
#endif
            // One A* search from src (window states) to the states whose bits are set in dstb (dstz: the coarse
            // cells they cover) within a pop budget: the state of dst reached (its chain in the nodes' par), or -1.
            // `exhausted`: every reachable state was searched, nothing of dst can be reached.
            auto search = [&](const std::vector<uint32_t>& src, const uint64_t* dstb, const std::vector<uint32_t>& dstz, size_t budget, bool& exhausted, bool fwd) -> int64_t {
                if (++W.epoch == 0) { std::fill(W.nd.begin(), W.nd.end(), Node{0.f, 0, 0, 0}); W.epoch = 1; }  // 16-bit epochs
                auto is_target = [&](uint32_t q) { return (dstb[q >> 6] >> (q & 63)) & 1; };
                const bool exact = fwd && !tboxes.empty();
                const float vcx = R.via >= 0 ? float(VC) : INF, sx = float(G.s), hwx = float(o.hweight);
                auto hexact = [&](int l, int i, int j) {
                    float best = INF;
                    for (auto& b : tboxes) {
                        int dx = std::max({b.i0 - i, i - b.i1, 0}), dy = std::max({b.j0 - j, j - b.j1, 0}), dm = std::min(dx, dy);
                        float v = (dm * wdm + (dx - dm) * wxm + (dy - dm) * wym) * sx + (b.l == l ? 0.f : vcx);
                        best = std::min(best, v);
                    }
                    return best * hwx;
                };
                // Heuristic: per layer, the cheapest 8-connected walk on an empty grid (this layer's direction
                // costs) to dst on that layer, from a chamfer distance map at CS x CS cells per entry, lowered by
                // what a point can gain by being off its coarse cell's centre; and the nearest dst on any layer
                // plus a via. Every step costs at least its length times the direction factor, so this never
                // overestimates.
                double ts0 = now();
                if (!exact) {
                W.dmap.assign(CN * NL, INF);
                W.hmin.assign(CN, INF);
                for (uint32_t k : dstz) W.dmap[k] = 0;
                // (the map could be computed on the box of src and dst only, with an estimate outside; tried, and
                // the looser outside estimate cost far more pops than the map's time)
                const int bi0 = 0, bi1 = cw - 1, bj0 = 0, bj1 = ch - 1;
                for (int l = 0; l < NL; ++l) {
                    float* D = &W.dmap[size_t(l) * CN];
                    if (R.lay[l]) {
                        const float wx = dirf[l < 8 ? l : 0][0], wy = dirf[l < 8 ? l : 0][2], wd = dirf[l < 8 ? l : 0][4] * float(SQ2);
                        for (int j = bj0; j <= bj1; ++j)
                            for (int i = bi0; i <= bi1; ++i) {
                                float& v = D[size_t(j) * cw + i];
                                if (i > bi0) v = std::min(v, D[size_t(j) * cw + i - 1] + wx);
                                if (j > bj0) {
                                    v = std::min(v, D[size_t(j - 1) * cw + i] + wy);
                                    if (i > bi0) v = std::min(v, D[size_t(j - 1) * cw + i - 1] + wd);
                                    if (i < bi1) v = std::min(v, D[size_t(j - 1) * cw + i + 1] + wd);
                                }
                            }
                        for (int j = bj1; j >= bj0; --j)
                            for (int i = bi1; i >= bi0; --i) {
                                float& v = D[size_t(j) * cw + i];
                                if (i < bi1) v = std::min(v, D[size_t(j) * cw + i + 1] + wx);
                                if (j < bj1) {
                                    v = std::min(v, D[size_t(j + 1) * cw + i] + wy);
                                    if (i < bi1) v = std::min(v, D[size_t(j + 1) * cw + i + 1] + wd);
                                    if (i > bi0) v = std::min(v, D[size_t(j + 1) * cw + i - 1] + wd);
                                }
                            }
                    }
                    const float unit = float(CS * G.s), hw_ = float(o.hweight);
                    const float slack = CS > 1 ? float(std::max(1.0, o.wrong_way) * CS * G.s * 0.75) : 0.f;  // read bilinearly below
                    // outside the box: at least the walk to the box plus the least the map has on its boundary
                    float bmin = INF;
                    for (int j = bj0; j <= bj1; ++j)
                        for (int i = bi0; i <= bi1; i += (j == bj0 || j == bj1 || bi1 == bi0) ? 1 : bi1 - bi0) bmin = std::min(bmin, D[size_t(j) * cw + i]);
                    const float wx = dirf[l < 8 ? l : 0][0], wy = dirf[l < 8 ? l : 0][2], wd = dirf[l < 8 ? l : 0][4] * float(SQ2);
                    for (int j = 0; j < ch; ++j) {
                        float* row = D + size_t(j) * cw;
                        float* hm = W.hmin.data() + size_t(j) * cw;
                        int dy = std::max({bj0 - j, j - bj1, 0});
                        for (int i = 0; i < cw; ++i) {
                            float v;
                            if (i < bi0 || i > bi1 || dy) {
                                int dx = std::max({bi0 - i, i - bi1, 0}), dm = std::min(dx, dy);
                                v = bmin < INF ? bmin + dm * wd + (dx - dm) * wx + (dy - dm) * wy : INF;
                            } else v = row[i];
                            v = v < INF ? std::max(0.f, v * unit - slack) * hw_ : INF;
                            row[i] = v;
                            hm[i] = std::min(hm[i], v);
                        }
                    }
                }
                {
                    const float vc = R.via >= 0 ? float(VC * o.hweight) : INF;
                    for (auto& v : W.hmin) v = v < INF ? v + vc : INF;
                }
                }
                const float* dmap = W.dmap.data();
                const float* hmin = W.hmin.data();
                auto bilin = [&](const float* M, int l, int i, int j) -> float {
                    if (CS == 1) return M[size_t(l) * CN + size_t(j) * cw + i];
                    float fx = (i + 0.5f) / CS - 0.5f, fy = (j + 0.5f) / CS - 0.5f;
                    int ix = std::clamp(int(std::floor(fx)), 0, std::max(cw - 2, 0)), iy = std::clamp(int(std::floor(fy)), 0, std::max(ch - 2, 0));
                    float tx = std::clamp(fx - ix, 0.f, 1.f), ty = std::clamp(fy - iy, 0.f, 1.f);
                    int ix1 = std::min(ix + 1, cw - 1), iy1 = std::min(iy + 1, ch - 1);
                    const float* P = M + size_t(l) * CN;
                    float a = P[size_t(iy) * cw + ix], b = P[size_t(iy) * cw + ix1], c = P[size_t(iy1) * cw + ix], d = P[size_t(iy1) * cw + ix1];
                    if (!(a < INF && b < INF && c < INF && d < INF)) return INF;
                    return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
                };
                auto h = [&](int l, int i, int j) {
                    if (exact) return hexact(l, i, j);
                    return std::min(bilin(dmap, l, i, j), bilin(hmin, 0, i, j));
                };
                double ts1 = now();
                us_dmap += long((ts1 - ts0) * 1e6);
                if (o.verbose > 2) std::fprintf(stderr, "    search: %zu src, CS %d, CN %zu, heuristic %.1f ms\n", src.size(), CS, CN, (ts1 - ts0) * 1e3);
                W.heap.clear();
                Node* nd = W.nd.data();
                for (uint32_t q : src) {
                    int i = int(q % ww), j = int((q / ww) % wh), l = int(q / WN);
                    uint32_t c = uint32_t(j + wj0) * G.nx + (i + wi0);
                    W.touch[blk(l, i + wi0, j + wj0)] = 1;
                    float cm = cm_at(l, c, q);
                    if (!(cm < INF)) continue;  // a start under another net can't be used when legalizing
                    if (cm < 0) { if (R.st[l][c] != me) continue; cm = -cm; }
                    float g0 = float(G.s) * (cm - 1.f);  // a start on another net's footprint costs what stepping onto it would
                    if (nd[q].ep == W.epoch && nd[q].g <= g0 + 1.f) continue;
                    nd[q] = {g0, W.epoch, 255, 255};
                    W.heap.push_back({g0 * TIE + h(l, i, j), g0, q});
                }
                std::make_heap(W.heap.begin(), W.heap.end());
                int64_t found = -1;
                double ts2 = now();
                us_init += long((ts2 - ts1) * 1e6);
                ++n_search;
                const size_t budget0 = budget;
                size_t reexp = 0;
                int dbg = 0;
#ifdef PCB_GPU
                // An expensive search is finished on the GPU: the same costs (the cost maps, the net's own footprint
                // left out, the learned prior and corridor), solved exactly for the whole window by sweeping. Returns
                // the target reached (its chain written into the A* arrays), -1 if no target can be reached, -2 if
                // it didn't work out and the CPU search should carry on, -3 if the GPU is busy (--gpu-nowait).
                bool gpu_try = o.gpu_pops > 0 && !gd && !shove_win && ikc == 1.f && !PJ && !xz && gpu_available(), gpu_none = false;
                size_t gpu_at = o.gpu_pops;
                auto gpu_finish = [&]() -> int64_t {
                    const size_t NS = WN * NL;
                    GpuQuery Q;
                    Q.nx = G.nx; Q.ny = G.ny; Q.NL = NL; Q.wi0 = wi0; Q.wj0 = wj0; Q.ww = ww; Q.wh = wh;
                    Q.me = me; Q.step = float(G.s); Q.via_cost = R.via >= 0 ? float(VC) : -1.f;
                    for (int l = 0; l < 8; ++l) {
                        Q.lay[l] = l < NL ? R.lay[l] : 0;
                        Q.play[l] = l < NL && mp ? play[l] : -1;
                        for (int d = 0; d < 8; ++d) Q.dirf[l][d] = dirf[l][d];
                        if (l < NL) { Q.cst[l] = R.cst[l].data(); Q.st[l] = R.st[l].data(); }
                    }
                    Q.vcst = R.vcst.data();
                    Q.vst = R.vst.data();
                    Q.tbit = dstb;
                    // the net's own cells: their cost without its own stamp (what cm_at / vm_at give the CPU search)
                    if (anyown) {
                        for (int l = 0; l < NL; ++l)
                            for (auto& sp : J.stamps[(J.rc * 2) * NL + l]) {
                                if (sp.j < wj0 || sp.j > wj1) continue;
                                int a = std::max(sp.i0, wi0), b = std::min(sp.i1, wi1);
                                for (int i = a; i <= b; ++i) {
                                    uint32_t c = uint32_t(sp.j) * G.nx + i;
                                    float cm = cost_x(R, l, c, 1);
                                    if (cm < 0) cm = R.st[l][c] == me ? -cm : INF;
                                    Q.own_cf.emplace_back(uint32_t((size_t(l) * wh + (sp.j - wj0)) * ww + (i - wi0)), cm);
                                }
                            }
                        if (anyownv) {
                            std::unordered_map<uint32_t, int> cnt;  // cell -> layers where the net's own via copper is
                            for (int l = 0; l < NL; ++l)
                                for (auto& sp : J.stamps[(J.rc * 2 + 1) * NL + l]) {
                                    if (sp.j < wj0 || sp.j > wj1) continue;
                                    int a = std::max(sp.i0, wi0), b = std::min(sp.i1, wi1);
                                    for (int i = a; i <= b; ++i) ++cnt[uint32_t(sp.j) * G.nx + i];
                                }
                            for (auto& [c, own] : cnt) {
                                float vm = vcost_x(R, c, own);
                                if (vm < 0) vm = R.vst[c] == me ? -vm : INF;
                                Q.own_vcf.emplace_back(uint32_t(size_t(c / G.nx - wj0) * ww + (c % G.nx - wi0)), vm);
                            }
                        }
                    }
                    if (mp) {
                        Q.prior = mp->data.data(); Q.pi0 = mp->i0; Q.pj0 = mp->j0; Q.pw = mp->w; Q.ph = mp->h; Q.pnl = int(prior.layers.size());
                        Q.pcol = pcol.data(); Q.prow = prow.data();
                        for (int v = 0; v < 256; ++v) Q.mlf[v] = mlf[v];
                        Q.mthr = mthr; Q.corridor = mc;
                    }
                    std::unordered_map<uint32_t, float> g0s;
                    for (uint32_t q : src) {
                        int i = int(q % ww), j = int((q / ww) % wh), l = int(q / WN);
                        uint32_t c = uint32_t(j + wj0) * G.nx + (i + wi0);
                        float cm = cm_at(l, c, q);
                        if (!(cm < INF)) continue;
                        if (cm < 0) { if (R.st[l][c] != me) continue; cm = -cm; }
                        float g0 = float(G.s) * (cm - 1.f);
                        auto [it, fresh] = g0s.emplace(q, g0);
                        if (!fresh) it->second = std::min(it->second, g0);
                    }
                    for (auto& [q, g] : g0s) Q.starts.emplace_back(q, g);
                    GpuOut dist;
                    double tg0 = now();
                    int sw = gpu_solve(Q, dist, o.gpu_wait);
                    // the GPU read the whole window (its solved rectangle, at least): those blocks count as read
                    // by this search, so that a commit changing them makes the wave redo the net (see run)
                    if (sw >= 0)
                        for (int gj = (wj0 + dist.ry0) / BS; gj <= (wj0 + dist.ry0 + dist.rh - 1) / BS; ++gj)
                            for (int gi = (wi0 + dist.rx0) / BS; gi <= (wi0 + dist.rx0 + dist.rw - 1) / BS; ++gi) {
                                for (int l = 0; l < NL; ++l) W.touch[size_t(l) * BN + size_t(gj) * bw + gi] = 1;
                                W.touchv[size_t(gj) * bw + gi] = 1;
                            }
                    n_gpu_us += long((now() - tg0) * 1e6);
                    if (sw == -3) return -3;  // every GPU slot is busy: try again a little later
                    if (sw < 0) { ++n_gpu_why[0]; return -2; }
                    n_gpu_sweeps += sw;
                    int64_t best = dist.best;
                    if (best < 0) { ++n_gpu_none; return -1; }
                    // the cost factor of a state as the GPU had it: the CPU's, times the prior's factor
                    auto gcf = [&](int l, int i, int j, uint32_t q) -> float {
                        uint32_t cell = uint32_t(j + wj0) * G.nx + (i + wi0);
                        float cm = cm_at(l, cell, q);
                        if (!(cm < INF)) return INF;
                        if (cm < 0) { if (R.st[l][cell] != me) return INF; cm = -cm; }
                        if (mp) {
                            uint8_t pv = prior_at(mp, l, i + wi0, j + wj0);
                            if (mc && pv < mthr && !is_target(q)) return INF;
                            cm *= mlf[pv];
                        }
                        return cm;
                    };
                    // walk back from the target: at each state take the predecessor that explains its cost
                    const float ln[8] = {float(G.s), float(G.s), float(G.s), float(G.s), float(G.s * SQ2), float(G.s * SQ2), float(G.s * SQ2), float(G.s * SQ2)};
                    std::vector<std::pair<int64_t, uint8_t>> chain;
                    std::vector<uint32_t> viacells;
                    int64_t q = best;
                    int since_via = 1 << 20;  // track steps since the last via met on the chain
                    for (size_t guard = 0; guard < NS; ++guard) {
                        int i = int(q % ww), j = int((q / ww) % wh), l = int(q / int64_t(WN));
                        uint32_t cell = uint32_t(j + wj0) * G.nx + (i + wi0);
                        auto st0 = g0s.find(uint32_t(q));
                        if (st0 != g0s.end() && dist(q) >= st0->second - 1e-3f - 1e-5f * st0->second) { chain.push_back({q, 255}); break; }
                        float bc = 1e30f;
                        int64_t bp = -1;
                        uint8_t bcode = 0;
                        if (R.lay[l]) {
                            float cfq = gcf(l, i, j, uint32_t(q));
                            if (cfq < INF)
                                for (int dm = 0; dm < 8; ++dm) {
                                    int pi = i - di[dm], pj = j - dj[dm];
                                    if (pi < 0 || pj < 0 || pi >= ww || pj >= wh) continue;
                                    int64_t p = int64_t(l) * WN + int64_t(pj) * ww + pi;
                                    float c = dist(p) + ln[dm] * dirf[l < 8 ? l : 0][dm] * cfq;
                                    if (c < bc) { bc = c; bp = p; bcode = uint8_t(dm); }
                                }
                        }
                        if (R.via >= 0) {
                            bool land = R.lay[l] ? (ok(R.st[l][cell], me) && !(mc && !is_target(uint32_t(q)) && prior_at(mp, l, i + wi0, j + wj0) < mthr))
                                                 : bool(is_target(uint32_t(q)));
                            float vm = vm_at(cell, i, j);
                            if (vm < 0) vm = R.vst[cell] == me ? -vm : INF;
                            if (land && vm < INF) {
                                float vc = float(VC) * vm;
                                for (int l2 = 0; l2 < NL; ++l2) {
                                    if (l2 == l) continue;
                                    int64_t p = int64_t(l2) * WN + int64_t(j) * ww + i;
                                    if (dist(p) + vc < bc) { bc = dist(p) + vc; bp = p; bcode = uint8_t(8 + l2); }
                                }
                            }
                        }
                        if (bp < 0 || bc > dist(q) * 1.001f + 1.f) {
                            ++n_gpu_why[1];
                            if (getenv("GPU_DEBUG")) {
                                std::fprintf(stderr, "gpu chain broke at (%d,%d,l%d) d %.1f: best pred %.1f (code %d), cf %.1f, lay %d, target %d, %zu steps in; nbr d:", i, j, l, dist(q), bc, bp < 0 ? -1 : int(bcode), R.lay[l] ? gcf(l, i, j, uint32_t(q)) : -1.f, int(R.lay[l]), int(is_target(uint32_t(q))), chain.size());
                                for (int dm = 0; dm < 8; ++dm) { int pi = i - di[dm], pj = j - dj[dm]; if (pi < 0 || pj < 0 || pi >= ww || pj >= wh) continue; std::fprintf(stderr, " %.1f", dist(int64_t(l) * WN + int64_t(pj) * ww + pi)); }
                                for (int l2 = 0; l2 < NL; ++l2) std::fprintf(stderr, " |l%d %.1f", l2, dist(int64_t(l2) * WN + int64_t(j) * ww + i));
                                std::fprintf(stderr, " ; start? %d\n", int(g0s.count(uint32_t(q))));
                            }
                            return -2;
                        }
                        chain.push_back({q, bcode});
                        if (bcode >= 8) {  // the CPU search keeps a path's vias apart: their holes would touch
                            if (since_via < via_gapc) { ++n_gpu_why[2]; return -2; }
                            since_via = 0;
                            viacells.push_back(cell);
                        } else ++since_via;
                        q = bp;
                    }
                    if (chain.empty() || chain.back().second != 255) { ++n_gpu_why[1]; return -2; }
                    for (uint32_t vc : viacells)  // ...and off its old ones
                        for (uint32_t mv : myvias) {
                            double ddx = double(mv % G.nx) - double(vc % G.nx), ddy = double(mv / G.nx) - double(vc / G.nx);
                            if (mv != vc && ddx * ddx + ddy * ddy < via_gap2) { ++n_gpu_why[3]; return -2; }
                        }
                    for (auto& [s, code] : chain) nd[s] = {dist(s), W.epoch, code, 255};
                    return best;
                };
#endif
                while (!W.heap.empty() && budget--) {
                    std::pop_heap(W.heap.begin(), W.heap.end());
                    HeapItem cur = W.heap.back();
                    W.heap.pop_back();
                    uint32_t q = cur.s;
                    int i = int(q % ww), j = int((q / ww) % wh), l = int(q / WN);
                    float gq = nd[q].g;
                    if (cur.g > gq) continue;  // stale
#ifdef PCB_GPU
                    if (gpu_try && budget0 - budget > gpu_at) {
                        int64_t r = gpu_finish();
                        if (r == -3) gpu_at += 200000;
                        else gpu_try = false;
                        if (r >= 0) { found = r; ++n_gpu; break; }
                        if (r == -2) ++n_gpu_fail;
                        if (r == -1) {
                            if (getenv("GPU_CHECK")) gpu_none = true;  // carry on: the CPU search must then exhaust too
                            else { W.heap.clear(); break; }
                        }
                    }
#endif
                    ++J.pops; ++n_pops;
                    if (o.verbose > 2) {
                        if (W.popep.size() < WS) W.popep.assign(WS, 0);
                        if (W.popep[q] == W.epoch) ++reexp;
                        W.popep[q] = W.epoch;
                    }
                    if (is_target(q)) { found = q; break; }
                    uint32_t cell = uint32_t(j + wj0) * G.nx + (i + wi0);
                    const uint8_t rq = nd[q].rec, rstep = rq == 255 ? 255 : uint8_t(std::min(254, rq + 1));
                    auto relax = [&](uint32_t nq, float ng, int nl, int ni, int nj, uint8_t code) {
                        Node& n = nd[nq];
                        if (o.verbose > 2 && n.ep == W.epoch && ng < n.g && W.popep.size() >= WS && W.popep[nq] == W.epoch && dbg < 5) {
                            ++dbg;
                            std::fprintf(stderr, "      closed state improved: (%d,%d,l%d) g %.1f -> %.1f via code %d from (%d,%d,l%d) g %.1f; h from %.1f, h to %.1f, step %.1f\n",
                                         ni, nj, nl, n.g, ng, code, i, j, l, gq, h(l, i, j), h(nl, ni, nj), ng - gq);
                        }
                        if (n.ep != W.epoch || ng < n.g - 1.f) {  // 1 um: a real improvement, not rounding
                            n = {ng, W.epoch, code, uint8_t(code >= 8 ? 0 : rstep)};
                            W.heap.push_back({ng * TIE + h(nl, ni, nj), ng, nq});
                            std::push_heap(W.heap.begin(), W.heap.end());
                        }
                    };
                    if (R.lay[l]) {
                        const float* DF = dirf[l < 8 ? l : 0];
                        for (int d = 0; d < 8; ++d) {
                            int ni = i + di[d], nj = j + dj[d];
                            if (ni < 0 || nj < 0 || ni >= ww || nj >= wh) continue;
                            uint32_t nc = uint32_t(nj + wj0) * G.nx + (ni + wi0);
                            uint32_t nq = uint32_t(size_t(l) * WN + size_t(nj) * ww + ni);
                            W.touch[blk(l, ni + wi0, nj + wj0)] = 1;
                            float cm = cm_at(l, nc, nq);
                            if (!(cm < INF)) continue;
                            if (cm < 0) { if (R.st[l][nc] != me) continue; cm = -cm; }
                            if (gd && !J.guide[size_t(l) * GNn + gnode(ni + wi0, nj + wj0)]) continue;
                            cm = damp(cm);
                            if (PJ && !((nearb[nq >> 6] >> (nq & 63)) & 1)) cm *= pw;
                            if (mp) {
                                uint8_t pv = prior_at(mp, l, ni + wi0, nj + wj0);
                                if (mc && pv < mthr && !is_target(nq)) continue;
                                relax(nq, gq + len[d] * DF[d] * cm * mlf[pv], l, ni, nj, uint8_t(d));
                            } else
                                relax(nq, gq + len[d] * DF[d] * cm, l, ni, nj, uint8_t(d));
                        }
                    }
                    float vm = INF;
                    if (R.via >= 0) {
                        W.touchv[blk(0, i + wi0, j + wj0)] = 1;
                        vm = vm_at(cell, i, j);
                        if (vm < 0) vm = R.vst[cell] == me ? -vm : INF;
                        for (size_t k = 0; vm < INF && k < myvias.size(); ++k) {
                            double ddx = double(myvias[k] % G.nx) - double(cell % G.nx), ddy = double(myvias[k] / G.nx) - double(cell / G.nx);
                            if (myvias[k] != cell && ddx * ddx + ddy * ddy < via_gap2) vm = INF;
                        }
                    }
                    if (vm < INF && rq < via_gapc) vm = INF;  // a via too soon after this path's last one: their holes would touch
                    if (vm < INF) {
                        float cost = float(VC) * damp(vm);
                        if (xz && xzone[cell]) cost *= float(o.nt_xtal_via);
                        for (int l2 = 0; l2 < NL; ++l2) {
                            if (l2 == l) continue;
                            uint32_t nq = uint32_t(size_t(l2) * WN + size_t(j) * ww + i);
                            if (R.lay[l2] ? !ok(R.st[l2][cell], me) : !is_target(nq)) continue;
                            if (gd && !J.guide[size_t(l2) * GNn + gnode(i + wi0, j + wj0)]) continue;
                            if (mc && !is_target(nq) && prior_at(mp, l2, i + wi0, j + wj0) < mthr) continue;
                            relax(nq, gq + cost, l2, i, j, uint8_t(8 + l));
                        }
                    }
                }
                exhausted = found < 0 && W.heap.empty();
#ifdef PCB_GPU
                if (gpu_none && (found >= 0 || !exhausted))
                    std::fprintf(stderr, "GPU_CHECK: the GPU found no target reachable, the CPU %s (net %s, %zu pops, window %dx%d, corridor %d, hard %d, pres %.0f)\n",
                                 found >= 0 ? "found one" : "ran out of budget", B.nets[J.net].name.c_str(), budget0 - budget, ww, wh, int(mc), int(hard), pres);
#endif
                if (W.heap.capacity() > (size_t(4) << 20)) { W.heap.clear(); W.heap.shrink_to_fit(); }  // a huge search's heap isn't kept
                us_loop += long((now() - ts2) * 1e6);
                if (o.verbose > 2) std::fprintf(stderr, "      %s search: %s after %zu pops (%zu re-expansions)%s (heap %zu)\n", fwd ? "tree-side" : "piece-side", found >= 0 ? "found" : exhausted ? "exhausted" : "out of budget", budget0 - budget, reexp, exact ? ", exact heuristic" : "", W.heap.size());
                return found;
            };
            t_setupsearch += now() - tss;
            size_t budget = hard ? o.max_pops * 16 : o.max_pops << std::min(J.stuck, 3);  // grows while the net stays stuck
            if (o.budget_scale != 1.0 && J.difficulty > 0) budget = size_t(double(budget) * (1 + (o.budget_scale - 1) * J.difficulty));  // predicted hard: more from the start
            // First a probe from the pieces towards the tree within a small budget: a piece walled in (common when
            // legalizing) is found out cheaply, and a short hop is done with. Then the full search from the tree,
            // unless the tree is huge: then the full search runs from the pieces as well.
            bool exh = false, rev = false;
            int64_t found = -1;
            if (gst.empty() || tree_in == 0) exh = true;
            else {
                for (uint32_t q : W.tlist) W.tbit[q >> 6] = 0;
                W.tlist = gst;
                for (uint32_t q : gst) W.tbit[q >> 6] |= uint64_t(1) << (q & 63);
                // a huge tree, or many pieces (no exact heuristic for the search from the tree): the search from the
                // pieces gets the whole budget
                size_t probe = bigtree || tboxes.empty() ? budget : std::min(budget, o.probe_pops);
                if (probe > 0) { found = search(gst, W.treeb.data(), W.tcz, probe, exh, false); rev = true; }
                if (found < 0 && !exh && !bigtree) { found = search(tst, W.tbit.data(), W.gcz, budget, exh, true); rev = false; }
            }
            if (found < 0) {
                // widen the window, unless the search ran out of budget (it will be retried next round); when
                // legalizing, a target walled in by other nets is not worth a whole-board search
                if (exh && mc) { mc = false; ++n_ml_fallback; continue; }  // the learned corridor was a dead end
                if (exh && gd) { gd = false; continue; }  // the corridor was a dead end
                if (exh && stage < (hard ? 1 : 2) && !(sx && sx->has_win) && (wi0 > 0 || wj0 > 0 || wi1 < G.nx - 1 || wj1 < G.ny - 1)) { ++stage; continue; }
                if (exh && left.size() > 1) {
                    // the tree is walled off from every piece left: count one open connection and carry on
                    // joining the other pieces to each other, from the next one
                    ++J.unrouted;
                    tree = grp[left[0]].states;
                    left.erase(left.begin());
                    stage = 0;
                    cur_stage = -1;
                    continue;
                }
                J.unrouted += int(left.size());
                break;
            }
            std::vector<uint32_t> path;
            uint32_t start = uint32_t(found);
            for (int64_t q = found;;) {
                path.push_back(glob(q));
                uint8_t code = W.nd[q].par;
                if (code == 255) { start = uint32_t(q); break; }
                int i = int(q % ww), j = int((q / ww) % wh), l = int(q / (int64_t(ww) * wh));
                if (code < 8) { i -= di[code]; j -= dj[code]; }
                else l = code - 8;
                q = (int64_t(l) * wh + j) * ww + i;
            }
            int c = W.tcomp[rev ? start : uint32_t(found)];
            if (!rev) std::reverse(path.begin(), path.end());  // tree end first either way
            for (uint32_t s : path) add_tree(s);
            for (uint32_t s : grp[c].states) add_tree(s);
            tree.insert(tree.end(), path.begin(), path.end());
            tree.insert(tree.end(), grp[c].states.begin(), grp[c].states.end());
            note_vias(path);
            J.paths.push_back(std::move(path));
            left.erase(std::find(left.begin(), left.end(), c));
        }
        tidy_paths(J);
        return J.unrouted == 0;
    }

    // ---- geometry of a routed job ----
    void geometry(const Job& J, std::vector<Seg>& segs, std::vector<P>& vias) const {
        const size_t N = G.n();
        const RClass& R = rcs[J.rc];
        std::unordered_set<uint32_t> keep(J.keep.begin(), J.keep.end());
        for (auto& path : J.paths) {
            size_t a = 0;
            while (a < path.size()) {
                size_t b = a;
                while (b + 1 < path.size() && path[b + 1] / N == path[a] / N) ++b;
                // run a..b on one layer
                int l = int(path[a] / N);
                std::vector<uint32_t> pts{uint32_t(path[a] % N)};
                int pdi = 0, pdj = 0;
                for (size_t k = a + 1; k <= b; ++k) {
                    uint32_t c0 = uint32_t(path[k - 1] % N), c1 = uint32_t(path[k] % N);
                    int ddi = int(c1 % G.nx) - int(c0 % G.nx), ddj = int(c1 / G.nx) - int(c0 / G.nx);
                    if (k > a + 1 && (ddi != pdi || ddj != pdj || keep.count(path[k - 1]))) pts.push_back(c0);
                    pdi = ddi; pdj = ddj;
                }
                if (b > a) pts.push_back(uint32_t(path[b] % N));
                for (size_t k = 1; k < pts.size(); ++k) segs.push_back({l, G.center(pts[k - 1]), G.center(pts[k]), 2 * R.hw});
                if (b + 1 < path.size()) vias.push_back(G.center(uint32_t(path[b] % N)));
                a = b + 1;
            }
        }
        for (auto& [s, pts, hw] : J.stubs)
            for (size_t k = 1; k < pts.size(); ++k)
                if (norm(pts[k] - pts[k - 1]) > 0.5) segs.push_back({int(s / N), pts[k - 1], pts[k], 2 * hw});
        std::sort(vias.begin(), vias.end(), [](P a, P b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
        vias.erase(std::unique(vias.begin(), vias.end(), [](P a, P b) { return a.x == b.x && a.y == b.y; }), vias.end());
    }

    // ---- occupancy ----
    // Every grid id's cells changed by a stamp get their cost map entry recomputed.
    void refresh_costs(size_t id, const std::vector<Span>& spans) {
        size_t k = id / (2 * NL), kind = (id / NL) % 2, l = id % NL;
        RClass& R = rcs[k];
        if (R.cst.empty()) return;
        for (auto& sp : spans)
            for (int i = sp.i0; i <= sp.i1; ++i) {
                uint32_t c = uint32_t(sp.j) * G.nx + i;
                if (kind) st(R.vcst[c], vcost_of(R, c));
                else st(R.cst[l][c], cost_of(R, l, c));
            }
    }
    std::vector<Span> stamp_scratch[2];  // stamps are made one at a time (under run()'s lock, or single-threaded)
    void stamp(Job& J, int sign) {
        double t0 = now();
        struct Acc { double t0; ~Acc() { t_stamp += now() - t0; } } acc{t0};
        if (sign < 0) {
            for (size_t id = 0; id < J.stamps.size(); ++id) {
                auto& g = grid_of(id);
                for (auto& sp : J.stamps[id])
                    for (uint8_t *c = &g[size_t(sp.j) * G.nx + sp.i0], *e = c + (sp.i1 - sp.i0 + 1); c < e; ++c) st(*c, uint8_t(*c - 1));
                refresh_costs(id, J.stamps[id]);
                J.stamps[id].clear();
            }
            return;
        }
        ++n_stamp;
        const size_t N = G.n();
        const RClass& R = rcs[J.rc];
        // the path as horizontal runs of cells per layer, and its via cells
        std::vector<std::vector<Span>> runs(NL);
        std::vector<uint32_t> viac;
        for (auto& path : J.paths)
            for (size_t k = 0; k < path.size(); ++k) {
                int l = int(path[k] / N);
                uint32_t c = uint32_t(path[k] % N);
                int i = int(c % G.nx), j = int(c / G.nx);
                auto& rs = runs[l];
                if (!rs.empty() && k > 0 && path[k - 1] / N == size_t(l) && rs.back().j == j && rs.back().i1 + 1 == i) rs.back().i1 = i;
                else if (!rs.empty() && k > 0 && path[k - 1] / N == size_t(l) && rs.back().j == j && rs.back().i0 - 1 == i) rs.back().i0 = i;
                else rs.push_back({j, i, i});
                if (k + 1 < path.size() && path[k + 1] / N != size_t(l)) viac.push_back(c);
            }
        std::sort(viac.begin(), viac.end());
        viac.erase(std::unique(viac.begin(), viac.end()), viac.end());
        n_rows += viac.size();
        auto apply = [&](const Kern& K, int j, int i0, int i1, std::vector<Span>& out) {
            for (auto& [dj, w] : K.rows) {
                int jj = j + dj;
                if (jj < 0 || jj >= G.ny) continue;
                out.push_back({jj, std::max(0, i0 - w), std::min(G.nx - 1, i1 + w)});
            }
        };
        for (size_t k = 0; k < rcs.size(); ++k) {
            const RClass& K = rcs[k];
            double c = std::max(R.clr, K.clr);
            const Kern *kt = &kern[J.rc][k * 4], *kv = &kern[J.rc][k * 4 + 2];
            for (int l = 0; l < NL; ++l) {
                // built in scratch space: a board-spanning net at a fine grid makes hundreds of thousands of spans
                // before merging, and the merged rows are a tiny part of that (the job only keeps those)
                std::vector<Span>&to = stamp_scratch[0], &tv = stamp_scratch[1];
                to.clear(); tv.clear();
                for (auto& r : runs[l]) { apply(kt[0], r.j, r.i0, r.i1, to); apply(kt[1], r.j, r.i0, r.i1, tv); }
                for (uint32_t v : viac) { apply(kv[0], int(v / G.nx), int(v % G.nx), int(v % G.nx), to); apply(kv[1], int(v / G.nx), int(v % G.nx), int(v % G.nx), tv); }
                for (auto& [s, pts, hw] : J.stubs) {  // off-grid escape stubs: exact capsules
                    if (int(s / N) != l) continue;
                    for (size_t p = 1; p < pts.size(); ++p) {
                        capsule_spans(G, pts[p - 1], pts[p], hw + K.hw + c + mg(hw + K.hw + c), to);
                        capsule_spans(G, pts[p - 1], pts[p], hw + K.vr + c + mg(hw + K.vr + c), tv);
                    }
                }
                merge_spans(to);
                merge_spans(tv);
                J.stamps[(k * 2 + 0) * NL + l].assign(to.begin(), to.end());
                J.stamps[(k * 2 + 1) * NL + l].assign(tv.begin(), tv.end());
            }
        }
        for (size_t id = 0; id < J.stamps.size(); ++id) {
            auto& g = grid_of(id);
            for (auto& sp : J.stamps[id])
                for (uint8_t *c = &g[size_t(sp.j) * G.nx + sp.i0], *e = c + (sp.i1 - sp.i0 + 1); c < e; ++c)
                    if (*c < 255) st(*c, uint8_t(*c + 1));
            refresh_costs(id, J.stamps[id]);
        }
    }
    std::vector<uint8_t>& grid_of(size_t id) {
        size_t k = id / (2 * NL), kind = (id / NL) % 2, l = id % NL;
        return kind ? rcs[k].ov[l] : rcs[k].oc[l];
    }

    // number of path states sitting on another net's footprint (own stamp counts 1 there)
    int count_conflicts(const Job& J, bool mark_hist, float hinc) {
        const size_t N = G.n();
        const RClass& R = rcs[J.rc];
        int n = 0;
        for (auto& path : J.paths)
            for (size_t k = 0; k < path.size(); ++k) {
                int l = int(path[k] / N);
                uint32_t c = uint32_t(path[k] % N);
                bool bad = R.oc[l][c] > 1;
                if (k + 1 < path.size() && path[k + 1] / N != path[k] / N)  // a via here
                    for (int l2 = 0; l2 < NL; ++l2) bad |= R.ov[l2][c] > 1;
                if (bad) {
                    ++n;
                    if (mark_hist) hist[l][c] += hinc;
                }
            }
        for (auto& [s, pts, hw] : J.stubs) {
            int l = int(s / N);
            for (size_t k = 1; k < pts.size(); ++k) {
                double len = norm(pts[k] - pts[k - 1]);
                for (double t = 0; t <= len; t += G.s / 2) {
                    P p = pts[k - 1] + (pts[k] - pts[k - 1]) * (len > 0 ? t / len : 0);
                    int i = int((p.x - G.x0) / G.s), j = int((p.y - G.y0) / G.s);
                    if (i < 0 || j < 0 || i >= G.nx || j >= G.ny) continue;
                    if (R.oc[l][uint32_t(j) * G.nx + i] > 1) ++n;
                }
            }
        }
        return n;
    }

    // the blocks (BS x BS cells, per layer; via grids apart) a job's stamps cover, into a wave's change record
    void mark_blocks(const Job& J, std::vector<uint8_t>& chg, std::vector<uint8_t>& chgv) const {
        for (size_t id = 0; id < J.stamps.size(); ++id) {
            size_t kind = (id / NL) % 2, l = id % NL;
            uint8_t* rec = kind ? chgv.data() : chg.data() + l * BN;
            for (auto& sp : J.stamps[id]) {
                size_t row = size_t(sp.j / BS) * bw;
                for (int i = sp.i0 / BS; i <= sp.i1 / BS; ++i) rec[row + i] = 1;
            }
        }
    }
    void restamp(Job& J) { stamp(J, -1); stamp(J, +1); }

    // ---- live events (--events) ----
    // A net is shown again when its copper (paths, stubs) or its open count changed since it was last shown.
    uint64_t ev_signature(const Job& J) const {
        uint64_t h = 1469598103934665603ull;
        auto mix = [&](uint64_t v) { h = (h ^ v) * 1099511628211ull; };
        mix(uint64_t(J.unrouted));
        for (auto& p : J.paths) { mix(p.size()); for (uint32_t x : p) mix(x); }
        for (auto& st : J.stubs) { mix(st.s); mix(st.pts.size()); for (P q : st.pts) { mix(uint64_t(std::llround(q.x))); mix(uint64_t(std::llround(q.y))); } }
        return h;
    }
    void ev_show(Job& J, uint64_t sig) {
        NetResult r;
        std::vector<P> vias;
        geometry(J, r.segs, vias);
        for (P v : vias) r.vias.push_back({v, rcs[J.rc].via});
        g_events.net(B, J.net, r.segs, r.vias, J.unrouted);
        J.ev_sig = sig;
        J.ev_shown = true;
    }
    void ev_maybe(Job& J) {
        uint64_t sig = ev_signature(J);
        if (sig == J.ev_sig && J.ev_shown) return;
        if (!J.ev_shown && J.paths.empty() && J.stubs.empty()) return;  // nothing to draw yet
        ev_show(J, sig);
    }
    int ev_conns = 0;  // connections in all, for the progress events
    int ev_pass = 0;
    double ev_last = 0;  // when the nets were last shown (rate limit on big boards)
    static constexpr double EV_EVERY = 0.2;
    void ev_progress(int conflicts) {
        int open = dead_total - std::accumulate(jobs.begin(), jobs.end(), 0, [](int a, const Job& J) { return a + J.dead; });
        for (auto& J : jobs) open += J.unrouted;
        g_events.progress(ev_pass, std::max(0, ev_conns - open), open, conflicts);
    }
    // every net whose copper changed (force: whatever the time since the last sweep)
    void ev_sweep(bool force, int conflicts = -1) {
        if (!g_events.on()) return;
        double t = now();
        if (!force && t - ev_last < EV_EVERY) return;
        ev_last = t;
        for (auto& J : jobs) ev_maybe(J);
        if (conflicts >= 0) ev_progress(conflicts);
    }

    void run(RouteStats& S) {
        const double t_run = now();
        int T = o.threads > 0 ? o.threads : std::max(1u, std::thread::hardware_concurrency());
        {
            // Every thread's search window can grow to the whole board: 8 bytes a state for the nodes, half a byte
            // of bits, and the heap. A 6-layer board at 25 um is 80 M states, 0.8 GB a thread: with 16 threads and
            // the grids that took 18-29 GB and the kernel killed the router (and the desktop with it). So the
            // thread count is capped by the memory the windows may take: --mem, else 45 % of what is free.
            double per_thread = double(G.n()) * NL * 12.0;  // nodes, bits, heap, and this net's stamps and copies
            // the grids: per class and layer 2 B static + 1 B + 1 B occupancy + 4 B cost map, per cell 2 B + 4 B for
            // vias, and the history; plus a fifth for the routes, stamps and copies
            double maps = 1.2 * (double(rcs.size()) * (double(G.n()) * NL * 8 + double(G.n()) * 6) + double(G.n()) * NL * 4);
            double budget = o.mem_gb > 0 ? o.mem_gb * 1e9 : 0;
            if (budget <= 0) {
                if (FILE* f = std::fopen("/proc/meminfo", "r")) {
                    char line[256];
                    while (std::fgets(line, sizeof line, f))
                        if (!std::strncmp(line, "MemAvailable:", 13)) budget = 0.45 * std::atof(line + 13) * 1024;
                    std::fclose(f);
                }
            }
            if (budget > 0) {
                int tm = std::max(1, int((budget * 0.85 - maps) / per_thread));
                if (tm < T) {
                    if (o.verbose) std::fprintf(stderr, "threads: %d, not %d: a search window can take %.0f MB a thread and %.1f GB is the budget (--mem)\n", tm, T, per_thread / 1e6, budget / 1e9);
                    T = tm;
                }
            }
        }
        std::vector<Work> works(T);
        std::sort(jobs.begin(), jobs.end(), [](const Job& a, const Job& b) {
            return (a.box.x1 - a.box.x0) + (a.box.y1 - a.box.y0) > (b.box.x1 - b.box.x0) + (b.box.y1 - b.box.y0);
        });
        if (!o.order.empty())  // the learned order first (--order), the rest in the default order after it
            std::stable_sort(jobs.begin(), jobs.end(), [](const Job& a, const Job& b) {
                return (a.prio < 0 ? INT32_MAX : a.prio) < (b.prio < 0 ? INT32_MAX : b.prio);
            });
        nt_order();
        // Route `todo` in waves of up to T nets in parallel, each seeing every other net's route as it stands (its
        // own left out), then commit them in order. Every search records the blocks of cells it read; a net whose
        // blocks a net committed before it in the same wave has changed (its old or its new footprint) goes back
        // in the queue with its old route, since its search could have come out differently. So the result is
        // exactly that of routing the nets one by one in this order, whatever the thread count; nets that don't
        // interact share a wave. A wave takes non-overlapping nets first and fills the idle threads with
        // overlapping ones on speculation.
        if (g_events.on()) {
            ev_conns = dead_total - std::accumulate(jobs.begin(), jobs.end(), 0, [](int a, const Job& J) { return a + J.dead; });
            for (auto& J : jobs) ev_conns += int(J.comps.size()) - 1 + J.dead;
            ev_last = now();
        }
        size_t waves = 0, retries = 0;
        struct Saved { std::vector<std::vector<uint32_t>> paths; std::vector<uint32_t> keep; std::vector<Stub> stubs; int unrouted; };
        // Route `todo` with T threads, committing the nets one by one in a fixed order: the queue regrouped into
        // maximal sets of nets whose boxes don't overlap, group after group, so that the nets being routed at the
        // same time mostly don't interact. Each thread takes the next net in that order and routes it against the
        // board as committed so far (its own route left out). A net is committed once every net before it is: it
        // is accepted if no commit since its search started touched a block of cells the search read, else it is
        // put back with its old route and routed again (the nets after it keep their routes: they are checked
        // against its new route the same way when their turn comes). So the result is exactly that of routing the
        // nets one by one in this order, whatever the thread count and timing; the threads run ahead of the
        // commit point on speculation.
        auto route_waves = [&](const std::vector<int>& todo, float pres, bool hard) {
            std::vector<int> order, rest(todo.begin(), todo.end());
            while (!rest.empty()) {
                std::vector<int> grp_, left_;
                for (int j : rest) {
                    bool clash = false;
                    for (int k : grp_) clash |= jobs[j].box.overlaps(jobs[k].box, 1000);
                    (clash ? left_ : grp_).push_back(j);
                }
                order.insert(order.end(), grp_.begin(), grp_.end());
                rest.swap(left_);
            }
            const size_t n = order.size();
            // --nt-pair: a pair's second net follows the first's track once that is committed (it waits for it); if
            // the first is routed after it in this call, it doesn't follow it (it would read a track being changed)
            std::vector<int> dep(n, -1);
            bool anydep = false;
            for (size_t i = 0; i < n; ++i)
                if (jobs[order[i]].pair >= 0) {
                    auto it = std::find(order.begin(), order.end(), jobs[order[i]].pair);
                    size_t pi = size_t(it - order.begin());
                    jobs[order[i]].pair_ok = pi >= n || pi < i;
                    if (pi < i) { dep[i] = int(pi); anydep = true; }
                }
            size_t lookahead = size_t(T);  // how far ahead of the commit point the threads route: adapts to the rejections
            std::vector<Saved> saved(n);
            std::vector<char> done(n, 0), inflight(n, 0);
            std::vector<size_t> startver(n);                 // commits made when the net's search started
            std::vector<std::vector<uint32_t>> commits;     // per commit: the blocks it changed (via blocks offset by BN * NL)
            size_t cursor = 0, next = 0;
            std::mutex mu;
            std::condition_variable cv;
            std::vector<uint8_t> scratch(BN * NL + BN, 0);
            std::vector<int> ev_dirty;  // --events: nets committed since they were last shown
            auto blocks_of = [&](const Job& J, std::vector<uint32_t>& out) {
                for (size_t id = 0; id < J.stamps.size(); ++id) {
                    size_t kind = (id / NL) % 2, l = id % NL, base = kind ? BN * NL : l * BN;
                    for (auto& sp : J.stamps[id]) {
                        size_t row = size_t(sp.j / BS) * bw;
                        for (int i = sp.i0 / BS; i <= sp.i1 / BS; ++i) {
                            size_t b = base + row + i;
                            if (!scratch[b]) { scratch[b] = 1; out.push_back(uint32_t(b)); }
                        }
                    }
                }
                for (uint32_t b : out) scratch[b] = 0;
            };
            auto worker = [&](int w) {
                Work& W = works[w];
                for (;;) {
                    size_t i, v0;
                    {
                        std::unique_lock<std::mutex> lk(mu);
                        cv.wait(lk, [&] {  // the next net nobody has routed or is routing
                            while (next < n && (inflight[next] || done[next])) ++next;
                            return cursor >= n || (next < n && next < cursor + lookahead && (!anydep || dep[next] < int(cursor)));
                        });
                        if (cursor >= n) return;
                        i = next++;
                        inflight[i] = 1;
                        v0 = commits.size();
                        Job& J = jobs[order[i]];
                        saved[i] = {J.paths, J.keep, J.stubs, J.unrouted};
                    }
                    double t = now();
                    Job& J = jobs[order[i]];
                    std::fill(W.touch.begin(), W.touch.end(), 0);
                    std::fill(W.touchv.begin(), W.touchv.end(), 0);
                    if (o.full_every > 0 && J.stuck % o.full_every == o.full_every - 1) J.paths.clear();  // local repairs keep failing: reroute it all
                    route_job(J, W, pres, hard);
                    if (hard && J.unrouted > J.dead) {      // could not repair around the others: start over
                        J.paths.clear();
                        route_job(J, W, pres, hard);
                    }
                    J.seconds += now() - t;
                    std::unique_lock<std::mutex> lk(mu);
                    J.touch = W.touch;
                    J.touchv = W.touchv;
                    startver[i] = v0;
                    done[i] = 1;
                    inflight[i] = 0;
                    while (cursor < n && done[cursor]) {
                        Job& C = jobs[order[cursor]];
                        bool fine = true;
                        for (size_t c = startver[cursor]; c < commits.size() && fine; ++c)
                            for (uint32_t b : commits[c])
                                if (b < BN * NL ? C.touch[b] : C.touchv[b - BN * NL]) { fine = false; break; }
                        if (fine) {
                            std::vector<uint32_t> blocks;
                            blocks_of(C, blocks);
                            restamp(C);
                            blocks_of(C, blocks);
                            commits.push_back(std::move(blocks));
                            if (g_events.on()) {  // committed nets stay as they are for the rest of this call
                                ev_dirty.push_back(order[cursor]);
                                if (now() - ev_last >= EV_EVERY) {
                                    for (int j : ev_dirty) ev_maybe(jobs[j]);
                                    ev_dirty.clear();
                                    ev_last = now();
                                }
                            }
                            ++cursor;
                            ++waves;
                            lookahead = std::min(size_t(T) * 2, lookahead + 1);
                        } else {
                            ++retries;
                            lookahead = std::max<size_t>(2, lookahead * 3 / 4);  // speculation is being wasted: less of it
                            C.paths = saved[cursor].paths; C.keep = saved[cursor].keep; C.stubs = saved[cursor].stubs; C.unrouted = saved[cursor].unrouted;
                            done[cursor] = 0;
                            next = std::min(next, cursor);  // routed again, now against the board it will be committed on
                            break;
                        }
                    }
                    cv.notify_all();
                }
            };
            int nt = std::min<int>(T, int(n));
            std::vector<std::thread> th;
            for (int w = 1; w < nt; ++w) th.emplace_back(worker, w);
            worker(0);
            for (auto& t : th) t.join();
            for (int j : ev_dirty) ev_maybe(jobs[j]);
            for (int j : order) jobs[j].pair_ok = true;
        };
        std::vector<int> todo(jobs.size());
        std::iota(todo.begin(), todo.end(), 0);
        float pres = 0.5f;
        int it = 0, best = 1 << 30, since = 0, same = 0, last = -1;
        // --time-limit: negotiation gets half the time left, rip-ups until 80 %, the last forced pass until the
        // deadline (then what still overlaps is dropped), rescue and shove whatever is left
        const double t_neg = o.deadline > 0 ? t_run + 0.50 * (o.deadline - t_run) : 0;
        const double t_rip = o.deadline > 0 ? t_run + 0.80 * (o.deadline - t_run) : 0;
        auto limit = [&](const std::string& what) {  // what was cut short, each once
            if (S.limited.find(what) == std::string::npos) S.limited += (S.limited.empty() ? "" : ", ") + what;
        };
        double last_round = 0;
        for (; it < o.iterations && !todo.empty(); ++it) {
            if (it > 0 && o.deadline > 0 && now() + last_round > t_neg) {
                char buf[96];
                std::snprintf(buf, sizeof buf, "negotiation stopped after %d rounds", it);
                limit(buf);
                break;
            }
            double t0 = now();
            size_t w0 = waves, r0 = retries;
            long p0 = n_pops;
            ev_pass = it + 1;
            g_events.pass(ev_pass, "route");
            rebuild_costs(pres, false);
            route_waves(todo, pres, false);
            malloc_trim(0);  // freed stamps, heaps and copies back to the system (fragmentation held gigabytes)
            todo.clear();
            int total = 0;
            for (size_t j = 0; j < jobs.size(); ++j) {
                { double t = now(); jobs[j].conflicts = count_conflicts(jobs[j], true, float(o.hist_inc)); t_conf += now() - t; }
                jobs[j].conflicts += 100 * (jobs[j].unrouted - jobs[j].dead);  // open connections: retry them
                if (jobs[j].conflicts) { todo.push_back(int(j)); total += jobs[j].conflicts; ++jobs[j].stuck; ++jobs[j].conflict_rounds; }
                else jobs[j].stuck = 0;
            }
            ev_sweep(true, int(todo.size()));
            last_round = now() - t0;
            if (o.verbose)
                std::fprintf(stderr, "  iteration %2d: %zu commits, %zu rejected, %zu nets conflicting (%d cells), pres %.2f, %.1fM pops, %.2f s\n",
                             it + 1, waves - w0, retries - r0, todo.size(), total, pres, (n_pops - p0) / 1e6, now() - t0);
            if (o.verbose > 2) {
                std::fprintf(stderr, "    in conflict:");
                for (int j : todo) std::fprintf(stderr, " %s(%d)", B.nets[jobs[j].net].name.c_str(), jobs[j].conflicts);
                std::fprintf(stderr, "\n");
            }
            pres *= float(o.pres_growth);
            if (o.pres_max > 0) pres = std::min(pres, float(o.pres_max));
            if (total < 0.9 * best) { best = total; since = 0; }  // progress: at least 10 % better
            else if (++since >= o.patience) { ++it; break; }  // stalled: legalize
            same = total == last ? same + 1 : 0;  // the very same conflicts three rounds running: structural, legalize
            last = total;
            if (same >= 2) { ++it; break; }
        }
        S.iterations = it;
        // Legalize: repair what still conflicts with the other nets' copper as a hard obstacle, first in parallel
        // waves, then one net at a time. A net that can't get through that way is routed across the others at a
        // very high price (the fewest crossings), and the nets it crosses repair themselves around it in turn
        // (rip-up and reroute; each net can push its way through o.ripups times). A last pass leaves every net
        // legal, if incomplete.
        auto legal_route = [&](Job& J) {
            std::fill(works[0].touch.begin(), works[0].touch.end(), 0);
            std::fill(works[0].touchv.begin(), works[0].touchv.end(), 0);
            route_job(J, works[0], 0, true);
            if (J.unrouted > J.dead) { J.paths.clear(); route_job(J, works[0], 0, true); }
            restamp(J);
        };
        // Legalize the nets in todo0 (see above) with a fresh rip-up budget: repairs, pushes, a last forced pass,
        // then the drop of what still overlaps.
        auto legalize = [&](const std::vector<int>& todo0, bool report) {
            double t0 = now();
            size_t w0 = waves;
            std::vector<int> rips(jobs.size(), 0);
            int steps = 0, ripups = 0;
            std::vector<int> q = todo0;
            if (report) g_events.pass(++ev_pass, "ripup");
            if (!o.legalize_par) {  // the earlier way: one net at a time, each seeing the repairs before it
                route_waves(q, 0, true);
                std::deque<int> dq;
                std::vector<char> queued(jobs.size(), 0);
                for (int j : q)
                    if (count_conflicts(jobs[j], false, 0) > 0 || jobs[j].unrouted > jobs[j].dead) { dq.push_back(j); queued[j] = 1; }
                while (!dq.empty() && steps++ < 20 * int(jobs.size())) {
                    if (o.deadline > 0 && now() > t_rip) { limit("rip-ups stopped"); break; }
                    int j = dq.front();
                    dq.pop_front();
                    queued[j] = 0;
                    Job& J = jobs[j];
                    legal_route(J);
                    if (J.unrouted > J.dead && rips[j] < o.ripups) {
                        ++rips[j];
                        ++ripups;
                        rebuild_costs(1e4f, false);
                        route_job(J, works[0], 1e4f, false);
                        rebuild_costs(0, true);
                        restamp(J);
                        for (size_t k = 0; k < jobs.size(); ++k)
                            if (int(k) != j && !queued[k] && count_conflicts(jobs[k], false, 0) > 0) { dq.push_back(int(k)); queued[k] = 1; }
                        if (count_conflicts(J, false, 0) > 0 && !queued[j]) { dq.push_back(j); queued[j] = 1; }
                    }
                }
                q.clear();
            }
            while (!q.empty() && steps++ < 20) {
                if (o.deadline > 0 && now() > t_rip) { limit("rip-ups stopped"); break; }
                route_waves(q, 0, true);  // everyone in conflict repairs around the others, in parallel
                // what could not get through pushes its way across (one at a time: each one changes the board)
                std::vector<int> pushed;
                for (int j : q) {
                    Job& J = jobs[j];
                    if (o.deadline > 0 && now() > t_rip) { limit("rip-ups stopped"); break; }
                    if (J.unrouted > J.dead && rips[j] < o.ripups) {
                        ++rips[j];
                        ++ripups;
                        rebuild_costs(1e4f, false);
                        std::fill(works[0].touch.begin(), works[0].touch.end(), 0);
                        std::fill(works[0].touchv.begin(), works[0].touchv.end(), 0);
                        route_job(J, works[0], 1e4f, false);  // through the others, crossing as little as it can
                        rebuild_costs(0, true);
                        restamp(J);
                        pushed.push_back(j);
                    }
                }
                ev_sweep(false);
                if (pushed.empty()) break;
                q.clear();  // the nets it crossed (and itself, where it still overlaps) repair next
                for (size_t k = 0; k < jobs.size(); ++k)
                    if (count_conflicts(jobs[k], false, 0) > 0) q.push_back(int(k));
            }
            // whatever still overlaps gets a last hard route; the paths that can't be made legal that way are
            // dropped (open connections rather than a violation), the rest of their nets stays
            int fixed = 0, dropped = 0;
            for (int pass = 0; pass < 4; ++pass) {
                bool any = false;
                bool late = false;
                for (auto& J : jobs)
                    if (count_conflicts(J, false, 0) > 0) {
                        if (o.deadline > 0 && now() > o.deadline) { late = true; break; }
                        any = true; legal_route(J); ++fixed;
                    }
                if (late) { limit("overlaps dropped at the deadline"); break; }
                if (!any) break;
            }
            for (auto& J : jobs)
                if (count_conflicts(J, false, 0) > 0) {
                    int was = J.unrouted, n = drop_conflicts(J);
                    dropped += n;
                    if (o.verbose && report) std::fprintf(stderr, "  %s: %d paths dropped, could not be made legal (%d more open)\n", B.nets[J.net].name.c_str(), n, J.unrouted - was);
                }
            if (report) ev_sweep(true, 0);
            if (o.verbose && report)
                std::fprintf(stderr, "  legalized %zu nets in %d parallel passes (%zu commits, %d rip-ups, %d forced, %d paths dropped), %.2f s\n", todo0.size(),
                             steps, waves - w0, ripups, fixed, dropped, now() - t0);
        };
        rebuild_costs(0, true);
        if (!todo.empty()) {
            std::sort(todo.begin(), todo.end(), [&](int a, int b) { return jobs[a].conflicts < jobs[b].conflicts; });
            legalize(todo, true);
        }
        // Rescue: a net still open pushes its way through the others (the fewest crossings), the nets it crosses
        // legalize around it (pushing in turn, with a fresh rip-up budget), and whatever still overlaps then is
        // dropped; the board is kept only if fewer
        // connections are open in all, else put back as it was. So this pass never makes the result worse.
        if (o.rescue > 0) {
            double t0 = now();
            // time cap: --rescue-time, else half the time routing took so far (at least 10 s)
            double cap = o.rescue_seconds > 0 ? o.rescue_seconds : std::max(10.0, 0.5 * (t0 - t_run));
            if (o.deadline > 0 && o.deadline - t0 < cap) {
                cap = std::max(0.0, o.deadline - t0);
                limit("rescue cut short");
            }
            auto open_total = [&] { int n = 0; for (auto& J : jobs) n += J.unrouted - J.dead; return n; };
            int tried = 0, kept = 0, before_all = open_total();
            if (before_all > 0) g_events.pass(++ev_pass, "ripup");
            for (int round = 0; round < o.rescue && open_total() > 0; ++round) {
                bool better = false;
                for (size_t j = 0; j < jobs.size(); ++j) {
                    if (jobs[j].unrouted <= jobs[j].dead) continue;
                    if (now() - t0 > cap) break;
                    ++tried;
                    int before = open_total();
                    std::vector<Saved> snap(jobs.size());
                    for (size_t k = 0; k < jobs.size(); ++k) snap[k] = {jobs[k].paths, jobs[k].keep, jobs[k].stubs, jobs[k].unrouted};
                    Job& J = jobs[j];
                    rebuild_costs(1e4f, false);
                    std::fill(works[0].touch.begin(), works[0].touch.end(), 0);
                    std::fill(works[0].touchv.begin(), works[0].touchv.end(), 0);
                    J.paths.clear();
                    route_job(J, works[0], 1e4f, false);
                    rebuild_costs(0, true);
                    restamp(J);
                    if (o.verbose > 1) std::fprintf(stderr, "    rescue %s: pushed through, %d open, %d cells over others\n", B.nets[J.net].name.c_str(), J.unrouted - J.dead, count_conflicts(J, false, 0));
                    // the nets it crosses repair around it first (pushing in turn if they must), then everyone
                    std::vector<int> q;
                    for (size_t k = 0; k < jobs.size(); ++k)
                        if (k != j && count_conflicts(jobs[k], false, 0) > 0) q.push_back(int(k));
                    if (q.empty() && count_conflicts(J, false, 0) > 0) q.push_back(int(j));
                    if (!q.empty()) legalize(q, false);
                    int after = open_total();
                    if (after < before) { ++kept; better = true; continue; }
                    for (size_t k = 0; k < jobs.size(); ++k)  // the cascade may have changed any net: put back those it did
                        if (jobs[k].paths != snap[k].paths || jobs[k].unrouted != snap[k].unrouted) {
                            Job& K = jobs[k];
                            K.paths = snap[k].paths; K.keep = snap[k].keep; K.stubs = snap[k].stubs; K.unrouted = snap[k].unrouted;
                            restamp(K);
                        }
                    ev_sweep(false);
                }
                if (!better) break;
            }
            if (o.verbose && tried)
                std::fprintf(stderr, "  rescue: %d of %d attempts kept, open %d -> %d, %.2f s\n", kept, tried, before_all, open_total(), now() - t0);
        }
        if (o.shove > 0) {
            if (o.deadline > 0 && now() > o.deadline) limit("shove skipped");
            else shove_pass(works[0]);  // last-mile shove (shove.inc)
        }
        ev_sweep(true, 0);
        S.unrouted += dead_total - std::accumulate(jobs.begin(), jobs.end(), 0, [](int a, const Job& J) { return a + J.dead; });
        S.connections += S.unrouted;
        S.conns = dead_by_net;  // a net with no job: its dead connections only
        S.conns.resize(B.nets.size(), 0);
        for (auto& J : jobs) {
            S.connections += int(J.comps.size()) - 1 + J.dead;
            S.conns[size_t(J.net)] = int(J.comps.size()) - 1 + J.dead;
            S.unrouted += J.unrouted;
            if (J.unrouted && o.verbose) {
                std::fprintf(stderr, "  unrouted: %s, %d of %zu connections (terminal cells:", B.nets[J.net].name.c_str(),
                             J.unrouted, J.comps.size() - 1);
                for (auto& C : J.comps)
                    std::fprintf(stderr, " %zu@(%.2f,%.2f)", C.states.size(), (C.box.x0 + C.box.x1) / 2000, -(C.box.y0 + C.box.y1) / 2000);
                std::fprintf(stderr, ")\n");
            }
        }
        S.nets = int(jobs.size());
        if (o.verbose > 1) {
            std::vector<const Job*> v;
            for (auto& J : jobs) v.push_back(&J);
            std::sort(v.begin(), v.end(), [](auto a, auto b) { return a->seconds > b->seconds; });
            for (size_t k = 0; k < std::min<size_t>(12, v.size()); ++k)
                std::fprintf(stderr, "  %-12s %6.2f s %10zu pops, %zu terminals, box %.0fx%.0f mm\n", B.nets[v[k]->net].name.c_str(),
                             v[k]->seconds, v[k]->pops, v[k]->comps.size(), (v[k]->box.x1 - v[k]->box.x0) / 1000, (v[k]->box.y1 - v[k]->box.y0) / 1000);
        }
    }

#include "shove.inc"

    // The history map at the end (the cells that were in conflict, by round), max-pooled to ~0.25 mm pixels as
    // bytes (hist x 10, capped), for ml/tools/route_all.py. File: "HST1", f64 pitch, x0, y0 (um, DSN axes, like the
    // ML prior), u32 w, h, layers, rounds, then the layer names and w x h bytes per layer (rows y up).
    void dump_hist(const std::string& path, int rounds) const {
        FILE* f = std::fopen(path.c_str(), "wb");
        if (!f) throw std::runtime_error("cannot write " + path);
        int P = std::max(1, int(std::lround(250.0 / G.s)));
        uint32_t w = uint32_t((G.nx + P - 1) / P), h = uint32_t((G.ny + P - 1) / P), nl = uint32_t(NL), r = uint32_t(rounds);
        double pitch = P * G.s, x0 = G.x0, y0 = G.y0;
        std::fwrite("HST1", 1, 4, f);
        std::fwrite(&pitch, 8, 1, f); std::fwrite(&x0, 8, 1, f); std::fwrite(&y0, 8, 1, f);
        std::fwrite(&w, 4, 1, f); std::fwrite(&h, 4, 1, f); std::fwrite(&nl, 4, 1, f); std::fwrite(&r, 4, 1, f);
        for (int l = 0; l < NL; ++l) { uint32_t n = uint32_t(B.layers[l].size()); std::fwrite(&n, 4, 1, f); std::fwrite(B.layers[l].data(), 1, n, f); }
        std::vector<uint8_t> row(w);
        for (int l = 0; l < NL; ++l)
            for (uint32_t pj = 0; pj < h; ++pj) {
                for (uint32_t pi = 0; pi < w; ++pi) {
                    float m = 0;
                    for (int j = int(pj) * P; j < std::min(G.ny, int(pj + 1) * P); ++j)
                        for (int i = int(pi) * P; i < std::min(G.nx, int(pi + 1) * P); ++i) m = std::max(m, hist[l][size_t(j) * G.nx + i]);
                    row[pi] = uint8_t(std::min(255.0f, std::round(m * 10)));
                }
                std::fwrite(row.data(), 1, w, f);
            }
        std::fclose(f);
    }

    // Per-net statistics as JSON (training data for an ordering / difficulty model): what each net cost the router.
    void dump_nets(const std::string& path) const {
        FILE* f = std::fopen(path.c_str(), "w");
        if (!f) throw std::runtime_error("cannot write " + path);
        std::fprintf(f, "[");
        for (size_t k = 0; k < jobs.size(); ++k) {
            const Job& J = jobs[k];
            std::vector<Seg> segs;
            std::vector<P> vias;
            geometry(J, segs, vias);
            double len = 0;
            for (auto& sg : segs) len += norm(sg.b - sg.a);
            std::fprintf(f, "%s\n{\"name\":\"", k ? "," : "");
            for (unsigned char c : B.nets[J.net].name) {
                if (c == '"' || c == '\\') std::fprintf(f, "\\%c", c);
                else if (c < 0x20) std::fprintf(f, "\\u%04x", c);
                else std::fputc(c, f);
            }
            std::fputc('"', f);
            std::fprintf(f, ",\"cls\":%d,\"width\":%.0f,\"terminals\":%zu,\"connections\":%zu,\"open\":%d,\"dead\":%d,\"conflict_rounds\":%d,"
                           "\"pops\":%zu,\"seconds\":%.3f,\"length\":%.1f,\"vias\":%zu,\"box\":[%.0f,%.0f,%.0f,%.0f]}",
                         J.rc, rcs[J.rc].hw * 2, J.comps.size(), J.comps.size() - 1 + size_t(J.dead), J.unrouted, J.dead, J.conflict_rounds,
                         J.pops, J.seconds, len, vias.size(), J.box.x0, J.box.y0, J.box.x1, J.box.y1);
        }
        std::fprintf(f, "]\n");
        std::fclose(f);
    }

    std::vector<NetResult> results() const {
        std::vector<NetResult> out;
        for (auto& J : jobs) {
            NetResult r;
            r.net = J.net;
            std::vector<P> vias;
            geometry(J, r.segs, vias);
            for (P v : vias) r.vias.push_back({v, rcs[J.rc].via});
            r.open = J.unrouted;
            if (!r.segs.empty() || !r.vias.empty()) out.push_back(std::move(r));
        }
        return out;
    }
};

}  // namespace

// The pours of plane's net on its layer that overlap or touch it, directly or through each other, plane first:
// KiCad joins their fills (and cuts the outline of the higher priority one from the other's).
std::vector<const Item*> pour_group(const Board& B, const Item& plane) {
    std::vector<const Item*> grp{&plane};
    if (plane.net < 0) return grp;
    for (size_t k = 0; k < grp.size(); ++k)
        for (auto& z : B.items)
            if (z.kind == Kind::Plane && z.net == plane.net && z.layer == plane.layer && std::find(grp.begin(), grp.end(), &z) == grp.end() &&
                z.shape.box().overlaps(grp[k]->shape.box(), 1) && shape_dist(z.shape, grp[k]->shape) <= 1)
                grp.push_back(&z);
    return grp;
}

// A pour refilled around other nets' copper, on a grid of o.step: a cell is pour where its centre lies inside the
// plane, at least the clearance (the larger of the classes' and the zone's, from the DSN's zone rules or else
// --pour-clearance) plus half the minimum width from other copper, keepouts and the plane's own outline and
// filleted corners (and the edge clearance from the board edge), and from the fill of each pour of another net that
// wins the overlap (pour_wins). That is the pour eroded by half its minimum width, so its 4-connected islands are
// those of the real fill with necks narrower than the minimum width cut, as KiCad does. The erosion is half a cell
// diagonal short of that: the cells a neck of the eroded pour crosses then stay, however thin it is (seen: a neck
// exactly the minimum width between 0.5 mm zone clearances on kicanvas, which KiCad keeps), at the price of joining
// through necks up to a cell diagonal narrower than the minimum width. The pours of the net that touch it on the
// layer (pour_group) are refilled with it, each by its own rules, and their union is cut into islands.
// A winning pour is knocked out by its own fill (refilled the same way, around this net's copper too, keeping the
// islands that touch its net's copper), not by its outline: KiCad fills zones in priority order and a lower one
// takes what a higher one's island removal left (seen: OpenFC's +3.3V inside +1.1V's outline, where a track cut
// +1.1V's fill off). Deeper than `depth` 6, the winner's outline stands in.
namespace {
struct PourFill {
    Grid g;
    std::vector<int> lab;                // per cell: -2 no pour, else its island
    std::vector<std::vector<int>> isl;   // per island: the indices into `own` of the items touching it
};

PourFill fill_pour(const Board& B, const Options& o, const Item& plane, const std::vector<const Item*>& own,
                   const std::vector<const Item*>& other, int depth) {
    const std::vector<const Item*> grp = pour_group(B, plane);
    Box pb;
    double hm = 1e300;
    for (const Item* P : grp) { pb.add(P->shape.box()); hm = std::min(hm, pour_min_width(B, o, *P) / 2); }
    PourFill F;
    Grid& g = F.g;
    g.s = o.step;
    g.x0 = pb.x0;
    g.y0 = pb.y0;
    g.nx = int((pb.x1 - pb.x0) / g.s) + 1;
    g.ny = int((pb.y1 - pb.y0) / g.s) + 1;
    std::vector<int>& lab = F.lab;
    lab.assign(g.n(), -2);  // -2 outside the pour, -1 pour not labelled yet, >= 0 island
    auto clr = [&](int net) { return B.classes[net < 0 ? 0 : B.nets[net].cls].clearance; };
    std::vector<char> in(g.n());
    std::map<const Item*, PourFill> winners;  // the fills of the pours of other nets that win an overlap
    for (const Item* P : grp) {
        const double he = std::max(0.0, pour_min_width(B, o, *P) / 2 - o.step * M_SQRT1_2);
        std::fill(in.begin(), in.end(), 0);
        raster(g, P->shape, 0, [&](uint32_t c) { in[c] = 1; });
        auto block = [&](uint32_t c) { in[c] = 0; };
        Shape rim = P->shape;
        rim.poly.clear();
        rim.r = 0;
        raster(g, rim, he, block);
        if (P->zone >= 0) fillet_cuts(g, P->shape.poly, B.zones[P->zone].fillet, he, block);
        const Box pp = P->shape.box(), reach = pp.grown(he + 5000);
        if (B.has_outline) {
            Shape edge = B.outline;
            edge.poly.clear();
            raster(g, edge, o.edge_clearance + he, block);
        }
        const double pc = std::max(clr(P->net), pour_clearance(B, o, *P));
        for (const Item* it : other) {
            if ((it->layer >= 0 && it->layer != P->layer) || it->kind == Kind::Plane || !it->shape.box().overlaps(reach)) continue;
            raster(g, it->shape, std::max(pc, clr(it->net)) + he, block);
        }
        for (auto& z : B.items) {
            if (z.kind != Kind::Plane || z.net == P->net || z.layer != P->layer || !pour_wins(B, z, *P) || !z.shape.box().overlaps(reach)) continue;
            const double c = std::max({pc, clr(z.net), pour_clearance(B, o, z)}) + he;
            if (depth >= 6 || z.net < 0) { raster(g, z.shape, c, block); continue; }
            auto w = winners.find(&z);
            if (w == winners.end()) {
                std::vector<const Item*> zown, zother(own);
                for (const Item* it : other) (it->net == z.net ? zown : zother).push_back(it);
                w = winners.emplace(&z, fill_pour(B, o, z, zown, zother, depth + 1)).first;
            }
            const PourFill& Z = w->second;
            auto kept = [&](int i, int j) {
                if (i < 0 || j < 0 || i >= Z.g.nx || j >= Z.g.ny) return false;
                int l = Z.lab[size_t(j) * Z.g.nx + i];
                return l >= 0 && !Z.isl[l].empty();
            };
            // this pour's cells in a kept cell of the winner's fill, and within c of its edge cells
            for (size_t q = 0; q < g.n(); ++q) {
                if (!in[q]) continue;
                ::P at = g.center(uint32_t(q));
                if (kept(int(std::floor((at.x - Z.g.x0) / Z.g.s)), int(std::floor((at.y - Z.g.y0) / Z.g.s)))) in[q] = 0;
            }
            for (int j = 0; j < Z.g.ny; ++j)
                for (int i = 0; i < Z.g.nx; ++i)
                    if (kept(i, j) && !(kept(i - 1, j) && kept(i + 1, j) && kept(i, j - 1) && kept(i, j + 1)))
                        raster(g, Shape::circle(Z.g.center(i, j), Z.g.s * M_SQRT1_2), c, block);
        }
        for (auto& k : B.keepouts)
            if (k.wires && (k.layer < 0 || k.layer == P->layer) && k.shape.box().overlaps(reach)) raster(g, k.shape, he, block);
        for (size_t c = 0; c < g.n(); ++c)
            if (in[c]) lab[c] = -1;
    }
    int n = 0;
    std::vector<uint32_t> q;
    for (size_t c0 = 0; c0 < g.n(); ++c0) {
        if (lab[c0] != -1) continue;
        q.assign(1, uint32_t(c0));
        lab[c0] = n;
        while (!q.empty()) {
            uint32_t c = q.back();
            q.pop_back();
            int i = int(c % g.nx), j = int(c / g.nx);
            auto go = [&](int a, int b) {
                if (a < 0 || b < 0 || a >= g.nx || b >= g.ny) return;
                uint32_t d = uint32_t(b) * g.nx + a;
                if (lab[d] == -1) { lab[d] = n; q.push_back(d); }
            };
            go(i - 1, j); go(i + 1, j); go(i, j - 1); go(i, j + 1);
        }
        ++n;
    }
    // the net's copper touches an island where it comes within half the minimum width (plus half a cell) of its cells
    F.isl.resize(n);
    for (size_t k = 0; k < own.size(); ++k) {
        const Item* it = own[k];
        if ((it->layer >= 0 && it->layer != plane.layer) || it->kind == Kind::Plane || !it->shape.box().overlaps(pb, hm + g.s)) continue;
        raster(g, it->shape, hm + g.s / 2, [&](uint32_t c) {
            if (lab[c] >= 0 && (F.isl[lab[c]].empty() || F.isl[lab[c]].back() != int(k))) F.isl[lab[c]].push_back(int(k));
        });
    }
    return F;
}
}  // namespace

std::vector<std::vector<int>> pour_islands(const Board& B, const Options& o, const Item& plane, const std::vector<const Item*>& own,
                                           const std::vector<const Item*>& other) {
    std::vector<std::vector<int>> isl = fill_pour(B, o, plane, own, other, 0).isl;
    isl.erase(std::remove_if(isl.begin(), isl.end(), [](auto& v) { return v.empty(); }), isl.end());
    return isl;
}

std::vector<NetResult> route_board(const Board& B, const Options& o, RouteStats& S) {
    double t0 = now();
    Router R(B, o);
#ifdef PCB_GPU
    if (o.gpu_pops > 0) gpu_configure(o.gpu_slots, size_t(o.gpu_mem_gb * (1 << 30) / 11));  // ~11 bytes per state on the GPU
#endif
    R.setup();
    S.dead = R.dead_by_net;
    S.dead.resize(B.nets.size(), 0);
    S.seconds_setup = now() - t0;
    if (o.verbose)
        std::fprintf(stderr, "grid %dx%d x %d layers at %.0f um, %zu classes, %zu nets to route, setup %.2f s\n",
                     R.G.nx, R.G.ny, R.NL, R.G.s, R.rcs.size(), R.jobs.size(), S.seconds_setup);
    if (o.verbose && R.necked)
        std::fprintf(stderr, "%d fine-pitch pads reachable only by a neck-down (a stub narrower than the class width)\n", R.necked);
    if (o.dump_r > 0 && !R.jobs.empty()) {
        // '#' blocked, '.' free, 'o' this net's own copper, 'T' terminal cell
        const auto& J = R.jobs[0];
        const auto& K = R.rcs[J.rc];
        std::unordered_set<uint32_t> term;
        for (auto& C : J.comps) term.insert(C.states.begin(), C.states.end());
        int ci = int((o.dump_x * 1000 - R.G.x0) / R.G.s), cj = int((-o.dump_y * 1000 - R.G.y0) / R.G.s), rr = int(o.dump_r * 1000 / R.G.s);
        for (int l = 0; l < R.NL; ++l) {
            std::fprintf(stderr, "%s (via grid on the right)\n", B.layers[l].c_str());
            for (int j = cj + rr; j >= cj - rr; --j) {
                for (int pass = 0; pass < 2; ++pass) {
                    for (int i = ci - rr; i <= ci + rr; ++i) {
                        if (i < 0 || j < 0 || i >= R.G.nx || j >= R.G.ny) { std::fputc(' ', stderr); continue; }
                        uint32_t c = uint32_t(j) * R.G.nx + i;
                        uint16_t v = pass ? K.vst[c] : K.st[l][c];
                        char ch = v == 0 ? '.' : v == J.net + 1 ? 'o' : '#';
                        if (!pass && term.count(uint32_t(l) * R.G.n() + c)) ch = 'T';
                        std::fputc(ch, stderr);
                    }
                    std::fputs("  ", stderr);
                }
                std::fputc('\n', stderr);
            }
        }
    }
    double t1 = now();
    R.run(S);
    if (o.verbose && (!o.ml_prior.empty() || o.ml_prior_get) && o.ml_corridor > 0) std::fprintf(stderr, "%ld searches left the learned corridor\n", long(n_ml_fallback));
    if (o.verbose && (n_gpu || n_gpu_none || n_gpu_fail))
        std::fprintf(stderr, "%ld searches finished on the GPU, %ld found unreachable there (%.1f s, %.0f sweeps each); %ld back on the CPU (%ld solve failed, %ld chain, %ld via spacing, %ld via near an old one)\n",
                     long(n_gpu), long(n_gpu_none), n_gpu_us / 1e6, double(n_gpu_sweeps) / std::max(1L, long(n_gpu) + long(n_gpu_none)), long(n_gpu_fail),
                     long(n_gpu_why[0]), long(n_gpu_why[1]), long(n_gpu_why[2]), long(n_gpu_why[3]));
    S.seconds_route = now() - t1;
    if (!o.dump_hist.empty()) R.dump_hist(o.dump_hist, S.iterations);
    if (!o.dump_nets.empty()) R.dump_nets(o.dump_nets);
    if (o.verbose > 1)
        std::fprintf(stderr, "  phases: search setup %.2f thread-s, stamping %.2f s, conflict counting %.2f s; %ld searches: heuristic %.2f, heap init %.2f, loop %.2f thread-s; %ld pops, %ld stamps\n",
                     t_setupsearch, t_stamp, t_conf, long(n_search), us_dmap / 1e6, us_init / 1e6, us_loop / 1e6, long(n_pops), long(n_stamp));
    return R.results();
}

// --plane-mode refill: the router takes a pour as one piece of its net wherever it is, but other nets' tracks may
// cut the refilled pour into islands (smoothing too: it doesn't know the pours). Repair, after smoothing: the nets
// left open by that are routed again, with every route so far fixed (static copper), their pours dropped, and their
// copper that touches a common island joined (one group). A few rounds, as a repair route may cut another net's
// pour. Returns the number of nets routed again.
int repair_pours(const Board& B, const Options& o, std::vector<NetResult>& R, RouteStats& S) {
    int repaired = 0;
    if (!o.plane_refill || o.only.size()) return 0;
    std::vector<char> has_plane(B.nets.size(), 0);
    for (auto& it : B.items)
        if (it.kind == Kind::Plane && it.net >= 0) has_plane[it.net] = 1;
    if (std::find(has_plane.begin(), has_plane.end(), 1) == has_plane.end()) return 0;
    std::vector<int> done(B.nets.size(), 0);  // repair rounds a net has had
    for (int round = 0; round < 3; ++round) {
        double t0 = now();
        std::vector<int> open, routed_open(B.nets.size(), 0);
        auto extra = result_items(B, R);
        open_connections(B, o, extra, 0, nullptr, nullptr, &open);
        // what the routing already counted open: a net's result, or its dead terminals where it got none (all its
        // terminals unreachable: counting them again here made a board's open exceed its connections)
        for (size_t n = 0; n < S.dead.size() && n < routed_open.size(); ++n) routed_open[n] = S.dead[n];
        for (auto& r : R) routed_open[r.net] = r.open;
        std::vector<int> fix;
        for (size_t n = 0; n < B.nets.size(); ++n)
            if (has_plane[n] && open[n] > routed_open[n] && done[n] < 2) fix.push_back(int(n));
        if (fix.empty()) break;
        repaired += int(fix.size());
        Board B2 = B;
        for (auto& N : B2.nets) N.has_pins = false;
        for (size_t n = 0; n < extra.size(); ++n)  // the routes so far: fixed copper
            for (auto& it : extra[n]) {
                B2.items.push_back(it);
                if (std::find(fix.begin(), fix.end(), int(n)) != fix.end()) B2.nets[n].items.push_back(int(B2.items.size() - 1));
            }
        std::vector<const Item*> all;
        for (auto& it : B2.items) all.push_back(&it);
        for (int n : fix) {
            Net& N = B2.nets[n];
            N.has_pins = true;
            ++done[n];
            std::vector<const Item*> own, other;
            std::vector<int> own_idx, planes;
            for (int k : N.items) {
                if (B2.items[k].kind == Kind::Plane) planes.push_back(k);
                else { own.push_back(&B2.items[k]); own_idx.push_back(k); }
            }
            for (auto* it : all)
                if (it->net != n && it->kind != Kind::Plane) other.push_back(it);
            std::map<int, int> up;  // groups joined by an island
            std::function<int(int)> find = [&](int a) { auto i = up.find(a); return i == up.end() || i->second == a ? a : i->second = find(i->second); };
            std::set<const Item*> seen;  // a pour refilled with another of its group already
            for (int p : planes) {
                if (seen.count(&B2.items[p])) continue;
                for (const Item* z : pour_group(B2, B2.items[p])) seen.insert(z);
                for (auto& g : pour_islands(B2, o, B2.items[p], own, other))
                    for (size_t k = 1; k < g.size(); ++k) {
                        int a = find(own[g[0]]->group), b = find(own[g[k]]->group);
                        if (a != b) up[b] = a;
                    }
            }
            for (int k : own_idx) B2.items[k].group = find(B2.items[k].group);
            N.items = own_idx;
        }
        Options o2 = o;
        o2.ml_prior.clear();
        o2.ml_prior_get = nullptr;
        o2.ml_congestion.clear();
        o2.order.clear();
        o2.dump_hist.clear();
        o2.dump_nets.clear();
        o2.dump_r = 0;
        o2.verbose = std::min(o.verbose, 1);
        RouteStats S2;
        auto R2 = route_board(B2, o2, S2);
        int before = 0, before_conns = 0;
        for (int n : fix) before += routed_open[n];
        for (int n : fix) before_conns += size_t(n) < S.conns.size() ? S.conns[size_t(n)] : 0;
        std::map<int, size_t> at;
        for (size_t k = 0; k < R.size(); ++k) at[R[k].net] = k;
        for (auto& r : R2) {
            auto i = at.find(r.net);
            if (i == at.end()) { at[r.net] = R.size(); R.push_back(r); continue; }
            NetResult& d = R[i->second];
            d.segs.insert(d.segs.end(), r.segs.begin(), r.segs.end());
            d.vias.insert(d.vias.end(), r.vias.begin(), r.vias.end());
            d.open = r.open;
        }
        for (int n : fix)  // a repaired net that needed no copper is joined now
            if (std::none_of(R2.begin(), R2.end(), [&](const NetResult& r) { return r.net == n; }))
                if (auto i = at.find(n); i != at.end()) R[i->second].open = 0;
        S.unrouted += S2.unrouted - before;
        S.connections += S2.connections - before_conns;  // the islands split the net into more (or fewer) pieces
        for (int n : fix) if (size_t(n) < S.conns.size() && size_t(n) < S2.conns.size()) S.conns[size_t(n)] = S2.conns[size_t(n)];
        for (int n : fix) if (size_t(n) < S.dead.size() && size_t(n) < S2.dead.size()) S.dead[size_t(n)] = S2.dead[size_t(n)];
        S.seconds_route += now() - t0;
        if (o.verbose)
            std::fprintf(stderr, "refilled pours: %zu nets cut into islands, routed again: %d connections open (were %d), %.2f s\n",
                         fix.size(), S2.unrouted, before, now() - t0);
    }
    return repaired;
}

// ---- independent check of the output against the exact geometry ----
int check_routes(const Board& B, const Options& o, const std::vector<NetResult>& RS, int verbose) {
    struct Cu { int net, layer; Shape sh; Box box; bool fresh; };
    std::vector<Cu> cu;
    for (auto& it : B.items)
        if (it.kind != Kind::Plane) cu.push_back({it.net, it.layer, it.shape, it.shape.box(), false});
    for (auto& r : RS) {
        for (auto& s : r.segs) {
            Shape sh = Shape::path({s.a, s.b}, s.width / 2);
            cu.push_back({r.net, s.layer, sh, sh.box(), true});
        }
        for (auto& v : r.vias) {
            Shape sh = Shape::circle(v.at, B.vias[v.type].diameter / 2);
            cu.push_back({r.net, -1, sh, sh.box(), true});
        }
    }
    auto clr = [&](int net) { return B.classes[net < 0 ? 0 : B.nets[net].cls].clearance; };
    int bad = 0;
    auto report = [&](const std::string& what, const Cu& a) {
        if (verbose && bad < 20) {
            P c = a.sh.center();
            std::fprintf(stderr, "  violation: %s, net %s at (%.3f, %.3f) mm\n", what.c_str(),
                         a.net >= 0 ? B.nets[a.net].name.c_str() : "-", c.x / 1000, -c.y / 1000);
        }
        ++bad;
    };
    // bucket the copper by 2 mm cells
    const double BK = 2000;
    Box all;
    for (auto& c : cu) all.add(c.box);
    int bx = int((all.x1 - all.x0) / BK) + 1, by = int((all.y1 - all.y0) / BK) + 1;
    std::vector<std::vector<int>> bucket(size_t(bx) * by);
    auto range = [&](const Box& b, double g, auto f) {
        int i0 = std::max(0, int((b.x0 - g - all.x0) / BK)), i1 = std::min(bx - 1, int((b.x1 + g - all.x0) / BK));
        int j0 = std::max(0, int((b.y0 - g - all.y0) / BK)), j1 = std::min(by - 1, int((b.y1 + g - all.y0) / BK));
        for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) f(size_t(j) * bx + i);
    };
    for (size_t k = 0; k < cu.size(); ++k) range(cu[k].box, 0, [&](size_t q) { bucket[q].push_back(int(k)); });
    double maxc = 0;
    for (auto& c : B.classes) maxc = std::max(maxc, c.clearance);
    for (size_t k = 0; k < cu.size(); ++k) {
        const Cu& a = cu[k];
        if (!a.fresh) continue;
        std::unordered_set<int> seen;
        bool hit = false;
        range(a.box, maxc, [&](size_t q) {
            for (int j : bucket[q]) {
                if (hit || j == int(k) || !seen.insert(j).second) continue;
                const Cu& b = cu[j];
                if (b.net == a.net && a.net >= 0) continue;
                if (a.layer >= 0 && b.layer >= 0 && a.layer != b.layer) continue;
                if (b.fresh && j < int(k)) continue;  // pair already checked
                double need = std::max(clr(a.net), clr(b.net)) - 1;
                if (!a.box.overlaps(b.box, need)) continue;
                if (shape_dist(a.sh, b.sh) < need) { report("clearance to " + std::string(b.fresh ? "new" : "existing") + " copper of " + (b.net >= 0 ? B.nets[b.net].name : "-"), a); hit = true; }
            }
        });
        for (auto& ko : B.keepouts) {
            if (hit) break;
            bool via = a.layer < 0;
            if ((via && !ko.vias) || (!via && !ko.wires)) continue;
            if (!via && ko.layer >= 0 && ko.layer != a.layer) continue;
            if (ko.shape.box().overlaps(a.box) && shape_dist(a.sh, ko.shape) < 1) { report("keepout", a); hit = true; }
        }
        if (!hit && B.has_outline) {
            Shape edge = B.outline;
            edge.poly.clear();
            bool inside = true;
            for (auto& [p, q] : a.sh.segs) inside &= point_in_poly(B.outline.poly, p);
            if (!inside || shape_dist(a.sh, edge) < o.edge_clearance - 1) report("board edge", a);
        }
    }
    return bad;
}

void write_ses(const Board& B, const std::vector<NetResult>& R, const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) throw std::runtime_error("cannot write " + path);
    double k = B.resolution;
    auto I = [&](double v) { return (long long)std::llround(v * k); };
    std::fprintf(f, "(session %s\n  (base_design %s)\n  (routes\n    (resolution um %g)\n", B.session_name.c_str(),
                 B.session_name.c_str(), k);
    std::fprintf(f, "    (parser\n      (host_cad \"pcbroute\")\n      (host_version \"0.1\")\n    )\n    (library_out\n");
    for (auto& v : B.vias) {
        std::fprintf(f, "      (padstack %s\n", v.raw.c_str());
        for (auto& l : B.layers) std::fprintf(f, "        (shape (circle %s %lld 0 0))\n", l.c_str(), I(v.diameter));
        std::fprintf(f, "        (attach off)\n      )\n");
    }
    std::fprintf(f, "    )\n    (network_out\n");
    for (auto& r : R) {
        std::fprintf(f, "      (net %s\n", B.nets[r.net].raw.c_str());
        for (auto& s : r.segs)
            std::fprintf(f, "        (wire (path %s %lld %lld %lld %lld %lld))\n", B.layers[s.layer].c_str(), I(s.width),
                         I(s.a.x), I(s.a.y), I(s.b.x), I(s.b.y));
        for (auto& v : r.vias)
            std::fprintf(f, "        (via %s %lld %lld)\n", B.vias[v.type].raw.c_str(), I(v.at.x), I(v.at.y));
        std::fprintf(f, "      )\n");
    }
    std::fprintf(f, "    )\n  )\n)\n");
    std::fclose(f);
}
