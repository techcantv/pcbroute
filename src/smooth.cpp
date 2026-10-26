// Post-routing clean-up: the grid paths are staircases. Each chain of track between two fixed points (a pad, a
// via, a junction) is redrawn greedily with the fewest octilinear segments: from each vertex, the farthest vertex
// down the chain reachable with one straight 0/45/90 degree segment, or two of them (one bend), that clears every
// other net's copper, the keepouts and the board edge exactly. Nets are smoothed one after the other, each against
// the others' latest copper.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <unordered_set>

#include "router.h"

namespace {

struct Cu {
    int net, layer;  // layer -1: every layer (a via, a through pad)
    Shape sh;
    Box box;
    double clr;
    bool keepout;
    bool fresh = false;  // routed copper (the static copper gets CLR_SAFETY more, see geom.h)
};

struct Index {
    const Board& B;
    double edge_clr;
    std::vector<Cu> cu;
    std::vector<char> alive;
    Box all;
    double BK = 1000;
    int bx = 0, by = 0;
    std::vector<std::vector<int>> bucket;
    Shape edge;
    double maxclr = 0;

    Index(const Board& b, double ec) : B(b), edge_clr(ec) {
        edge = B.outline;
        edge.poly.clear();
        for (auto& c : B.classes) maxclr = std::max(maxclr, c.clearance);
    }
    double clr(int net) const { return B.classes[net < 0 ? 0 : B.nets[net].cls].clearance; }
    void init(const Box& b) {
        all = b.grown(5000);
        bx = int((all.x1 - all.x0) / BK) + 1;
        by = int((all.y1 - all.y0) / BK) + 1;
        bucket.assign(size_t(bx) * by, {});
    }
    template <class F> void cells(const Box& b, double g, F f) const {
        int i0 = std::clamp(int((b.x0 - g - all.x0) / BK), 0, bx - 1), i1 = std::clamp(int((b.x1 + g - all.x0) / BK), 0, bx - 1);
        int j0 = std::clamp(int((b.y0 - g - all.y0) / BK), 0, by - 1), j1 = std::clamp(int((b.y1 + g - all.y0) / BK), 0, by - 1);
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) f(size_t(j) * bx + i);
    }
    int add(Cu c) {
        int k = int(cu.size());
        cells(c.box, 0, [&](size_t q) { bucket[q].push_back(k); });
        cu.push_back(std::move(c));
        alive.push_back(1);
        return k;
    }
    // A track of half width hw on layer l along ab clears everything that isn't net `net`.
    bool clear(int l, P a, P b, double hw, int net) const {
        Shape cap = Shape::path({a, b}, hw);
        Box bb = cap.box();
        if (B.has_outline && (!point_in_poly(B.outline.poly, a) || !point_in_poly(B.outline.poly, b) ||
                              shape_dist(cap, edge) < edge_clr))
            return false;
        double c0 = clr(net);
        bool ok = true;
        cells(bb, std::max(c0, maxclr), [&](size_t q) {
            if (!ok) return;
            for (int k : bucket[q]) {
                if (!alive[k]) continue;
                const Cu& c = cu[k];
                if (c.layer >= 0 && c.layer != l) continue;
                if (c.keepout) {
                    if (c.box.overlaps(bb, 1) && shape_dist(cap, c.sh) < 1) { ok = false; return; }  // as check_routes judges it
                    continue;
                }
                if (c.net == net && net >= 0) continue;
                double need = std::max(c0, c.clr) + (c.fresh ? 0 : CLR_SAFETY);  // the static copper: see geom.h
                if (c.box.overlaps(bb, need) && shape_dist(cap, c.sh) < need) { ok = false; return; }
            }
        });
        return ok;
    }
};

struct Key {
    long long x, y;
    int l;
    bool operator<(const Key& o) const { return x < o.x || (x == o.x && (y < o.y || (y == o.y && l < o.l))); }
};
Key key(P p, int l) { return {std::llround(p.x * 100), std::llround(p.y * 100), l}; }

bool octilinear(P a, P b) {
    double dx = std::fabs(b.x - a.x), dy = std::fabs(b.y - a.y);
    return dx < 1e-6 || dy < 1e-6 || std::fabs(dx - dy) < 1e-6;
}

}  // namespace

int smooth_routes(const Board& B, const Options& o, std::vector<NetResult>& R) {
    Index ix(B, o.edge_clearance);
    Box all;
    if (B.has_outline) all = B.outline.box();
    for (auto& it : B.items) all.add(it.shape.box());
    ix.init(all);
    for (auto& it : B.items)
        if (it.kind != Kind::Plane) ix.add({it.net, it.layer, it.shape, it.shape.box(), ix.clr(it.net), false});
    for (auto& k : B.keepouts)
        if (k.wires) ix.add({-1, k.layer, k.shape, k.shape.box(), 0, true});
    std::vector<std::vector<int>> mine(R.size());
    for (size_t r = 0; r < R.size(); ++r) {
        for (auto& s : R[r].segs) {
            Shape sh = Shape::path({s.a, s.b}, s.width / 2);
            mine[r].push_back(ix.add({R[r].net, s.layer, sh, sh.box(), ix.clr(R[r].net), false, true}));
        }
        for (auto& v : R[r].vias) {
            Shape sh = Shape::circle(v.at, B.vias[v.type].diameter / 2);
            ix.add({R[r].net, -1, sh, sh.box(), ix.clr(R[r].net), false, true});
        }
    }
    int before = 0, after = 0;
    for (size_t r = 0; r < R.size(); ++r) {
        NetResult& N = R[r];
        before += int(N.segs.size());
        // vertices that must stay: vias, pad centres, ends and junctions (degree != 2 on the layer)
        std::map<Key, int> degree;
        for (auto& s : N.segs) { ++degree[key(s.a, s.layer)]; ++degree[key(s.b, s.layer)]; }
        std::set<std::pair<long long, long long>> fixed_xy;
        for (auto& v : N.vias) fixed_xy.insert({std::llround(v.at.x * 100), std::llround(v.at.y * 100)});
        for (auto& it : B.items)
            if (it.net == N.net && it.kind == Kind::Pad) fixed_xy.insert({std::llround(it.center.x * 100), std::llround(it.center.y * 100)});
        auto is_fixed = [&](P p, int l) {
            return degree[key(p, l)] != 2 || fixed_xy.count({std::llround(p.x * 100), std::llround(p.y * 100)});
        };
        // chain the segments: walk from each fixed vertex along unvisited segments
        std::map<Key, std::vector<int>> at;
        for (size_t k = 0; k < N.segs.size(); ++k) { at[key(N.segs[k].a, N.segs[k].layer)].push_back(int(k)); at[key(N.segs[k].b, N.segs[k].layer)].push_back(int(k)); }
        std::vector<char> used(N.segs.size(), 0);
        std::vector<std::pair<std::vector<P>, std::pair<int, double>>> chains;  // points, (layer, width)
        auto walk = [&](int k0, P start) {
            std::vector<P> pts{start};
            int l = N.segs[k0].layer;
            double w = N.segs[k0].width;
            int k = k0;
            P cur = start;
            while (true) {
                used[k] = 1;
                const Seg& s = N.segs[k];
                P nx = (key(s.a, l).x == key(cur, l).x && key(s.a, l).y == key(cur, l).y) ? s.b : s.a;
                pts.push_back(nx);
                cur = nx;
                if (is_fixed(cur, l)) break;
                int nk = -1;
                for (int c : at[key(cur, l)])
                    if (!used[c] && N.segs[c].width == w) nk = c;
                if (nk < 0) break;
                k = nk;
            }
            chains.push_back({pts, {l, w}});
        };
        for (size_t k = 0; k < N.segs.size(); ++k) {
            if (used[k]) continue;
            const Seg& s = N.segs[k];
            if (is_fixed(s.a, s.layer)) walk(int(k), s.a);
            else if (is_fixed(s.b, s.layer)) walk(int(k), s.b);
        }
        for (size_t k = 0; k < N.segs.size(); ++k)
            if (!used[k]) walk(int(k), N.segs[k].a);  // loops without a fixed vertex
        // take this net's tracks out of the index while it is redrawn
        for (int k : mine[r]) ix.alive[k] = 0;
        std::vector<Seg> out;
        for (auto& [pts, lw] : chains) {
            auto [l, w] = lw;
            double hw = w / 2;
            std::vector<P> np{pts[0]};
            size_t i = 0;
            while (i + 1 < pts.size()) {
                size_t best = i + 1;
                P bend{};
                bool bent = false;
                for (size_t j = std::min(pts.size() - 1, i + 400); j > i + 1; --j) {
                    P a = pts[i], b = pts[j];
                    if (octilinear(a, b)) {
                        if (ix.clear(l, a, b, hw, N.net)) { best = j; bent = false; break; }
                        continue;
                    }
                    double dx = b.x - a.x, dy = b.y - a.y, m = std::min(std::fabs(dx), std::fabs(dy));
                    P d1{a.x + std::copysign(m, dx), a.y + std::copysign(m, dy)};      // diagonal first
                    P d2{b.x - std::copysign(m, dx), b.y - std::copysign(m, dy)};      // straight first
                    bool got = false;
                    for (P c : {d1, d2})
                        if (ix.clear(l, a, c, hw, N.net) && ix.clear(l, c, b, hw, N.net)) { bend = c; got = true; break; }
                    if (got) { best = j; bent = true; break; }
                }
                if (bent) np.push_back(bend);
                np.push_back(pts[best]);
                i = best;
            }
            // drop collinear interior points
            std::vector<P> q{np[0]};
            for (size_t k = 1; k < np.size(); ++k) {
                if (q.size() >= 2) {
                    P u = q.back() - q[q.size() - 2], v = np[k] - q.back();
                    if (std::fabs(cross(u, v)) < 1e-6 * (norm(u) * norm(v) + 1) && dot(u, v) > 0) { q.back() = np[k]; continue; }
                }
                if (norm(np[k] - q.back()) > 1e-6) q.push_back(np[k]);
            }
            for (size_t k = 1; k < q.size(); ++k) out.push_back({l, q[k - 1], q[k], w});
        }
        N.segs = out;
        mine[r].clear();
        for (auto& s : N.segs) {
            Shape sh = Shape::path({s.a, s.b}, s.width / 2);
            mine[r].push_back(ix.add({N.net, s.layer, sh, sh.box(), ix.clr(N.net), false, true}));
        }
        after += int(N.segs.size());
    }
    if (o.verbose) std::fprintf(stderr, "smoothed: %d track segments -> %d\n", before, after);
    return after;
}
