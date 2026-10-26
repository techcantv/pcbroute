// pcbroute --eval: judge a Specctra session from any router against the board's own rules, so that routers can
// be compared by one yardstick: open connections (pieces of a net its copper plus the session leave apart),
// clearance / keepout / edge violations (check_routes), track length and via count.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>

#include "dsn.h"
#include "router.h"
#include "sexpr.h"

namespace {

double unit_scale(const std::string& u) {
    if (u == "um") return 1;
    if (u == "mm") return 1000;
    if (u == "mil") return 25.4;
    if (u == "inch") return 25400;
    if (u == "cm") return 10000;
    throw std::runtime_error("SES: unknown unit " + u);
}

// connected pieces of a set of copper items (pads, planes, wires, vias): items of one group are joined, and so
// are items on a common layer (or a through item) whose copper touches. --plane-mode refill: a plane joins nothing
// by its outline; each of its refilled islands (around `other`, the other nets' copper) joins the items touching it,
// and a plane is no piece of its own. No `other`: the planes join nothing.
int pieces(const Board& B, const Options& o, const std::vector<Item>& extra, const std::vector<int>& idx,
           const std::vector<const Item*>* other) {
    std::vector<const Item*> it;
    for (int k : idx) it.push_back(&B.items[k]);
    for (auto& e : extra) it.push_back(&e);
    size_t n = it.size();
    std::vector<int> up(n);
    std::iota(up.begin(), up.end(), 0);
    std::function<int(int)> find = [&](int a) { return up[a] == a ? a : up[a] = find(up[a]); };
    std::vector<Box> bx(n);
    for (size_t a = 0; a < n; ++a) bx[a] = it[a]->shape.box();
    auto refilled = [&](size_t a) { return o.plane_refill && it[a]->kind == Kind::Plane; };
    for (size_t a = 0; a < n; ++a)
        for (size_t b = a + 1; b < n; ++b) {
            if (find(int(a)) == find(int(b)) || refilled(a) || refilled(b)) continue;
            const Item &A = *it[a], &Bi = *it[b];
            bool join = A.group == Bi.group && A.group >= 0;
            if (!join && (A.layer < 0 || Bi.layer < 0 || A.layer == Bi.layer) && bx[a].overlaps(bx[b], 2))
                join = shape_dist(A.shape, Bi.shape) <= 1;
            if (join) up[find(int(a))] = find(int(b));
        }
    std::set<const Item*> seen;  // a pour refilled with another of its group already (pour_group)
    for (size_t a = 0; a < n && other; ++a)
        if (refilled(a) && !seen.count(it[a])) {
            for (const Item* z : pour_group(B, *it[a])) seen.insert(z);
            for (auto& g : pour_islands(B, o, *it[a], it, *other))
                for (size_t k = 1; k < g.size(); ++k) up[find(g[k])] = find(g[0]);
        }
    int k = 0;
    for (size_t a = 0; a < n; ++a) k += find(int(a)) == int(a) && !refilled(a);
    return k;
}

}  // namespace

int open_connections(const Board& B, const Options& o, const std::vector<std::vector<Item>>& extra, int verbose, int* nets,
                     int* connections, std::vector<int>* per_net) {
    if (per_net) per_net->assign(B.nets.size(), 0);
    // --plane-mode refill: the connections to make are those of the net's copper without its pours (a pour joins
    // what it touches only where it is still copper once refilled, which depends on the routes); after routing, the
    // pours refilled around everything else (the other nets' copper, the board's and the routes')
    std::vector<const Item*> all;
    std::vector<int> owner;
    bool planes = false;
    for (auto& it : B.items) {
        planes |= it.kind == Kind::Plane;
        if (it.kind != Kind::Plane) { all.push_back(&it); owner.push_back(it.net); }
    }
    for (size_t n = 0; n < extra.size(); ++n)
        for (auto& it : extra[n]) { all.push_back(&it); owner.push_back(int(n)); }
    int nn = 0, nc = 0, open = 0;
    std::vector<const Item*> other;
    for (size_t n = 0; n < B.nets.size(); ++n) {
        const Net& N = B.nets[n];
        if (!N.has_pins) continue;
        other.clear();
        bool mine = false;
        if (o.plane_refill && planes) {
            for (int k : N.items) mine |= B.items[k].kind == Kind::Plane;
            if (mine)
                for (size_t k = 0; k < all.size(); ++k)
                    if (owner[k] != int(n)) other.push_back(all[k]);
        }
        int before = pieces(B, o, {}, N.items, nullptr);
        if (before < 2) continue;
        ++nn;
        nc += before - 1;
        int after = pieces(B, o, n < extra.size() ? extra[n] : std::vector<Item>{}, N.items, &other);
        if (per_net) (*per_net)[n] = after - 1;
        open += after - 1;
        if (after > 1 && verbose) std::fprintf(stderr, "  open: %s, %d of %d connections\n", N.name.c_str(), after - 1, before - 1);
    }
    if (nets) *nets = nn;
    if (connections) *connections = nc;
    return open;
}

std::vector<std::vector<Item>> result_items(const Board& B, const std::vector<NetResult>& R) {
    std::vector<std::vector<Item>> extra(B.nets.size());
    int group = 1 << 30;
    for (auto& r : R) {
        for (auto& s : r.segs) {
            Item it;
            it.kind = Kind::Wire;
            it.net = r.net;
            it.layer = s.layer;
            it.group = group++;
            it.shape = Shape::path({s.a, s.b}, s.width / 2);
            it.center = it.shape.center();
            extra[r.net].push_back(std::move(it));
        }
        for (auto& v : r.vias) {
            Item it;
            it.kind = Kind::Via;
            it.net = r.net;
            it.layer = -1;
            it.group = group++;
            it.shape = Shape::circle(v.at, B.vias[v.type].diameter / 2);
            it.center = v.at;
            extra[r.net].push_back(std::move(it));
        }
    }
    return extra;
}

int eval_session(Board B, const Options& o, const std::string& path, int verbose) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    Sx root = parse_sexpr(ss.str());
    const Sx* routes = root.find("routes");
    if (!routes) throw std::runtime_error("SES: no (routes ...)");
    double k = 1;  // session units per micrometre
    if (auto r = routes->find("resolution")) k = r->num(2) / unit_scale(r->str(1));
    // via padstacks the session defines (a router may add its own)
    if (auto lib = routes->find("library_out"))
        for (auto ps : lib->all("padstack")) {
            bool known = false;
            for (auto& v : B.vias) known |= v.name == ps->str(1);
            if (known) continue;
            ViaType vt;
            vt.name = ps->str(1);
            vt.raw = ps->kids[1].raw;
            for (auto sh : ps->all("shape"))
                for (auto& c : sh->kids)
                    if (c.list && c.head() == "circle") vt.diameter = std::max(vt.diameter, c.num(2) / k);
            B.vias.push_back(vt);
        }
    std::vector<NetResult> R;
    std::vector<std::vector<Item>> extra(B.nets.size());
    double length = 0;
    int nvias = 0, group = 1 << 30;
    if (auto nw = routes->find("network_out"))
        for (auto n : nw->all("net")) {
            auto ni = B.net_index.find(n->kids[1].atom);
            if (ni == B.net_index.end()) { std::fprintf(stderr, "eval: unknown net %s\n", n->kids[1].atom.c_str()); continue; }
            NetResult r;
            r.net = ni->second;
            for (auto& c : n->kids) {
                if (!c.list) continue;
                if (c.head() == "wire") {
                    for (auto& p : c.kids) {
                        if (!p.list || p.head() != "path") continue;
                        int layer = B.layer_index(p.str(1));
                        double w = p.num(2) / k;
                        std::vector<P> pts;
                        for (size_t i = 3; i + 1 < p.kids.size() && !p.kids[i].list && !p.kids[i + 1].list; i += 2)
                            pts.push_back({p.num(i) / k, p.num(i + 1) / k});
                        for (size_t i = 1; i < pts.size(); ++i) {
                            r.segs.push_back({layer, pts[i - 1], pts[i], w});
                            length += norm(pts[i] - pts[i - 1]);
                        }
                        Item it;
                        it.kind = Kind::Wire;
                        it.net = r.net;
                        it.layer = layer;
                        it.group = group++;
                        it.shape = Shape::path(pts, w / 2);
                        it.center = it.shape.center();
                        extra[r.net].push_back(std::move(it));
                    }
                } else if (c.head() == "via") {
                    int type = -1;
                    for (size_t v = 0; v < B.vias.size(); ++v)
                        if (B.vias[v].name == c.str(1)) type = int(v);
                    if (type < 0) throw std::runtime_error("SES: unknown via padstack " + c.str(1));
                    P at{c.num(2) / k, c.num(3) / k};
                    r.vias.push_back({at, type});
                    ++nvias;
                    Item it;
                    it.kind = Kind::Via;
                    it.net = r.net;
                    it.layer = -1;
                    it.group = group++;
                    it.shape = Shape::circle(at, B.vias[type].diameter / 2);
                    it.center = at;
                    extra[r.net].push_back(std::move(it));
                }
            }
            R.push_back(std::move(r));
        }
    int nets = 0, connections = 0;
    int open = open_connections(B, o, extra, verbose, &nets, &connections);
    int violations = check_routes(B, o, R, verbose);
    std::printf("eval: %d nets, %d connections, %d open, %d violations, %.1f mm of track, %d vias\n", nets, connections, open,
                violations, length / 1000, nvias);
    return open || violations ? 2 : 0;
}
