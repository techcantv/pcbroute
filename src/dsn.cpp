#include "dsn.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

#include "sexpr.h"

int Board::layer_index(const std::string& name) const {
    for (size_t i = 0; i < layers.size(); ++i)
        if (layers[i] == name) return int(i);
    return -1;
}

namespace {

double unit_scale(const std::string& u) {
    if (u == "um") return 1;
    if (u == "mm") return 1000;
    if (u == "mil") return 25.4;
    if (u == "inch") return 25400;
    if (u == "cm") return 10000;
    throw std::runtime_error("DSN: unknown unit " + u);
}

struct ShapeOnLayer {
    std::string layer;
    Shape shape;
};

// A DSN shape: (circle L d [x y]) | (rect L x1 y1 x2 y2) | (polygon L aperture x y ...) | (path L width x y ...)
bool read_shape(const Sx& s, double k, ShapeOnLayer& out) {
    const std::string& h = s.head();
    if (h == "circle") {
        double d = s.num(2) * k;
        P c{s.kids.size() > 4 ? s.num(3) * k : 0, s.kids.size() > 4 ? s.num(4) * k : 0};
        out = {s.str(1), Shape::circle(c, d / 2)};
        return true;
    }
    if (h == "rect") {
        double x1 = s.num(2) * k, y1 = s.num(3) * k, x2 = s.num(4) * k, y2 = s.num(5) * k;
        out = {s.str(1), Shape::polygon({{x1, y1}, {x2, y1}, {x2, y2}, {x1, y2}})};
        return true;
    }
    if (h == "polygon" || h == "path") {
        double w = s.num(2) * k;
        std::vector<P> pts;
        for (size_t i = 3; i + 1 < s.kids.size() && !s.kids[i].list && !s.kids[i + 1].list; i += 2)
            pts.push_back({s.num(i) * k, s.num(i + 1) * k});
        out = {s.str(1), h == "polygon" ? Shape::polygon(pts, w / 2) : Shape::path(pts, w / 2)};
        return true;
    }
    return false;
}

// Apply f to every point of the shape.
template <class F> Shape transformed(const Shape& s, F f) {
    Shape t = s;
    for (auto& [a, b] : t.segs) { a = f(a); b = f(b); }
    for (auto& p : t.poly) p = f(p);
    return t;
}

P rot(P p, double deg) {
    double a = deg * M_PI / 180, c = std::cos(a), s = std::sin(a);
    if (std::fmod(std::fabs(deg), 90.0) == 0) { c = std::round(c); s = std::round(s); }
    return {p.x * c - p.y * s, p.x * s + p.y * c};
}

}  // namespace

Board read_dsn(const std::string& path, bool image_keepouts) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    Sx root = parse_sexpr(ss.str());
    if (root.head() != "pcb") throw std::runtime_error("not a DSN file (no (pcb ...))");

    Board B;
    if (root.kids.size() > 1 && !root.kids[1].list) {
        B.session_name = root.kids[1].atom;
        auto sl = B.session_name.find_last_of('/');
        if (sl != std::string::npos) B.session_name = B.session_name.substr(sl + 1);
        auto dot = B.session_name.rfind('.');
        if (dot != std::string::npos) B.session_name = B.session_name.substr(0, dot);
    }
    double k = 1;
    if (auto u = root.find("unit")) k = unit_scale(u->str(1));
    else if (auto r = root.find("resolution")) k = unit_scale(r->str(1));
    if (auto r = root.find("resolution")) B.resolution = r->num(2) / unit_scale(r->str(1));  // per micrometre

    const Sx* st = root.find("structure");
    if (!st) throw std::runtime_error("DSN: no structure");
    for (auto l : st->all("layer")) B.layers.push_back(l->str(1));
    int NL = int(B.layers.size());
    auto lay = [&](const std::string& n) { return B.layer_index(n); };

    for (auto b : st->all("boundary")) {
        for (auto& c : b->kids) {
            ShapeOnLayer sl;
            if (c.list && read_shape(c, k, sl) && sl.layer == "pcb" && !B.has_outline) {
                std::vector<P> pts;
                for (auto& [a, e] : sl.shape.segs) pts.push_back(a);
                if (c.head() == "path" && !sl.shape.segs.empty()) pts.push_back(sl.shape.segs.back().second);
                B.outline = Shape::polygon(pts);
                B.has_outline = true;
            }
        }
    }

    // Via types
    std::vector<std::string> via_names;
    if (auto v = st->find("via"))
        for (size_t i = 1; i < v->kids.size(); ++i)
            if (!v->kids[i].list) via_names.push_back(v->kids[i].atom);

    // Default rule
    NetClass def;
    def.name = "default";
    def.layers.assign(NL, true);
    auto read_rule = [&](const Sx* r, NetClass& c) {
        if (!r) return;
        for (auto& x : r->kids) {
            if (!x.list) continue;
            if (x.head() == "width") c.width = x.num(1) * k;
            if (x.head() == "clearance") {
                if (x.kids.size() == 2) c.clearance = x.num(1) * k;
                else if (auto t = x.find("type"); t && t->str(1) == "smd_smd") B.smd_smd_clearance = x.num(1) * k;
            }
        }
    };
    read_rule(st->find("rule"), def);

    for (auto& ko : st->kids) {
        if (!ko.list) continue;
        const std::string& h = ko.head();
        if (h != "keepout" && h != "via_keepout" && h != "wire_keepout") continue;
        for (auto& c : ko.kids) {
            ShapeOnLayer sl;
            if (c.list && read_shape(c, k, sl)) {
                Keepout K;
                K.layer = lay(sl.layer);
                K.shape = sl.shape;
                K.wires = h != "via_keepout";
                K.vias = h != "wire_keepout";
                B.keepouts.push_back(K);
            }
        }
    }

    // Library: padstacks and images
    const Sx* lib = root.find("library");
    std::map<std::string, std::vector<ShapeOnLayer>> padstacks;
    std::map<std::string, const Sx*> images;
    if (lib) {
        for (auto ps : lib->all("padstack")) {
            auto& v = padstacks[ps->str(1)];
            for (auto sh : ps->all("shape"))
                for (auto& c : sh->kids) {
                    ShapeOnLayer sl;
                    if (c.list && read_shape(c, k, sl)) v.push_back(sl);
                }
        }
        for (auto im : lib->all("image")) images[im->str(1)] = im;
    }
    for (auto& n : via_names) {
        ViaType vt;
        vt.name = n;
        vt.raw = n.find_first_of(" ()[]:") != std::string::npos ? "\"" + n + "\"" : n;
        auto it = padstacks.find(n);
        if (it == padstacks.end() || it->second.empty()) throw std::runtime_error("DSN: no padstack for via " + n);
        for (auto& s : it->second) vt.diameter = std::max(vt.diameter, 2 * s.shape.r);
        B.vias.push_back(vt);
    }
    for (size_t i = 0; i < B.vias.size(); ++i) def.vias.push_back(int(i));
    B.classes.push_back(def);

    // Nets (network first so that pins know their nets)
    const Sx* nw = root.find("network");
    std::map<std::string, int> pin_net;
    if (nw) {
        for (auto n : nw->all("net")) {
            Net N;
            N.name = n->kids[1].atom;
            N.raw = n->kids[1].raw;
            int id = int(B.nets.size());
            if (auto p = n->find("pins"))
                for (size_t i = 1; i < p->kids.size(); ++i) { pin_net[p->kids[i].atom] = id; N.has_pins = true; }
            B.net_index[N.name] = id;
            B.nets.push_back(N);
        }
        for (auto c : nw->all("class")) {
            NetClass C = def;
            C.name = c->str(1);
            read_rule(c->find("rule"), C);
            if (auto cir = c->find("circuit")) {
                if (auto uv = cir->find("use_via")) {
                    C.vias.clear();
                    for (size_t i = 1; i < uv->kids.size(); ++i)
                        for (size_t j = 0; j < B.vias.size(); ++j)
                            if (B.vias[j].name == uv->kids[i].atom) C.vias.push_back(int(j));
                }
                if (auto ul = cir->find("use_layer")) {
                    C.layers.assign(NL, false);
                    for (size_t i = 1; i < ul->kids.size(); ++i)
                        if (int L = lay(ul->kids[i].atom); L >= 0) C.layers[L] = true;
                }
            }
            int ci = int(B.classes.size());
            B.classes.push_back(C);
            for (size_t i = 2; i < c->kids.size(); ++i)
                if (!c->kids[i].list)
                    if (auto it = B.net_index.find(c->kids[i].atom); it != B.net_index.end()) B.nets[it->second].cls = ci;
        }
    }

    int group = 0;
    auto add_item = [&](Item it) {
        if (it.net >= 0) B.nets[it.net].items.push_back(int(B.items.size()));
        B.items.push_back(std::move(it));
    };

    // Placement: every pin of every component
    if (const Sx* pl = root.find("placement")) {
        for (auto comp : pl->all("component")) {
            auto im = images.find(comp->str(1));
            if (im == images.end()) throw std::runtime_error("DSN: no image " + comp->str(1));
            for (auto place : comp->all("place")) {
                std::string ref = place->str(1);
                P at{place->num(2) * k, place->num(3) * k};
                bool back = place->str(4) == "back";
                double crot = place->num(5);
                for (auto pin : im->second->all("pin")) {
                    std::vector<const Sx*> atoms;
                    double prot = 0;
                    for (size_t i = 1; i < pin->kids.size(); ++i) {
                        if (pin->kids[i].list) { if (pin->kids[i].head() == "rotate") prot = pin->kids[i].num(1); }
                        else atoms.push_back(&pin->kids[i]);
                    }
                    if (atoms.size() < 4) continue;
                    const std::string& ps = atoms[0]->atom;
                    std::string pid = atoms[1]->atom;
                    P pp{std::strtod(atoms[2]->atom.c_str(), nullptr) * k, std::strtod(atoms[3]->atom.c_str(), nullptr) * k};
                    auto f = [&](P q) {
                        P r = rot(q, prot) + pp;
                        if (back) r.x = -r.x;
                        return rot(r, crot) + at;
                    };
                    auto pst = padstacks.find(ps);
                    if (pst == padstacks.end()) throw std::runtime_error("DSN: no padstack " + ps);
                    auto pn = pin_net.find(ref + "-" + pid);
                    int net = pn == pin_net.end() ? -1 : pn->second;
                    std::set<int> layers_used;
                    for (auto& s : pst->second) layers_used.insert(lay(s.layer));
                    bool smd = layers_used.size() == 1 && *layers_used.begin() >= 0;
                    for (auto& s : pst->second) {
                        Item it;
                        it.kind = Kind::Pad;
                        it.net = net;
                        it.layer = lay(s.layer);
                        if (back && it.layer >= 0) it.layer = NL - 1 - it.layer;
                        it.group = group;
                        it.smd = smd;
                        it.shape = transformed(s.shape, f);
                        it.center = f(P{0, 0});
                        add_item(std::move(it));
                    }
                    ++group;
                }
                // The image's own keepouts (KiCad writes one per copper layer for every bare NPTH hole, and the
                // footprint's rule areas), placed like its pins; a back-side part mirrors them to the other layers.
                if (!image_keepouts) continue;
                auto f = [&](P q) {
                    if (back) q.x = -q.x;
                    return rot(q, crot) + at;
                };
                for (auto& ko : im->second->kids) {
                    if (!ko.list) continue;
                    const std::string& h = ko.head();
                    if (h != "keepout" && h != "via_keepout" && h != "wire_keepout") continue;
                    for (auto& c : ko.kids) {
                        ShapeOnLayer sl;
                        if (!c.list || !read_shape(c, k, sl)) continue;
                        Keepout K;
                        K.layer = lay(sl.layer);
                        if (back && K.layer >= 0) K.layer = NL - 1 - K.layer;
                        K.shape = transformed(sl.shape, f);
                        K.wires = h != "via_keepout";
                        K.vias = h != "wire_keepout";
                        B.keepouts.push_back(K);
                    }
                }
            }
        }
    }

    // make_dsn.py's board rules: the copper-to-edge clearance (the --edge default)
    if (auto br = st->find("pcbroute_rules"))
        if (auto ec = br->find("edge_clearance"); ec && ec->kids.size() > 1) B.edge_clearance = std::max(0.0, ec->num(1));

    // Planes (structure) and pre-routed copper (wiring)
    // make_dsn.py's zone rules: the k-th (zone) belongs to the k-th (plane), if their nets and layers agree
    std::vector<const Sx*> planes = st->all("plane"), zrs;
    if (auto zs = st->find("pcbroute_zones")) zrs = zs->all("zone");
    bool rules = !zrs.empty() && zrs.size() == planes.size();
    for (size_t i = 0; rules && i < planes.size(); ++i) {
        const Sx* sh = planes[i]->kids.size() > 2 && planes[i]->kids[2].list ? &planes[i]->kids[2] : nullptr;
        rules = zrs[i]->kids.size() > 2 && zrs[i]->str(1) == planes[i]->str(1) && sh && sh->kids.size() > 1 && zrs[i]->str(2) == sh->str(1);
    }
    if (!zrs.empty() && !rules) std::fprintf(stderr, "DSN: (pcbroute_zones) doesn't match the planes, ignored\n");
    if (rules)
        for (auto z : zrs) {
            ZoneRule r;
            for (auto& x : z->kids) {
                if (!x.list || x.kids.size() < 2) continue;
                if (x.head() == "priority") r.priority = int(x.num(1));
                else if (x.head() == "clearance") r.clearance = x.num(1);
                else if (x.head() == "min_width") r.min_width = x.num(1);
                else if (x.head() == "fillet" || x.head() == "chamfer") r.fillet = x.num(1);
            }
            B.zones.push_back(r);
        }
    for (size_t pi = 0; pi < planes.size(); ++pi) {
        auto pl = planes[pi];
        auto ni = B.net_index.find(pl->kids[1].atom);
        for (auto& c : pl->kids) {
            ShapeOnLayer sl;
            if (c.list && read_shape(c, k, sl)) {
                Item it;
                it.kind = Kind::Plane;
                it.net = ni == B.net_index.end() ? -1 : ni->second;
                it.zone = rules ? int(pi) : -1;
                it.layer = lay(sl.layer);
                it.group = group++;
                it.shape = sl.shape;
                it.center = sl.shape.center();
                add_item(std::move(it));
            }
        }
    }
    if (const Sx* w = root.find("wiring")) {
        for (auto& c : w->kids) {
            if (!c.list) continue;
            int net = -1;
            if (auto n = c.find("net"))
                if (auto it = B.net_index.find(n->kids[1].atom); it != B.net_index.end()) net = it->second;
            if (c.head() == "wire") {
                for (auto& s : c.kids) {
                    ShapeOnLayer sl;
                    if (s.list && read_shape(s, k, sl)) {
                        Item it;
                        it.kind = Kind::Wire;
                        it.net = net;
                        it.layer = lay(sl.layer);
                        it.group = group++;
                        it.shape = sl.shape;
                        it.center = sl.shape.center();
                        add_item(std::move(it));
                    }
                }
            } else if (c.head() == "via") {
                auto pst = padstacks.find(c.str(1));
                if (pst == padstacks.end()) throw std::runtime_error("DSN: no padstack " + c.str(1));
                P at{c.num(2) * k, c.num(3) * k};
                for (auto& s : pst->second) {
                    Item it;
                    it.kind = Kind::Via;
                    it.net = net;
                    it.layer = lay(s.layer);
                    it.group = group;
                    it.shape = transformed(s.shape, [&](P q) { return q + at; });
                    it.center = at;
                    add_item(std::move(it));
                }
                ++group;
            }
        }
    }
    return B;
}
