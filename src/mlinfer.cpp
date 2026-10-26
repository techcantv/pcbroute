// In-process RouteNet prior: ml/mlroute.py (board_from_dump, predict, write_prior) and ml/raster.py (board_maps,
// net_inputs) in C++, with the model run by ONNX Runtime. The model was trained on boards drawn by Pillow, so the
// drawing primitives below are Pillow's own (libImaging/Draw.c, 12.3.0, the 32-bit ink paths) and the board goes
// through the same roundings as the Python path (pcbroute --dump-board prints micrometres with one decimal, the
// hull rounds to 1e-6 mm, Pillow truncates coordinates to int): the inputs are the same bit for bit, and the prior
// differs from mlroute's only by the model's floating point. This file is compiled with -ffp-contract=off (no FMA),
// like Pillow and numpy.
#include "mlinfer.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#ifdef PCB_ONNXRT
#include <dlfcn.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cerrno>
#include <condition_variable>
#include <deque>

#include <onnxruntime_cxx_api.h>
#include <onnxruntime_run_options_config_keys.h>
#endif

namespace {

constexpr double RES = 0.25;    // mm per pixel (raster.RES)
constexpr int NIN = 12, NOUT = 4;
constexpr double MARGIN = 10.0; // mm of map around each net's terminals (mlroute.MARGIN)
constexpr double BATCH_PX = 16e6;

double now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

// a coordinate as the JSON dump carries it ("%.1f" micrometres)
double snap(double v) {
    char b[64];
    std::snprintf(b, sizeof b, "%.1f", v);
    return std::strtod(b, nullptr);
}

using Pt = std::array<double, 2>;
using Poly = std::vector<Pt>;

// ---------------------------------------------------------------------------------------------------------------
// Pillow's drawing, 32-bit images, no blending, no mask. A sink has xsize, ysize and set(x, y).

// ROUND_UP / ROUND_DOWN with C's promotions: a float argument adds 0.5F in float, its fabs() is a double
float round_up(float f) { return f >= 0.0 ? float(std::floor(double(float(f + 0.5F)))) : float(-std::floor(std::fabs(double(f)) + 0.5F)); }
float round_down(float f) { return f >= 0.0 ? float(std::ceil(double(float(f - 0.5F)))) : float(-std::ceil(std::fabs(double(f)) - 0.5F)); }
int round_up_i(float f) { return int(round_up(f)); }
int round_down_i(float f) { return int(round_down(f)); }
int round_up_d(double f) { return int(f >= 0.0 ? std::floor(f + 0.5F) : -std::floor(std::fabs(f) + 0.5F)); }
int round_down_d(double f) { return int(f >= 0.0 ? std::ceil(f - 0.5F) : -std::ceil(std::fabs(f) - 0.5F)); }

template <class S> inline void point32(S& im, int x, int y) {
    if (x >= 0 && x < im.xsize && y >= 0 && y < im.ysize) im.set(x, y);
}

template <class S> inline void hline32(S& im, int x0, int y0, int x1) {
    if (y0 >= 0 && y0 < im.ysize) {
        if (x0 < 0) x0 = 0;
        else if (x0 >= im.xsize) return;
        if (x1 < 0) return;
        else if (x1 >= im.xsize) x1 = im.xsize - 1;
        while (x0 <= x1) im.set(x0++, y0);
    }
}

template <class S> void line32(S& im, int x0, int y0, int x1, int y1) {
    int i, n, e, dx, dy, xs, ys;
    dx = x1 - x0;
    if (dx < 0) dx = -dx, xs = -1;
    else xs = 1;
    dy = y1 - y0;
    if (dy < 0) dy = -dy, ys = -1;
    else ys = 1;
    n = (dx > dy) ? dx : dy;
    if (dx == 0) {
        for (i = 0; i < dy; i++) { point32(im, x0, y0); y0 += ys; }
    } else if (dy == 0) {
        for (i = 0; i < dx; i++) { point32(im, x0, y0); x0 += xs; }
    } else if (dx > dy) {
        n = dx;
        dy += dy;
        e = dy - dx;
        dx += dx;
        for (i = 0; i < n; i++) {
            point32(im, x0, y0);
            if (e >= 0) { y0 += ys; e -= dx; }
            e += dy;
            x0 += xs;
        }
    } else {
        n = dy;
        dx += dx;
        e = dx - dy;
        dy += dy;
        for (i = 0; i < n; i++) {
            point32(im, x0, y0);
            if (e >= 0) { x0 += xs; e -= dy; }
            e += dx;
            y0 += ys;
        }
    }
}

struct Edge {
    int d;
    int x0, y0;
    int xmin, ymin, xmax, ymax;
    float dx;
};

void add_edge(Edge* e, int x0, int y0, int x1, int y1) {
    if (x0 <= x1) e->xmin = x0, e->xmax = x1;
    else e->xmin = x1, e->xmax = x0;
    if (y0 <= y1) e->ymin = y0, e->ymax = y1;
    else e->ymin = y1, e->ymax = y0;
    if (y0 == y1) {
        e->d = 0;
        e->dx = 0.0;
    } else {
        e->dx = float(x1 - x0) / float(y1 - y0);
        e->d = y0 == e->ymin ? 1 : -1;
    }
    e->x0 = x0;
    e->y0 = y0;
}

template <class S> void polygon_generic(S& im, int n, Edge* e) {
    if (n <= 0) return;
    std::vector<Edge*> edge_table(n);
    int edge_count = 0;
    int ymin = im.ysize - 1, ymax = 0;
    int i, j, k;
    float adjacent_line_x, adjacent_line_x_other_edge;
    for (i = 0; i < n; i++) {
        if (ymin > e[i].ymin) ymin = e[i].ymin;
        if (ymax < e[i].ymax) ymax = e[i].ymax;
        if (e[i].ymin == e[i].ymax) {
            hline32(im, e[i].xmin, e[i].ymin, e[i].xmax);
            continue;
        }
        edge_table[edge_count++] = e + i;
    }
    if (ymin < 0) ymin = 0;
    if (ymax > im.ysize) ymax = im.ysize;
    std::vector<float> xx(size_t(edge_count) * 2 + 1);
    for (; ymin <= ymax; ymin++) {
        j = 0;
        for (i = 0; i < edge_count; i++) {
            Edge* current = edge_table[i];
            if (ymin >= current->ymin && ymin <= current->ymax) {
                xx[j++] = float(ymin - current->y0) * current->dx + float(current->x0);
                if (ymin == current->ymax && ymin < ymax) {
                    xx[j] = xx[j - 1];
                    j++;
                } else if ((ymin == current->ymin || ymin == current->ymax) && current->dx != 0) {
                    for (k = 0; k < i; k++) {
                        Edge* other_edge = edge_table[k];
                        if ((ymin != other_edge->ymin && ymin != other_edge->ymax) || other_edge->dx == 0) continue;
                        if (std::roundf(xx[j - 1]) == std::roundf(float(ymin - other_edge->y0) * other_edge->dx + float(other_edge->x0))) {
                            int offset = ymin == current->ymax ? -1 : 1;
                            adjacent_line_x = float(ymin + offset - current->y0) * current->dx + float(current->x0);
                            if (ymin + offset >= other_edge->ymin && ymin + offset <= other_edge->ymax) {
                                adjacent_line_x_other_edge = float(ymin + offset - other_edge->y0) * other_edge->dx + float(other_edge->x0);
                                if (xx[j - 1] > adjacent_line_x + 1 && xx[j - 1] > adjacent_line_x_other_edge + 1) {
                                    xx[j - 1] = std::roundf(float(std::fmax(double(adjacent_line_x), double(adjacent_line_x_other_edge)))) + 1;
                                } else if (xx[j - 1] < adjacent_line_x - 1 && xx[j - 1] < adjacent_line_x_other_edge - 1) {
                                    xx[j - 1] = std::roundf(float(std::fmin(double(adjacent_line_x), double(adjacent_line_x_other_edge)))) - 1;
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }
        std::sort(xx.begin(), xx.begin() + j);
        for (i = 1; i < j; i += 2) hline32(im, round_up_i(xx[i - 1]), ymin, round_down_i(xx[i]));
    }
}

// ImagingDrawPolygon with fill
template <class S> void draw_polygon(S& im, const std::vector<int>& xy) {
    int count = int(xy.size() / 2);
    if (count <= 0) return;
    std::vector<Edge> e(count);
    int i, n, x0, y0, x1, y1;
    for (i = n = 0; i < count - 1; i++) {
        x0 = xy[i * 2];
        y0 = xy[i * 2 + 1];
        x1 = xy[i * 2 + 2];
        y1 = xy[i * 2 + 3];
        if (y0 == y1 && i != 0 && y0 == xy[i * 2 - 1]) {
            Edge* last_e = &e[n - 1];
            if (x1 > x0 && x0 > xy[i * 2 - 2]) { last_e->xmax = x1; continue; }
            else if (x1 < x0 && x0 < xy[i * 2 - 2]) { last_e->xmin = x1; continue; }
        }
        add_edge(&e[n++], x0, y0, x1, y1);
    }
    if (xy[i * 2] != xy[0] || xy[i * 2 + 1] != xy[1]) add_edge(&e[n++], xy[i * 2], xy[i * 2 + 1], xy[0], xy[1]);
    polygon_generic(im, n, e.data());
}

template <class S> void draw_wide_line(S& im, int x0, int y0, int x1, int y1, int width) {
    int dx = x1 - x0, dy = y1 - y0;
    if (dx == 0 && dy == 0) { point32(im, x0, y0); return; }
    double big_hypotenuse = std::hypot(double(dx), double(dy));
    double small_hypotenuse = (width - 1) / 2.0;
    double ratio_max = round_up_d(small_hypotenuse) / big_hypotenuse;
    double ratio_min = round_down_d(small_hypotenuse) / big_hypotenuse;
    int dxmin = round_down_d(ratio_min * dy), dxmax = round_down_d(ratio_max * dy);
    int dymin = round_down_d(ratio_min * dx), dymax = round_down_d(ratio_max * dx);
    int v[4][2] = {{x0 - dxmin, y0 + dymax}, {x1 - dxmin, y1 + dymax}, {x1 + dxmax, y1 - dymin}, {x0 + dxmax, y0 - dymin}};
    Edge e[4];
    for (int k = 0; k < 4; ++k) add_edge(e + k, v[k][0], v[k][1], v[(k + 1) % 4][0], v[(k + 1) % 4][1]);
    polygon_generic(im, 4, e);
}

struct quarter_state {
    int32_t a, b, cx, cy, ex, ey;
    int64_t a2, b2, a2b2;
    int8_t finished;
};

void quarter_init(quarter_state* s, int32_t a, int32_t b) {
    if (a < 0 || b < 0) {
        s->finished = 1;
    } else {
        s->a = a;
        s->b = b;
        s->cx = a;
        s->cy = b % 2;
        s->ex = a % 2;
        s->ey = b;
        s->a2 = int64_t(a) * a;
        s->b2 = int64_t(b) * b;
        s->a2b2 = s->a2 * s->b2;
        s->finished = 0;
    }
}

int64_t quarter_delta(quarter_state* s, int64_t x, int64_t y) { return std::llabs(s->a2 * y * y + s->b2 * x * x - s->a2b2); }

int8_t quarter_next(quarter_state* s, int32_t* ret_x, int32_t* ret_y) {
    if (s->finished) return -1;
    *ret_x = s->cx;
    *ret_y = s->cy;
    if (s->cx == s->ex && s->cy == s->ey) {
        s->finished = 1;
    } else {
        int32_t nx = s->cx, ny = s->cy + 2;
        int64_t ndelta = quarter_delta(s, nx, ny);
        if (nx > 1) {
            int64_t newdelta = quarter_delta(s, s->cx - 2, s->cy + 2);
            if (ndelta > newdelta) { nx = s->cx - 2; ny = s->cy + 2; ndelta = newdelta; }
            newdelta = quarter_delta(s, s->cx - 2, s->cy);
            if (ndelta > newdelta) { nx = s->cx - 2; ny = s->cy; }
        }
        s->cx = nx;
        s->cy = ny;
    }
    return 0;
}

struct ellipse_state {
    quarter_state st_o, st_i;
    int32_t py, pl, pr;
    int32_t cy[4], cl[4], cr[4];
    int8_t bufcnt, finished, leftmost;
};

void ellipse_init(ellipse_state* s, int32_t a, int32_t b, int32_t w) {
    s->bufcnt = 0;
    s->leftmost = int8_t(a % 2);
    quarter_init(&s->st_o, a, b);
    if (w < 1 || quarter_next(&s->st_o, &s->pr, &s->py) == -1) {
        s->finished = 1;
    } else {
        s->finished = 0;
        quarter_init(&s->st_i, a - 2 * (w - 1), b - 2 * (w - 1));
        s->pl = s->leftmost;
    }
}

int8_t ellipse_next(ellipse_state* s, int32_t* ret_x0, int32_t* ret_y, int32_t* ret_x1) {
    if (s->bufcnt == 0) {
        if (s->finished) return -1;
        int32_t y = s->py, l = s->pl, r = s->pr, cx = 0, cy = 0;
        int8_t next_ret;
        while ((next_ret = quarter_next(&s->st_o, &cx, &cy)) != -1 && cy <= y) {}
        if (next_ret == -1) s->finished = 1;
        else { s->pr = cx; s->py = cy; }
        while ((next_ret = quarter_next(&s->st_i, &cx, &cy)) != -1 && cy <= y) l = cx;
        s->pl = next_ret == -1 ? s->leftmost : cx;
        if ((l > 0 || l < r) && y > 0) {
            s->cl[s->bufcnt] = l == 0 ? 2 : l;
            s->cy[s->bufcnt] = y;
            s->cr[s->bufcnt] = r;
            ++s->bufcnt;
        }
        if (y > 0) {
            s->cl[s->bufcnt] = -r;
            s->cy[s->bufcnt] = y;
            s->cr[s->bufcnt] = -l;
            ++s->bufcnt;
        }
        if (l > 0 || l < r) {
            s->cl[s->bufcnt] = l == 0 ? 2 : l;
            s->cy[s->bufcnt] = -y;
            s->cr[s->bufcnt] = r;
            ++s->bufcnt;
        }
        s->cl[s->bufcnt] = -r;
        s->cy[s->bufcnt] = -y;
        s->cr[s->bufcnt] = -l;
        ++s->bufcnt;
    }
    --s->bufcnt;
    *ret_x0 = s->cl[s->bufcnt];
    *ret_y = s->cy[s->bufcnt];
    *ret_x1 = s->cr[s->bufcnt];
    return 0;
}

// ImagingDrawEllipse with fill
template <class S> void draw_ellipse(S& im, int x0, int y0, int x1, int y1) {
    int a = x1 - x0, b = y1 - y0;
    if (a < 0 || b < 0) return;
    ellipse_state st;
    ellipse_init(&st, a, b, a + b);
    int32_t X0, Y, X1;
    while (ellipse_next(&st, &X0, &Y, &X1) != -1) hline32(im, x0 + (X0 + a) / 2, y0 + (Y + b) / 2, x0 + (X1 + a) / 2);
}

// ---------------------------------------------------------------------------------------------------------------
// raster.py's Canvas over the drawing primitives

struct ValImg {  // an "I" image
    int xsize, ysize;
    int32_t val = 0;
    std::vector<int32_t> px;
    ValImg(int w, int h) : xsize(w), ysize(h), px(size_t(w) * h, 0) {}
    void set(int x, int y) { px[size_t(y) * xsize + x] = val; }
};

struct Canvas {
    double x0, y0;
    int w, h;
    double pxx(double x) const { return (x - x0) / RES; }
    double pxy(double y) const { return (y - y0) / RES; }
    template <class S> void poly(S& img, const Poly& pts) const {
        if (pts.size() < 3) return;
        std::vector<int> xy(pts.size() * 2);
        for (size_t i = 0; i < pts.size(); ++i) { xy[2 * i] = int(pxx(pts[i][0])); xy[2 * i + 1] = int(pxy(pts[i][1])); }
        draw_polygon(img, xy);
    }
    template <class S> void seg(S& img, Pt a, Pt b, double width) const {
        double ax = pxx(a[0]), ay = pxy(a[1]), bx = pxx(b[0]), by = pxy(b[1]);
        double r = std::max(0.5, width / RES / 2);
        if (ax != bx || ay != by) {
            int lw = std::max(1, int(std::nearbyint(2 * r)));  // Python's round(): half to even
            if (lw == 1) {
                line32(img, int(ax), int(ay), int(bx), int(by));
                point32(img, int(bx), int(by));
            } else {
                draw_wide_line(img, int(ax), int(ay), int(bx), int(by), lw);
            }
        }
        for (auto [x, y] : {std::pair{ax, ay}, std::pair{bx, by}}) draw_ellipse(img, int(x - r), int(y - r), int(x + r), int(y + r));
    }
};

// ---------------------------------------------------------------------------------------------------------------
// mlroute.py's board_from_dump

int fold(const std::string& name) { return name == "F.Cu" ? 0 : name == "B.Cu" ? 1 : 2; }

std::vector<int> fold_table(const std::vector<std::string>& layers) {
    std::vector<int> t;
    for (auto& n : layers) t.push_back(fold(n));
    if (!t.empty() && std::find(t.begin(), t.end(), 0) == t.end()) t[0] = 0;
    if (t.size() > 1 && std::find(t.begin(), t.end(), 1) == t.end()) t.back() = 1;
    return t;
}

double cross(const Pt& o2, const Pt& o1, const Pt& p) {  // as hull()'s turn test, operand order kept
    return (o1[0] - o2[0]) * (p[1] - o2[1]) - (o1[1] - o2[1]) * (p[0] - o2[0]);
}

Poly hull(Poly pts) {
    for (auto& p : pts) for (double& v : p) v = std::nearbyint(v * 1e6) / 1e6;  // np.round(pts, 6)
    std::sort(pts.begin(), pts.end());
    if (pts.size() < 3) return pts;
    auto half = [](auto first, auto last) {
        Poly out;
        for (auto it = first; it != last; ++it) {
            while (out.size() >= 2 && cross(out[out.size() - 2], out.back(), *it) <= 0) out.pop_back();
            out.push_back(*it);
        }
        return out;
    };
    Poly lo = half(pts.begin(), pts.end()), up = half(pts.rbegin(), pts.rend());
    Poly r(lo.begin(), lo.end() - 1);
    r.insert(r.end(), up.begin(), up.end() - 1);
    return r;
}

struct Trig {
    double c[16], s[16];
    Trig() {
        const double step = (2 * M_PI - 0.0) / 16;  // np.linspace(0, 2 pi, 16, endpoint=False)
        for (int k = 0; k < 16; ++k) { double t = double(k) * step; c[k] = std::cos(t); s[k] = std::sin(t); }
    }
};

Poly shape_poly(const Shape& s) {
    static const Trig T;
    std::vector<std::array<double, 4>> segs;
    for (auto& [a, b] : s.segs) segs.push_back({snap(a.x) / 1000.0, -(snap(a.y) / 1000.0), snap(b.x) / 1000.0, -(snap(b.y) / 1000.0)});
    if (s.poly.size() >= 3) {
        Poly p;
        for (auto& g : segs) p.push_back({g[0], g[1]});
        return p;
    }
    double r = snap(s.r) / 1000.0;
    Poly pts;
    for (auto& g : segs)
        for (int k = 0; k < 16; ++k) {
            pts.push_back({g[0] + r * T.c[k], g[1] + r * T.s[k]});
            pts.push_back({g[2] + r * T.c[k], g[3] + r * T.s[k]});
        }
    return hull(pts);
}

struct PadR { int net; bool F, B, drilled; Poly poly; };
struct ZoneR { int net; std::vector<int> layers; Poly poly; };
struct BoardR {
    std::vector<Poly> outline;
    std::vector<PadR> pads;
    std::vector<ZoneR> zones;
    int copper_layers = 0;
    std::vector<int> plane_nets;
};

BoardR board_from(const Board& B) {
    BoardR R;
    auto ft = fold_table(B.layers);
    struct Group { int net; bool F = false, B = false, smd; Kind kind; bool has = false; Poly poly; };
    std::vector<Group> groups;
    std::map<int, size_t> gidx;
    for (auto& it : B.items) {
        int net = it.net >= 0 ? it.net + 1 : 0;
        if (it.kind == Kind::Plane) {
            if (it.layer >= 0) {
                R.zones.push_back({net, {ft[it.layer]}, shape_poly(it.shape)});
                if (std::find(R.plane_nets.begin(), R.plane_nets.end(), net) == R.plane_nets.end()) R.plane_nets.push_back(net);
            }
            continue;
        }
        if (it.kind == Kind::Via && it.layer >= 0 && ft[it.layer] == 2) continue;
        auto [gi, fresh] = gidx.emplace(it.group, groups.size());
        if (fresh) { Group g; g.net = net; g.smd = it.smd; g.kind = it.kind; groups.push_back(g); }
        Group& g = groups[gi->second];
        bool on0 = it.layer < 0 || ft[it.layer] == 0, on1 = it.layer < 0 || ft[it.layer] == 1;
        g.F |= on0;
        g.B |= on1;
        if (!g.has || on0) { g.poly = shape_poly(it.shape); g.has = true; }
    }
    for (auto& g : groups)
        if (g.has && g.poly.size() >= 3 && (g.F || g.B)) R.pads.push_back({g.net, g.F, g.B, !g.smd && g.kind != Kind::Wire, g.poly});
    for (auto& k : B.keepouts)
        if (k.wires) {
            std::vector<int> ls = k.layer < 0 ? std::vector<int>{0, 1, 2} : std::vector<int>{ft[k.layer]};
            R.zones.push_back({-1, ls, shape_poly(k.shape)});
        }
    if (B.has_outline) {
        Poly o;
        for (auto& p : B.outline.poly) o.push_back({snap(p.x) / 1000.0, -(snap(p.y) / 1000.0)});
        R.outline.push_back(o);
    } else {
        if (R.pads.empty()) throw std::runtime_error("ML model: no outline and no pads");
        double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
        for (auto& p : R.pads)
            for (auto& q : p.poly) { x0 = std::min(x0, q[0]); y0 = std::min(y0, q[1]); x1 = std::max(x1, q[0]); y1 = std::max(y1, q[1]); }
        R.outline.push_back({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}});
    }
    for (auto& n : B.layers) R.copper_layers += n.size() >= 3 && n.compare(n.size() - 3, 3, ".Cu") == 0;
    if (!R.copper_layers) R.copper_layers = int(B.layers.size());
    return R;
}

// ---------------------------------------------------------------------------------------------------------------
// raster.py's board_maps and net_inputs

struct Maps {
    Canvas cv;
    std::vector<uint8_t> interior, drill, rats;
    std::vector<int32_t> padF, padB, pourF, pourB, pourI;
    std::vector<int> rat_nets;                         // nets with a ratsnest, in raster.py's dict order
    std::map<int, std::vector<std::pair<Pt, Pt>>> rat; // net -> MST edges between pad centres
};

std::vector<std::pair<int, int>> mst_edges(const std::vector<Pt>& pts) {
    size_t n = pts.size();
    std::vector<std::pair<int, int>> edges;
    if (n < 2) return edges;
    std::vector<char> inside(n, 0);
    inside[0] = 1;
    auto dist = [&](size_t i, size_t k) {
        double dx = pts[i][0] - pts[k][0], dy = pts[i][1] - pts[k][1];
        return std::sqrt(dx * dx + dy * dy);
    };
    std::vector<double> best(n);
    std::vector<int> frm(n, 0);
    for (size_t i = 0; i < n; ++i) best[i] = dist(i, 0);
    for (size_t step = 0; step + 1 < n; ++step) {
        size_t k = 0;
        double bk = INFINITY;
        bool any = false;
        for (size_t i = 0; i < n; ++i) {
            double b = inside[i] ? INFINITY : best[i];
            if (!any || b < bk) { bk = b; k = i; any = true; }
        }
        edges.push_back({frm[k], int(k)});
        inside[k] = 1;
        for (size_t i = 0; i < n; ++i) {
            double d = dist(i, k);
            if (d < best[i]) { best[i] = d; frm[i] = int(k); }
        }
    }
    return edges;
}

struct RatSink {  // adds 1 (mod 256) to each pixel one net's ratsnest covers
    int xsize, ysize, id = 0;
    std::vector<int32_t>& stamp;
    std::vector<uint8_t>& rats;
    void set(int x, int y) {
        size_t i = size_t(y) * xsize + x;
        if (stamp[i] != id) { stamp[i] = id; rats[i] = uint8_t(rats[i] + 1); }
    }
};

std::vector<uint8_t> to_u8(const std::vector<int32_t>& v) {
    std::vector<uint8_t> o(v.size());
    for (size_t i = 0; i < v.size(); ++i) o[i] = uint8_t(v[i]);
    return o;
}

Maps board_maps(const BoardR& bd) {
    double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
    auto take = [&](const Poly& p) { for (auto& q : p) { x0 = std::min(x0, q[0]); y0 = std::min(y0, q[1]); x1 = std::max(x1, q[0]); y1 = std::max(y1, q[1]); } };
    for (auto& o : bd.outline) take(o);
    for (auto& p : bd.pads) take(p.poly);
    x0 -= 2.0, y0 -= 2.0, x1 += 2.0, y1 += 2.0;
    Maps M{Canvas{x0, y0, int(std::ceil((x1 - x0) / RES)), int(std::ceil((y1 - y0) / RES))}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}};
    const Canvas& cv = M.cv;
    const int W = cv.w, H = cv.h;
    ValImg interior(W, H), padF(W, H), padB(W, H), drill(W, H);
    interior.val = 1;
    for (auto& o : bd.outline) cv.poly(interior, o);
    std::map<int, std::vector<Pt>> centers;
    std::vector<int> order;
    drill.val = 1;
    for (auto& p : bd.pads) {
        int32_t v = std::max(0, p.net) + 1;
        padF.val = padB.val = v;
        if (p.F) cv.poly(padF, p.poly);
        if (p.B) cv.poly(padB, p.poly);
        if (p.drilled) cv.poly(drill, p.poly);
        if (p.net > 0) {
            double sx = 0, sy = 0;
            for (auto& q : p.poly) { sx += q[0]; sy += q[1]; }
            double n = double(p.poly.size());
            auto [it, fresh] = centers.emplace(p.net, std::vector<Pt>{});
            if (fresh) order.push_back(p.net);
            it->second.push_back({sx / n, sy / n});
        }
    }
    ValImg pour[3] = {ValImg(W, H), ValImg(W, H), ValImg(W, H)};
    for (auto& z : bd.zones)
        for (int L : z.layers) { pour[L].val = std::max(0, z.net) + 1; cv.poly(pour[L], z.poly); }
    M.rats.assign(size_t(W) * H, 0);
    std::vector<int32_t> stamp(size_t(W) * H, 0);
    RatSink rs{W, H, 0, stamp, M.rats};
    for (int net : order) {
        auto& pts = centers[net];
        if (pts.size() < 2) continue;
        auto& E = M.rat[net];
        for (auto [a, b] : mst_edges(pts)) E.push_back({pts[a], pts[b]});
        M.rat_nets.push_back(net);
        rs.id = int(M.rat_nets.size());
        for (auto& [a, b] : E) cv.seg(rs, a, b, RES);
    }
    M.interior = to_u8(interior.px);
    M.drill = to_u8(drill.px);
    auto i16 = [](std::vector<int32_t>& v) { for (auto& x : v) x = int16_t(x); return std::move(v); };  // .astype(np.int16)
    M.padF = i16(padF.px);
    M.padB = i16(padB.px);
    M.pourF = i16(pour[0].px);
    M.pourB = i16(pour[1].px);
    M.pourI = i16(pour[2].px);
    return M;
}

struct Box { int i0, j0, w, h; };

struct OwnSink {
    int xsize, ysize;
    float* px;
    void set(int x, int y) { px[size_t(y) * xsize + x] = 1.f; }
};

// (NIN, h, w) float32 for one net over its box, written to out (zeroed by the caller)
void net_inputs(const Maps& m, int net, bool multilayer, const Box& bx, float* out) {
    const int W = m.cv.w, H = m.cv.h, bw = bx.w, bh = bx.h;
    const size_t plane = size_t(bw) * bh;
    auto ch = [&](int c) { return out + c * plane; };
    for (size_t i = 0; i < plane; ++i) ch(11)[i] = multilayer ? 1.f : 0.f;
    int si0 = std::max(0, bx.i0), sj0 = std::max(0, bx.j0), si1 = std::min(W, bx.i0 + bw), sj1 = std::min(H, bx.j0 + bh);
    if (si1 <= si0 || sj1 <= sj0) {
        for (size_t i = 0; i < plane; ++i) ch(11)[i] = 0.f;  // raster.py returns before the multilayer flag
        return;
    }
    // own ratsnest, drawn on a canvas of the box
    auto rit = m.rat.find(net);
    if (rit != m.rat.end()) {
        Canvas sub{m.cv.x0 + bx.i0 * RES, m.cv.y0 + bx.j0 * RES, bw, bh};
        OwnSink os{bw, bh, ch(6)};
        for (auto& [a, b] : rit->second) sub.seg(os, a, b, RES);
    }
    const int32_t me = net + 1;
    for (int y = sj0; y < sj1; ++y) {
        size_t s = size_t(y) * W, d = size_t(y - bx.j0) * bw - bx.i0;
        for (int x = si0; x < si1; ++x) {
            int32_t pf = m.padF[s + x], pb = m.padB[s + x];
            ch(0)[d + x] = m.interior[s + x];
            ch(1)[d + x] = pf == me;
            ch(2)[d + x] = pb == me;
            ch(3)[d + x] = pf > 0 && pf != me;
            ch(4)[d + x] = pb > 0 && pb != me;
            ch(5)[d + x] = m.drill[s + x];
            float others = float(m.rats[s + x]) - ch(6)[d + x];
            ch(7)[d + x] = std::min(std::max(others, 0.f), 4.f) / 4;
            int32_t zf = m.pourF[s + x], zb = m.pourB[s + x], zi = m.pourI[s + x];
            ch(8)[d + x] = zf > 0 && zf != me;
            ch(9)[d + x] = zb > 0 && zb != me;
            ch(10)[d + x] = zi > 0 && zi != me;
        }
    }
}

struct Job {
    int net;
    Box box;
};

// ---------------------------------------------------------------------------------------------------------------
// ONNX Runtime

#ifdef PCB_ONNXRT
#ifndef PCB_NVIDIA_DIR
#define PCB_NVIDIA_DIR ""
#endif

// The CUDA execution provider needs the CUDA 13 libraries and dlopens cuDNN; the pip wheels keep them under
// site-packages/nvidia (torch's copies), which no loader path names, so they are loaded here first, as
// onnxruntime.preload_dlls() does in Python. PCBROUTE_NVIDIA_DIR overrides the directory.
std::string nvidia_dir() {
    const char* env = std::getenv("PCBROUTE_NVIDIA_DIR");
    return env && *env ? env : PCB_NVIDIA_DIR;
}

int preload(const std::string& dir, std::vector<std::string> libs, int verbose) {
    if (dir.empty()) return 0;
    for (auto& l : libs) l = dir + "/" + l;
    for (int pass = 0; pass < 4 && !libs.empty(); ++pass) {  // a library whose dependency isn't loaded yet gets another go
        std::vector<std::string> next;
        for (auto& p : libs)
            if (!dlopen(p.c_str(), RTLD_NOW | RTLD_GLOBAL)) next.push_back(p);
        if (next.size() == libs.size()) break;
        libs.swap(next);
    }
    if (verbose > 1)
        for (auto& p : libs) std::fprintf(stderr, "ML model: could not preload %s\n", p.c_str());
    return int(libs.size());
}

struct Model {
    Ort::Env env{ORT_LOGGING_LEVEL_ERROR, "pcbroute"};
    std::unique_ptr<Ort::Session> sess;
    std::string device;
    int workers = 1;  // batches run at once (CUDA: one's CPU-side set-up and copies overlap the others' kernels)

    Model(const MlModelOptions& m) {
        auto make = [&](bool cuda) {
            Ort::SessionOptions so;
            // CUDA: the basic rewrites only (the extended and layout ones are CPU kernels, and slower here)
            so.SetGraphOptimizationLevel(cuda ? GraphOptimizationLevel::ORT_ENABLE_BASIC : GraphOptimizationLevel::ORT_ENABLE_ALL);
            so.SetIntraOpNumThreads(cuda ? 1 : int(std::max(1u, std::thread::hardware_concurrency())));
            if (cuda) {
                // dlopen serializes on the loader lock, so the libraries load one after the other; the CUDA context
                // (cudaFree(0), 0.2-0.5 s; the primary context, which ORT then uses) is made on the side meanwhile
                double tp = now();
                const std::string dir = nvidia_dir();
                preload(dir, {"cu13/lib/libcudart.so.13"}, m.verbose);
                dlopen("libcuda.so.1", RTLD_NOW | RTLD_GLOBAL);  // the driver, which the context needs, before the loader lock is busy
                std::promise<void> ctx_made;
                std::shared_future<void> ctx_ready = ctx_made.get_future().share();
                std::thread ctx([v = m.verbose, &ctx_made] {
                    double t = now();
                    if (void* rt = dlopen("libcudart.so.13", RTLD_NOW | RTLD_GLOBAL | RTLD_NOLOAD))
                        if (auto cuda_free = reinterpret_cast<int (*)(void*)>(dlsym(rt, "cudaFree"))) cuda_free(nullptr);
                    ctx_made.set_value();
                    if (v > 1) std::fprintf(stderr, "ML model: CUDA context made in %.2f s\n", now() - t);
                });
                preload(dir, {"cu13/lib/libcublasLt.so.13", "cu13/lib/libcublas.so.13", "cu13/lib/libcurand.so.10"}, m.verbose);
                try {
                    append_cuda(so, m.verbose);
                } catch (...) { ctx.join(); throw; }
                if (m.verbose > 1) std::fprintf(stderr, "ML model: CUDA and the provider loaded in %.2f s\n", now() - tp);
                // the session needs cuDNN only when it first runs: loaded meanwhile
                std::thread cudnn([dir, v = m.verbose, ctx_ready] {
                    double t = now();
                    preload(dir, {"cudnn/lib/libcudnn.so.9", "cudnn/lib/libcudnn_graph.so.9", "cudnn/lib/libcudnn_engines_precompiled.so.9",
                                  "cudnn/lib/libcudnn_heuristic.so.9", "cudnn/lib/libcudnn_ops.so.9", "cudnn/lib/libcudnn_cnn.so.9"}, v);
                    // and their one-time set-up (a handle made and dropped), which the first batch would otherwise pay
                    if (!std::getenv("PCBROUTE_ML_NO_WARM")) {
                        ctx_ready.wait();
                        void* h = nullptr;
                        if (void* dn = dlopen("libcudnn.so.9", RTLD_NOW | RTLD_GLOBAL | RTLD_NOLOAD)) {
                            auto mk = reinterpret_cast<int (*)(void**)>(dlsym(dn, "cudnnCreate"));
                            auto rm = reinterpret_cast<int (*)(void*)>(dlsym(dn, "cudnnDestroy"));
                            if (mk && rm && mk(&h) == 0) rm(h);
                        }
                        if (void* bl = dlopen("libcublas.so.13", RTLD_NOW | RTLD_GLOBAL | RTLD_NOLOAD)) {
                            auto mk = reinterpret_cast<int (*)(void**)>(dlsym(bl, "cublasCreate_v2"));
                            auto rm = reinterpret_cast<int (*)(void*)>(dlsym(bl, "cublasDestroy_v2"));
                            if (mk && rm && mk(&h) == 0) rm(h);
                        }
                    }
                    if (v > 1) std::fprintf(stderr, "ML model: cuDNN loaded (and warmed) in %.2f s\n", now() - t);
                });
                workers = 3;  // 2: 10-30 % slower runs, 4: no faster
                if (const char* e = std::getenv("PCBROUTE_ML_WORKERS")) workers = std::max(1, std::atoi(e));
                double ts = now();
                try {
                    sess = std::make_unique<Ort::Session>(env, m.model.c_str(), so);
                } catch (...) { cudnn.join(); ctx.join(); throw; }
                if (m.verbose > 1) std::fprintf(stderr, "ML model: session (cuda) created in %.2f s\n", now() - ts);
                cudnn.join();
                ctx.join();
            } else {
                double ts = now();
                sess = std::make_unique<Ort::Session>(env, m.model.c_str(), so);
                if (m.verbose > 1) std::fprintf(stderr, "ML model: session (cpu) created in %.2f s\n", now() - ts);
            }
            device = cuda ? "cuda" : "cpu";
        };
        if (m.device == "cpu") make(false);
        else if (m.device == "cuda") make(true);
        else {
            try { make(true); } catch (std::exception& e) {
                if (m.verbose) std::fprintf(stderr, "ML model: no CUDA (%s), running on the CPU\n", e.what());
                make(false);
            }
        }
    }

    static void append_cuda(Ort::SessionOptions& so, int verbose) {
        const OrtApi& api = Ort::GetApi();
        OrtCUDAProviderOptionsV2* cu = nullptr;
        Ort::ThrowOnError(api.CreateCUDAProviderOptions(&cu));
        // Every batch shape is new to cuDNN, and the default exhaustive benchmark (or even HEURISTIC, the cuDNN
        // frontend's plans) costs ~100 ms per shape: DEFAULT takes the stock algorithm. Not fuse_conv_bias: faster,
        // but its cuDNN graphs give different maps from run to run. No TF32: the maps stay float32.
        // PCBROUTE_ORT_CUDA="key=value,..." adds or overrides options.
        std::vector<std::pair<std::string, std::string>> kv = {{"device_id", "0"}, {"cudnn_conv_algo_search", "DEFAULT"},
            {"use_tf32", "0"}, {"arena_extend_strategy", "kNextPowerOfTwo"}};
        if (const char* e = std::getenv("PCBROUTE_ORT_CUDA")) {
            std::string s = e;
            for (size_t a = 0; a < s.size();) {
                size_t b = s.find(',', a);
                if (b == std::string::npos) b = s.size();
                std::string item = s.substr(a, b - a);
                size_t eq = item.find('=');
                if (eq != std::string::npos) {
                    std::string k = item.substr(0, eq), v = item.substr(eq + 1);
                    bool found = false;
                    for (auto& p : kv) if (p.first == k) { p.second = v; found = true; }
                    if (!found) kv.push_back({k, v});
                }
                a = b + 1;
            }
        }
        std::vector<const char*> keys, vals;
        for (auto& p : kv) { keys.push_back(p.first.c_str()); vals.push_back(p.second.c_str()); }
        OrtStatus* st = api.UpdateCUDAProviderOptions(cu, keys.data(), vals.data(), keys.size());
        if (st) { api.ReleaseCUDAProviderOptions(cu); Ort::ThrowOnError(st); }
        double ta = now();
        try {
            so.AppendExecutionProvider_CUDA_V2(*cu);
        } catch (...) { api.ReleaseCUDAProviderOptions(cu); throw; }
        api.ReleaseCUDAProviderOptions(cu);
        if (verbose > 1) std::fprintf(stderr, "ML model: CUDA provider loaded in %.2f s\n", now() - ta);
    }

    // x: (n, NIN, h, w) -> probabilities (n, NOUT, h, w); thread-safe
    std::vector<float> run(std::vector<float>& x, int n, int h, int w) {
        Ort::MemoryInfo mi = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        int64_t shape[4] = {n, NIN, h, w};
        Ort::Value in = Ort::Value::CreateTensor<float>(mi, x.data(), x.size(), shape, 4);
        const char* inames[] = {"x"};
        const char* onames[] = {"p"};
        auto out = sess->Run(Ort::RunOptions{nullptr}, inames, &in, 1, onames, 1);
        const float* p = out[0].GetTensorData<float>();
        return std::vector<float>(p, p + size_t(n) * NOUT * h * w);
    }

    // the same into the caller's memory (the server: the client's memfd)
    void run_into(float* x, float* p, int n, int h, int w, const Ort::RunOptions& ro) {
        Ort::MemoryInfo mi = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        int64_t ishape[4] = {n, NIN, h, w}, oshape[4] = {n, NOUT, h, w};
        Ort::Value in = Ort::Value::CreateTensor<float>(mi, x, size_t(n) * NIN * h * w, ishape, 4);
        Ort::Value out = Ort::Value::CreateTensor<float>(mi, p, size_t(n) * NOUT * h * w, oshape, 4);
        const char* inames[] = {"x"};
        const char* onames[] = {"p"};
        sess->Run(ro, inames, &in, 1, onames, &out, 1);
    }
};
#else
struct Model {
    std::string device = "-";
    int workers = 1;
    explicit Model(const MlModelOptions&) { throw std::runtime_error("--ml-model: this pcbroute was built without ONNX Runtime (make ONNXRT=1)"); }
    std::vector<float> run(std::vector<float>&, int, int, int) { return {}; }
};
#endif

// ---------------------------------------------------------------------------------------------------------------
// The resident model (pcbroute --ml-serve SOCK): the CUDA start-up (context, libraries, session, lazy kernel loading,
// tear-down: ~1 s) is paid once. A client connects to the unix socket, says which model and options it would load
// (identity(): only the same ones are served, so the maps are those of the in-process session), then sends its
// batches one at a time: the header (n, h, w) with a memfd (SCM_RIGHTS) holding the input tensor followed by room for
// the output one, which the server's session reads and writes in place. The client keeps its own batching, workers
// (one connection each) and big-batch lock, so the server runs exactly the batches the in-process path would.

#ifdef PCB_ONNXRT
constexpr uint32_t SERVE_MAGIC = 0x3153'4c4d;  // "MLS1"

std::string identity(const MlModelOptions& m) {
    char rp[PATH_MAX];
    if (!realpath(m.model.c_str(), rp)) throw std::runtime_error("cannot find " + m.model);
    struct stat st{};
    if (stat(rp, &st) != 0) throw std::runtime_error("cannot stat " + m.model);
    const char* cu = std::getenv("PCBROUTE_ORT_CUDA");
    char b[PATH_MAX + 512];
    std::snprintf(b, sizeof b, "model=%s\nsize=%lld\nmtime=%lld.%09ld\nort=%s\ncuda=%s\n", rp, (long long)st.st_size,
                  (long long)st.st_mtim.tv_sec, (long)st.st_mtim.tv_nsec, Ort::GetVersionString().c_str(), cu ? cu : "");
    return b;
}

bool write_all(int fd, const void* p, size_t n) {
    const char* c = static_cast<const char*>(p);
    while (n) {
        ssize_t k = ::send(fd, c, n, MSG_NOSIGNAL);
        if (k < 0 && errno == EINTR) continue;
        if (k <= 0) return false;
        c += k;
        n -= size_t(k);
    }
    return true;
}

bool read_all(int fd, void* p, size_t n) {
    char* c = static_cast<char*>(p);
    while (n) {
        ssize_t k = ::recv(fd, c, n, 0);
        if (k < 0 && errno == EINTR) continue;
        if (k <= 0) return false;
        c += k;
        n -= size_t(k);
    }
    return true;
}

bool write_msg(int fd, int32_t status, const std::string& msg) {
    uint32_t hdr[2] = {uint32_t(status), uint32_t(msg.size())};
    return write_all(fd, hdr, 8) && write_all(fd, msg.data(), msg.size());
}

bool read_msg(int fd, int32_t& status, std::string& msg) {
    uint32_t hdr[2];
    if (!read_all(fd, hdr, 8) || hdr[1] > (1u << 20)) return false;
    status = int32_t(hdr[0]);
    msg.resize(hdr[1]);
    return read_all(fd, msg.data(), msg.size());
}

void set_timeout(int fd, double s) {
    timeval tv{time_t(s), suseconds_t((s - double(time_t(s))) * 1e6)};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
}

sockaddr_un sock_addr(const std::string& path) {
    sockaddr_un a{};
    a.sun_family = AF_UNIX;
    if (path.size() >= sizeof a.sun_path) throw std::runtime_error("socket path too long: " + path);
    std::memcpy(a.sun_path, path.c_str(), path.size() + 1);
    return a;
}

// a connection to the server that accepted our identity, or -1 (err says why)
int connect_server(const std::string& path, const std::string& ident, std::string& err) {
    int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) { err = std::strerror(errno); return -1; }
    sockaddr_un a = sock_addr(path);
    if (::connect(fd, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0) {
        err = std::strerror(errno);
        ::close(fd);
        return -1;
    }
    set_timeout(fd, 5);
    uint32_t hdr[2] = {SERVE_MAGIC, uint32_t(ident.size())};
    int32_t st = -1;
    if (!write_all(fd, hdr, 8) || !write_all(fd, ident.data(), ident.size()) || !read_msg(fd, st, err)) {
        if (err.empty()) err = "no answer";
        ::close(fd);
        return -1;
    }
    if (st != 0) { ::close(fd); return -1; }
    set_timeout(fd, 600);  // a batch: a big board on a busy GPU takes seconds, a hung server must not hang us
    return fd;
}

// one batch over a connection: false when the connection failed (err), throws on the server's errors
bool remote_run(int fd, const std::vector<float>& x, int n, int h, int w, std::vector<float>& out, std::string& err) {
    const size_t nin = size_t(n) * NIN * h * w, nout = size_t(n) * NOUT * h * w;
    const size_t bytes = (nin + nout) * sizeof(float);
    int mfd = memfd_create("pcbroute-ml", MFD_CLOEXEC);
    if (mfd < 0) { err = std::string("memfd: ") + std::strerror(errno); return false; }
    if (ftruncate(mfd, off_t(bytes)) != 0) { err = std::string("memfd: ") + std::strerror(errno); ::close(mfd); return false; }
    void* mp = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, 0);
    if (mp == MAP_FAILED) { err = std::string("mmap: ") + std::strerror(errno); ::close(mfd); return false; }
    float* f = static_cast<float*>(mp);
    std::memcpy(f, x.data(), nin * sizeof(float));
    uint32_t hdr[4] = {SERVE_MAGIC, uint32_t(n), uint32_t(h), uint32_t(w)};
    iovec iov{hdr, sizeof hdr};
    alignas(cmsghdr) char ctl[CMSG_SPACE(sizeof(int))] = {};
    msghdr mh{};
    mh.msg_iov = &iov;
    mh.msg_iovlen = 1;
    mh.msg_control = ctl;
    mh.msg_controllen = sizeof ctl;
    cmsghdr* c = CMSG_FIRSTHDR(&mh);
    c->cmsg_level = SOL_SOCKET;
    c->cmsg_type = SCM_RIGHTS;
    c->cmsg_len = CMSG_LEN(sizeof(int));
    std::memcpy(CMSG_DATA(c), &mfd, sizeof(int));
    ssize_t k;
    do k = ::sendmsg(fd, &mh, MSG_NOSIGNAL); while (k < 0 && errno == EINTR);
    ::close(mfd);
    int32_t st = -1;
    std::string msg;
    bool ok = k == ssize_t(sizeof hdr) && read_msg(fd, st, msg);
    if (ok && st == 0) out.assign(f + nin, f + nin + nout);
    munmap(mp, bytes);
    if (!ok) { err = "lost the server"; return false; }
    if (st != 0) throw std::runtime_error("ML server: " + msg);
    return true;
}

// the server's side of a connection; run(n, h, w, in, out) runs one batch
template <class RunFn> void serve_conn(int fd, const std::string& ident, RunFn&& run, int verbose) {
    uint32_t hdr[4];
    std::string theirs;
    if (!read_all(fd, hdr, 8) || hdr[0] != SERVE_MAGIC || hdr[1] > 65536) return;
    theirs.resize(hdr[1]);
    if (!read_all(fd, theirs.data(), theirs.size())) return;
    if (theirs != ident) {
        if (verbose) {
            std::string t = theirs;
            std::replace(t.begin(), t.end(), '\n', ' ');
            std::fprintf(stderr, "ML server: refused a client for another model or options: %s\n", t.c_str());
        }
        write_msg(fd, 1, "it serves another model or other options");
        return;
    }
    if (!write_msg(fd, 0, "cuda")) return;
    set_timeout(fd, 0);  // a client may sit between batches as long as it likes
    for (;;) {
        iovec iov{hdr, sizeof hdr};
        alignas(cmsghdr) char ctl[CMSG_SPACE(sizeof(int))] = {};
        msghdr mh{};
        mh.msg_iov = &iov;
        mh.msg_iovlen = 1;
        mh.msg_control = ctl;
        mh.msg_controllen = sizeof ctl;
        ssize_t k;
        do k = ::recvmsg(fd, &mh, MSG_CMSG_CLOEXEC | MSG_WAITALL); while (k < 0 && errno == EINTR);
        int mfd = -1;
        for (cmsghdr* c = CMSG_FIRSTHDR(&mh); c; c = CMSG_NXTHDR(&mh, c))
            if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_RIGHTS) std::memcpy(&mfd, CMSG_DATA(c), sizeof(int));
        if (k != ssize_t(sizeof hdr) || hdr[0] != SERVE_MAGIC || mfd < 0) {
            if (mfd >= 0) ::close(mfd);
            return;
        }
        const int n = int(hdr[1]), h = int(hdr[2]), w = int(hdr[3]);
        const size_t nin = size_t(n) * NIN * h * w, nout = size_t(n) * NOUT * h * w, bytes = (nin + nout) * sizeof(float);
        struct stat st{};
        std::string err;
        void* mp = MAP_FAILED;
        if (n <= 0 || h <= 0 || w <= 0 || fstat(mfd, &st) != 0 || size_t(st.st_size) != bytes) err = "bad batch";
        else if ((mp = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, 0)) == MAP_FAILED) err = std::strerror(errno);
        ::close(mfd);
        if (err.empty()) {
            try {
                run(n, h, w, static_cast<float*>(mp), static_cast<float*>(mp) + nin);
            } catch (std::exception& e) { err = e.what(); }
            munmap(mp, bytes);
        }
        if (!write_msg(fd, err.empty() ? 0 : 2, err)) return;
    }
}
#endif

// What predict() runs the batches on: the resident model at mo.server when it answers for our model and options,
// else (and from a batch on which the server failed) a session of our own.
struct Engine {
    MlModelOptions mo;
    std::string device;
    int workers = 1;
    std::shared_ptr<Model> local;
    std::mutex mu;
    std::string sock, ident;
    std::vector<int> conns;  // idle connections to the server
    bool remote = false;

    explicit Engine(const MlModelOptions& m) : mo(m) {
#ifdef PCB_ONNXRT
        if (!mo.server.empty() && mo.device != "cpu") {
            std::string err;
            try {
                ident = identity(mo);
                int fd = connect_server(mo.server, ident, err);
                if (fd >= 0) {
                    conns.push_back(fd);
                    sock = mo.server;
                    remote = true;
                    device = "cuda (served by " + sock + ")";
                    workers = 3;
                    if (const char* e = std::getenv("PCBROUTE_ML_WORKERS")) workers = std::max(1, std::atoi(e));
                    return;
                }
            } catch (std::exception& e) { err = e.what(); }
            if (mo.verbose) std::fprintf(stderr, "ML model: no server at %s (%s), loading the model here\n", mo.server.c_str(), err.c_str());
            if (mo.spawn_server) spawn();
        }
#endif
        local = std::make_shared<Model>(mo);
        device = local->device;
        workers = local->workers;
    }

    ~Engine() {
#ifdef PCB_ONNXRT
        for (int fd : conns) ::close(fd);
#endif
    }

#ifdef PCB_ONNXRT
    // pcbroute --ml-serve, detached (own session, no inherited descriptors, output to SOCK.log); a second one started
    // by another client meanwhile sees the lock and leaves
    void spawn() {
        char self[PATH_MAX];
        ssize_t n = readlink("/proc/self/exe", self, sizeof self - 1);
        if (n <= 0) return;
        self[n] = 0;
        char model[PATH_MAX], sockdir[PATH_MAX];
        if (!realpath(mo.model.c_str(), model)) return;
        std::string sock = mo.server;  // made absolute: the server runs in /
        if (sock[0] != '/' && getcwd(sockdir, sizeof sockdir)) sock = std::string(sockdir) + "/" + sock;
        std::vector<std::string> args = {self, "--ml-serve", sock, "--ml-model", model, "--ml-device", mo.device == "auto" ? "cuda" : mo.device};
        std::string log = sock + ".log";
        pid_t pid = fork();
        if (pid < 0) return;
        if (pid == 0) {
            setsid();
            if (fork() != 0) _exit(0);
            if (chdir("/") != 0) _exit(126);
            int nul = open("/dev/null", O_RDONLY);
            int lg = open(log.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0600);
            if (nul >= 0) dup2(nul, 0);
            if (lg >= 0) { dup2(lg, 1); dup2(lg, 2); }
            for (int fd = 3; fd < 1024; ++fd) ::close(fd);
            std::vector<char*> av;
            for (auto& a : args) av.push_back(a.data());
            av.push_back(nullptr);
            execv(self, av.data());
            _exit(127);
        }
        waitpid(pid, nullptr, 0);
        if (mo.verbose) std::fprintf(stderr, "ML model: started a server at %s for the next runs (log %s)\n", mo.server.c_str(), log.c_str());
    }
#endif

    std::shared_ptr<Model> local_model() {
        std::lock_guard<std::mutex> g(mu);
        if (!local) {
            local = std::make_shared<Model>(mo);
            device = local->device + " (the server failed)";
        }
        return local;
    }

    // thread-safe, as Model::run
    std::vector<float> run(std::vector<float>& x, int n, int h, int w) {
#ifdef PCB_ONNXRT
        if (remote) {
            std::string err;
            int fd = -1;
            {
                std::lock_guard<std::mutex> g(mu);
                if (remote && !conns.empty()) { fd = conns.back(); conns.pop_back(); }
            }
            if (fd < 0 && remote) fd = connect_server(sock, ident, err);
            if (fd >= 0) {
                std::vector<float> out;
                bool ok;
                try {
                    ok = remote_run(fd, x, n, h, w, out, err);
                } catch (...) { ::close(fd); throw; }
                if (ok) {
                    std::lock_guard<std::mutex> g(mu);
                    conns.push_back(fd);
                    return out;
                }
                ::close(fd);
            }
            {
                std::lock_guard<std::mutex> g(mu);
                if (remote && mo.verbose) std::fprintf(stderr, "ML model: the server at %s failed (%s), carrying on here\n", sock.c_str(), err.c_str());
                remote = false;
            }
        }
#endif
        return local_model()->run(x, n, h, w);
    }
};

using ModelFuture = std::shared_future<std::shared_ptr<Engine>>;

ModelFuture load_model(const MlModelOptions& mo) {
    if (mo.model.empty()) {
        std::promise<std::shared_ptr<Engine>> none;
        none.set_value(nullptr);
        return none.get_future().share();
    }
    return std::async(std::launch::async, [mo] {
        double t = now();
        auto m = std::make_shared<Engine>(mo);
        if (mo.verbose > 1) std::fprintf(stderr, "ML model: %s loaded on %s in %.2f s\n", mo.model.c_str(), m->device.c_str(), now() - t);
        return m;
    }).share();
}

MlPrior predict(const Board& B, const MlModelOptions& mo, const ModelFuture& mf);

}  // namespace

struct MlPredictor::State {
    MlModelOptions mo;
    ModelFuture model;
    std::promise<MlPrior> done;
    std::shared_future<MlPrior> prior;
    std::thread th;
};

MlPredictor::MlPredictor(const MlModelOptions& mo) : s(std::make_shared<State>()) {
    s->mo = mo;
    s->model = load_model(mo);
}

MlPredictor::~MlPredictor() {
    if (s->th.joinable()) s->th.join();
    else if (s->model.valid()) s->model.wait();
}

void MlPredictor::start(const Board& B) {
    s->prior = s->done.get_future().share();
    State* st = s.get();
    s->th = std::thread([st, &B] {
        try {
            st->done.set_value(predict(B, st->mo, st->model));
        } catch (...) {
            st->done.set_exception(std::current_exception());
        }
        // the session and its GPU memory go while the router works. Not the CUDA context: it is the device's primary
        // context, which the GPU build's own CUDA runtime shares (cudaDeviceReset would pull it from under the solver)
        double t = now();
        st->model = ModelFuture();
        if (st->mo.verbose > 1) std::fprintf(stderr, "ML model: session released in %.2f s\n", now() - t);
    });
}

const MlPrior& MlPredictor::get() { return s->prior.get(); }

#ifdef PCB_ONNXRT
namespace {
volatile sig_atomic_t g_serve_stop = 0;
void serve_stop(int) { g_serve_stop = 1; }
}  // namespace
#endif

int ml_serve(const MlModelOptions& m, const std::string& sock, double idle_s, double trim_s) {
#ifndef PCB_ONNXRT
    (void)m, (void)sock, (void)idle_s, (void)trim_s;
    throw std::runtime_error("this pcbroute was built without ONNX Runtime (make ONNXRT=1)");
#else
    const int v = m.verbose;
    // one server per socket: the lock is held for the server's life
    const std::string lockp = sock + ".lock";
    int lk = ::open(lockp.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (lk < 0) throw std::runtime_error("cannot open " + lockp + ": " + std::strerror(errno));
    if (flock(lk, LOCK_EX | LOCK_NB) != 0) {
        if (v) std::fprintf(stderr, "ML server: %s is already served (or starting)\n", sock.c_str());
        return 0;
    }
    const double t0 = now();
    const std::string ident = identity(m);
    Model model(m);
    if (model.device != "cuda") throw std::runtime_error("the model did not load on CUDA: nothing to gain from serving it");

    // the batches run on a fixed set of threads, so the CUDA provider's per-thread state (streams, cuDNN and cuBLAS
    // handles) and the lazily loaded kernels stay warm from client to client
    struct Job {
        int n, h, w;
        float *in, *out;
        const Ort::RunOptions* ro;
        std::promise<void> done;
    };
    std::mutex qmu;
    std::condition_variable qcv;
    std::deque<Job*> q;
    std::atomic<int> warm{0};
    int nthreads = 4;
    if (const char* e = std::getenv("PCBROUTE_ML_SERVE_THREADS")) nthreads = std::max(1, std::atoi(e));
    Ort::RunOptions plain{nullptr};
    std::vector<std::thread> pool;
    for (int t = 0; t < nthreads; ++t)
        pool.emplace_back([&] {
            {   // the kernels this thread will need, loaded now rather than in a client's first batch
                std::vector<float> x(size_t(NIN) * 64 * 64, 0.f), p(size_t(NOUT) * 64 * 64);
                try { model.run_into(x.data(), p.data(), 1, 64, 64, plain); } catch (...) {}
                ++warm;
            }
            for (;;) {
                Job* j;
                {
                    std::unique_lock<std::mutex> g(qmu);
                    qcv.wait(g, [&] { return !q.empty(); });
                    j = q.front();
                    q.pop_front();
                }
                try {
                    model.run_into(j->in, j->out, j->n, j->h, j->w, *j->ro);
                    j->done.set_value();
                } catch (...) { j->done.set_exception(std::current_exception()); }
            }
        });
    for (auto& t : pool) t.detach();  // they live as long as the process (which ends with _exit)
    while (warm < nthreads) std::this_thread::sleep_for(std::chrono::milliseconds(5));

    std::atomic<int> conns{0}, busy{0};
    std::atomic<double> last{now()};
    std::atomic<bool> trimmed{false};
    std::atomic<long> served{0};
    std::mutex big;  // one big batch at a time over all clients (GPU memory), as each client does on its own
    auto submit = [&](int n, int h, int w, float* in, float* out, const Ort::RunOptions& ro) {
        Job j{n, h, w, in, out, &ro, {}};
        auto f = j.done.get_future();
        {
            std::lock_guard<std::mutex> g(qmu);
            q.push_back(&j);
        }
        qcv.notify_one();
        f.get();
    };
    auto run = [&](int n, int h, int w, float* in, float* out) {
        ++busy;
        trimmed = false;
        double tb = now();
        try {
            std::unique_lock<std::mutex> lk2(big, std::defer_lock);
            if (double(n) * h * w > 4e6) lk2.lock();
            submit(n, h, w, in, out, plain);
        } catch (...) { last = now(); --busy; throw; }
        last = now();
        --busy;
        ++served;
        if (v > 2) std::fprintf(stderr, "ML server: batch %d x %dx%d in %.3f s\n", n, w, h, now() - tb);
    };

    // the socket, made under another name and renamed into place once it listens
    // (from here on, the pool's threads use this frame: errors end the process, not the function)
    const std::string tmp = sock + "~" + std::to_string(getpid());
    int lfd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    sockaddr_un a{};
    a.sun_family = AF_UNIX;
    if (tmp.size() >= sizeof a.sun_path) {
        std::fprintf(stderr, "pcbroute --ml-serve: socket path too long (%zu bytes, at most %zu): %s\n", tmp.size(), sizeof a.sun_path - 1, tmp.c_str());
        _exit(1);
    }
    std::memcpy(a.sun_path, tmp.c_str(), tmp.size() + 1);
    ::unlink(tmp.c_str());
    if (lfd < 0 || ::bind(lfd, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0 || ::listen(lfd, 64) != 0 || chmod(tmp.c_str(), 0600) != 0 ||
        ::rename(tmp.c_str(), sock.c_str()) != 0) {
        std::fprintf(stderr, "pcbroute --ml-serve: cannot serve on %s: %s\n", sock.c_str(), std::strerror(errno));
        _exit(1);
    }
    struct stat mine{};
    stat(sock.c_str(), &mine);
    signal(SIGPIPE, SIG_IGN);
    signal(SIGTERM, serve_stop);
    signal(SIGINT, serve_stop);
    signal(SIGHUP, serve_stop);
    if (v) std::fprintf(stderr, "ML server: %s on %s, ready in %.2f s (pid %d; exits after %.0f s idle)\n", m.model.c_str(), sock.c_str(),
                        now() - t0, int(getpid()), idle_s);
    std::fflush(stderr);
    last = now();
    while (!g_serve_stop) {
        pollfd p{lfd, POLLIN, 0};
        int r = poll(&p, 1, 500);
        if (r > 0 && (p.revents & POLLIN)) {
            int fd = ::accept4(lfd, nullptr, nullptr, SOCK_CLOEXEC);
            if (fd < 0) continue;
            ++conns;
            last = now();
            std::thread([&, fd] {
                serve_conn(fd, ident, run, v);
                ::close(fd);
                last = now();
                --conns;
            }).detach();
            continue;
        }
        const double idle = now() - last;
        if (conns == 0 && busy == 0 && idle > idle_s) break;
        if (!trimmed && busy == 0 && idle > trim_s) {
            // the arena's free GPU memory back to the device (a tiny batch run with the shrink option); the session,
            // the context and the loaded kernels stay
            Ort::RunOptions shrink;
            shrink.AddConfigEntry(kOrtRunOptionsConfigEnableMemoryArenaShrinkage, "gpu:0");
            std::vector<float> x(size_t(NIN) * 32 * 32, 0.f), y(size_t(NOUT) * 32 * 32);
            try { submit(1, 32, 32, x.data(), y.data(), shrink); } catch (...) {}
            trimmed = true;
            if (v > 1) std::fprintf(stderr, "ML server: idle %.0f s, GPU arena trimmed\n", idle);
            std::fflush(stderr);
        }
    }
    struct stat now_st{};
    if (stat(sock.c_str(), &now_st) == 0 && now_st.st_ino == mine.st_ino && now_st.st_dev == mine.st_dev) ::unlink(sock.c_str());
    if (v) std::fprintf(stderr, "ML server: %s closed after %ld batches (%s)\n", sock.c_str(), served.load(), g_serve_stop ? "signal" : "idle");
    std::fflush(stderr);
    _exit(0);  // not the CUDA and ORT tear-down (detached connection threads may still sit in recv)
#endif
}

bool ml_infer_available() {
#ifdef PCB_ONNXRT
    return true;
#else
    return false;
#endif
}

MlPrior predict_ml_prior(const Board& B, const MlModelOptions& mo) {
    MlPredictor P(mo);
    P.start(B);
    return P.get();
}

namespace {

// write_prior for one net: per router layer max(its folded map, union_share * union of the three track maps), grown
// by `grow` px (maxpool, zero padding), 0-255, rows flipped to DSN axes (y up)
NetPrior net_prior(const float* pr, int w, int h, const std::vector<int>& fl, const MlModelOptions& mo, const Box& box, int H) {
    const int NL = int(fl.size());
    const size_t plane = size_t(w) * h;
    const float share = float(mo.union_share);
    std::vector<float> L(size_t(NL) * plane), tmp(plane);
    for (int l = 0; l < NL; ++l) {
        const float* own = pr + fl[l] * plane;
        float* d = L.data() + l * plane;
        for (size_t i = 0; i < plane; ++i) {
            float u = std::max(std::max(pr[i], pr[plane + i]), pr[2 * plane + i]);
            d[i] = std::max(own[i], share * u);
        }
        if (mo.grow > 0) {
            const int r = mo.grow;
            for (int y = 0; y < h; ++y)
                for (int xx = 0; xx < w; ++xx) {
                    float v = d[size_t(y) * w + xx];
                    for (int xd = std::max(0, xx - r); xd <= std::min(w - 1, xx + r); ++xd) v = std::max(v, d[size_t(y) * w + xd]);
                    tmp[size_t(y) * w + xx] = std::max(v, 0.f);
                }
            for (int y = 0; y < h; ++y)
                for (int xx = 0; xx < w; ++xx) {
                    float v = tmp[size_t(y) * w + xx];
                    for (int yd = std::max(0, y - r); yd <= std::min(h - 1, y + r); ++yd) v = std::max(v, tmp[size_t(yd) * w + xx]);
                    d[size_t(y) * w + xx] = v;
                }
        }
    }
    NetPrior N;
    N.i0 = box.i0;
    N.j0 = H - (box.j0 + h);
    N.w = w;
    N.h = h;
    N.data.resize(size_t(NL) * plane);
    for (int l = 0; l < NL; ++l)
        for (int y = 0; y < h; ++y) {
            const float* src = L.data() + l * plane + size_t(h - 1 - y) * w;
            uint8_t* dst = N.data.data() + l * plane + size_t(y) * w;
            for (int xx = 0; xx < w; ++xx) dst[xx] = uint8_t(std::min(255.f, std::max(0.f, std::nearbyint(src[xx] * 255.f))));
        }
    return N;
}

MlPrior predict(const Board& B, const MlModelOptions& mo, const ModelFuture& mf) {
    double t0 = now();
    BoardR bd = board_from(B);
    Maps m = board_maps(bd);
    const bool multilayer = bd.copper_layers > 2;
    const int H = m.cv.h, padpx = int(MARGIN / RES);
    std::vector<Job> jobs;
    for (int net : m.rat_nets) {
        if (std::find(bd.plane_nets.begin(), bd.plane_nets.end(), net) != bd.plane_nets.end()) continue;
        double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
        for (auto& [a, b] : m.rat.at(net))
            for (const Pt& p : {a, b}) { x0 = std::min(x0, p[0]); y0 = std::min(y0, p[1]); x1 = std::max(x1, p[0]); y1 = std::max(y1, p[1]); }
        int i0 = int((x0 - m.cv.x0) / RES) - padpx, j0 = int((y0 - m.cv.y0) / RES) - padpx;
        int i1 = int((x1 - m.cv.x0) / RES) + padpx, j1 = int((y1 - m.cv.y0) / RES) + padpx;
        auto up32 = [](int v) { int q = v + 31; return (q >= 0 ? q / 32 : -((-q + 31) / 32)) * 32; };  // Python's //
        jobs.push_back({net, {i0, j0, up32(i1 - i0), up32(j1 - j0)}});
    }
    std::stable_sort(jobs.begin(), jobs.end(), [](const Job& a, const Job& b) { return int64_t(a.box.w) * a.box.h < int64_t(b.box.w) * b.box.h; });
    // batches as mlroute makes them: consecutive nets of one map size, up to 16 Mpx
    std::vector<std::pair<size_t, size_t>> batches;
    for (size_t k = 0; k < jobs.size();) {
        const int w = jobs[k].box.w, h = jobs[k].box.h;
        size_t nb = 1;
        while (k + nb < jobs.size() && jobs[k + nb].box.w == w && jobs[k + nb].box.h == h && double(nb + 1) * w * h < BATCH_PX) ++nb;
        batches.push_back({k, nb});
        k += nb;
    }
    double t_raster = now() - t0;

    MlPrior M;
    M.pitch = RES * 1000.0;
    M.x0 = m.cv.x0 * 1000.0;
    M.y0 = -(m.cv.y0 + H * RES) * 1000.0;
    M.layers = B.layers;
    std::vector<int> fl;
    for (auto& n : B.layers) fl.push_back(fold(n));
    if (!mo.dump_inputs.empty()) {
        FILE* dump = std::fopen(mo.dump_inputs.c_str(), "wb");
        if (!dump) throw std::runtime_error("cannot write " + mo.dump_inputs);
        uint32_t n = uint32_t(jobs.size());
        std::fwrite("MLX1", 1, 4, dump);
        std::fwrite(&n, 4, 1, dump);
        for (auto& J : jobs) {
            std::vector<float> x(size_t(NIN) * J.box.w * J.box.h, 0.f);
            net_inputs(m, J.net, multilayer, J.box, x.data());
            int32_t hdr[5] = {J.net, J.box.i0, J.box.j0, J.box.w, J.box.h};
            std::fwrite(hdr, 4, 5, dump);
            std::fwrite(x.data(), 4, x.size(), dump);
        }
        std::fclose(dump);
    }
    std::shared_ptr<Engine> model = mf.get();  // loading since the start
    double t_wait = now() - t0 - t_raster, t1 = now();
    if (model) {
        // workers take the batches in turn; one big batch at a time (GPU memory), the results go in in batch order
        std::vector<std::vector<NetPrior>> res(batches.size());
        std::atomic<size_t> next{0};
        std::mutex big;
        std::exception_ptr err;
        std::mutex err_mu;
        auto work = [&] {
            for (size_t bi; (bi = next++) < batches.size();) {
                try {
                    auto [k, nb] = batches[bi];
                    const int w = jobs[k].box.w, h = jobs[k].box.h;
                    const size_t per = size_t(NIN) * w * h, plane = size_t(w) * h;
                    std::vector<float> x(per * nb, 0.f);
                    for (size_t b = 0; b < nb; ++b) net_inputs(m, jobs[k + b].net, multilayer, jobs[k + b].box, x.data() + b * per);
                    std::unique_lock<std::mutex> lk(big, std::defer_lock);
                    if (double(nb) * plane > 4e6) lk.lock();
                    double tb = now();
                    std::vector<float> p = model->run(x, int(nb), h, w);
                    if (lk.owns_lock()) lk.unlock();
                    if (mo.verbose > 2) std::fprintf(stderr, "ML model: batch %zu: %zu x %dx%d in %.3f s\n", bi, nb, w, h, now() - tb);
                    for (size_t b = 0; b < nb; ++b) res[bi].push_back(net_prior(p.data() + b * NOUT * plane, w, h, fl, mo, jobs[k + b].box, H));
                } catch (...) {
                    std::lock_guard<std::mutex> g(err_mu);
                    if (!err) err = std::current_exception();
                    next = batches.size();
                }
            }
        };
        std::vector<std::thread> th;
        for (int t = 1; t < std::min<int>(model->workers, int(batches.size())); ++t) th.emplace_back(work);
        work();
        for (auto& t : th) t.join();
        if (err) std::rethrow_exception(err);
        for (size_t bi = 0; bi < batches.size(); ++bi)
            for (size_t b = 0; b < res[bi].size(); ++b) M.nets[B.nets[jobs[batches[bi].first + b].net - 1].name] = std::move(res[bi][b]);
    }
    if (mo.verbose)
        std::fprintf(stderr, "ML model: RouteNet predicted %zu nets in %zu batches on %dx%d px (%.2f mm) on %s in %.2f s (raster %.2f, waiting for "
                     "the model %.2f, run %.2f); %zu pour nets left to the router\n", M.nets.size(), batches.size(), m.cv.w, H, RES,
                     model ? model->device.c_str() : "-", now() - t0, t_raster, t_wait, now() - t1, bd.plane_nets.size());
    return M;
}

}  // namespace

void write_ml_prior(const MlPrior& M, const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) throw std::runtime_error("cannot write " + path);
    auto u32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
    auto str = [&](const std::string& s) { u32(uint32_t(s.size())); std::fwrite(s.data(), 1, s.size(), f); };
    std::fwrite("MLP1", 1, 4, f);
    std::fwrite(&M.pitch, 8, 1, f);
    std::fwrite(&M.x0, 8, 1, f);
    std::fwrite(&M.y0, 8, 1, f);
    u32(uint32_t(M.layers.size()));
    for (auto& l : M.layers) str(l);
    u32(uint32_t(M.nets.size()));
    for (auto& [name, N] : M.nets) {
        str(name);
        int32_t ij[2] = {N.i0, N.j0};
        std::fwrite(ij, 4, 2, f);
        u32(uint32_t(N.w));
        u32(uint32_t(N.h));
        std::fwrite(N.data.data(), 1, N.data.size(), f);
    }
    std::fclose(f);
}
