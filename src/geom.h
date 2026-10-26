// Geometry: every copper or keepout outline is a Shape, a set of segments (a point is a zero-length segment)
// thickened by a radius, optionally with a filled polygon interior. Units: micrometres, DSN axes (y up).
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

// Routing keeps this much more than the clearance from the static copper: the DSN approximates rounded pads by
// polygons whose vertices lie on the arc, so their copper is up to a few micrometres inside the real pad, and
// KiCad's DRC measures to the real one (seen: a track 2 um short of the clearance at a rounded 0603 pad corner).
constexpr double CLR_SAFETY = 5;  // um

struct P {
    double x = 0, y = 0;
};
inline P operator+(P a, P b) { return {a.x + b.x, a.y + b.y}; }
inline P operator-(P a, P b) { return {a.x - b.x, a.y - b.y}; }
inline P operator*(P a, double k) { return {a.x * k, a.y * k}; }
inline double dot(P a, P b) { return a.x * b.x + a.y * b.y; }
inline double cross(P a, P b) { return a.x * b.y - a.y * b.x; }
inline double norm(P a) { return std::sqrt(dot(a, a)); }

inline double seg_point_dist(P a, P b, P p) {
    P d = b - a;
    double l2 = dot(d, d);
    double t = l2 > 0 ? std::clamp(dot(p - a, d) / l2, 0.0, 1.0) : 0.0;
    return norm(a + d * t - p);
}

inline bool seg_intersect(P a, P b, P c, P d) {
    double d1 = cross(b - a, c - a), d2 = cross(b - a, d - a), d3 = cross(d - c, a - c), d4 = cross(d - c, b - c);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0;
}

inline double seg_seg_dist(P a, P b, P c, P d) {
    if (seg_intersect(a, b, c, d)) return 0;
    return std::min({seg_point_dist(a, b, c), seg_point_dist(a, b, d), seg_point_dist(c, d, a), seg_point_dist(c, d, b)});
}

inline bool point_in_poly(const std::vector<P>& poly, P p) {
    bool in = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const P &a = poly[i], &b = poly[j];
        if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
    }
    return in;
}

struct Box {
    double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
    void add(P p, double r = 0) {
        x0 = std::min(x0, p.x - r); y0 = std::min(y0, p.y - r);
        x1 = std::max(x1, p.x + r); y1 = std::max(y1, p.y + r);
    }
    void add(const Box& b) { x0 = std::min(x0, b.x0); y0 = std::min(y0, b.y0); x1 = std::max(x1, b.x1); y1 = std::max(y1, b.y1); }
    bool empty() const { return x0 > x1; }
    bool overlaps(const Box& b, double m = 0) const {
        return x0 - m <= b.x1 && b.x0 - m <= x1 && y0 - m <= b.y1 && b.y0 - m <= y1;
    }
    Box grown(double m) const { return {x0 - m, y0 - m, x1 + m, y1 + m}; }
};

struct Shape {
    std::vector<std::pair<P, P>> segs;  // the core (edges of the polygon if filled)
    std::vector<P> poly;                // filled interior (empty if none)
    double r = 0;                       // thickening radius around the core

    static Shape circle(P c, double r) { Shape s; s.segs.push_back({c, c}); s.r = r; return s; }
    static Shape path(const std::vector<P>& pts, double r) {
        Shape s; s.r = r;
        if (pts.size() == 1) s.segs.push_back({pts[0], pts[0]});
        for (size_t i = 1; i < pts.size(); ++i) s.segs.push_back({pts[i - 1], pts[i]});
        return s;
    }
    static Shape polygon(std::vector<P> pts, double r = 0) {
        if (pts.size() > 1 && pts.front().x == pts.back().x && pts.front().y == pts.back().y) pts.pop_back();
        Shape s; s.r = r; s.poly = pts;
        for (size_t i = 0; i < pts.size(); ++i) s.segs.push_back({pts[i], pts[(i + 1) % pts.size()]});
        return s;
    }
    Box box() const {
        Box b;
        for (auto& [a, c] : segs) { b.add(a, r); b.add(c, r); }
        return b;
    }
    // Distance from p to this shape's copper (0 inside).
    double dist(P p) const {
        if (poly.size() >= 3 && point_in_poly(poly, p)) return 0;
        double d = 1e300;
        for (auto& [a, b] : segs) d = std::min(d, seg_point_dist(a, b, p));
        return std::max(0.0, d - r);
    }
    P center() const { Box b = box(); return {(b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2}; }
};

// Area of a filled polygon (0 if none).
inline double poly_area(const std::vector<P>& p) {
    double a = 0;
    for (size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) a += cross(p[j], p[i]);
    return p.size() >= 3 ? std::fabs(a) / 2 : 0;
}

// Gap between two shapes' copper (0 if they touch or overlap).
inline double shape_dist(const Shape& A, const Shape& B) {
    if (A.poly.size() >= 3)
        for (auto& [a, b] : B.segs) if (point_in_poly(A.poly, a)) return 0;
    if (B.poly.size() >= 3)
        for (auto& [a, b] : A.segs) if (point_in_poly(B.poly, a)) return 0;
    double d = 1e300;
    for (auto& [a, b] : A.segs)
        for (auto& [c, e] : B.segs) d = std::min(d, seg_seg_dist(a, b, c, e));
    return std::max(0.0, d - A.r - B.r);
}
