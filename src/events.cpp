#include "events.h"

#include <chrono>
#include <cmath>
#include <set>

Events g_events;

static double mono() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

static void json_str(std::string& s, const std::string& v) {
    s += '"';
    char buf[8];
    for (unsigned char c : v) {
        if (c == '"' || c == '\\') { s += '\\'; s += char(c); }
        else if (c < 0x20) { std::snprintf(buf, sizeof buf, "\\u%04x", c); s += buf; }
        else s += char(c);
    }
    s += '"';
}

bool Events::open(const std::string& path) {
    f = std::fopen(path.c_str(), "a");  // the studio truncates it before the run; a reader may already be tailing it
    t0 = mono();
    return f != nullptr;
}

double Events::elapsed() const { return mono() - t0; }

std::string Events::head(const char* e) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "{\"e\":\"%s\",\"t\":%.3f", e, elapsed());
    return buf;
}

void Events::line(std::string& s) {
    s += "}\n";
    std::lock_guard<std::mutex> lk(mu);
    std::fwrite(s.data(), 1, s.size(), f);
    std::fflush(f);
}

void Events::begin(const Board& B) {
    if (!f) return;
    shown_open.assign(B.nets.size(), -1);
    int conns = 0, nets = 0;
    for (auto& N : B.nets) {
        std::set<int> pins;  // pins - 1 per net, as Product Studio counts them from the DSN
        for (int k : N.items)
            if (B.items[k].kind == Kind::Pad) pins.insert(B.items[k].group >= 0 ? B.items[k].group : -2 - k);
        if (pins.size() >= 2) { conns += int(pins.size()) - 1; ++nets; }
    }
    std::string s = "{\"e\":\"begin\",\"v\":1,\"router\":\"pcbroute\",\"resolution_um\":";
    char buf[64];
    std::snprintf(buf, sizeof buf, "%g,\"connections\":%d,\"nets\":%d,\"layers\":[", B.resolution, conns, nets);
    s += buf;
    for (size_t l = 0; l < B.layers.size(); ++l) { if (l) s += ','; json_str(s, B.layers[l]); }
    s += ']';
    line(s);
}

void Events::pass(int n, const char* phase) {
    if (!f) return;
    passes = n;
    std::string s = head("pass");
    char buf[64];
    std::snprintf(buf, sizeof buf, ",\"n\":%d,\"phase\":\"%s\"", n, phase);
    s += buf;
    line(s);
}

void Events::progress(int pass, int routed, int unrouted, int conflicts) {
    if (!f) return;
    std::string s = head("progress");
    char buf[128];
    std::snprintf(buf, sizeof buf, ",\"pass\":%d,\"routed\":%d,\"unrouted\":%d,\"conflicts\":%d", pass, routed, unrouted, conflicts);
    s += buf;
    line(s);
}

void Events::net(const Board& B, int net, const std::vector<Seg>& segs, const std::vector<ViaOut>& vias, int open) {
    if (!f) return;
    const double k = B.resolution;
    auto I = [&](double v) { return (long long)std::llround(v * k); };
    std::string s = head("net");
    s += ",\"net\":";
    json_str(s, B.nets[net].name);
    s += ",\"wires\":[";
    char buf[160];
    for (size_t i = 0; i < segs.size(); ++i) {
        auto& g = segs[i];
        if (i) s += ',';
        s += '[';
        json_str(s, B.layers[g.layer]);
        std::snprintf(buf, sizeof buf, ",%lld,[%lld,%lld,%lld,%lld]]", I(g.width), I(g.a.x), I(g.a.y), I(g.b.x), I(g.b.y));
        s += buf;
    }
    s += "],\"vias\":[";
    for (size_t i = 0; i < vias.size(); ++i) {
        if (i) s += ',';
        s += '[';
        json_str(s, B.vias[vias[i].type].name);
        std::snprintf(buf, sizeof buf, ",%lld,%lld]", I(vias[i].at.x), I(vias[i].at.y));
        s += buf;
    }
    std::snprintf(buf, sizeof buf, "],\"open\":%d", open);
    s += buf;
    {
        std::lock_guard<std::mutex> lk(mu);
        if (size_t(net) < shown_open.size()) shown_open[net] = open;
    }
    line(s);
}

void Events::final_nets(const Board& B, const std::vector<NetResult>& R) {
    if (!f) return;
    std::vector<char> has(B.nets.size(), 0);
    for (auto& r : R) { has[r.net] = 1; net(B, r.net, r.segs, r.vias, r.open); }
    for (size_t n = 0; n < shown_open.size(); ++n)
        if (!has[n] && shown_open[n] >= 0) net(B, int(n), {}, {}, shown_open[n]);
}

void Events::end(int routed, int unrouted, bool ok, const std::string& error, int violations) {
    if (!f) return;
    std::string s = head("end");
    char buf[128];
    std::snprintf(buf, sizeof buf, ",\"routed\":%d,\"unrouted\":%d,\"ok\":%s", routed, unrouted, ok ? "true" : "false");
    s += buf;
    if (violations >= 0) { std::snprintf(buf, sizeof buf, ",\"violations\":%d", violations); s += buf; }
    if (!error.empty()) { s += ",\"error\":"; json_str(s, error); }
    line(s);
    std::fclose(f);
    f = nullptr;
}
