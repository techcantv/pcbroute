// Router events (--events FILE): protocol v1 of Product Studio's live routing view (its docs/ROUTE_EVENTS.md).
// JSON lines, one flushed line per event, written from any thread. Off unless --events is given: every call is
// then a no-op, so the router costs nothing extra.
#pragma once
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include "dsn.h"
#include "router.h"

struct Events {
    bool open(const std::string& path);
    bool on() const { return f != nullptr; }
    void begin(const Board& B);
    void pass(int n, const char* phase);
    void progress(int pass, int routed, int unrouted, int conflicts);
    // all the copper of net `net` now (session units, as write_ses prints them), `open` connections still missing
    void net(const Board& B, int net, const std::vector<Seg>& segs, const std::vector<ViaOut>& vias, int open);
    // the final copper of every net (after smoothing), and an empty one for each net shown earlier that has none
    void final_nets(const Board& B, const std::vector<NetResult>& R);
    void end(int routed, int unrouted, bool ok, const std::string& error = "", int violations = -1);
    double elapsed() const;
    int passes = 0;  // the last pass number written

private:
    FILE* f = nullptr;
    std::mutex mu;
    double t0 = 0;
    std::vector<int> shown_open;  // per net: open connections last shown, -1 if never shown
    void line(std::string& s);    // appends "}\n", writes, flushes (under mu)
    std::string head(const char* e);
};

extern Events g_events;
