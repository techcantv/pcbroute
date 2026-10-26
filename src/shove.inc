    // ---- last-mile shove (--shove D), included inside struct Router (router.cpp) ----
    // For a connection still open at the end: route it into the others' clearance as little as it can (a soft
    // search: such a cell costs (1 + history)(1 + 1e4 x occupancy) instead of being a wall), and onto their copper
    // only at --shove-cross times that price (a crossing: the crossed net's local repair then hops to another layer
    // under it). That path stays put (frozen). Each net it now overlaps is pushed aside: cut where it is in the way (--shove-cut each
    // side) and rerouted within a small window around the cut (--shove-reach) with everything else as a hard
    // obstacle, which hugs the frozen path: the least displacement that clears it. A pushed net that can't get
    // around that way crosses the others as little as it can in the same window, is frozen in turn, and pushes
    // the nets it now overlaps, down to D levels. The attempt is kept only if the connection is closed, nothing
    // overlaps, no pushed net lost a connection, and the exact clearance check (the --eval rules) passes on the
    // copper it changed; else every net it touched is put back. Single-threaded, in job order: the result does not
    // depend on the thread count.
    struct ShoveCtx {
        bool soft = false;   // other nets' copper can be crossed, at a price (see route_job's cm_at / vm_at)
        float pres = 1e4f;
        bool has_win = false;  // search only these cells (no widening)
        int wi0 = 0, wj0 = 0, wi1 = 0, wj1 = 0;
        int repair = -1;     // cells cut out each side of a conflict (-1: route_job's own)
        std::unordered_set<uint32_t> cut;  // path states cut out as if in conflict (they put a frozen net in conflict)
        // the frozen nets' footprint (the pusher, and every net that pushed in turn), which a soft search may not
        // cross: per class, the NL track grids and then the via grid, over fw x fh cells from (fi0, fj0)
        int nx = 0, fi0 = 0, fj0 = 0, fw = 0, fh = 0, nl = 0;
        std::vector<uint8_t> fz;
        bool frozen(int k, int l, uint32_t c) const {
            if (fz.empty()) return false;
            int i = int(c % uint32_t(nx)) - fi0, j = int(c / uint32_t(nx)) - fj0;
            if (i < 0 || j < 0 || i >= fw || j >= fh) return false;
            return fz[((size_t(k) * (nl + 1) + l) * fh + j) * fw + i];
        }
        // where the searching net's track (via) centre would put its copper on another net's copper, over the window:
        // a soft search may squeeze into the others' clearance, never through their tracks (that is a crossing,
        // which no push can undo); NL track masks then the via mask
        std::vector<uint8_t> core;
        bool cored(int l, uint32_t c) const {
            if (core.empty()) return false;
            int i = int(c % uint32_t(nx)) - wi0, j = int(c / uint32_t(nx)) - wj0, w = wi1 - wi0 + 1, h = wj1 - wj0 + 1;
            if (i < 0 || j < 0 || i >= w || j >= h) return true;
            return core[(size_t(l) * h + j) * w + i];
        }
        float cross = 0;  // a soft step onto another net's copper costs this many times a step into its clearance (0: never)
        float over(int l, uint32_t c) const { return !cored(l, c) ? 1.f : cross > 0 ? cross : INF; }
    };
    const ShoveCtx* sx = nullptr;  // set only while the shove pass runs a search (single-threaded)
    ShoveCtx sctx;
    struct ShoveSnap { std::vector<std::vector<uint32_t>> paths; std::vector<uint32_t> keep; std::vector<Stub> stubs; int unrouted; };
    std::map<int, ShoveSnap> shove_snap;     // the nets the current attempt changed, as they were
    std::vector<char> shove_frz;             // ...and which of them are frozen
    std::vector<int> shove_win;              // ...the windows it searched (i0, j0, i1, j1 each), for the exact check
    int shove_steps = 0;                     // nets pushed in the current attempt (capped)
    long sh_pushed = 0, sh_hard = 0, sh_soft = 0, sh_fail_path = 0, sh_fail_push = 0, sh_fail_check = 0, sh_fail_open = 0;

    // A net's cells on another net's footprint: (kind, layer, cell), kind 1 a via (its cell on that layer's via grid)
    void shove_bad(const Job& X, std::vector<std::array<uint32_t, 3>>& out) const {
        const size_t N = G.n();
        const RClass& R = rcs[X.rc];
        for (auto& p : X.paths)
            for (size_t k = 0; k < p.size(); ++k) {
                int l = int(p[k] / N);
                uint32_t c = uint32_t(p[k] % N);
                if (R.oc[l][c] > 1) out.push_back({0, uint32_t(l), c});
                if (k + 1 < p.size() && p[k + 1] / N != p[k] / N)
                    for (int l2 = 0; l2 < NL; ++l2) if (R.ov[l2][c] > 1) out.push_back({1, uint32_t(l2), c});
            }
        for (auto& [s, pts, hw] : X.stubs) {
            int l = int(s / N);
            for (size_t k = 1; k < pts.size(); ++k) {
                double len = norm(pts[k] - pts[k - 1]);
                for (double t = 0; t <= len; t += G.s / 2) {
                    P q = pts[k - 1] + (pts[k] - pts[k - 1]) * (len > 0 ? t / len : 0);
                    int i = int((q.x - G.x0) / G.s), j = int((q.y - G.y0) / G.s);
                    if (i < 0 || j < 0 || i >= G.nx || j >= G.ny) continue;
                    uint32_t c = uint32_t(j) * G.nx + i;
                    if (R.oc[l][c] > 1) out.push_back({0, uint32_t(l), c});
                }
            }
        }
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    }
    // whether job K's footprint (its stamps on class rc's grids) covers any of those cells
    bool shove_covers(const Job& K, int rc, const std::vector<std::array<uint32_t, 3>>& bad) const {
        for (auto& [kind, l, c] : bad) {
            const auto& sp = K.stamps[(size_t(rc) * 2 + kind) * NL + l];
            int j = int(c / G.nx), i = int(c % G.nx);
            auto it = std::lower_bound(sp.begin(), sp.end(), j, [](const Span& s, int jj) { return s.j < jj; });
            for (; it != sp.end() && it->j == j; ++it)
                if (it->i0 <= i && i <= it->i1) return true;
        }
        return false;
    }
    // the nets in conflict with job x, in job order: those whose footprint covers x's copper, and those whose copper
    // x's footprint covers
    std::vector<int> shove_blockers(int x) const {
        const Job& X = jobs[size_t(x)];
        std::vector<std::array<uint32_t, 3>> bad, other;
        shove_bad(X, bad);
        std::vector<int> out;
        for (size_t k = 0; k < jobs.size(); ++k) {
            if (int(k) == x) continue;
            const Job& K = jobs[k];
            bool hit = !bad.empty() && shove_covers(K, X.rc, bad);
            if (!hit) {
                other.clear();
                shove_bad(K, other);
                hit = !other.empty() && shove_covers(X, K.rc, other);
            }
            if (hit) out.push_back(int(k));
        }
        return out;
    }
    void shove_save(int k) {
        if (shove_snap.count(k)) return;
        const Job& K = jobs[size_t(k)];
        shove_snap[k] = {K.paths, K.keep, K.stubs, K.unrouted};
    }
    // Frozen net k: per class, the cells where a track (via) centre would overlap k's footprint, or would put k's
    // copper on its own footprint (the grids are not symmetric: a track's footprint on the via grid is a capsule,
    // a via's on the track grid a disc, so a track may stay off a via's footprint and still make the via conflict)
    void shove_freeze(int k) {
        shove_frz[size_t(k)] = 1;
        const Job& F = jobs[size_t(k)];
        ShoveCtx& S = sctx;
        const size_t N = G.n();
        auto row = [&](size_t kc, int m, int j, int i0, int i1) {
            j -= S.fj0;
            if (j < 0 || j >= S.fh) return;
            int a = std::max(i0 - S.fi0, 0), b = std::min(i1 - S.fi0, S.fw - 1);
            if (a <= b) std::memset(&S.fz[((kc * (NL + 1) + m) * S.fh + j) * S.fw + a], 1, size_t(b - a + 1));
        };
        auto put = [&](size_t kc, int m, const Kern& K, int ci, int cj) { for (auto& [dj, hw] : K.rows) row(kc, m, cj + dj, ci - hw, ci + hw); };
        for (size_t kc = 0; kc < rcs.size(); ++kc) {
            for (int l = 0; l < NL; ++l)
                for (int kind = 0; kind < 2; ++kind)
                    for (auto& sp : F.stamps[(kc * 2 + kind) * NL + l]) row(kc, kind ? NL : l, sp.j, sp.i0, sp.i1);
            const Kern* kk = &kern[kc][size_t(F.rc) * 4];  // class kc's track / via on F's class grids
            for (auto& p : F.paths)
                for (size_t x = 0; x < p.size(); ++x) {
                    int l = int(p[x] / N);
                    uint32_t c = uint32_t(p[x] % N);
                    int i = int(c % G.nx), j = int(c / G.nx);
                    if (i < S.fi0 - 64 || i >= S.fi0 + S.fw + 64 || j < S.fj0 - 64 || j >= S.fj0 + S.fh + 64) continue;
                    put(kc, l, kk[0], i, j);
                    put(kc, NL, kk[2], i, j);
                    if (x + 1 < p.size() && p[x + 1] / N != p[x] / N) {
                        for (int l2 = 0; l2 < NL; ++l2) put(kc, l2, kk[1], i, j);
                        put(kc, NL, kk[3], i, j);
                    }
                }
            for (auto& [s0, pts, shw] : F.stubs)
                for (size_t x = 1; x < pts.size(); ++x) {
                    double len = norm(pts[x] - pts[x - 1]);
                    for (double t = 0; t <= len; t += G.s / 2) {
                        P q = pts[x - 1] + (pts[x] - pts[x - 1]) * (len > 0 ? t / len : 0);
                        int i = int((q.x - G.x0) / G.s), j = int((q.y - G.y0) / G.s);
                        put(kc, int(s0 / N), kk[0], i, j);
                        put(kc, NL, kk[2], i, j);
                    }
                }
        }
    }
    // the core mask of the window for job k's searches (see ShoveCtx::cored)
    void shove_core(int k) {
        ShoveCtx& S = sctx;
        const RClass& Rk = rcs[jobs[size_t(k)].rc];
        const int w = S.wi1 - S.wi0 + 1, h = S.wj1 - S.wj0 + 1;
        S.core.assign(size_t(NL + 1) * w * h, 0);
        const size_t N = G.n();
        auto put = [&](int m, const Kern& K, int ci, int cj) {
            for (auto& [dj, hw] : K.rows) {
                int j = cj + dj - S.wj0;
                if (j < 0 || j >= h) continue;
                int a = std::max(0, ci - hw - S.wi0), b = std::min(w - 1, ci + hw - S.wi0);
                if (a <= b) std::memset(&S.core[(size_t(m) * h + j) * w + a], 1, size_t(b - a + 1));
            }
        };
        std::vector<Span> sp;
        for (size_t q = 0; q < jobs.size(); ++q) {
            if (int(q) == k) continue;
            const Job& K = jobs[q];
            const RClass& Rc = rcs[K.rc];
            const Kern kt = make_kern(Rk.hw + Rc.hw, true), ktv = make_kern(Rk.vr + Rc.hw, true);
            const Kern kvt = make_kern(Rk.hw + Rc.vr, false), kvv = make_kern(Rk.vr + Rc.vr, false);
            const int M = int(std::ceil((std::max(Rk.hw, Rk.vr) + std::max(Rc.hw, Rc.vr)) / G.s)) + 2;
            for (auto& p : K.paths)
                for (size_t x = 0; x < p.size(); ++x) {
                    int l = int(p[x] / N);
                    uint32_t c = uint32_t(p[x] % N);
                    int i = int(c % G.nx), j = int(c / G.nx);
                    if (i < S.wi0 - M || i > S.wi1 + M || j < S.wj0 - M || j > S.wj1 + M) continue;
                    put(l, kt, i, j);
                    put(NL, ktv, i, j);
                    if (x + 1 < p.size() && p[x + 1] / N != p[x] / N) {
                        for (int l2 = 0; l2 < NL; ++l2) put(l2, kvt, i, j);
                        put(NL, kvv, i, j);
                    }
                }
            for (auto& [s0, pts, shw] : K.stubs)
                for (size_t x = 1; x < pts.size(); ++x)
                    for (int m : {int(s0 / N), NL}) {
                        sp.clear();
                        capsule_spans(G, pts[x - 1], pts[x], shw + (m == NL ? Rk.vr : Rk.hw), sp);
                        for (auto& r : sp) {
                            int j = r.j - S.wj0;
                            if (j < 0 || j >= h) continue;
                            int a = std::max(0, r.i0 - S.wi0), b = std::min(w - 1, r.i1 - S.wi0);
                            if (a <= b) std::memset(&S.core[(size_t(m) * h + j) * w + a], 1, size_t(b - a + 1));
                        }
                    }
        }
    }
    // route job k with the shove context: a local window (or the job's own), soft or hard
    void shove_route(int k, Work& W, bool soft) {
        sctx.soft = soft;
        if (soft) shove_core(k);
        else sctx.core.clear();
        sx = &sctx;
        std::fill(W.touch.begin(), W.touch.end(), 0);
        std::fill(W.touchv.begin(), W.touchv.end(), 0);
        route_job(jobs[size_t(k)], W, 0, true);
        sx = nullptr;
        sctx.cut.clear();
        if (sctx.has_win) shove_win.insert(shove_win.end(), {sctx.wi0, sctx.wj0, sctx.wi1, sctx.wj1});
    }
    // job k's path states whose footprint covers a frozen net's copper (the grids are not symmetric: k's own cells
    // there need not be in conflict)
    std::unordered_set<uint32_t> shove_harm(int k) const {
        std::unordered_set<uint32_t> out;
        const Job& K = jobs[size_t(k)];
        const size_t N = G.n();
        auto in = [](const Kern& kk, int di, int dj) {
            for (auto& [r, w] : kk.rows) if (r == dj) return std::abs(di) <= w;
            return false;
        };
        std::vector<std::array<uint32_t, 3>> bad;
        for (size_t f = 0; f < jobs.size(); ++f) {
            if (!shove_frz[f] || int(f) == k) continue;
            bad.clear();
            shove_bad(jobs[f], bad);
            if (bad.empty()) continue;
            const Kern* kk = &kern[size_t(K.rc)][size_t(jobs[f].rc) * 4];
            for (auto& p : K.paths)
                for (size_t x = 0; x < p.size(); ++x) {
                    int l = int(p[x] / N);
                    uint32_t c = uint32_t(p[x] % N);
                    int i = int(c % G.nx), j = int(c / G.nx);
                    bool via = x + 1 < p.size() && p[x + 1] / N != p[x] / N;
                    for (auto& [kind, bl, bc] : bad) {
                        int di = int(bc % G.nx) - i, dj = int(bc / G.nx) - j;
                        if (std::abs(di) > 64 || std::abs(dj) > 64) continue;
                        bool hit = kind == 0 ? (int(bl) == l && in(kk[0], di, dj)) || (via && in(kk[2], di, dj))
                                             : in(kk[1], di, dj) || (via && in(kk[3], di, dj));
                        if (hit) { out.insert(p[x]); if (via) out.insert(p[x + 1]); break; }
                    }
                }
        }
        return out;
    }
    bool why(int k, int depth, const char* what) const {
        if (o.verbose > 1) std::fprintf(stderr, "      push %s (level %d) failed: %s\n", B.nets[jobs[size_t(k)].net].name.c_str(), depth, what);
        return false;
    }
    // Push net k out of the way of the frozen copper: reroute its stretches in conflict locally, around everything
    // (hard); failing that, across the others (soft), freezing it and pushing those in turn. True if k is legal and
    // has no more open connections than before the attempt.
    bool shove_net(int k, int depth, Work& W) {
        if (++shove_steps > 64) return why(k, depth, "too many pushes");
        Job& K = jobs[size_t(k)];
        shove_save(k);
        ++sh_pushed;
        const int was = shove_snap[k].unrouted;
        const size_t N = G.n();
        const int cutc = std::max(2, int(std::lround(o.shove_cut / G.s))), reach = std::max(2, int(std::lround(o.shove_reach / G.s)));
        // the window: the stretches route_job will cut out (the conflict plus the cut each side), plus the reach
        int i0 = INT_MAX, j0 = INT_MAX, i1 = -1, j1 = -1;
        auto add = [&](uint32_t c) { int i = int(c % G.nx), j = int(c / G.nx); i0 = std::min(i0, i); i1 = std::max(i1, i); j0 = std::min(j0, j); j1 = std::max(j1, j); };
        const RClass& R = rcs[K.rc];
        std::unordered_set<uint32_t> harm = shove_harm(k);
        for (auto& p : K.paths) {
            int f = -1, b = -1;
            for (size_t q = 0; q < p.size(); ++q) {
                int l = int(p[q] / N);
                uint32_t c = uint32_t(p[q] % N);
                bool bad = R.oc[l][c] > 1 || harm.count(p[q]);
                if (q + 1 < p.size() && p[q + 1] / N != p[q] / N)
                    for (int l2 = 0; l2 < NL; ++l2) bad |= R.ov[l2][c] > 1;
                if (bad) { if (f < 0) f = int(q); b = int(q); }
            }
            if (f < 0) continue;
            for (int q = std::max(0, f - cutc - 1); q <= std::min(int(p.size()) - 1, b + cutc + 1); ++q) add(uint32_t(p[size_t(q)] % N));
        }
        {
            std::vector<std::array<uint32_t, 3>> bad;
            shove_bad(K, bad);
            if (bad.empty() && harm.empty()) return K.unrouted <= was;  // nothing in the way (any more)
            for (auto& b : bad) add(b[2]);
        }
        sctx.has_win = true;
        sctx.wi0 = std::max(0, i0 - reach); sctx.wj0 = std::max(0, j0 - reach);
        sctx.wi1 = std::min(G.nx - 1, i1 + reach); sctx.wj1 = std::min(G.ny - 1, j1 + reach);
        sctx.repair = cutc;
        sctx.cut = harm;
        const int wi0 = sctx.wi0, wj0 = sctx.wj0, wi1 = sctx.wi1, wj1 = sctx.wj1;
        const ShoveSnap entry{K.paths, K.keep, K.stubs, K.unrouted};
        shove_route(k, W, false);
        if (o.verbose > 1)
            std::fprintf(stderr, "      push %s (level %d): window %.1fx%.1f mm at (%.2f, %.2f), around: %d open (was %d)\n", B.nets[K.net].name.c_str(), depth,
                         (wi1 - wi0) * G.s / 1000, (wj1 - wj0) * G.s / 1000, G.center(uint32_t(wj0 + wj1) / 2 * G.nx + uint32_t(wi0 + wi1) / 2).x / 1000,
                         -G.center(uint32_t(wj0 + wj1) / 2 * G.nx + uint32_t(wi0 + wi1) / 2).y / 1000, K.unrouted, was);
        if (K.unrouted <= was) {
            restamp(K);
            ++sh_hard;
            // around everything by its own grids, but the grids are not symmetric: a net it now makes conflict moves too
            if (depth < o.shove)
                for (int m : shove_blockers(k)) {
                    if (shove_frz[size_t(m)]) return why(m, depth + 1, "frozen");
                    if (!shove_net(m, depth + 1, W)) return false;
                }
            return count_conflicts(K, false, 0) == 0 || why(k, depth, "still overlaps");
        }
        if (depth >= o.shove) return why(k, depth, "no level left");
        // around everything did not work: across the others, then they move (from the paths as they were: the failed
        // repair trimmed its cut pieces back to the pads)
        K.paths = entry.paths; K.keep = entry.keep; K.stubs = entry.stubs; K.unrouted = entry.unrouted;
        sctx.repair = cutc;
        sctx.cut = harm;
        sctx.has_win = true;
        sctx.wi0 = wi0; sctx.wj0 = wj0; sctx.wi1 = wi1; sctx.wj1 = wj1;
        shove_route(k, W, true);
        if (o.verbose > 1) std::fprintf(stderr, "      push %s (level %d): across: %d open\n", B.nets[K.net].name.c_str(), depth, K.unrouted);
        if (K.unrouted > was) return why(k, depth, "no way across");
        restamp(K);
        ++sh_soft;
        shove_freeze(k);
        for (int m : shove_blockers(k)) {
            if (shove_frz[size_t(m)]) return why(m, depth + 1, "frozen");
            if (!shove_net(m, depth + 1, W)) return false;
        }
        return count_conflicts(K, false, 0) == 0 || why(k, depth, "still overlaps");
    }
    // One attempt at net j's open connections. Returns the nets pushed if kept, -1 if everything was put back.
    // the frozen box (see ShoveCtx): cells i0..i1, j0..j1 grown by what the pushes can reach
    void shove_fbox(int i0, int j0, int i1, int j1) {
        int m = int((o.shove_cut + o.shove_reach) / G.s) * (o.shove + 1) + 4;
        sctx.fi0 = std::max(0, i0 - m); sctx.fj0 = std::max(0, j0 - m);
        sctx.fw = std::min(G.nx - 1, i1 + m) - sctx.fi0 + 1; sctx.fh = std::min(G.ny - 1, j1 + m) - sctx.fj0 + 1;
        sctx.fz.assign(rcs.size() * (NL + 1) * size_t(sctx.fw) * sctx.fh, 0);
    }
    // Net j's open connections: a few tries, each one's path kept off the nets that could not be pushed in the
    // tries before. Returns the nets pushed if kept, -1 if everything was put back.
    int shove_attempt(int j, Work& W) {
        std::vector<char> nopush(jobs.size(), 0);
        for (int t = 0; t < 4; ++t) {
            int failed = -1;
            int r = shove_try(j, W, nopush, failed);
            if (r >= 0) return r;
            if (failed < 0 || nopush[size_t(failed)]) break;
            nopush[size_t(failed)] = 1;
            if (o.verbose > 1) std::fprintf(stderr, "    shove %s: again, keeping off %s\n", B.nets[jobs[size_t(j)].net].name.c_str(), B.nets[jobs[size_t(failed)].net].name.c_str());
        }
        return -1;
    }
    int shove_try(int j, Work& W, const std::vector<char>& nopush, int& failed) {
        Job& J = jobs[size_t(j)];
        shove_snap.clear();
        shove_win.clear();
        shove_frz.assign(jobs.size(), 0);
        shove_steps = 0;
        sctx = ShoveCtx{};
        sctx.nx = G.nx;
        sctx.nl = NL;
        sctx.cross = float(o.shove_cross);
        shove_save(j);
        const int open0 = J.unrouted;
        auto give_up = [&](long& why) {
            ++why;
            sx = nullptr;
            if (getenv("SHOVE_KEEP")) return -1;  // debugging: leave the failed attempt's copper as it is
            for (auto& [k, s] : shove_snap) {
                Job& K = jobs[size_t(k)];
                K.paths = s.paths; K.keep = s.keep; K.stubs = s.stubs; K.unrouted = s.unrouted;
                restamp(K);
            }
            return -1;
        };
        // the open connection, squeezing into the others' clearance as little as it can, in the net's usual window
        {
            Box win = J.box.grown(o.window);
            sctx.has_win = true;
            sctx.wi0 = std::max(0, int((win.x0 - G.x0) / G.s)); sctx.wj0 = std::max(0, int((win.y0 - G.y0) / G.s));
            sctx.wi1 = std::min(G.nx - 1, int((win.x1 - G.x0) / G.s)); sctx.wj1 = std::min(G.ny - 1, int((win.y1 - G.y0) / G.s));
        }
        const bool keepoff = std::find(nopush.begin(), nopush.end(), 1) != nopush.end();
        if (keepoff) {  // the nets that could not be pushed are frozen from the start
            shove_fbox(sctx.wi0, sctx.wj0, sctx.wi1, sctx.wj1);
            for (size_t k = 0; k < jobs.size(); ++k) if (nopush[k]) shove_freeze(int(k));
        }
        shove_route(j, W, true);
        if (J.unrouted >= open0) {
            if (o.verbose > 1) std::fprintf(stderr, "    shove %s: no path even through the others' clearance\n", B.nets[J.net].name.c_str());
            return give_up(sh_fail_path);
        }
        restamp(J);
        if (o.verbose > 1) {
            std::vector<int> bl = shove_blockers(j);
            std::fprintf(stderr, "    shove %s: path across %d cells of others, in the way:", B.nets[J.net].name.c_str(), count_conflicts(J, false, 0));
            for (int k : bl) std::fprintf(stderr, " %s", B.nets[jobs[size_t(k)].net].name.c_str());
            std::fprintf(stderr, "\n");
        }
        // the frozen box: where the pushes can reach from what the path overlaps
        std::vector<std::array<uint32_t, 3>> bad;
        shove_bad(J, bad);
        if (!bad.empty()) {
            if (!keepoff) {
                int i0 = INT_MAX, j0 = INT_MAX, i1 = -1, j1 = -1;
                for (auto& b : bad) { int i = int(b[2] % G.nx), jj = int(b[2] / G.nx); i0 = std::min(i0, i); i1 = std::max(i1, i); j0 = std::min(j0, jj); j1 = std::max(j1, jj); }
                shove_fbox(i0, j0, i1, j1);
            }
            shove_freeze(j);
            for (int k : shove_blockers(j))
                if (shove_frz[size_t(k)] || !shove_net(k, 1, W)) { failed = k; return give_up(sh_fail_push); }
        }
        // kept only if the connection is closed, nothing overlaps, no pushed net lost a connection...
        for (auto& [k, s] : shove_snap) {
            const Job& K = jobs[size_t(k)];
            if (count_conflicts(K, false, 0) > 0) {
                if (o.verbose > 1) {
                    std::vector<std::array<uint32_t, 3>> bad;
                    shove_bad(K, bad);
                    std::fprintf(stderr, "      %s still overlaps %d cells (e.g. %s at (%.2f, %.2f), layer %u), in the way:", B.nets[K.net].name.c_str(), count_conflicts(K, false, 0),
                                 bad.empty() ? "-" : bad[0][0] ? "via" : "track", bad.empty() ? 0 : G.center(bad[0][2]).x / 1000, bad.empty() ? 0 : -G.center(bad[0][2]).y / 1000, bad.empty() ? 0 : bad[0][1]);
                    for (int m : shove_blockers(k)) std::fprintf(stderr, " %s%s", B.nets[jobs[size_t(m)].net].name.c_str(), shove_frz[size_t(m)] ? "(frozen)" : "");
                    std::fprintf(stderr, "\n");
                }
                return give_up(sh_fail_push);
            }
            if (k != j && K.unrouted > s.unrouted) return give_up(sh_fail_open);
        }
        if (J.unrouted >= open0) return give_up(sh_fail_open);
        // ...and the copper of the nets it changed passes the exact check against everything near the windows
        {
            Box reg;
            for (size_t q = 0; q + 3 < shove_win.size(); q += 4) {
                reg.add(G.center(uint32_t(shove_win[q + 1]) * G.nx + shove_win[q]));
                reg.add(G.center(uint32_t(shove_win[q + 3]) * G.nx + shove_win[q + 2]));
            }
            for (auto& b : bad) reg.add(G.center(b[2]));
            reg = reg.grown(1000 + maxclr);
            const size_t N = G.n();
            std::vector<NetResult> res;
            for (size_t k = 0; k < jobs.size(); ++k) {
                const Job& K = jobs[k];
                bool take = shove_snap.count(int(k)) > 0;
                for (size_t p = 0; p < K.paths.size() && !take; ++p)
                    for (uint32_t s : K.paths[p]) {
                        P c = G.center(uint32_t(s % N));
                        if (c.x >= reg.x0 && c.x <= reg.x1 && c.y >= reg.y0 && c.y <= reg.y1) { take = true; break; }
                    }
                for (auto& st : K.stubs) if (!take) for (P c : st.pts) if (c.x >= reg.x0 && c.x <= reg.x1 && c.y >= reg.y0 && c.y <= reg.y1) take = true;
                if (!take) continue;
                if (count_conflicts(K, false, 0) > 0) {
                    if (o.verbose > 1) std::fprintf(stderr, "      %s, near the pushes, overlaps\n", B.nets[K.net].name.c_str());
                    return give_up(sh_fail_push);
                }
                NetResult r;
                r.net = K.net;
                std::vector<P> vias;
                geometry(K, r.segs, vias);
                for (P v : vias) r.vias.push_back({v, rcs[K.rc].via});
                res.push_back(std::move(r));
            }
            if (check_routes(B, o, res, o.verbose > 2) > 0) return give_up(sh_fail_check);
        }
        if (o.verbose > 1)
            std::fprintf(stderr, "    shove %s: %d more connected, %zu nets pushed aside\n", B.nets[J.net].name.c_str(), open0 - J.unrouted, shove_snap.size() - 1);
        return int(shove_snap.size()) - 1;
    }
    void shove_pass(Work& W) {
        double t0 = now();
        auto open_total = [&] { int n = 0; for (auto& J : jobs) n += J.unrouted - J.dead; return n; };
        const int before = open_total();
        if (before == 0) return;
        if (W.touch.size() != BN * NL) { W.touch.assign(BN * NL, 0); W.touchv.assign(BN, 0); }
        g_events.pass(++ev_pass, "ripup");
        int tried = 0, kept = 0, moved = 0;
        for (int round = 0; round < o.shove_rounds; ++round) {
            bool better = false;
            for (size_t j = 0; j < jobs.size(); ++j) {
                if (jobs[j].unrouted <= jobs[j].dead) continue;
                ++tried;
                int n = shove_attempt(int(j), W);
                if (n >= 0) { ++kept; moved += n; better = true; ev_sweep(false); }
            }
            if (!better) break;
        }
        sx = nullptr;
        shove_snap.clear();
        if (o.verbose)
            std::fprintf(stderr, "  shove: %d of %d attempts kept, open %d -> %d, %d nets moved (%ld pushes: %ld around, %ld across; failed: %ld no path, %ld push, %ld open, %ld check), %.2f s\n",
                         kept, tried, before, open_total(), moved, sh_pushed, sh_hard, sh_soft, sh_fail_path, sh_fail_push, sh_fail_open, sh_fail_check, now() - t0);
    }
