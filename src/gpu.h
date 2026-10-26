// Optional CUDA back end: an exact shortest-path solver for one search window, used when a CPU A* search has run
// past its pop threshold (build with `make GPU=1`). It reads what the CPU search reads: the class's cost maps
// (RClass::cst / vcst), the net's own footprint, the target bits and the learned prior, so it can finish any search
// in any round, with or without the prior.
#pragma once
#include <cstdint>
#include <utility>
#include <vector>

struct GpuQuery {
    int nx, ny, NL;                // the full grid (the host arrays below are laid out layer by layer: [l][j * nx + i])
    int wi0, wj0, ww, wh;          // search window
    uint16_t me;                   // the net, as stored in the static grids (0: free, 0xFFFF: blocked)
    float via_cost, step;          // via_cost < 0: no vias
    float dirf[8][8];              // per layer, cost factor of each of the 8 moves
    uint8_t lay[8];                // layers a track may use
    const float* cst[8];           // per layer, the cost map: INF blocked, negative = some net's own copper (st says whose)
    const uint16_t* st[8];         // static obstacles for a track centre, per layer
    const float* vcst;             // the same for a via centre (summed over the layers)
    const uint16_t* vst;
    const uint64_t* tbit;          // target bits over the window states (layer * ww * wh + j * ww + i)
    std::vector<std::pair<uint32_t, float>> own_cf;   // window state -> cost factor of the net's own cells (its stamp left out; INF if it can't go there)
    std::vector<std::pair<uint32_t, float>> own_vcf;  // window cell -> via factor where the net's own vias are
    // learned prior of the net (null: none): its map (pnl layers of ph rows of pw bytes, pixel (pi0, pj0) first),
    // the fine column / row -> prior pixel tables of the full grid, the router layer -> prior layer table, the cost
    // factor per prior value, and the corridor threshold (0: no corridor)
    const uint8_t* prior = nullptr;
    int pi0 = 0, pj0 = 0, pw = 0, ph = 0, pnl = 0;
    const int* pcol = nullptr;
    const int* prow = nullptr;
    int play[8];
    float mlf[256];
    uint8_t mthr = 0;
    bool corridor = false;
    std::vector<std::pair<uint32_t, float>> starts;  // window state, initial cost
};

// The solution: costs for the part of the window that was solved (cells outside it read as unreachable).
struct GpuOut {
    int rx0 = 0, ry0 = 0, rw = 0, rh = 0, NL = 0, ww = 0, wh = 0;
    int64_t best = -1;                   // the cheapest target reached, as a window state (-1: none)
    float* d = nullptr;                  // pinned host memory from a pool, given back by the destructor
    size_t cap = 0;
    GpuOut() = default;
    GpuOut(const GpuOut&) = delete;
    GpuOut& operator=(const GpuOut&) = delete;
    ~GpuOut();
    void reserve(size_t n);
    float operator()(int64_t q) const;   // cost of a window state
};

bool gpu_available();
// How many searches the GPU solves at once (each with its own buffers), and the most states one may take.
void gpu_configure(int slots, size_t max_states);
// Returns the sweeps it took, -1 on failure (or a window too big for the GPU), -3 if every GPU slot is busy and
// wait is false (with wait, the call blocks for a slot: the result then doesn't depend on timing).
int gpu_solve(const GpuQuery& q, GpuOut& out, bool wait = true);
