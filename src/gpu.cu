#include "gpu.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>
#include <cuda_runtime.h>

#define INF 1e30f
#define BLOCK 0xFFFF

namespace {

// One GPU slot: a stream and its buffers. A CPU thread takes a slot for one solve; several solves run at once on
// their own streams.
template <class T> struct Buf {  // a device array grown on demand
    T* p = nullptr;
    size_t cap = 0;
    bool need(size_t n) {
        if (cap >= n) return p != nullptr;
        cudaFree(p);
        p = nullptr;
        cap = n + n / 8;
        if (cudaMalloc(&p, cap * sizeof(T)) != cudaSuccess) { p = nullptr; cap = 0; }
        return p != nullptr;
    }
};

// One GPU slot: a stream and its buffers. A CPU thread takes a slot for one solve; several solves run at once on
// their own streams.
struct Dev {
    cudaStream_t stream = nullptr;
    Buf<float> d, cf, vcf, mlf, lval;
    Buf<uint16_t> st, vst;
    Buf<uint8_t> vok, prior;
    Buf<uint64_t> tbit;
    Buf<int> pcol, prow;
    Buf<uint32_t> lst;
    Buf<float2> sum;
    int* flag = nullptr;
    unsigned long long* ub = nullptr;
    bool busy = false;
};
int SLOTS = 4;
std::vector<Dev> DS;
std::mutex slot_mu;
std::condition_variable slot_cv;
bool g_ok = false;
size_t MAXN = size_t(48) << 20;  // states a solve may take on the GPU (PCB_GPU_MAX_STATES)

struct KP {
    int rw, rh, NL;          // the rectangle solved
    int rx0, ry0, ww, wh;    // its place in the window (the target bits are over the window)
    uint16_t me;
    float via_cost, step;
    float dirf[8][8];
    uint8_t lay[8];
    int play[8];
    int pi0, pj0, pw, ph, pnl;
    int has_prior, corridor;
    uint8_t mthr;
};

__device__ inline float ubof(const unsigned long long* ub) { return __uint_as_float(unsigned(*ub >> 32)); }
__device__ inline bool tgbit(const KP& kp, const uint64_t* tbit, int l, int i, int j) {
    size_t wq = (size_t(l) * kp.wh + (j + kp.ry0)) * kp.ww + (i + kp.rx0);
    return (tbit[wq >> 6] >> (wq & 63)) & 1;
}

// The cost factor of every state of the rectangle, as the CPU search sees it: the cost map, the own copper
// resolved (whose it is), the learned prior's factor and corridor; and where a via may land on each layer.
__global__ void prep_cf(const KP kp, float* cf, const uint16_t* st, uint8_t* vok, const uint64_t* tbit, const uint8_t* prior,
                        const int* pcol, const int* prow, const float* mlf) {
    size_t RN = size_t(kp.rw) * kp.rh;
    size_t idx = size_t(blockIdx.x) * blockDim.x + threadIdx.x;
    if (idx >= RN * kp.NL) return;
    int l = int(idx / RN);
    size_t c = idx % RN;
    int j = int(c / kp.rw), i = int(c % kp.rw);
    bool tg = tgbit(kp, tbit, l, i, j);
    uint16_t s = st[idx];
    bool sok = s == 0 || s == kp.me;
    float v = cf[idx], r;
    if (!kp.lay[l] || !(v < INF)) r = INF;
    else if (v < 0) r = s == kp.me ? -v : INF;
    else r = v;
    int pv = 255;
    if (kp.has_prior && kp.lay[l]) {
        int pl = kp.play[l];
        if (pl >= 0) {
            int x = pcol[i] - kp.pi0, y = prow[j] - kp.pj0;
            pv = (x < 0 || y < 0 || x >= kp.pw || y >= kp.ph) ? 0 : prior[(size_t(pl) * kp.ph + y) * kp.pw + x];
        }
        if (kp.corridor && pv < kp.mthr && !tg) r = INF;
        else if (r < INF) r *= mlf[pv];
    }
    cf[idx] = r;
    vok[idx] = kp.lay[l] ? (sok && (!kp.corridor || tg || pv >= kp.mthr)) : tg;
}

__global__ void prep_v(const KP kp, float* vcf, const uint16_t* vst) {
    size_t RN = size_t(kp.rw) * kp.rh;
    size_t idx = size_t(blockIdx.x) * blockDim.x + threadIdx.x;
    if (idx >= RN) return;
    float v = vcf[idx];
    uint16_t s = vst[idx];
    float r = !(v < INF) ? INF : v < 0 ? (s == kp.me ? -v : INF) : v;
    vcf[idx] = r < INF ? r * kp.via_cost : INF;
}

__global__ void setv(float* a, const uint32_t* idx, const float* val, int n) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k < n) a[idx[k]] = val[k];
}

// A scan along a line is d'[i] = min(d[i], d'[i-1] + c[i]), a min-plus recurrence. Cell i is the function
// f_i(x) = min(x + c_i, d_i) (a blocked cell ignores x: c_i = INF), and those compose into (C, D) pairs:
// f(x) = min(x + C, D). So each line is cut into chunks of CH cells that are summarised in parallel, chained
// (a short serial walk over the chunks) and then applied in parallel, which keeps the GPU busy where one
// thread per line left it waiting on its own memory loads.
constexpr int CH = 32;

__device__ inline bool line_of(int ww, int wh, int dx, int dy, int t, int& i, int& j, int& n) {
    if (dx != 0 && t < wh) { i = dx > 0 ? 0 : ww - 1; j = t; }
    else {
        int u = dx != 0 ? t - wh : t;
        if (dy == 0 || u < 0 || u >= ww) return false;
        i = u;
        j = dy > 0 ? 0 : wh - 1;
        if (dx != 0 && (i == (dx > 0 ? 0 : ww - 1))) return false;  // that corner is a column start already
    }
    n = 1 << 30;
    if (dx) n = min(n, dx > 0 ? ww - i : i + 1);
    if (dy) n = min(n, dy > 0 ? wh - j : j + 1);
    return true;
}

// where chunk ck of line t starts; false if the chunk lies beyond the line
__device__ inline bool chunk_of(const KP& kp, int dx, int dy, int t, int ck, int& i, int& j, int& m) {
    int n;
    if (!line_of(kp.rw, kp.rh, dx, dy, t, i, j, n)) return false;
    int s = ck * CH;
    if (s >= n) return false;
    i += dx * s; j += dy * s; m = min(CH, n - s);
    return true;
}

// cost of a move into a state whose factor is c; < 0: not allowed
__device__ inline float into(float c, float fac) { return c < INF ? c * fac : -1.f; }

// Horizontal scans: a warp takes 32 rows x one chunk, loads that tile coalesced into shared memory, and each lane then
// walks its own row there (a thread per row reading global memory would touch a new cache line at every row).
struct Tile { float d[32][33]; float c[32][33]; };

template <bool APPLY>
__global__ void hscan(const KP kp, float* d, const float* cf, int dx, int dm, int nch, float2* sum, int* flag, const unsigned long long* ubp) {
    int l = blockIdx.y;
    if (!kp.lay[l]) return;
    const float ub = ubof(ubp);
    __shared__ Tile T[4];
    int w = threadIdx.x >> 5, lane = threadIdx.x & 31;
    Tile& t = T[w];
    const int ww = kp.rw, wh = kp.rh, nlg = (wh + 31) / 32;
    int wid = blockIdx.x * 4 + w, lg = wid % nlg, ck = wid / nlg;
    if (ck >= nch) return;  // uniform over the warp
    int s = ck * CH, m = min(CH, ww - s);
    if (m <= 0) return;
    size_t base = size_t(l) * ww * wh;
    int col = dx > 0 ? s + lane : ww - 1 - s - lane;
    bool cv = lane < m;
    #pragma unroll 8
    for (int r = 0; r < 32; ++r) {
        int row = lg * 32 + r;
        if (row < wh && cv) {
            size_t idx = base + size_t(row) * ww + col;
            t.d[r][lane] = d[idx]; t.c[r][lane] = cf[idx];
        }
    }
    __syncwarp();
    int row = lg * 32 + lane;
    if (row < wh) {
        const float fac = kp.step * kp.dirf[l][dm];
        size_t si = (size_t(l) * wh + row) * nch + ck;
        if (!APPLY) {
            float C = 0.f, D = INF;
            for (int k = 0; k < m; ++k) {
                float c = into(t.c[lane][k], fac), di = t.d[lane][k];
                if (c < 0) { C = INF; D = di; }
                else { C = fminf(C + c, INF); D = fminf(fminf(D + c, INF), di); }
            }
            sum[si] = make_float2(C, D);
        } else {
            float prev = sum[si].x;
            bool changed = false;
            for (int k = 0; k < m; ++k) {
                float cur = t.d[lane][k];
                if (prev < ub) {
                    float c = into(t.c[lane][k], fac);
                    if (c >= 0 && prev + c < cur && prev + c < ub) { cur = prev + c; t.d[lane][k] = cur; changed = true; }
                }
                prev = cur;
            }
            if (changed) *flag = 1;
        }
    }
    if (APPLY) {
        __syncwarp();
        for (int r = 0; r < 32; ++r) {
            int rw = lg * 32 + r;
            if (rw < wh && cv) d[base + size_t(rw) * ww + col] = t.d[r][lane];
        }
    }
}

__global__ void scan_sum(const KP kp, const float* d, const float* cf, int dx, int dy, int dm, int lines, int nch, float2* sum) {
    int l = blockIdx.y;
    if (!kp.lay[l]) return;
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= lines * nch) return;
    int t = tid % lines, ck = tid / lines, i, j, m;
    if (!chunk_of(kp, dx, dy, t, ck, i, j, m)) return;
    size_t base = size_t(l) * kp.rw * kp.rh;
    const float fac = ((dx != 0 && dy != 0) ? kp.step * 1.41421356f : kp.step) * kp.dirf[l][dm];
    float C = 0.f, D = INF;
    for (int k = 0; k < m; ++k, i += dx, j += dy) {
        size_t idx = base + size_t(j) * kp.rw + i;
        float c = into(cf[idx], fac), di = d[idx];
        if (c < 0) { C = INF; D = di; }
        else { C = fminf(C + c, INF); D = fminf(fminf(D + c, INF), di); }
    }
    sum[(size_t(l) * lines + t) * nch + ck] = make_float2(C, D);
}

// turns the chunk summaries of each line into the value entering each chunk (stored in .x)
__global__ void scan_carry(const KP kp, int dx, int dy, int lines, int nch, float2* sum) {
    int l = blockIdx.y;
    if (!kp.lay[l]) return;
    int t = blockIdx.x * blockDim.x + threadIdx.x;
    int i, j, n;
    if (t >= lines || !line_of(kp.rw, kp.rh, dx, dy, t, i, j, n)) return;
    float x = INF;
    float2* p = sum + (size_t(l) * lines + t) * nch;
    for (int ck = 0; ck * CH < n; ++ck) {
        float2 s = p[ck];
        p[ck].x = x;
        x = fminf(fminf(x + s.x, INF), s.y);
    }
}

__global__ void scan_apply(const KP kp, float* d, const float* cf, int dx, int dy, int dm, int lines, int nch, const float2* sum, int* flag, const unsigned long long* ubp) {
    int l = blockIdx.y;
    if (!kp.lay[l]) return;
    const float ub = ubof(ubp);
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= lines * nch) return;
    int t = tid % lines, ck = tid / lines, i, j, m;
    if (!chunk_of(kp, dx, dy, t, ck, i, j, m)) return;
    size_t base = size_t(l) * kp.rw * kp.rh;
    const float fac = ((dx != 0 && dy != 0) ? kp.step * 1.41421356f : kp.step) * kp.dirf[l][dm];
    float prev = sum[(size_t(l) * lines + t) * nch + ck].x;
    bool changed = false;
    for (int k = 0; k < m; ++k, i += dx, j += dy) {
        size_t idx = base + size_t(j) * kp.rw + i;
        float cur = d[idx];
        if (prev < ub) {
            float c = into(cf[idx], fac);
            if (c >= 0 && prev + c < cur && prev + c < ub) { cur = prev + c; d[idx] = cur; changed = true; }
        }
        prev = cur;
    }
    if (changed) *flag = 1;
}

// Diagonal scans. A diagonal is keyed by c (i - j, or i + j for the anti-diagonals) and cut into chunks of CH rows, so
// that neighbouring lanes (neighbouring diagonals) touch neighbouring cells of the same row at every step.
template <bool APPLY>
__global__ void dscan(const KP kp, float* d, const float* cf, int dx, int dy, int dm, int lines, int nch, float2* sum, int* flag, const unsigned long long* ubp) {
    int l = blockIdx.y;
    if (!kp.lay[l]) return;
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= lines * nch) return;
    const int ww = kp.rw, wh = kp.rh, sg = dx * dy;
    int t = tid % lines, ck = tid / lines;
    int c = sg > 0 ? t - (wh - 1) : t;
    size_t base = size_t(l) * ww * wh;
    const float fac = kp.step * 1.41421356f * kp.dirf[l][dm], ub = ubof(ubp);
    size_t si = (size_t(l) * lines + t) * nch + ck;
    float C = 0.f, D = INF, prev = APPLY ? sum[si].x : INF;
    bool changed = false;
    for (int k = 0; k < CH; ++k) {
        int r = ck * CH + k;
        if (r >= wh) break;
        int j = dy > 0 ? r : wh - 1 - r;
        int i = sg > 0 ? c + j : c - j;
        if (i < 0 || i >= ww) continue;
        size_t idx = base + size_t(j) * ww + i;
        float cur = d[idx];
        float cc = into(cf[idx], fac);
        if (!APPLY) {
            if (cc < 0) { C = INF; D = cur; }
            else { C = fminf(C + cc, INF); D = fminf(fminf(D + cc, INF), cur); }
        } else {
            if (prev < ub && cc >= 0 && prev + cc < cur && prev + cc < ub) { cur = prev + cc; d[idx] = cur; changed = true; }
            prev = cur;
        }
    }
    if (APPLY) { if (changed) *flag = 1; }
    else sum[si] = make_float2(C, D);
}

__global__ void dcarry(const KP kp, int lines, int nch, float2* sum) {
    int l = blockIdx.y;
    if (!kp.lay[l]) return;
    int t = blockIdx.x * blockDim.x + threadIdx.x;
    if (t >= lines) return;
    float x = INF;
    float2* p = sum + (size_t(l) * lines + t) * nch;
    for (int ck = 0; ck < nch; ++ck) {
        float2 s = p[ck];
        p[ck].x = x;
        x = fminf(fminf(x + s.x, INF), s.y);
    }
}

// Layer changes at a cell: from the cheapest layer to every layer a via may land on.
__global__ void vias(const KP kp, float* d, const float* vcf, const uint8_t* vok, int* flag, const unsigned long long* ubp) {
    const float ub = ubof(ubp);
    size_t cell = size_t(blockIdx.x) * blockDim.x + threadIdx.x;
    size_t n = size_t(kp.rw) * kp.rh;
    if (cell >= n) return;
    float c = vcf[cell];
    if (!(c < INF)) return;
    float best = INF;
    for (int l = 0; l < kp.NL; ++l) best = fminf(best, d[size_t(l) * n + cell]);
    if (best >= INF) return;
    for (int l = 0; l < kp.NL; ++l) {
        size_t idx = size_t(l) * n + cell;
        if (!vok[idx]) continue;
        if (best + c < d[idx] && best + c < ub) {
            // the cheapest source layer must differ from l; if l itself is the minimum nothing improves
            d[idx] = best + c; *flag = 1;
        }
    }
}

// the cheapest target reached so far bounds the search: nothing costlier than it can lie on the best path
__global__ void bound(const KP kp, const float* d, const uint64_t* tbit, unsigned long long* ub) {
    size_t RN = size_t(kp.rw) * kp.rh;
    size_t idx = size_t(blockIdx.x) * blockDim.x + threadIdx.x;
    if (idx >= RN * kp.NL) return;
    float v = d[idx];
    if (!(v < ubof(ub))) return;
    int l = int(idx / RN);
    size_t c = idx % RN;
    if (!tgbit(kp, tbit, l, int(c % kp.rw), int(c / kp.rw))) return;
    atomicMin(ub, (static_cast<unsigned long long>(__float_as_uint(v)) << 32) | unsigned(idx));
}

__global__ void fill(float* d, size_t n) {
    size_t i = size_t(blockIdx.x) * blockDim.x + threadIdx.x;
    if (i < n) d[i] = INF;
}

double tnow() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

std::mutex pool_mu;
std::vector<std::pair<float*, size_t>> pool;

}  // namespace

void gpu_configure(int slots, size_t max_states) {
    std::lock_guard<std::mutex> g(slot_mu);
    if (DS.empty()) SLOTS = std::max(1, slots);  // (the slots are made on first use)
    if (max_states) MAXN = max_states;
}

bool gpu_available() {
    static std::once_flag once;
    std::call_once(once, [] {
        int n = 0;
        g_ok = cudaGetDeviceCount(&n) == cudaSuccess && n > 0;
        if (g_ok) { std::lock_guard<std::mutex> g(slot_mu); DS.resize(SLOTS); }
    });
    return g_ok;
}

void GpuOut::reserve(size_t n) {
    if (d && cap >= n) return;
    if (d) { std::lock_guard<std::mutex> g(pool_mu); pool.push_back({d, cap}); d = nullptr; }
    {
        std::lock_guard<std::mutex> g(pool_mu);
        for (size_t i = 0; i < pool.size(); ++i)
            if (pool[i].second >= n) { d = pool[i].first; cap = pool[i].second; pool.erase(pool.begin() + i); return; }
    }
    cap = n + n / 4;
    if (cudaHostAlloc((void**)&d, cap * sizeof(float), cudaHostAllocDefault) != cudaSuccess) { d = nullptr; cap = 0; }
}

GpuOut::~GpuOut() {
    if (d) { std::lock_guard<std::mutex> g(pool_mu); pool.push_back({d, cap}); }
}

float GpuOut::operator()(int64_t q) const {
    int64_t WN = int64_t(ww) * wh;
    int l = int(q / WN);
    int64_t c = q % WN;
    int i = int(c % ww) - rx0, j = int(c / ww) - ry0;
    if (i < 0 || j < 0 || i >= rw || j >= rh) return 1e30f;
    return d[(size_t(l) * rh + j) * rw + i];
}

namespace {

// One shortest-path solve over the rectangle [rx0, rx0 + rw) x [ry0, ry0 + rh) of the query's window. Returns the sweeps
// it took (or -1), and the cheapest target cost in ub_out (its rectangle state in best).
int solve_rect(const GpuQuery& q, Dev& D, int rx0, int ry0, int rw, int rh, GpuOut& out, float& ub_out, int64_t& best, double& t_up, double& t_sw) {
    cudaStream_t S = D.stream;
    double t0 = tnow();
    const size_t RN = size_t(rw) * rh, N = RN * q.NL;
    if (N > MAXN) return -1;
    if (!D.d.need(N) || !D.cf.need(N) || !D.st.need(N) || !D.vok.need(N) || !D.vcf.need(RN) || !D.vst.need(RN) ||
        !D.pcol.need(rw) || !D.prow.need(rh) || !D.mlf.need(256))
        return -1;
    if (!D.flag) { cudaMalloc(&D.flag, sizeof(int)); cudaMalloc(&D.ub, sizeof(unsigned long long)); }
    // rectangle copies of the host arrays
    size_t hoff = size_t(q.wj0 + ry0) * q.nx + q.wi0 + rx0;
    for (int l = 0; l < q.NL; ++l) {
        size_t off = size_t(l) * RN;
        cudaMemcpy2DAsync(D.cf.p + off, rw * 4, q.cst[l] + hoff, q.nx * 4, rw * 4, rh, cudaMemcpyHostToDevice, S);
        cudaMemcpy2DAsync(D.st.p + off, rw * 2, q.st[l] + hoff, q.nx * 2, rw * 2, rh, cudaMemcpyHostToDevice, S);
    }
    cudaMemcpy2DAsync(D.vcf.p, rw * 4, q.vcst + hoff, q.nx * 4, rw * 4, rh, cudaMemcpyHostToDevice, S);
    cudaMemcpy2DAsync(D.vst.p, rw * 2, q.vst + hoff, q.nx * 2, rw * 2, rh, cudaMemcpyHostToDevice, S);
    KP k{};
    k.rw = rw; k.rh = rh; k.NL = q.NL; k.rx0 = rx0; k.ry0 = ry0; k.ww = q.ww; k.wh = q.wh;
    k.me = q.me; k.via_cost = q.via_cost; k.step = q.step;
    for (int l = 0; l < 8; ++l) { k.lay[l] = q.lay[l]; k.play[l] = q.play[l]; for (int d = 0; d < 8; ++d) k.dirf[l][d] = q.dirf[l][d]; }
    k.has_prior = q.prior != nullptr;
    k.pi0 = q.pi0; k.pj0 = q.pj0; k.pw = q.pw; k.ph = q.ph; k.pnl = q.pnl; k.corridor = q.corridor; k.mthr = q.mthr;
    if (q.prior) {
        cudaMemcpyAsync(D.pcol.p, q.pcol + q.wi0 + rx0, rw * sizeof(int), cudaMemcpyHostToDevice, S);
        cudaMemcpyAsync(D.prow.p, q.prow + q.wj0 + ry0, rh * sizeof(int), cudaMemcpyHostToDevice, S);
    }
    // window states of the full window -> states of this rectangle (-1: outside it)
    const size_t FWN = size_t(q.ww) * q.wh;
    auto conv = [&](uint32_t st) -> int64_t {
        size_t l = st / FWN, c = st % FWN;
        int i = int(c % q.ww) - rx0, j = int(c / q.ww) - ry0;
        if (i < 0 || j < 0 || i >= rw || j >= rh) return -1;
        return int64_t(l * RN + size_t(j) * rw + i);
    };
    std::vector<uint32_t> li;
    std::vector<float> lv;
    auto send_list = [&](const std::vector<std::pair<uint32_t, float>>& src, bool cells, float* dst) {
        li.clear(); lv.clear();
        for (auto& [a, b] : src) {
            int64_t r = conv(a);
            if (r < 0) continue;
            if (cells) r %= int64_t(RN);  // a cell: the state's place on any layer
            li.push_back(uint32_t(r)); lv.push_back(b);
        }
        if (li.empty()) return;
        if (!D.lst.need(li.size()) || !D.lval.need(li.size())) return;
        cudaMemcpyAsync(D.lst.p, li.data(), li.size() * 4, cudaMemcpyHostToDevice, S);
        cudaMemcpyAsync(D.lval.p, lv.data(), lv.size() * 4, cudaMemcpyHostToDevice, S);
        setv<<<unsigned((li.size() + 255) / 256), 256, 0, S>>>(dst, D.lst.p, D.lval.p, int(li.size()));
        cudaStreamSynchronize(S);  // the lists are reused by the next call
    };
    send_list(q.own_cf, false, D.cf.p);
    send_list(q.own_vcf, true, D.vcf.p);
    prep_cf<<<unsigned((N + 255) / 256), 256, 0, S>>>(k, D.cf.p, D.st.p, D.vok.p, D.tbit.p, D.prior.p, D.pcol.p, D.prow.p, D.mlf.p);
    prep_v<<<unsigned((RN + 255) / 256), 256, 0, S>>>(k, D.vcf.p, D.vst.p);
    fill<<<unsigned((N + 255) / 256), 256, 0, S>>>(D.d.p, N);
    send_list(q.starts, false, D.d.p);
    {
        unsigned long long inf = (static_cast<unsigned long long>(0x7F800000u) << 32) | 0xFFFFFFFFu;  // +inf, no state
        cudaMemcpyAsync(D.ub, &inf, 8, cudaMemcpyHostToDevice, S);
        cudaStreamSynchronize(S);
    }
    double t1 = tnow();
    t_up += t1 - t0;
    // moves: code order of the CPU search: 0 +x, 1 -x, 2 +y, 3 -y, 4 (+,+), 5 (+,-), 6 (-,+), 7 (-,-)
    const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1}, DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    int sweeps = 0;
    const bool prof = getenv("GPU_PROF") != nullptr;
    double tk[8][3] = {};
    for (; sweeps < 4000; ++sweeps) {
        cudaMemsetAsync(D.flag, 0, sizeof(int), S);
        for (int dm = 0; dm < 8; ++dm) {
            const bool horiz = DY[dm] == 0, diag = DX[dm] != 0 && DY[dm] != 0;
            int lines = diag ? rw + rh - 1 : horiz ? rh : rw;
            int maxn = horiz ? rw : rh;
            int nch = (maxn + CH - 1) / CH;
            size_t need = size_t(q.NL) * lines * nch;
            if (!D.sum.need(need)) return -1;
            dim3 grid((lines * nch + 127) / 128, q.NL), gl((lines + 127) / 128, q.NL);
            dim3 hgrid((((rh + 31) / 32) * nch + 3) / 4, q.NL);
            if (prof) cudaStreamSynchronize(S);
            double a0 = tnow();
            if (horiz) hscan<false><<<hgrid, 128, 0, S>>>(k, D.d.p, D.cf.p, DX[dm], dm, nch, D.sum.p, D.flag, D.ub);
            else if (diag) dscan<false><<<grid, 128, 0, S>>>(k, D.d.p, D.cf.p, DX[dm], DY[dm], dm, lines, nch, D.sum.p, D.flag, D.ub);
            else scan_sum<<<grid, 128, 0, S>>>(k, D.d.p, D.cf.p, DX[dm], DY[dm], dm, lines, nch, D.sum.p);
            if (prof) { cudaStreamSynchronize(S); tk[dm][0] += tnow() - a0; a0 = tnow(); }
            if (diag) dcarry<<<gl, 128, 0, S>>>(k, lines, nch, D.sum.p);
            else scan_carry<<<gl, 128, 0, S>>>(k, DX[dm], DY[dm], lines, nch, D.sum.p);
            if (prof) { cudaStreamSynchronize(S); tk[dm][1] += tnow() - a0; a0 = tnow(); }
            if (horiz) hscan<true><<<hgrid, 128, 0, S>>>(k, D.d.p, D.cf.p, DX[dm], dm, nch, D.sum.p, D.flag, D.ub);
            else if (diag) dscan<true><<<grid, 128, 0, S>>>(k, D.d.p, D.cf.p, DX[dm], DY[dm], dm, lines, nch, D.sum.p, D.flag, D.ub);
            else scan_apply<<<grid, 128, 0, S>>>(k, D.d.p, D.cf.p, DX[dm], DY[dm], dm, lines, nch, D.sum.p, D.flag, D.ub);
            if (prof) { cudaStreamSynchronize(S); tk[dm][2] += tnow() - a0; }
        }
        if (q.via_cost >= 0) vias<<<unsigned((RN + 255) / 256), 256, 0, S>>>(k, D.d.p, D.vcf.p, D.vok.p, D.flag, D.ub);
        bound<<<unsigned((N + 255) / 256), 256, 0, S>>>(k, D.d.p, D.tbit.p, D.ub);
        int f = 0;
        cudaMemcpyAsync(&f, D.flag, sizeof(int), cudaMemcpyDeviceToHost, S);
        cudaStreamSynchronize(S);
        if (!f) break;
    }
    if (cudaGetLastError() != cudaSuccess) return -1;
    if (prof) { for (int dm = 0; dm < 8; ++dm) fprintf(stderr, "dir %d: sum %.1f carry %.1f apply %.1f ms\n", dm, tk[dm][0]*1e3, tk[dm][1]*1e3, tk[dm][2]*1e3); }
    double t2 = tnow();
    t_sw += t2 - t1;
    unsigned long long ubk = 0;
    cudaMemcpyAsync(&ubk, D.ub, 8, cudaMemcpyDeviceToHost, S);
    out.reserve(N);
    if (!out.d) return -1;
    cudaMemcpyAsync(out.d, D.d.p, N * 4, cudaMemcpyDeviceToHost, S);
    cudaStreamSynchronize(S);
    unsigned ub_bits = unsigned(ubk >> 32), bi = unsigned(ubk & 0xFFFFFFFFu);
    float ubf;
    memcpy(&ubf, &ub_bits, 4);
    ub_out = ubf;
    best = -1;
    if (ubf < INF && bi != 0xFFFFFFFFu) {
        size_t l = bi / RN, c = bi % RN;
        best = int64_t((l * q.wh + (c / rw + ry0)) * q.ww + (c % rw + rx0));  // back to a window state
    }
    out.rx0 = rx0; out.ry0 = ry0; out.rw = rw; out.rh = rh; out.NL = q.NL; out.ww = q.ww; out.wh = q.wh;
    return sweeps + 1;
}

}  // namespace

int gpu_solve(const GpuQuery& q, GpuOut& out, bool wait) {
    if (!gpu_available() || q.NL > 8) return -1;
    Dev* dp = nullptr;
    {
        std::unique_lock<std::mutex> lk(slot_mu);
        auto free_slot = [&]() -> Dev* { for (auto& c : DS) if (!c.busy) return &c; return nullptr; };
        dp = free_slot();
        if (!dp && !wait) return -3;  // every slot is busy
        while (!dp) { slot_cv.wait(lk); dp = free_slot(); }
        dp->busy = true;
    }
    struct Release { Dev* d; ~Release() { { std::lock_guard<std::mutex> g(slot_mu); d->busy = false; } slot_cv.notify_one(); } } rel{dp};
    Dev& D = *dp;
    if (!D.stream) cudaStreamCreateWithFlags(&D.stream, cudaStreamNonBlocking);
    double t0 = tnow(), t_up = 0, t_sw = 0;
    const size_t FWN = size_t(q.ww) * q.wh, WS = FWN * q.NL, WB = WS / 64 + 1;
    // what every rectangle shares: the target bits, the prior map and its factor table
    if (!D.tbit.need(WB)) return -1;
    cudaMemcpyAsync(D.tbit.p, q.tbit, WB * 8, cudaMemcpyHostToDevice, D.stream);
    if (q.prior) {
        size_t pn = size_t(q.pw) * q.ph * q.pnl;
        if (!D.prior.need(pn) || !D.mlf.need(256)) return -1;
        cudaMemcpyAsync(D.prior.p, q.prior, pn, cudaMemcpyHostToDevice, D.stream);
        cudaMemcpyAsync(D.mlf.p, q.mlf, 256 * sizeof(float), cudaMemcpyHostToDevice, D.stream);
    }
    // The starts and targets span a box; a path of cost ub only passes cells with (octile distance from the starts) +
    // (octile distance to the targets) <= ub. Solve a rectangle around the box first, and widen it only while that
    // bound says a cheaper path might lie outside.
    int sx0 = INT_MAX, sy0 = INT_MAX, sx1 = -1, sy1 = -1, tx0 = INT_MAX, ty0 = INT_MAX, tx1 = -1, ty1 = -1;
    auto upd = [&](size_t st, int& x0, int& y0, int& x1, int& y1) {
        size_t c = st % FWN;
        int i = int(c % q.ww), j = int(c / q.ww);
        x0 = std::min(x0, i); x1 = std::max(x1, i); y0 = std::min(y0, j); y1 = std::max(y1, j);
    };
    for (auto& p : q.starts) upd(p.first, sx0, sy0, sx1, sy1);
    for (size_t w = 0; w < WB; ++w) {
        uint64_t b = q.tbit[w];
        while (b) { int k = __builtin_ctzll(b); b &= b - 1; size_t st = w * 64 + k; if (st < WS) upd(st, tx0, ty0, tx1, ty1); }
    }
    if (sx1 < 0 || tx1 < 0) return -1;
    float minfac = 1.f;
    for (int l = 0; l < q.NL; ++l) if (q.lay[l]) for (int d = 0; d < 8; ++d) minfac = std::min(minfac, q.dirf[l][d]);
    const double unit = double(q.step) * minfac;
    auto oct = [&](int i, int j, int x0, int y0, int x1, int y1) {
        double dx = std::max({0, x0 - i, i - x1}), dy = std::max({0, y0 - j, j - y1});
        return unit * (std::max(dx, dy) + 0.41421356 * std::min(dx, dy));
    };
    auto lb = [&](int i, int j) { return oct(i, j, sx0, sy0, sx1, sy1) + oct(i, j, tx0, ty0, tx1, ty1); };
    int hx0 = std::min(sx0, tx0), hy0 = std::min(sy0, ty0), hx1 = std::max(sx1, tx1), hy1 = std::max(sy1, ty1);
    int M = 80, sweeps = 0, tries = 0;
    out.best = -1;
    for (;; M *= 3, ++tries) {
        int rx0 = std::max(0, hx0 - M), ry0 = std::max(0, hy0 - M), rx1 = std::min(q.ww, hx1 + M + 1), ry1 = std::min(q.wh, hy1 + M + 1);
        if (tries == 0 && double(rx1 - rx0) * (ry1 - ry0) > 0.5 * double(q.ww) * q.wh) { rx0 = ry0 = 0; rx1 = q.ww; ry1 = q.wh; }
        bool full = rx0 == 0 && ry0 == 0 && rx1 == q.ww && ry1 == q.wh;
        float ub = INF;
        int64_t best = -1;
        sweeps = solve_rect(q, D, rx0, ry0, rx1 - rx0, ry1 - ry0, out, ub, best, t_up, t_sw);
        if (sweeps < 0) return -1;
        out.best = best;
        if (full) break;
        if (ub < INF * 0.5f) {
            double lim = double(ub) * 1.0001 + 1e-3, mn = 1e300;
            if (rx0 > 0) for (int j = 0; j < q.wh; ++j) mn = std::min(mn, lb(rx0 - 1, j));
            if (rx1 < q.ww) for (int j = 0; j < q.wh; ++j) mn = std::min(mn, lb(rx1, j));
            if (ry0 > 0) for (int i = 0; i < q.ww; ++i) mn = std::min(mn, lb(i, ry0 - 1));
            if (ry1 < q.wh) for (int i = 0; i < q.ww; ++i) mn = std::min(mn, lb(i, ry1));
            if (mn > lim) break;
        }
    }
    if (getenv("GPU_TIMES")) fprintf(stderr, "gpu: window %dx%dx%d rect %dx%d (%d tries) up %.0f ms, %d sweeps %.0f ms, total %.0f ms%s\n", q.ww, q.wh, q.NL, out.rw, out.rh, tries + 1, t_up * 1e3, sweeps, t_sw * 1e3, (tnow() - t0) * 1e3, q.prior ? " (prior)" : "");
    return sweeps;
}
