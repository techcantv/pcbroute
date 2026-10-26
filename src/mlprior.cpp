#include "mlprior.h"

#include <cstdio>
#include <fstream>
#include <stdexcept>

#include "router.h"

namespace {

struct Reader {
    std::ifstream f;
    template <class T> T get() {
        T v;
        if (!f.read(reinterpret_cast<char*>(&v), sizeof v)) throw std::runtime_error("ML prior: truncated file");
        return v;
    }
    std::string str() {
        uint32_t n = get<uint32_t>();
        if (n > (1u << 20)) throw std::runtime_error("ML prior: bad string length");
        std::string s(n, '\0');
        if (n && !f.read(s.data(), n)) throw std::runtime_error("ML prior: truncated file");
        return s;
    }
};

void json_str(FILE* f, const std::string& s) {
    std::fputc('"', f);
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') std::fprintf(f, "\\%c", c);
        else if (c < 0x20) std::fprintf(f, "\\u%04x", c);
        else std::fputc(c, f);
    }
    std::fputc('"', f);
}

void json_shape(FILE* f, const Shape& s) {
    std::fprintf(f, "\"r\":%.1f,\"segs\":[", s.r);
    for (size_t i = 0; i < s.segs.size(); ++i)
        std::fprintf(f, "%s[%.1f,%.1f,%.1f,%.1f]", i ? "," : "", s.segs[i].first.x, s.segs[i].first.y, s.segs[i].second.x, s.segs[i].second.y);
    std::fprintf(f, "],\"poly\":%s", s.poly.size() >= 3 ? "1" : "0");
}

}  // namespace

// File: "MLP1", f64 pitch, x0, y0, u32 layers + names, u32 nets, then per net: name, i32 i0, j0, u32 w, h, u8 maps.
MlPrior read_ml_prior(const std::string& path) {
    Reader r{std::ifstream(path, std::ios::binary)};
    if (!r.f) throw std::runtime_error("cannot open " + path);
    char magic[4];
    if (!r.f.read(magic, 4) || std::string(magic, 4) != "MLP1") throw std::runtime_error("not an ML prior file: " + path);
    MlPrior M;
    M.pitch = r.get<double>();
    M.x0 = r.get<double>();
    M.y0 = r.get<double>();
    if (!(M.pitch > 0)) throw std::runtime_error("ML prior: bad pitch");
    uint32_t nl = r.get<uint32_t>();
    for (uint32_t i = 0; i < nl; ++i) M.layers.push_back(r.str());
    uint32_t nn = r.get<uint32_t>();
    for (uint32_t k = 0; k < nn; ++k) {
        std::string name = r.str();
        NetPrior N;
        N.i0 = r.get<int32_t>();
        N.j0 = r.get<int32_t>();
        N.w = int(r.get<uint32_t>());
        N.h = int(r.get<uint32_t>());
        size_t sz = size_t(N.w) * N.h * nl;
        if (N.w < 0 || N.h < 0 || sz > (size_t(1) << 31)) throw std::runtime_error("ML prior: bad map size");
        N.data.resize(sz);
        if (sz && !r.f.read(reinterpret_cast<char*>(N.data.data()), std::streamsize(sz))) throw std::runtime_error("ML prior: truncated file");
        M.nets[name] = std::move(N);
    }
    return M;
}

void dump_board(const Board& B, const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) throw std::runtime_error("cannot write " + path);
    std::fprintf(f, "{\"layers\":[");
    for (size_t i = 0; i < B.layers.size(); ++i) { std::fprintf(f, "%s", i ? "," : ""); json_str(f, B.layers[i]); }
    std::fprintf(f, "],\n\"outline\":");
    if (B.has_outline) {
        std::fprintf(f, "[");
        for (size_t i = 0; i < B.outline.poly.size(); ++i) std::fprintf(f, "%s[%.1f,%.1f]", i ? "," : "", B.outline.poly[i].x, B.outline.poly[i].y);
        std::fprintf(f, "]");
    } else std::fprintf(f, "null");
    std::fprintf(f, ",\n\"classes\":[");
    for (size_t i = 0; i < B.classes.size(); ++i)
        std::fprintf(f, "%s{\"width\":%.1f,\"clearance\":%.1f}", i ? "," : "", B.classes[i].width, B.classes[i].clearance);
    std::fprintf(f, "],\n\"nets\":[");
    for (size_t i = 0; i < B.nets.size(); ++i) {
        std::fprintf(f, "%s\n{\"name\":", i ? "," : "");
        json_str(f, B.nets[i].name);
        std::fprintf(f, ",\"cls\":%d,\"pins\":%d}", B.nets[i].cls, B.nets[i].has_pins ? 1 : 0);
    }
    std::fprintf(f, "],\n\"items\":[");
    static const char* kinds[] = {"pad", "wire", "via", "plane"};
    for (size_t i = 0; i < B.items.size(); ++i) {
        const Item& it = B.items[i];
        std::fprintf(f, "%s\n{\"k\":\"%s\",\"net\":%d,\"layer\":%d,\"group\":%d,\"smd\":%d,", i ? "," : "", kinds[int(it.kind)], it.net, it.layer,
                     it.group, it.smd ? 1 : 0);
        json_shape(f, it.shape);
        std::fprintf(f, "}");
    }
    std::fprintf(f, "],\n\"keepouts\":[");
    for (size_t i = 0; i < B.keepouts.size(); ++i) {
        const Keepout& k = B.keepouts[i];
        std::fprintf(f, "%s\n{\"layer\":%d,\"wires\":%d,\"vias\":%d,", i ? "," : "", k.layer, k.wires ? 1 : 0, k.vias ? 1 : 0);
        json_shape(f, k.shape);
        std::fprintf(f, "}");
    }
    std::fprintf(f, "]}\n");
    std::fclose(f);
}
