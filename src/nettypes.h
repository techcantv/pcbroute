// Net types (ground, power, differential pair, high-speed, crystal, analog, plain signal), from the net names, the
// net classes and the footprints the pins belong to. They drive the routing guidelines of the --nt-* options
// (router.cpp), all off by default; --nt-dump writes the classification for checking by eye.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "dsn.h"

enum class NetType : uint8_t { Signal, Ground, Power, Diff, HighSpeed, Crystal, Analog };

struct NetInfo {
    NetType type = NetType::Signal;
    int pair = -1;       // the other net of a differential pair (any pair named X_P / X_N, X+ / X-, XDP / XDM, ...)
    bool plane = false;  // the net has a pour (DSN plane) on some layer
    std::string why;     // the rule that decided
};

const char* net_type_name(NetType t);
// fast: routed first and kept short (--nt-hs-first, --nt-hs-via, --nt-hs-len)
inline bool nt_fast(NetType t) { return t == NetType::Diff || t == NetType::HighSpeed || t == NetType::Crystal; }
inline bool nt_supply(NetType t) { return t == NetType::Ground || t == NetType::Power; }

std::vector<NetInfo> classify_nets(const Board& B, const std::string& dsn_path);
// JSON: per net its type, pair, plane layers, class and rule, and its pads ([x, y] in um, layer (-1: all), smd, pin group) for nt_metrics.py
void dump_net_types(const Board& B, const std::vector<NetInfo>& T, const std::string& path);
