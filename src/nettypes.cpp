#include "nettypes.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

#include "sexpr.h"

const char* net_type_name(NetType t) {
    static const char* names[] = {"signal", "ground", "power", "diff", "highspeed", "crystal", "analog"};
    return names[int(t)];
}

namespace {

std::string upper(std::string s) {
    for (char& c : s) c = char(std::toupper((unsigned char)c));
    return s;
}

std::vector<std::string> split(const std::string& s, const char* seps) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (std::strchr(seps, c)) { if (!cur.empty()) out.push_back(cur); cur.clear(); }
        else cur += c;
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

// What a net is called, without the sheet path and KiCad's overbar braces, upper case; for KiCad's automatic names
// ("Net-(U2-USB_DP)") the pin's name, if it has one ("Pad1" doesn't count).
struct Name {
    std::string base, path;
};
Name split_name(const std::string& full) {
    std::string s;
    for (size_t i = 0; i < full.size(); ++i) {
        if (full.compare(i, 2, "~{") == 0) { ++i; continue; }
        if (full[i] == '}') continue;
        s += full[i];
    }
    Name n;
    if (s.rfind("Net-(", 0) == 0 || s.rfind("unconnected-(", 0) == 0) {
        size_t a = s.find('(') + 1, b = s.rfind(')');
        std::string in = s.substr(a, b == std::string::npos ? std::string::npos : b - a);
        size_t d = in.find('-');
        std::string pin = d == std::string::npos ? in : in.substr(d + 1);
        if (size_t pd = pin.find("-Pad"); pd != std::string::npos) pin = pin.substr(0, pd);  // unconnected-(U1-5V-Pad14)
        if (std::regex_match(pin, std::regex("Pad\\d*[A-Z]?\\d*"))) pin.clear();
        n.base = upper(pin);
        return n;
    }
    size_t k = s.size();
    while (k > 0 && s[k - 1] == '/') --k;  // "//AS"
    size_t slash = s.rfind('/', k ? k - 1 : 0);
    if (slash != std::string::npos && slash < k) {
        n.path = upper(s.substr(0, slash));
        n.base = upper(s.substr(slash + 1));
    } else n.base = upper(s);
    return n;
}

bool any_of(const std::vector<std::string>& toks, std::initializer_list<const char*> set) {
    for (auto& t : toks)
        for (const char* w : set)
            if (t == w) return true;
    return false;
}

}  // namespace

std::vector<NetInfo> classify_nets(const Board& B, const std::string& dsn_path) {
    // the footprints each net's pins belong to: "REF-PIN" in the network, REF -> image in the placement
    std::map<std::string, std::string> ref_image;
    std::map<std::string, std::vector<std::string>> net_refs;
    {
        std::ifstream f(dsn_path);
        if (!f) throw std::runtime_error("cannot open " + dsn_path);
        std::stringstream ss;
        ss << f.rdbuf();
        Sx root = parse_sexpr(ss.str());
        if (const Sx* pl = root.find("placement"))
            for (auto comp : pl->all("component"))
                for (auto place : comp->all("place")) ref_image[place->str(1)] = comp->str(1);
        if (const Sx* nw = root.find("network"))
            for (auto n : nw->all("net"))
                if (auto p = n->find("pins"))
                    for (size_t i = 1; i < p->kids.size(); ++i) {
                        const std::string& pin = p->kids[i].atom;
                        size_t d = pin.rfind('-');
                        net_refs[n->kids[1].atom].push_back(d == std::string::npos ? pin : pin.substr(0, d));
                    }
    }
    auto images_of = [&](const std::string& net) {
        std::string all;
        for (auto& r : net_refs[net]) all += " " + upper(r) + ":" + upper(ref_image[r]);
        return all;
    };

    const size_t n = B.nets.size();
    std::vector<NetInfo> T(n);
    for (auto& it : B.items)
        if (it.kind == Kind::Plane && it.net >= 0) T[it.net].plane = true;

    static const std::regex re_gnd("[A-Z]*GND[A-Z0-9_]*|VSS[A-Z0-9_]*|0V|EARTH");
    static const std::regex re_rail("[+-]?\\d+([.,]\\d+)?V\\d*([A-Z0-9_.+]*)?");     // 3V3, +3.3V, 5V_SYS, 1V8_A, 12V
    static const std::regex re_vname("(V(CC|DD|EE|IN|BUS|BAT|BATT|SYS|MOT|M|PP|IO|CORE|DDA|DDIO|CCA|S)|AVDD|DVDD|AVCC|BATT?|PWR|EXPWR)[A-Z0-9_.+]*");
    static const std::regex re_vtok("\\d+([.,]\\d+)?V\\d*|VDD\\w*|VCC\\w*|VIN|VBUS|VBAT|VSYS|VMOT|PWR\\d*|BATT?");
    static const std::regex re_cls_pwr(".*(PWR|POWER|SUPPLY|VCC).*");
    static const std::regex re_xtal_img(".*(CRYSTAL|XTAL|RESONATOR|OSCILLATOR).*");
    static const std::regex re_xtal_name("(XIN|XOUT|XI|XO|XTAL\\w*|\\w*_XTAL\\w*|OSC_?(IN|OUT)\\w*|OSC\\d*_(IN|OUT)\\w*|X32\\w*|OSC32\\w*)");
    static const std::regex re_hs_tok("(?!SWCLK)\\w*CLK\\w*|SCK|SCLK|QSPI\\w*|SDIO\\w*|DDR\\w*|DQS?\\d*|RGMII\\w*|RMII\\w*|ULPI\\w*|MII");
    static const std::regex re_an_tok("AIN\\w*|VREF\\w*|AREF|MIC\\w*|ANALOG\\w*|AUDIO\\w*|SENSE\\w*|ISENSE\\w*|SNS|IOUT|FILT|INA|INP|INN|CAP[PM]|SPK\\w*");

    std::vector<Name> names(n);
    for (size_t i = 0; i < n; ++i) names[i] = split_name(B.nets[i].name);

    // supply first: ground, then power
    for (size_t i = 0; i < n; ++i) {
        const Net& N = B.nets[i];
        const std::string& b = names[i].base;
        auto toks = split(b, "_ -");
        NetInfo& I = T[i];
        const std::string cls = upper(B.classes[N.cls].name);
        static const std::initializer_list<const char*> ctrl = {"EN", "ENABLE", "FB", "PG", "PGOOD", "SENSE", "SNS", "DET",
            "DETECT", "LEVEL", "LVL", "ADC", "MON", "ISET", "ILIM", "BST", "BOOT", "OK", "GOOD", "FAULT", "INA", "CTL", "SW"};
        if (b.empty()) continue;
        if (std::regex_match(b, re_gnd)) { I.type = NetType::Ground; I.why = "name"; continue; }
        if (any_of(toks, ctrl)) continue;  // EN_VBAT, FB_5V2, INA_VBUS: control or sense of a rail, not the rail
        // VIN+ / VIN- of an amplifier: an input pair, not a rail
        if ((b.back() == '+' || b.back() == '-') && B.net_index.count(N.name.substr(0, N.name.size() - (N.name.back() == ')' ? 2 : 1)) +
                                                                       (b.back() == '+' ? "-" : "+") + (N.name.back() == ')' ? ")" : "")))
            continue;
        if (std::regex_match(b, re_rail) || std::regex_match(b, re_vname) || (b.size() > 1 && b[0] == '+' && std::isalnum((unsigned char)b[1]))) {
            I.type = NetType::Power; I.why = "name"; continue;
        }
        // a rail token first (3V3_B2, VCC_PROG), or last after rail words only (AUDIO_5V, BASE_3.3V, BME_PWR); not
        // a signal that only says its level (DOUT_5V, RAD_IRQ_3V3)
        bool tok = !toks.empty() && std::regex_match(toks[0], re_vtok);
        if (!tok && toks.size() > 1 && std::regex_match(toks.back(), re_vtok)) {
            tok = true;
            for (size_t k = 0; k + 1 < toks.size(); ++k)
                tok &= toks.back().rfind("PWR", 0) == 0 || any_of({toks[k]}, {"AUDIO", "BASE", "USB", "SYS", "OUT", "MAIN", "AUX", "BUS", "A", "D", "IO",
                                                                            "ANALOG", "DIGITAL", "LOGIC", "MCU", "CORE", "PROG", "PERIPH", "SD"});
        }
        if (tok) { I.type = NetType::Power; I.why = "name token"; continue; }
        if (std::regex_match(cls, re_cls_pwr)) { I.type = NetType::Power; I.why = "class " + B.classes[N.cls].name; continue; }
    }
    // differential pairs: X_P / X_N (or XP / XM), X+ / X-, CANH / CANL; neither a supply
    for (size_t i = 0; i < n; ++i) {
        if (nt_supply(T[i].type) || T[i].pair >= 0) continue;
        const std::string& s = B.nets[i].name;
        size_t e = s.size();
        while (e > 0 && (s[e - 1] == ')' || s[e - 1] == '"')) --e;
        if (e < 2) continue;
        std::string core = s.substr(0, e - 1), tail = s.substr(e);
        char c = s[e - 1];
        std::vector<std::string> cand;
        if (c == '+') cand = {core + "-" + tail};
        else if (c == 'P') cand = {core + "N" + tail, core + "M" + tail};
        else if (c == 'p') cand = {core + "n" + tail, core + "m" + tail};
        else if (c == 'H' && upper(s).find("CAN") != std::string::npos) cand = {core + "L" + tail};
        for (auto& o : cand) {
            auto it = B.net_index.find(o);
            if (it == B.net_index.end() || size_t(it->second) == i || nt_supply(T[it->second].type) || T[it->second].pair >= 0) continue;
            T[i].pair = it->second;
            T[it->second].pair = int(i);
            break;
        }
    }
    for (size_t i = 0; i < n; ++i) {
        NetInfo& I = T[i];
        if (nt_supply(I.type)) continue;
        const Name& nm = names[i];
        const std::string& b = nm.base;
        const std::string imgs = images_of(B.nets[i].name);
        auto toks = split(b, "_ -.+");
        auto ptoks = split(nm.path, "/_ -.");
        // crystal: a pin on a crystal or oscillator footprint, or named so
        bool xt = false;
        for (auto& r : net_refs[B.nets[i].name]) xt |= std::regex_match(upper(ref_image[r]), re_xtal_img);
        if (xt || std::regex_match(b, re_xtal_name)) { I.type = NetType::Crystal; I.why = xt ? "crystal footprint" : "name"; continue; }
        bool hs = false;
        for (auto& t : toks) hs |= std::regex_match(t, re_hs_tok);
        if (I.pair >= 0) {
            const std::string u = upper(B.nets[i].name) + " " + upper(B.nets[I.pair].name);
            bool fast = hs;
            for (const char* k : {"USB", "ETH", "PAIR", "HDMI", "DSI", "CSI", "LVDS", "PCIE", "SATA", "MIPI", "TMDS", "SERDES", "CAN", "485", "422"})
                fast |= u.find(k) != std::string::npos;
            for (auto& t : toks) fast |= std::regex_match(t, std::regex("[TR]X\\d*[A-Z]?\\d*|HS\\w*"));
            fast |= std::regex_match(b, std::regex("(.*_)?(U?D[A-Z]?[+-]|U?D[PMN])")) || imgs.find("USB") != std::string::npos;
            if (fast) { I.type = NetType::Diff; I.why = "pair"; continue; }
        }
        if (hs) { I.type = NetType::HighSpeed; I.why = "clock/bus name"; continue; }
        bool an = false;
        for (auto& t : toks) an |= std::regex_match(t, re_an_tok);
        if (!an && !toks.empty() && toks[0] == "ADC")  // ADC_IA, ADC2: an input; ADC_CS, ADC_MISO: the converter's bus
            an = !any_of(toks, {"CS", "MISO", "MOSI", "SCK", "SCLK", "CLK", "RESET", "RST", "BUSY", "CONVST", "RANGE", "DRDY", "START", "SYNC"});
        if (!an && toks.size() == 1 && std::regex_match(toks[0], std::regex("ADC\\d*|A?IN\\d+[A-D]?"))) an = true;
        for (auto& t : ptoks) an |= t.rfind("AUDIO", 0) == 0;
        if (an) { I.type = NetType::Analog; I.why = I.pair >= 0 ? "pair, analog name" : "analog name"; continue; }
        if (I.pair >= 0) I.why = "pair";
    }
    return T;
}

void dump_net_types(const Board& B, const std::vector<NetInfo>& T, const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) throw std::runtime_error("cannot write " + path);
    auto js = [&](const std::string& s) {
        std::fputc('"', f);
        for (char c : s) {
            if (c == '"' || c == '\\') std::fputc('\\', f);
            std::fputc(c, f);
        }
        std::fputc('"', f);
    };
    std::fprintf(f, "{\"layers\":[");
    for (size_t l = 0; l < B.layers.size(); ++l) { std::fprintf(f, "%s", l ? "," : ""); js(B.layers[l]); }
    std::fprintf(f, "],\n\"nets\":[");
    for (size_t i = 0; i < B.nets.size(); ++i) {
        const Net& N = B.nets[i];
        std::fprintf(f, "%s\n{\"name\":", i ? "," : "");
        js(N.name);
        std::fprintf(f, ",\"type\":\"%s\",\"pair\":", net_type_name(T[i].type));
        if (T[i].pair >= 0) js(B.nets[T[i].pair].name); else std::fprintf(f, "null");
        std::fprintf(f, ",\"plane\":[");  // the layers of its pours
        std::set<int> pl;
        for (int k : N.items) if (B.items[k].kind == Kind::Plane) pl.insert(B.items[k].layer);
        for (int l : pl) std::fprintf(f, "%s%d", l == *pl.begin() ? "" : ",", l);
        std::fprintf(f, "],\"class\":");
        js(B.classes[N.cls].name);
        std::fprintf(f, ",\"width\":%.1f,\"why\":", B.classes[N.cls].width);
        js(T[i].why);
        std::fprintf(f, ",\"pads\":[");
        bool first = true;
        for (int k : N.items) {
            const Item& it = B.items[k];
            if (it.kind != Kind::Pad) continue;
            std::fprintf(f, "%s[%.1f,%.1f,%d,%d,%d]", first ? "" : ",", it.center.x, it.center.y, it.layer, it.smd ? 1 : 0, it.group);
            first = false;
        }
        std::fprintf(f, "],\"vias\":[");  // vias the DSN already has
        first = true;
        std::set<int> groups;
        for (int k : N.items) {
            const Item& it = B.items[k];
            if (it.kind != Kind::Via || !groups.insert(it.group).second) continue;
            std::fprintf(f, "%s[%.1f,%.1f]", first ? "" : ",", it.center.x, it.center.y);
            first = false;
        }
        std::fprintf(f, "]}");
    }
    std::fprintf(f, "]}\n");
    std::fclose(f);
}
