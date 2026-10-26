// The routing problem read from a Specctra DSN file.
#pragma once
#include <map>
#include <string>
#include <vector>

#include "geom.h"

enum class Kind { Pad, Wire, Via, Plane };

struct Item {              // a piece of copper on one layer (or on every layer: layer -1)
    Kind kind;
    int net = -1;          // -1: no net (an unconnected pin)
    int layer = -1;
    int group = -1;        // items of one pin or via share a group: they are connected across layers
    bool smd = false;      // a pad on one layer only
    int zone = -1;         // a plane: its index into Board::zones (-1: the DSN carries no zone rules)
    Shape shape;
    P center;
};

// A plane's KiCad zone settings, which the Specctra export drops: make_dsn.py writes them in a (pcbroute_zones (zone
// NET LAYER (priority N) (clearance UM) (min_width UM) (fillet UM)) ...) scope of the structure, one per plane in order.
struct ZoneRule {
    int priority = 0;      // where pours of different nets overlap, the higher priority fills
    double clearance = 0;  // the zone's own clearance to other nets' copper (um; the larger of it and the classes' holds)
    double min_width = 0;  // the fill's narrowest neck (um; 0: unknown)
    double fillet = 0;     // corner radius of the outline (um; a chamfer is taken as a fillet)
};

struct Keepout {
    int layer = -1;        // -1: every layer
    bool wires = true, vias = true;
    Shape shape;
};

struct ViaType {
    std::string name, raw;
    double diameter = 0;
};

struct NetClass {
    std::string name;
    double width = 250, clearance = 200;
    std::vector<int> vias;        // indices into Board::vias, preferred first
    std::vector<bool> layers;     // usable layers
};

struct Net {
    std::string name, raw;
    int cls = 0;
    std::vector<int> items;
    bool has_pins = false;
};

struct Board {
    std::vector<std::string> layers;
    Shape outline;
    bool has_outline = false;
    std::vector<Item> items;
    std::vector<Keepout> keepouts;
    std::vector<ZoneRule> zones;
    std::vector<ViaType> vias;
    std::vector<NetClass> classes;
    std::vector<Net> nets;
    std::map<std::string, int> net_index;
    double resolution = 10;       // output units per micrometre in the session file
    std::string session_name = "board";
    double smd_smd_clearance = 0;
    double edge_clearance = -1;   // copper to board edge, um, from make_dsn.py's (pcbroute_rules (edge_clearance UM)) (-1: none)

    int layer_index(const std::string& name) const;  // -1 for "signal" / any layer
};

// image_keepouts: also the keepouts inside the library images (placed with each component); false reads only the
// structure's, as pcbroute did before 2026-10-19.
Board read_dsn(const std::string& path, bool image_keepouts = true);
