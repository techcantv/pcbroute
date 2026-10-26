// Learned routing prior (written by ml/mlroute.py): for each net, a coarse map per layer of how likely the net's
// copper is to run through each cell, 0-255. Units: micrometres, DSN axes, like the rest of the router.
#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>

struct NetPrior {
    int i0 = 0, j0 = 0, w = 0, h = 0;  // the map covers prior pixels [i0, i0 + w) x [j0, j0 + h)
    std::vector<uint8_t> data;         // layer-major, then rows (y up), then columns
};

struct MlPrior {
    double pitch = 0, x0 = 0, y0 = 0;  // pixel (i, j) spans x0 + i * pitch .. + pitch, y0 + j * pitch .. + pitch
    std::vector<std::string> layers;   // the layers of the maps, by name
    std::map<std::string, NetPrior> nets;
};

MlPrior read_ml_prior(const std::string& path);
