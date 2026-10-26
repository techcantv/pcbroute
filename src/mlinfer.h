// In-process RouteNet prior (--ml-model): the board rasterized as ml/raster.py does it, the per-net maps predicted
// with ONNX Runtime (make ONNXRT=1), turned into the MlPrior that ml/mlroute.py would have written. No Python.
#pragma once
#include <memory>
#include <string>

#include "dsn.h"
#include "mlprior.h"

struct MlModelOptions {
    std::string model;              // RouteNet exported by ml/tools/export_onnx.py
    std::string device = "auto";    // auto (CUDA when it loads, else CPU), cuda, cpu
    double union_share = 0.5;       // weight of the layer-agnostic prediction in each layer's map (mlroute --union-share)
    int grow = 2;                   // pixels the maps are grown by (mlroute --grow)
    std::string dump_inputs;        // debugging: write the model's input tensors (and the jobs) to this file
    std::string server;             // a resident model (pcbroute --ml-serve) on this unix socket runs the batches when it
                                    // answers for the same model and options; else (or when it fails) they run in process
    bool spawn_server = false;      // no server answering at `server`: start one (detached) for the next runs
    int verbose = 1;
};

bool ml_infer_available();  // built with ONNX Runtime

// pcbroute --ml-serve SOCK: keep m.model loaded (CUDA session, kernels warm) and run the batches clients send over
// the unix socket SOCK, until idle_s seconds pass without a client; the GPU arena is given back after trim_s idle
int ml_serve(const MlModelOptions& m, const std::string& sock, double idle_s, double trim_s);

// The prediction in the background: the model starts loading when this is made (before the DSN is read), the
// prediction when start() is called, and get() waits for it (pcbroute calls it at the end of the router's setup).
class MlPredictor {
public:
    explicit MlPredictor(const MlModelOptions& m);
    ~MlPredictor();
    void start(const Board& B);  // B must outlive the prediction
    const MlPrior& get();        // rethrows the prediction's errors
private:
    struct State;
    std::shared_ptr<State> s;
};

MlPrior predict_ml_prior(const Board& B, const MlModelOptions& m);  // load, predict, wait
void write_ml_prior(const MlPrior& M, const std::string& path);  // the MLP1 file read_ml_prior reads
