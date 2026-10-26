CXX ?= g++
CXXFLAGS ?= -O3 -march=native -std=c++17 -Wall -Wextra -Wno-unused-parameter
LDFLAGS ?= -pthread

BD ?= build

all: $(BD)/pcbroute
SRC := $(wildcard src/*.cpp)
OBJ := $(SRC:src/%.cpp=$(BD)/%.o)

# make GPU=1: also build the CUDA back end (needs nvcc; CUDA 12.0 wants g++ 12 or older as its host compiler)
ifeq ($(GPU),1)
NVCC ?= nvcc
CUDA_ARCH ?= sm_86
CXXFLAGS += -DPCB_GPU
OBJ += $(BD)/gpu.o
LDFLAGS += -lcudart
$(BD)/gpu.o: src/gpu.cu src/gpu.h | $(BD)
	$(NVCC) -arch=$(CUDA_ARCH) -ccbin $(or $(shell which g++-12),g++) -O3 -c -o $@ $<
endif

# make ONNXRT=1: also the in-process RouteNet prior (--ml-model, src/mlinfer.cpp) through ONNX Runtime's C++ API. The
# library comes from the onnxruntime-gpu wheel in ./venv (ORT_LIB), the headers of the same release from
# third_party (ORT_INC); its CUDA provider uses the CUDA 13 / cuDNN 9 libraries of the pip "nvidia" packages
# (ORT_NVIDIA, preloaded at run time; PCBROUTE_NVIDIA_DIR overrides it). Without ONNXRT=1, --ml-model is refused.
ifeq ($(ONNXRT),1)
ORT_VERSION ?= 1.31.0
ORT_SITE ?= $(CURDIR)/venv/lib/python3.12/site-packages
ORT_LIB ?= $(ORT_SITE)/onnxruntime/capi
ORT_INC ?= $(CURDIR)/third_party/onnxruntime-$(ORT_VERSION)/include
ORT_NVIDIA ?= $(ORT_SITE)/nvidia
# the wheel has no libonnxruntime.so.1 (the soname): links to its libraries in $(BD)/ort, the binary's run path
ORT_LINKS := $(BD)/ort/libonnxruntime.so.1
$(BD)/mlinfer.o: CXXFLAGS += -DPCB_ONNXRT -DPCB_NVIDIA_DIR='"$(ORT_NVIDIA)"' -I$(ORT_INC)
LDFLAGS += -L$(BD)/ort -l:libonnxruntime.so.1 -Wl,-rpath,$(abspath $(BD))/ort -ldl
$(BD)/pcbroute: $(ORT_LINKS)
$(ORT_LINKS): | $(BD)
	mkdir -p $(BD)/ort
	ln -sf $(ORT_LIB)/libonnxruntime.so.$(ORT_VERSION) $(BD)/ort/libonnxruntime.so.1
	for f in $(ORT_LIB)/libonnxruntime_providers_*.so; do ln -sf $$f $(BD)/ort/; done
endif

# the rasterizer reproduces Pillow's integer drawing and numpy's float rounding: no fused multiply-adds
$(BD)/mlinfer.o: CXXFLAGS += -ffp-contract=off

$(BD)/pcbroute: $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

$(BD)/%.o: src/%.cpp $(wildcard src/*.h src/*.inc) | $(BD)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BD):
	mkdir -p $(BD)

clean:
	rm -rf $(BD)

.PHONY: clean
