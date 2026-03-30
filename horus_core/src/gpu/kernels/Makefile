# HORUS GPU Kernels — build libhorus_kernels.so
#
# Usage:
#   make              # Build shared library
#   make clean        # Remove build artifacts
#   make install      # Copy to target/gpu/ for Rust dlopen

NVCC ?= nvcc
CUDA_ARCH ?= -gencode arch=compute_75,code=sm_75 \
             -gencode arch=compute_80,code=sm_80 \
             -gencode arch=compute_86,code=sm_86 \
             -gencode arch=compute_89,code=sm_89

NVCC_FLAGS = -O2 -shared -Xcompiler -fPIC $(CUDA_ARCH)

SRC = horus_kernels.cu
HDR = horus_kernels.h
OUT = libhorus_kernels.so

# Default target dir (overridden by build.rs)
TARGET_DIR ?= .

.PHONY: all clean install

all: $(OUT)

$(OUT): $(SRC) $(HDR)
	$(NVCC) $(NVCC_FLAGS) -o $(OUT) $(SRC)

clean:
	rm -f $(OUT)

install: $(OUT)
	mkdir -p $(TARGET_DIR)
	cp $(OUT) $(TARGET_DIR)/
