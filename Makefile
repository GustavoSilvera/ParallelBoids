TARGET = Simulator # name of binary
CUDA_TARGET = CudaSimulator

OBJ_DIR = objs
OUT_DIR = out

OBJS = $(OBJ_DIR)/Flock.o $(OBJ_DIR)/Boid.o $(OBJ_DIR)/Neighbourhood.o $(OBJ_DIR)/Tracer.o

CPU_OBJS += $(OBJ_DIR)/Simulator.o $(OBJS)
GPU_OBJS += $(OBJ_DIR)/cudaSimulator.o $(OBJS)

CXX = g++
# CXX = clang++
CFLAGS = -std=c++11 -Wall -Werror -pedantic -pthread -fopenmp -g 
CFLAGS += -O3 # optimization
CFLAGS += -DNDEBUG # comment to enforce asserts
# CFLAGS += -DNTRACE # comment to trace memory accesses


NVCCFLAGS= -std=c++11 -O3 -m64 --gpu-architecture compute_61 -ccbin /usr/bin/gcc
NVCC=nvcc

NV_LIBS += GL glut cudart
NV_LDLIBS  := $(addprefix -l, $(NV_LIBS))
NV_LDFRAMEWORKS := $(addprefix -framework , $(FRAMEWORKS))

SRC_DIR = source

LDFLAGS += $(LIBS)
NV_LDFLAGS=-L/usr/local/depot/cuda-10.2/lib64/ -lcudart

default: $(TARGET)

dirs:
	mkdir -p $(OUT_DIR)
	mkdir -p $(OBJ_DIR)

cuda: dirs $(GPU_OBJS)
	$(CXX) $(CFLAGS) -o $(CUDA_TARGET)  $(GPU_OBJS) $(NV_LDFLAGS) $(NV_LDLIBS) $(NV_LDFRAMEWORKS)

all: $(TARGET)

$(TARGET): dirs $(CPU_OBJS)
	$(CXX) $(CFLAGS) -o $@ $(CPU_OBJS) $(LDFLAGS) 

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cu
	$(NVCC) $< $(NVCCFLAGS) -c -o $@ 

DEPS = $(OBJS:%.o=%.d)
-include $(DEPS)

clean: 
	rm $(TARGET) || true
	rm $(CUDA_TARGET) || true
	rm -rf $(OBJ_DIR) || true
	rm -rf $(OUT_DIR) || true
