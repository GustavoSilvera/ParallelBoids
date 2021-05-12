TARGET = Simulator # name of binary

OBJ_DIR = objs
OUT_DIR = out
OBJS += $(OBJ_DIR)/Simulator.o $(OBJ_DIR)/Flock.o 
OBJS += $(OBJ_DIR)/Boid.o $(OBJ_DIR)/Neighbourhood.o 
OBJS += $(OBJ_DIR)/Tracer.o # all the cpp obj files
OBJS += cudaSimulator.o # cuda files

CXX = g++
# CXX = clang++
CFLAGS = -std=c++11 -Wall -Werror -pedantic -pthread -fopenmp -g 
CFLAGS += -O3 # optimization
CFLAGS += -DNDEBUG # comment to enforce asserts
CFLAGS += -DNTRACE # comment to trace memory accesses


NVCCFLAGS=-O3 -m64 --gpu-architecture compute_61 -ccbin /usr/bin/gcc
LIBS += GL glut cudart
NVCC=nvcc

SRC_DIR = source

LDFLAGS += $(LIBS)

default: $(TARGET)

dirs:
	mkdir -p $(OUT_DIR)
	mkdir -p $(OBJ_DIR)

all: $(TARGET)

$(TARGET): dirs $(OBJS)
	$(CXX) $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<
%.o: $(SRC_DIR)/%.cu
	$(NVCC) $< $(NVCCFLAGS) -c -o $@ 

DEPS = $(OBJS:%.o=%.d)
-include $(DEPS)

clean: 
	rm $(TARGET) || true
	rm -rf $(OBJ_DIR) || true
	rm -rf $(OUT_DIR) || true
