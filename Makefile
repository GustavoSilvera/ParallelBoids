TARGET = Simulator # name of binary
OBJS += Simulator.o Flock.o Boid.o Neighbourhood.o Tracer.o # all the cpp obj files

CXX = g++
# CXX = clang++
CFLAGS = -std=c++11 -Wall -Werror -pedantic -pthread -fopenmp -g 
CFLAGS += -O3 # optimization
CFLAGS += -DNDEBUG # comment to enforce asserts
CFLAGS += -DNTRACE # comment to trace memory accesses
SRC_DIR = source

LDFLAGS += $(LIBS)

default: $(TARGET)
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<

DEPS = $(OBJS:%.o=%.d)
-include $(DEPS)

clean: 
	rm $(TARGET) $(OBJS) || true
	rm -rf Out/*.ppm || true