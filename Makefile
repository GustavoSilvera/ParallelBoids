TARGET = Simulator # name of binary
OBJS += Simulator.o Flock.o Boid.o

CXX = clang++
# CFLAGS = -std=c++11 -Wall -Werror -pedantic -pthread -fopenmp -g -O3 -DNDEBUG 
CFLAGS = -std=c++11 -Wall -Werror -pedantic -pthread -fopenmp -g -O3 
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