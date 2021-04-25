TARGET = Boids
OBJS += Boids.o

CXX = clang++
CFLAGS = -std=c++11 -Wall -Werror -pedantic -pthread -fopenmp -g -O3 -DNDEBUG 
SOURCE = source
LDFLAGS += $(LIBS)

default: $(TARGET)
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: $(SOURCE)/%.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<

DEPS = $(OBJS:%.o=%.d)
-include $(DEPS)

clean: 
	rm $(TARGET) $(OBJS) $(DEPS) || true
	rm -rf Out/ || true