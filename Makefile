TARGET = Boids
OBJS += Boids.o

CXX = clang++
CFLAGS = -std=c++11 -Wall -Werror -pedantic -g -DNDEBUG 

LDFLAGS += $(LIBS)

default: $(TARGET)
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CFLAGS) -c -o $@ $<

DEPS = $(OBJS:%.o=%.d)
-include $(DEPS)

clean: 
	rm $(TARGET) $(OBJS) $(DEPS) || true