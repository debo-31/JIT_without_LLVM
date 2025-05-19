CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2

TARGET = jit_compiler
SRCS = main.cpp compiler_components.cpp jit_compiler.cpp
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean 