# Declaration of variables
CC = g++
CC_FLAGS = -ggdb -w -std=c++11 $(INCLUDES)
LD_FLAGS = -lvillas-ext

# File names
EXEC = DPSolver
SOURCES = $(wildcard *.cpp) $(wildcard Components/*.cpp) $(wildcard Examples/*.cpp)
OBJECTS = $(SOURCES:.cpp=.o)

INCLUDES = -I /usr/include/eigen3

# Main target
$(EXEC): $(OBJECTS)
	$(CC) $(OBJECTS) $(LD_FLAGS) -o $(EXEC)

# To obtain object files
%.o: %.cpp
	$(CC) -c $(CC_FLAGS) $< -o $@

# To remove generated files
clean:
	rm -f $(EXEC) $(OBJECTS)
