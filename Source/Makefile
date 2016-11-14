# Declaration of variables
CC = g++
CC_FLAGS = -ggdb -w -lm -std=c++11 $(INCLUDES)

# File names
EXEC = DPSolver
SOURCES = $(wildcard *.cpp) $(wildcard Components/*.cpp)
OBJECTS = $(SOURCES:.cpp=.o)

INCLUDES = -I /usr/local/include/eigen

# Main target
$(EXEC): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(EXEC)

# To obtain object files
%.o: %.cpp
	$(CC) -c $(CC_FLAGS) $< -o $@

# To remove generated files
clean:
	rm -f $(EXEC) $(OBJECTS)
