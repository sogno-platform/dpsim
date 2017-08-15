# Declaration of variables
CC = g++
CC_FLAGS = -03 -w -std=c++11 $(INCLUDES)
LD_FLAGS = -lvillas-ext -lrt -LCIM-XML-Parser/build/thirdparty/arabica -larabica -LCIM-XML-Parser/build -lCIMParser

# File names
EXEC = DPSolver
SOURCES = $(wildcard *.cpp) $(wildcard Components/*.cpp) $(wildcard Examples/*.cpp)
OBJECTS = $(filter-out DPsimMain.o,$(SOURCES:.cpp=.o))

TEST_SOURCES = $(wildcard Tests/Test*.cpp)
TEST_OBJS = $(TEST_SOURCES:.cpp=.o)
TEST_BINS = $(TEST_SOURCES:.cpp=)

INCLUDES = -I /usr/local/include/eigen -I /usr/include/eigen3 -ICIM-XML-Parser/src -ICIM-XML-Parser/IEC61970CIM16v29a

.PHONY: tests clean

# Main target
$(EXEC): $(OBJECTS) DPsimMain.o
	$(CC) $^ $(LD_FLAGS) -o $(EXEC)

# To obtain object files
%.o: %.cpp
	$(CC) -c $(CC_FLAGS) $< -o $@

# To remove generated files
clean:
	rm -f $(EXEC) $(OBJECTS) DPsimMain.o $(TEST_BINS)
 
Tests/Test%: $(OBJECTS) Tests/Test%.o
	$(CC) $^ $(LD_FLAGS) -o $@

tests: $(TEST_BINS)
