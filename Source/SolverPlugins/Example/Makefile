
CC = g++

.PHONY: all

all: plugin.so

%.o : %.cpp
	$(CC) $(CC_FLAGS) -I../dpsim/Include -c -fpic -o $@ $<

plugin.so: example.o
	$(CC) $(LD_FLAGS) -shared -o $@ $<
