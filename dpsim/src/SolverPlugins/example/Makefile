
CC = gcc

.PHONY: all

all: plugin.so

%.o : %.c
	$(CC) $(CC_FLAGS) -I../../../include -c -fpic -o $@ $<

plugin.so: example.o
	$(CC) $(LD_FLAGS) -shared -o $@ $<
