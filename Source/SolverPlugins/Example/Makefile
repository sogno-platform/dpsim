
CC = gcc

.PHONY: all

all: plugin.so

%.o : %.c
	$(CC) $(CC_FLAGS) -I../../../Include -c -fpic -o $@ $<

plugin.so: example.o
	$(CC) $(LD_FLAGS) -shared -o $@ $<
