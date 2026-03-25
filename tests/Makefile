CC      ?= gcc
CFLAGS  := -Wall -Wextra -std=c99 -g -O0 -I.
LDFLAGS := -lm

.PHONY: all test clean

all: test_unit

test_unit: test_unit.c zephyr_stubs.h test_runner.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

test: test_unit
	./test_unit

clean:
	rm -f test_unit
