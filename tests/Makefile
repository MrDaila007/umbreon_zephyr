CC      ?= gcc
CFLAGS  := -Wall -Wextra -std=c99 -g -O0 -I.
LDFLAGS := -lm

# Zephyr workspace
ZEPHYR_VENV   ?= $(HOME)/zephyrproject-v4.3/.venv/bin/activate
ZEPHYR_BASE   ?= $(HOME)/zephyrproject-v4.3/zephyr
ZTEST_SRC     := ztest
ZTEST_BUILD   := /tmp/umbreon_ztest
ZTEST_BOARD   := native_sim

.PHONY: all test test-host test-ztest clean clean-ztest

# ── Default: run both ─────────────────────────────────────────────────────────
all: test

test: test-host test-ztest

# ── Host unit tests (gcc, no Zephyr) ─────────────────────────────────────────
test_unit: test_unit.c zephyr_stubs.h test_runner.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

test-host: test_unit
	@echo "══ Host unit tests ══"
	@./test_unit

# ── Zephyr ztest (native_sim) ────────────────────────────────────────────────
test-ztest:
	@echo "══ Zephyr ztest ($(ZTEST_BOARD)) ══"
	@. $(ZEPHYR_VENV) && \
		ZEPHYR_BASE=$(ZEPHYR_BASE) \
		west build -b $(ZTEST_BOARD) -d $(ZTEST_BUILD) $(ZTEST_SRC) 2>&1 | tail -3
	@$(ZTEST_BUILD)/zephyr/zephyr.exe

# ── Clean ─────────────────────────────────────────────────────────────────────
clean:
	rm -f test_unit

clean-ztest:
	rm -rf $(ZTEST_BUILD)

clean-all: clean clean-ztest
