/*
 * test_runner.h — Minimal test framework for host unit tests
 */
#pragma once

#include <stdio.h>
#include <math.h>

static int _tests_run = 0;
static int _tests_passed = 0;
static int _tests_failed = 0;

#define TEST(name) static void name(void)

#define RUN_TEST(fn) do {                                   \
    _tests_run++;                                           \
    int _prev_fail = _tests_failed;                         \
    fn();                                                   \
    if (_tests_failed == _prev_fail) {                      \
        _tests_passed++;                                    \
        printf("  PASS  %s\n", #fn);                        \
    }                                                       \
} while (0)

#define ASSERT_EQ(a, b) do {                                \
    long long _a = (long long)(a), _b = (long long)(b);     \
    if (_a != _b) {                                         \
        printf("  FAIL  %s:%d: %s == %lld, expected %lld\n",\
               __FILE__, __LINE__, #a, _a, _b);             \
        _tests_failed++;                                    \
        return;                                             \
    }                                                       \
} while (0)

#define ASSERT_TRUE(expr) do {                              \
    if (!(expr)) {                                          \
        printf("  FAIL  %s:%d: %s is false\n",              \
               __FILE__, __LINE__, #expr);                  \
        _tests_failed++;                                    \
        return;                                             \
    }                                                       \
} while (0)

#define ASSERT_FALSE(expr) ASSERT_TRUE(!(expr))

#define ASSERT_FLOAT_EQ(a, b, eps) do {                     \
    float _a = (a), _b = (b);                               \
    if (fabsf(_a - _b) > (eps)) {                           \
        printf("  FAIL  %s:%d: %s == %.6f, expected %.6f "  \
               "(eps=%.6f)\n",                              \
               __FILE__, __LINE__, #a, _a, _b, (float)(eps));\
        _tests_failed++;                                    \
        return;                                             \
    }                                                       \
} while (0)

#define TEST_SUMMARY() do {                                 \
    printf("\n%d/%d tests passed",                          \
           _tests_passed, _tests_run);                      \
    if (_tests_failed)                                      \
        printf(", %d FAILED", _tests_failed);               \
    printf("\n");                                            \
    return _tests_failed ? 1 : 0;                           \
} while (0)
