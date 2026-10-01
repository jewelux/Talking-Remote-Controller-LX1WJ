// Minimal self-contained test runner for the host (g++) unit tests.
//
//   TEST(name) { CHECK(cond); CHECK_EQ(actual, expected); }
//
// Tests register themselves at static-init time; test_main.cpp runs them.
// A failed CHECK reports and continues, so one run shows every failure.
#pragma once

#include <cstdio>
#include <cstring>
#include <type_traits>

namespace test {

struct Case {
  const char *name;
  void (*fn)();
  Case *next;
};

inline Case *g_head = nullptr;
inline Case **g_tail = &g_head;
inline int g_checkFailures = 0;  // failed checks in the running test

struct Registrar {
  explicit Registrar(Case &c) {
    *g_tail = &c;
    g_tail = &c.next;
  }
};

template <typename T>
constexpr bool isCString = std::is_convertible_v<const T &, const char *>;

template <typename T>
void printValue(const T &v) {
  if constexpr (isCString<T>) {
    const char *s = v;
    if (s) {
      fprintf(stderr, "\"%s\"", s);
    } else {
      fputs("(null)", stderr);
    }
  } else if constexpr (std::is_same_v<T, bool>) {
    fputs(v ? "true" : "false", stderr);
  } else if constexpr (std::is_same_v<T, char>) {
    fprintf(stderr, "'%c'", v);
  } else if constexpr (std::is_enum_v<T>) {
    fprintf(stderr, "%lld", static_cast<long long>(v));
  } else if constexpr (std::is_integral_v<T> && std::is_signed_v<T>) {
    fprintf(stderr, "%lld", static_cast<long long>(v));
  } else if constexpr (std::is_integral_v<T>) {
    fprintf(stderr, "%llu", static_cast<unsigned long long>(v));
  } else if constexpr (std::is_floating_point_v<T>) {
    fprintf(stderr, "%g", static_cast<double>(v));
  } else {
    fputs("<unprintable>", stderr);
  }
}

// C strings compare by content, everything else with ==.
template <typename A, typename B>
bool equal(const A &a, const B &b) {
  if constexpr (isCString<A> && isCString<B>) {
    const char *sa = a;
    const char *sb = b;
    if (!sa || !sb) {
      return sa == sb;
    }
    return strcmp(sa, sb) == 0;
  } else {
    return a == b;
  }
}

inline void reportCheck(const char *file, int line, const char *expr) {
  fprintf(stderr, "  %s:%d: CHECK(%s) failed\n", file, line, expr);
  ++g_checkFailures;
}

template <typename A, typename B>
void reportEq(const char *file, int line, const char *actualExpr, const char *expectedExpr,
              const A &actual, const B &expected) {
  fprintf(stderr, "  %s:%d: CHECK_EQ(%s, %s) failed\n    actual:   ", file, line, actualExpr,
          expectedExpr);
  printValue(actual);
  fputs("\n    expected: ", stderr);
  printValue(expected);
  fputc('\n', stderr);
  ++g_checkFailures;
}

}  // namespace test

#define TEST(name)                                              \
  static void name();                                           \
  static ::test::Case name##_case{#name, name, nullptr};        \
  static const ::test::Registrar name##_registrar{name##_case}; \
  static void name()

#define CHECK(cond)                                    \
  do {                                                 \
    if (!(cond)) {                                     \
      ::test::reportCheck(__FILE__, __LINE__, #cond);  \
    }                                                  \
  } while (0)

#define CHECK_EQ(actual, expected)                                                        \
  do {                                                                                    \
    const auto &check_a_ = (actual);                                                      \
    const auto &check_e_ = (expected);                                                    \
    if (!::test::equal(check_a_, check_e_)) {                                             \
      ::test::reportEq(__FILE__, __LINE__, #actual, #expected, check_a_, check_e_);       \
    }                                                                                     \
  } while (0)
