// A minimal, dependency-free test harness: enough for assert-style unit tests
// without pulling in a third-party framework.
#pragma once

#include <cstdlib>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace testing {

struct Case {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> cases;
    return cases;
}

struct Registrar {
    Registrar(const std::string& name, std::function<void()> fn) {
        registry().push_back({name, std::move(fn)});
    }
};

inline int& failure_count() {
    static int count = 0;
    return count;
}

inline void report_failure(const char* file, int line, const std::string& message) {
    std::cerr << file << ":" << line << ": FAILED: " << message << "\n";
    failure_count()++;
}

inline int run_all() {
    for (auto& c : registry()) {
        std::cerr << "[ RUN      ] " << c.name << "\n";
        c.fn();
    }
    if (failure_count() == 0) {
        std::cerr << "[  ALL OK  ] " << registry().size() << " test case(s)\n";
        return 0;
    }
    std::cerr << "[  FAILED  ] " << failure_count() << " check(s) failed\n";
    return 1;
}

}  // namespace testing

#define TEST(name)                                                             \
    static void test_##name();                                                 \
    static ::testing::Registrar registrar_##name(#name, test_##name);          \
    static void test_##name()

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) ::testing::report_failure(__FILE__, __LINE__, #cond);     \
    } while (0)

#define CHECK_EQ(a, b)                                                         \
    do {                                                                       \
        auto _a = (a);                                                         \
        auto _b = (b);                                                         \
        if (!(_a == _b)) {                                                     \
            std::ostringstream _s;                                             \
            _s << #a " == " #b " (" << _a << " vs " << _b << ")";              \
            ::testing::report_failure(__FILE__, __LINE__, _s.str());           \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                  \
    do {                                                                       \
        double _a = (a), _b = (b), _eps = (eps);                               \
        if (std::abs(_a - _b) > _eps) {                                        \
            std::ostringstream _s;                                             \
            _s << #a " ~= " #b " (" << _a << " vs " << _b << ")";              \
            ::testing::report_failure(__FILE__, __LINE__, _s.str());           \
        }                                                                      \
    } while (0)

#define CHECK_THROWS(expr)                                                     \
    do {                                                                       \
        bool _threw = false;                                                   \
        try {                                                                  \
            (void)(expr);                                                      \
        } catch (...) {                                                        \
            _threw = true;                                                     \
        }                                                                      \
        if (!_threw) ::testing::report_failure(__FILE__, __LINE__, #expr " did not throw"); \
    } while (0)
