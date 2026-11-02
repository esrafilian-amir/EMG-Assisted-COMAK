// Standalone check of SettleMonitor (OpenSim/JAM/JAMUtilities.h).
// Build (VS 2019 x64 prompt):
//   cl /std:c++14 /EHsc /I..\..\OpenSim\JAM test_settle_monitor.cpp
#include "JAMUtilities.h"

#include <cmath>
#include <cstdio>

static int failures = 0;
#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("FAIL line %d: %s\n", __LINE__, #cond);                \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

// Feeds values until the monitor stops; returns the number of steps used, or
// -1 if it never stopped within n values.
template <class F>
static int run(SettleMonitor& m, int n, F value) {
    for (int i = 0; i < n; ++i)
        if (m.update(value(i))) return m.numSteps();
    return -1;
}

int main() {
    // 1. Threshold only (plateau off): stops at the first value <= threshold.
    {
        SettleMonitor m(1e-3);
        int steps = run(m, 1000, [](int i) { return 1.0 * std::pow(0.9, i); });
        CHECK(steps > 0);
        CHECK(m.reason() == "coordinate change below settle threshold");
        CHECK(!m.hitSafetyLimit());
        // 0.9^i <= 1e-3 first at i = 66 -> 67 steps
        CHECK(steps == 67);
    }

    // 2. A flat plateau well below the first step stops early when enabled,
    //    and never stops when disabled.
    {
        auto plateau = [](int i) { return i < 5 ? 1.0 / (i + 1) : 0.02; };
        SettleMonitor on(1e-4, 0.1, 10);
        int steps = run(on, 200, plateau);
        CHECK(steps > 0 && steps < 30);
        CHECK(on.reason() == "coordinate change plateaued");

        SettleMonitor off(1e-4, 0.0, 10);
        CHECK(run(off, 200, plateau) == -1);
    }

    // 3. A decaying tail that is still falling faster than the tolerance is
    //    not accepted as a plateau (5% decay per step -> ~40% over 10 steps).
    {
        SettleMonitor m(1e-6, 0.1, 10);
        int steps = run(m, 60, [](int i) { return 1.0 * std::pow(0.95, i); });
        CHECK(steps == -1);
    }

    // 4. A flat value that has NOT dropped below 10% of the first step is not
    //    a plateau (still moving as much as at the start).
    {
        SettleMonitor m(1e-6, 0.1, 10);
        int steps = run(m, 100, [](int) { return 0.5; });
        CHECK(steps == -1);
    }

    // 5. Oscillation within the tolerance band counts as a plateau.
    {
        SettleMonitor m(1e-6, 0.1, 10);
        int steps = run(m, 100, [](int i) {
            return i == 0 ? 1.0 : (i % 2 ? 0.020 : 0.019);
        });
        CHECK(steps > 0 && steps <= 12);
    }

    // 6. Oscillation wider than the tolerance band does not.
    {
        SettleMonitor m(1e-6, 0.1, 10);
        int steps = run(m, 100, [](int i) {
            return i == 0 ? 1.0 : (i % 2 ? 0.020 : 0.010);
        });
        CHECK(steps == -1);
    }

    // 7. Safety limit.
    {
        SettleMonitor m(1e-12, 0.0, 10);
        int steps =
                run(m, SettleMonitor::maxSteps() + 10, [](int) { return 1.0; });
        CHECK(steps == SettleMonitor::maxSteps());
        CHECK(m.hitSafetyLimit());
    }

    // 8. Need a full window before a plateau can be declared.
    {
        SettleMonitor m(1e-6, 0.1, 10);
        m.update(1.0);
        for (int i = 0; i < 8; ++i) CHECK(!m.update(0.02));
    }

    std::printf(failures ? "%d FAILURES\n" : "all SettleMonitor checks passed\n",
            failures);
    return failures ? 1 : 0;
}
