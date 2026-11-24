// Headless check: runs every scenario with ABS off and on, prints the results.
// Built for Node by `make test`; the ABS is the real JS controller (web/abs_example.js).
#include "../src/sim.h"
#include "../src/bridge.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

static Sim sim;

static void Report(const char *label)
{
    double v0 = sim.v0;
    double meanG = v0/sim.tBrake/9.81;
    // ABS cycles: count pressure drops on FL
    int cycles = 0;
    for (int k = 2; k < sim.telCount; k++)
        if (sim.tel[k].p[0] < sim.tel[k-1].p[0] - 0.5 && sim.tel[k-1].p[0] >= sim.tel[k-2].p[0] - 0.5) cycles++;
    double minWs = 1e9;
    for (int k = 0; k < sim.telCount; k++) if (sim.tel[k].v > 3) for (int i = 0; i < 4; i++) {
        double r = sim.tel[k].ws[i]/sim.tel[k].v; if (r < minWs) minWs = r; }
    printf("%-24s %-4s %7.1f m %6.2f s %5.2f g  yaw %6.1f deg  lat %5.2f m  cyclesFL %3d  minWs/v %.2f%s\n",
        SCENARIOS[sim.scenario].name, label, sim.dist, sim.tBrake, meanG,
        sim.maxYaw*57.2958, sim.maxLat, cycles, minWs, sim.absFault ? "  ABS FAULT" : "");
}

int main(int argc, char **argv)
{
    int driver = argc > 1 ? atoi(argv[1]) : 0;
    printf("driver: %s\n", driver ? "corrective" : "straight");
    for (int sc = 0; sc < SCENARIO_COUNT; sc++) {
        for (int abs = 0; abs <= 1; abs++) {
            BridgeAbsReset();
            SimReset(&sim, sc, abs, (DriverMode)driver, SCENARIOS[sc].v0Kmh, BridgeAbs, NULL);
            SimRunToEnd(&sim);
            Report(abs ? "ABS" : "off");
        }
    }
    return 0;
}
