// Bridge between the C simulation and the user's JavaScript ABS controller.
#include "bridge.h"
#include <emscripten/emscripten.h>

EM_JS(int, js_abs_step, (const float *inp, int n, float *out), {
    if (!Module.absStep) return 0;
    var r = Module.absStep(HEAPF32.subarray(inp >> 2, (inp >> 2) + n));
    if (!r || r.length !== 4) return 0;
    for (var i = 0; i < 4; i++) {
        var v = +r[i];
        if (!isFinite(v)) return 0;
        HEAPF32[(out >> 2) + i] = v;
    }
    return 1;
});

EM_JS(void, js_abs_reset, (void), {
    if (Module.absReset) Module.absReset();
});

int BridgeAbs(const float *in, float *out, void *user)
{
    (void)user;
    return js_abs_step(in, ABS_IN_COUNT, out);
}

void BridgeAbsReset(void)
{
    js_abs_reset();
}
