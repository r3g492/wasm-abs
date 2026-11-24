// ABS braking simulator: raylib renderer + page bridge.
#include "raylib.h"
#include "rlgl.h"
#include "sim.h"
#include "bridge.h"
#include <math.h>
#include <emscripten/emscripten.h>

#define SCREEN_W 1024
#define SCREEN_H 640
#define PLOT_H   190

static Sim sim;
static int running = 0;
static double simAccum = 0;
static int resultSent = 0;
static Camera3D camera;
static Vector3 camPos, camTarget;

static const Color WHEEL_COL[4] = {
    { 255, 107, 107, 255 }, { 77, 171, 247, 255 }, { 255, 212, 59, 255 }, { 105, 219, 124, 255 }
};
static const char *WHEEL_NAME[4] = { "FL", "FR", "RL", "RR" };

// ---------------------------------------------------------------------------
// Page bridge
// ---------------------------------------------------------------------------
EM_JS(void, js_register_scenario, (int i, const char *name, const char *desc, double v0), {
    if (Module.onScenario) Module.onScenario(i, UTF8ToString(name), UTF8ToString(desc), v0);
});

EM_JS(void, js_on_result, (int sc, int absOn, int driver, double v0, double dist, double time,
                           double yawDeg, double lat, int fault), {
    if (Module.onResult) Module.onResult({ scenario: sc, abs: !!absOn, driver: driver, v0: v0,
        dist: dist, time: time, yaw: yawDeg, lat: lat, fault: !!fault });
});

static void SendResult(const Sim *s)
{
    js_on_result(s->scenario, s->absOn, s->driver, s->v0*3.6, s->dist, s->tBrake,
                 s->maxYaw*RAD2DEG, s->maxLat, s->absFault);
}

EMSCRIPTEN_KEEPALIVE void app_run(int scenario, int absOn, int driver, float v0Kmh)
{
    if (scenario < 0 || scenario >= SCENARIO_COUNT) scenario = 0;
    if (v0Kmh < 5) v0Kmh = 5;
    if (v0Kmh > 250) v0Kmh = 250;
    BridgeAbsReset();
    SimReset(&sim, scenario, absOn, (DriverMode)driver, v0Kmh, BridgeAbs, NULL);
    running = 1;
    simAccum = 0;
    resultSent = 0;
}

// ---------------------------------------------------------------------------
// Drawing helpers: sim (x fwd, y left) -> world (X, up, Z = -y)
// ---------------------------------------------------------------------------
static Vector3 W(double x, double y, double h) { return (Vector3){ (float)x, (float)h, (float)-y }; }

static Color SurfaceColor(Surface s)
{
    switch (s) {
        case SURF_DRY:   return (Color){ 58, 60, 66, 255 };
        case SURF_WET:   return (Color){ 34, 40, 52, 255 };
        case SURF_WATER: return (Color){ 38, 92, 150, 255 };
        case SURF_ICE:   return (Color){ 196, 224, 240, 255 };
        default:         return GRAY;
    }
}

static void DrawRect3D(double x0, double x1, double y0, double y1, double h, Color c)
{
    Vector3 a = W(x0, y0, h), b = W(x1, y0, h), cc = W(x1, y1, h), d = W(x0, y1, h);
    // Two triangles, both windings so they show from above regardless of y order
    DrawTriangle3D(a, b, cc, c); DrawTriangle3D(a, cc, b, c);
    DrawTriangle3D(a, cc, d, c); DrawTriangle3D(a, d, cc, c);
}

#define ROAD_HALF 5.5
#define ROAD_X0   -250.0
#define ROAD_X1   450.0

static void DrawWorld(void)
{
    const Scenario *sc = &SCENARIOS[sim.scenario];
    // Layers never overlap at the same height; overlays sit >= 3 cm above to avoid z-fighting
    DrawPlane((Vector3){ 100, -0.08f, 0 }, (Vector2){ 900, 200 }, (Color){ 44, 70, 48, 255 });
    Color shoulder = (Color){ 90, 90, 90, 255 };
    DrawRect3D(ROAD_X0, ROAD_X1, ROAD_HALF, ROAD_HALF + 0.6, 0.0, shoulder);
    DrawRect3D(ROAD_X0, ROAD_X1, -ROAD_HALF - 0.6, -ROAD_HALF, 0.0, shoulder);
    DrawRect3D(ROAD_X0, 0, -ROAD_HALF, ROAD_HALF, 0.0, SurfaceColor(SURF_DRY));
    DrawRect3D(0, ROAD_X1, 0, ROAD_HALF, 0.0, SurfaceColor(sc->left));
    DrawRect3D(0, ROAD_X1, -ROAD_HALF, 0, 0.0, SurfaceColor(sc->right));

    // Edge lines, centre dashes, brake line, 10 m ticks
    Color line = (Color){ 235, 235, 235, 255 };
    DrawRect3D(ROAD_X0, ROAD_X1, ROAD_HALF - 0.15, ROAD_HALF, 0.03, line);
    DrawRect3D(ROAD_X0, ROAD_X1, -ROAD_HALF, -ROAD_HALF + 0.15, 0.03, line);
    for (double x = ROAD_X0; x < ROAD_X1; x += 9) DrawRect3D(x, x + 3, -0.06, 0.06, 0.03, line);
    for (int k = 0; k < 11; k++)
        DrawRect3D(0, 0.5, -ROAD_HALF + k, -ROAD_HALF + k + 1, 0.03, k % 2 ? RED : RAYWHITE);
    for (int m = 10; m <= 400; m += 10) {
        DrawRect3D(m - 0.08, m + 0.08, ROAD_HALF, ROAD_HALF + 0.6, 0.03, m % 50 ? LIGHTGRAY : YELLOW);
        DrawRect3D(m - 0.08, m + 0.08, -ROAD_HALF - 0.6, -ROAD_HALF, 0.03, m % 50 ? LIGHTGRAY : YELLOW);
    }
}

static void DrawSkids(void)
{
    for (int i = 0; i < 4; i++) {
        for (int k = 1; k < sim.skidCount[i]; k++) {
            SkidPoint *a = &sim.skid[i][k - 1], *b = &sim.skid[i][k];
            if (b->start) continue;
            float dx = b->x - a->x, dy = b->y - a->y;
            float len = sqrtf(dx*dx + dy*dy);
            if (len < 1e-4f || len > 2.0f) continue;
            float nx = -dy/len*0.1f, ny = dx/len*0.1f;
            Color c = b->surf == SURF_ICE ? (Color){ 245, 250, 255, 255 } :
                      b->surf == SURF_WATER ? (Color){ 70, 130, 190, 255 } : (Color){ 18, 18, 20, 255 };
            Vector3 p1 = W(a->x + nx, a->y + ny, 0.02), p2 = W(a->x - nx, a->y - ny, 0.02);
            Vector3 p3 = W(b->x - nx, b->y - ny, 0.02), p4 = W(b->x + nx, b->y + ny, 0.02);
            DrawTriangle3D(p1, p2, p3, c); DrawTriangle3D(p1, p3, p2, c);
            DrawTriangle3D(p1, p3, p4, c); DrawTriangle3D(p1, p4, p3, c);
        }
    }
}

static int WheelInAbs(int i) { return sim.phase == PHASE_BRAKING && sim.p[i] < sim.pm - 5; }

static void DrawCar(void)
{
    const double WX[4] = { CAR_A, CAR_A, -CAR_B, -CAR_B };
    const double WY[4] = { CAR_TRACK/2, -CAR_TRACK/2, CAR_TRACK/2, -CAR_TRACK/2 };

    rlPushMatrix();
    rlTranslatef((float)sim.x, 0, (float)-sim.y);
    rlRotatef((float)(sim.psi*RAD2DEG), 0, 1, 0);

    // Body (local: X forward, Z = right)
    Color body = (Color){ 220, 60, 50, 255 };
    DrawCube((Vector3){ (float)(CAR_A - CAR_B)/2, 0.62f, 0 }, 4.4f, 0.6f, 1.8f, body);
    DrawCubeWires((Vector3){ (float)(CAR_A - CAR_B)/2, 0.62f, 0 }, 4.4f, 0.6f, 1.8f, (Color){ 120, 20, 20, 255 });
    DrawCube((Vector3){ -0.35f, 1.15f, 0 }, 2.3f, 0.5f, 1.6f, (Color){ 40, 50, 70, 255 });
    DrawCube((Vector3){ 2.0f, 0.62f, 0.6f }, 0.05f, 0.15f, 0.35f, (Color){ 255, 250, 210, 255 });
    DrawCube((Vector3){ 2.0f, 0.62f, -0.6f }, 0.05f, 0.15f, 0.35f, (Color){ 255, 250, 210, 255 });
    if (sim.pedal > 0.05) {
        DrawCube((Vector3){ -2.42f, 0.7f, 0.65f }, 0.05f, 0.15f, 0.3f, RED);
        DrawCube((Vector3){ -2.42f, 0.7f, -0.65f }, 0.05f, 0.15f, 0.3f, RED);
    }

    for (int i = 0; i < 4; i++) {
        rlPushMatrix();
        rlTranslatef((float)WX[i], (float)CAR_R, (float)-WY[i]);
        if (i < 2) rlRotatef((float)(sim.steer*RAD2DEG), 0, 1, 0);
        rlRotatef((float)(-sim.spin[i]*RAD2DEG), 0, 0, 1);
        float side = WY[i] > 0 ? -1.0f : 1.0f;     // outward along local Z
        DrawCylinderEx((Vector3){ 0, 0, -0.12f }, (Vector3){ 0, 0, 0.12f }, (float)CAR_R, (float)CAR_R, 18,
                       (Color){ 25, 25, 28, 255 });
        Color hub = sim.slip[i] < -0.5 ? RED : (WheelInAbs(i) ? ORANGE : (Color){ 200, 200, 200, 255 });
        DrawCube((Vector3){ 0, 0, side*0.125f }, 0.5f, 0.08f, 0.02f, hub);
        DrawCube((Vector3){ 0, 0, side*0.125f }, 0.08f, 0.5f, 0.02f, hub);
        rlPopMatrix();
    }
    rlPopMatrix();
}

static void UpdateCam(float dt)
{
    Vector3 car = W(sim.x, sim.y, 0.6);
    // Chase camera: behind, above and slightly right of the car, looking ahead of it
    Vector3 pos = { car.x - 15, 7.5f, car.z + 5.0f };
    Vector3 tgt = { car.x + 10, 0, car.z };
    camera.up = (Vector3){ 0, 1, 0 };
    float k = dt <= 0 ? 1.0f : 1.0f - expf(-dt*6.0f);
    camPos.x += (pos.x - camPos.x)*k; camPos.y += (pos.y - camPos.y)*k; camPos.z += (pos.z - camPos.z)*k;
    camTarget = tgt;
    camera.position = camPos;
    camera.target = camTarget;
}

// ---------------------------------------------------------------------------
// HUD
// ---------------------------------------------------------------------------
static void DrawDistanceLabels(void)
{
    for (int m = 10; m <= 400; m += 10) {
        if (fabs(m - sim.x) > 70) continue;
        Vector3 p = W(m, -ROAD_HALF - 1.0, 0.05);
        Vector3 d = { p.x - camera.position.x, p.y - camera.position.y, p.z - camera.position.z };
        Vector3 f = { camera.target.x - camera.position.x, camera.target.y - camera.position.y, camera.target.z - camera.position.z };
        if (d.x*f.x + d.y*f.y + d.z*f.z <= 0) continue;
        Vector2 s = GetWorldToScreen(p, camera);
        if (s.x < -40 || s.x > SCREEN_W + 40 || s.y < 0 || s.y > SCREEN_H - PLOT_H) continue;
        DrawText(TextFormat("%d m", m), (int)s.x - 12, (int)s.y, 10, m % 50 ? LIGHTGRAY : YELLOW);
    }
}

static void Panel(int x, int y, int w, int h) { DrawRectangle(x, y, w, h, (Color){ 12, 14, 20, 200 }); }

static void DrawInfo(void)
{
    const Scenario *sc = &SCENARIOS[sim.scenario];
    double speed = sqrt(sim.vx*sim.vx + sim.vy*sim.vy)*3.6;
    Panel(10, 10, 300, 128);
    DrawText(sc->name, 20, 18, 20, RAYWHITE);
    DrawText(TextFormat("L: %s   R: %s", SURFACES[sc->left].name, SURFACES[sc->right].name), 20, 42, 10, LIGHTGRAY);
    DrawText(TextFormat("%5.1f km/h", speed), 20, 60, 30, RAYWHITE);
    DrawText(TextFormat("dist %5.1f m   t %5.2f s", sim.dist, sim.tBrake), 20, 94, 10, LIGHTGRAY);
    const char *absTxt = !sim.absOn ? "ABS OFF" : (sim.absFault ? "ABS FAULT: brakes unmodulated" : "ABS: your code");
    Color absCol = !sim.absOn ? GRAY : (sim.absFault ? RED : GREEN);
    DrawText(absTxt, 20, 110, 10, absCol);
    DrawText(TextFormat("driver: %s", sim.driver ? "counter-steers" : "holds straight"), 20, 124, 10, GRAY);

    // Wheel status diagram
    int cx = SCREEN_W - 120, cy = 80;
    Panel(SCREEN_W - 230, 10, 220, 128);
    DrawRectangleLines(cx - 22, cy - 50, 44, 100, GRAY);
    int wxs[4] = { cx - 34, cx + 22, cx - 34, cx + 22 };
    int wys[4] = { cy - 44, cy - 44, cy + 20, cy + 20 };
    for (int i = 0; i < 4; i++) {
        double sl = -sim.slip[i]; if (sl < 0) sl = 0; if (sl > 1) sl = 1;
        Color c = ColorLerp(GREEN, RED, (float)fmin(1.0, sl/0.3));
        DrawRectangle(wxs[i], wys[i], 12, 24, c);
        int tx = i % 2 ? wxs[i] + 18 : wxs[i] - 80;
        DrawText(TextFormat("%s %3.0f%%", WHEEL_NAME[i], sl*100), tx, wys[i], 10, WHEEL_COL[i]);
        DrawText(TextFormat("%5.1f bar", sim.p[i]), tx, wys[i] + 12, 10, LIGHTGRAY);
    }
    DrawText(TextFormat("master %3.0f bar", sim.pm), SCREEN_W - 220, 122, 10, GRAY);

    if (!running) {
        const char *t = "Press Run";
        DrawText(t, SCREEN_W/2 - MeasureText(t, 30)/2, 200, 30, RAYWHITE);
    } else if (sim.phase == PHASE_STOPPED) {
        const char *t = TextFormat("Stopped in %.1f m  (%.2f s)", sim.dist, sim.tBrake);
        int w = MeasureText(t, 26);
        Panel(SCREEN_W/2 - w/2 - 14, 160, w + 28, 64);
        DrawText(t, SCREEN_W/2 - w/2, 170, 26, RAYWHITE);
        const char *t2 = TextFormat("max yaw %.1f deg   max lateral %.2f m", sim.maxYaw*RAD2DEG, sim.maxLat);
        DrawText(t2, SCREEN_W/2 - MeasureText(t2, 10)/2, 204, 10, LIGHTGRAY);
    } else if (sim.phase == PHASE_RUNUP) {
        const char *t = "approaching brake line...";
        DrawText(t, SCREEN_W/2 - MeasureText(t, 20)/2, 170, 20, RAYWHITE);
    }
}

static void PlotFrame(Rectangle r, const char *title, float yMax, const char *unit, float tMax)
{
    DrawRectangleRec(r, (Color){ 16, 18, 24, 255 });
    for (int k = 0; k <= 4; k++) {
        float y = r.y + r.height - r.height*k/4.0f;
        DrawLine((int)r.x, (int)y, (int)(r.x + r.width), (int)y, (Color){ 40, 44, 54, 255 });
        DrawText(TextFormat("%.0f", yMax*k/4.0f), (int)r.x - 26, (int)y - 5, 10, GRAY);
    }
    int step = tMax > 20 ? 5 : (tMax > 8 ? 2 : 1);
    for (int t = 0; t <= (int)tMax; t += step) {
        float x = r.x + r.width*t/tMax;
        DrawLine((int)x, (int)r.y, (int)x, (int)(r.y + r.height), (Color){ 40, 44, 54, 255 });
        DrawText(TextFormat("%ds", t), (int)x - 6, (int)(r.y + r.height + 3), 10, GRAY);
    }
    DrawText(TextFormat("%s (%s)", title, unit), (int)r.x + 6, (int)r.y + 4, 10, LIGHTGRAY);
}

static void PlotLine(Rectangle r, float tMax, float yMax, int field, int wheel, Color c)
{
    Vector2 prev = { 0 };
    for (int k = 0; k < sim.telCount; k++) {
        const TelSample *s = &sim.tel[k];
        float v = field == 0 ? s->v*3.6f : field == 1 ? s->ws[wheel]*3.6f : field == 2 ? s->p[wheel] : s->pm;
        Vector2 p = { r.x + r.width*s->t/tMax, r.y + r.height*(1.0f - fminf(v/yMax, 1.02f)) };
        if (k > 0) DrawLineV(prev, p, c);
        prev = p;
    }
}

static void DrawPlots(void)
{
    int top = SCREEN_H - PLOT_H;
    DrawRectangle(0, top, SCREEN_W, PLOT_H, (Color){ 10, 11, 15, 255 });
    float tEnd = sim.telCount ? sim.tel[sim.telCount - 1].t : 0;
    float tMax = tEnd < 3 ? 3 : ceilf(tEnd);
    float vMax = (float)ceil(sim.v0*3.6/20.0)*20;
    Rectangle a = { 40, (float)top + 10, SCREEN_W/2.0f - 60, PLOT_H - 34 };
    Rectangle b = { SCREEN_W/2.0f + 30, (float)top + 10, SCREEN_W/2.0f - 50, PLOT_H - 34 };
    PlotFrame(a, "Speed: car (white) & wheels", vMax, "km/h", tMax);
    PlotFrame(b, "Wheel brake pressure, master (gray)", 175, "bar", tMax);
    PlotLine(b, tMax, 175, 3, 0, GRAY);
    for (int i = 3; i >= 0; i--) {
        PlotLine(a, tMax, vMax, 1, i, WHEEL_COL[i]);
        PlotLine(b, tMax, 175, 2, i, WHEEL_COL[i]);
    }
    PlotLine(a, tMax, vMax, 0, 0, RAYWHITE);
    for (int i = 0; i < 4; i++)
        DrawText(WHEEL_NAME[i], (int)(b.x + b.width - 90 + i*22), (int)b.y + 4, 10, WHEEL_COL[i]);
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
static void Frame(void)
{
    float dt = GetFrameTime();
    if (dt > 0.05f) dt = 0.05f;

    if (running && sim.phase != PHASE_STOPPED) {
        simAccum += dt;
        int steps = 0;
        while (simAccum >= SIM_DT && steps < 4000 && sim.phase != PHASE_STOPPED) {
            SimStep(&sim);
            simAccum -= SIM_DT;
            steps++;
        }
    }
    if (running && sim.phase == PHASE_STOPPED && !resultSent) {
        SendResult(&sim);
        resultSent = 1;
    }
    UpdateCam(dt);

    BeginDrawing();
        ClearBackground((Color){ 120, 150, 180, 255 });
        BeginMode3D(camera);
            DrawWorld();
            DrawSkids();
            DrawCar();
        EndMode3D();
        DrawDistanceLabels();
        DrawInfo();
        DrawPlots();
    EndDrawing();
}

int main(void)
{
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(SCREEN_W, SCREEN_H, "ABS braking simulator");

    camera.fovy = 50.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    for (int i = 0; i < SCENARIO_COUNT; i++)
        js_register_scenario(i, SCENARIOS[i].name, SCENARIOS[i].desc, SCENARIOS[i].v0Kmh);

    SimReset(&sim, 0, 1, DRIVER_STRAIGHT, SCENARIOS[0].v0Kmh, BridgeAbs, NULL);
    UpdateCam(0);

    emscripten_set_main_loop(Frame, 0, 1);
    CloseWindow();
    return 0;
}
