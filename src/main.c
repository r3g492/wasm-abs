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

#define ROAD_HALF   5.5
#define ROAD_BEHIND 300.0   // road drawn this far behind the car...
#define ROAD_AHEAD  700.0   // ...and this far ahead, so it never ends on screen

static void DrawWorld(void)
{
    const Scenario *sc = &SCENARIOS[sim.scenario];
    double x0 = floor((sim.x - ROAD_BEHIND)/9.0)*9.0;   // multiple of 9 keeps the dashes fixed in the world
    double x1 = sim.x + ROAD_AHEAD;

    // Layers never overlap at the same height; overlays sit >= 3 cm above to avoid z-fighting
    DrawPlane((Vector3){ (float)(x0 + x1)/2, -0.08f, 0 }, (Vector2){ (float)(x1 - x0) + 200, 600 }, (Color){ 44, 70, 48, 255 });
    Color shoulder = (Color){ 90, 90, 90, 255 };
    DrawRect3D(x0, x1, ROAD_HALF, ROAD_HALF + 0.6, 0.0, shoulder);
    DrawRect3D(x0, x1, -ROAD_HALF - 0.6, -ROAD_HALF, 0.0, shoulder);
    if (x0 < 0) DrawRect3D(x0, fmin(0, x1), -ROAD_HALF, ROAD_HALF, 0.0, SurfaceColor(SURF_DRY));
    if (x1 > 0) {
        DrawRect3D(fmax(0, x0), x1, 0, ROAD_HALF, 0.0, SurfaceColor(sc->left));
        DrawRect3D(fmax(0, x0), x1, -ROAD_HALF, 0, 0.0, SurfaceColor(sc->right));
    }

    // Edge lines, centre dashes, brake line, 10 m ticks
    Color line = (Color){ 235, 235, 235, 255 };
    DrawRect3D(x0, x1, ROAD_HALF - 0.15, ROAD_HALF, 0.03, line);
    DrawRect3D(x0, x1, -ROAD_HALF, -ROAD_HALF + 0.15, 0.03, line);
    for (double x = x0; x < x1; x += 9) DrawRect3D(x, x + 3, -0.06, 0.06, 0.03, line);
    if (x0 < 0.5 && x1 > 0)
        for (int k = 0; k < 11; k++)
            DrawRect3D(0, 0.5, -ROAD_HALF + k, -ROAD_HALF + k + 1, 0.03, k % 2 ? RED : RAYWHITE);
    for (int m = (int)fmax(10, ceil(x0/10)*10); m <= x1; m += 10) {
        DrawRect3D(m - 0.08, m + 0.08, ROAD_HALF, ROAD_HALF + 0.6, 0.03, m % 50 ? LIGHTGRAY : YELLOW);
        DrawRect3D(m - 0.08, m + 0.08, -ROAD_HALF - 0.6, -ROAD_HALF, 0.03, m % 50 ? LIGHTGRAY : YELLOW);
    }
}

static void DrawSkids(void)
{
    for (int i = 0; i < 4; i++) {
        for (int k = 1; k < sim.skidCount[i]; k++) {
            SkidPoint *a = &sim.skid[i][k - 1], *b = &sim.skid[i][k];
            if (b->start || b->x < sim.x - ROAD_BEHIND) continue;
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

// ---------------------------------------------------------------------------
// Car model (assets/*.glb from tools/make_car.py) with a small lit shader
// ---------------------------------------------------------------------------
static Model carBody, carWheel;
static Shader carShader;
static int viewPosLoc = -1;
static Color *tailLight = NULL;     // taillight material colour, brightened while braking
static Color tailLightOff;

static const char *CAR_VS =
    "#version 100\n"
    "attribute vec3 vertexPosition;\n"
    "attribute vec3 vertexNormal;\n"
    "uniform mat4 mvp;\n"
    "uniform mat4 matModel;\n"
    "uniform mat4 matNormal;\n"
    "varying vec3 fragPos;\n"
    "varying vec3 fragNormal;\n"
    "void main() {\n"
    "    fragPos = vec3(matModel*vec4(vertexPosition, 1.0));\n"
    "    fragNormal = normalize(vec3(matNormal*vec4(vertexNormal, 0.0)));\n"
    "    gl_Position = mvp*vec4(vertexPosition, 1.0);\n"
    "}\n";

// Colours from glTF are linear: light in linear space, then gamma-encode.
static const char *CAR_FS =
    "#version 100\n"
    "precision mediump float;\n"
    "varying vec3 fragPos;\n"
    "varying vec3 fragNormal;\n"
    "uniform vec4 colDiffuse;\n"
    "uniform vec3 viewPos;\n"
    "void main() {\n"
    "    vec3 n = normalize(fragNormal);\n"
    "    vec3 L = normalize(vec3(-0.35, 0.85, 0.4));\n"
    "    vec3 v = normalize(viewPos - fragPos);\n"
    "    float d = max(dot(n, L), 0.0);\n"
    "    vec3 amb = mix(vec3(0.08, 0.08, 0.07), vec3(0.30, 0.34, 0.40), 0.5 + 0.5*n.y);\n"
    "    float spec = pow(max(dot(n, normalize(L + v)), 0.0), 60.0)*0.6;\n"
    "    float fres = pow(1.0 - max(dot(n, v), 0.0), 4.0)*0.20;\n"
    "    vec3 c = colDiffuse.rgb*(amb + d*vec3(1.05, 1.0, 0.92)) + vec3(spec) + fres*vec3(0.55, 0.65, 0.8);\n"
    "    gl_FragColor = vec4(pow(c, vec3(1.0/2.2)), 1.0);\n"
    "}\n";

static void LoadCar(void)
{
    carShader = LoadShaderFromMemory(CAR_VS, CAR_FS);
    carShader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(carShader, "matModel");
    carShader.locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(carShader, "matNormal");
    viewPosLoc = GetShaderLocation(carShader, "viewPos");

    carBody = LoadModel("assets/car_body.glb");
    carWheel = LoadModel("assets/car_wheel.glb");
    for (int i = 0; i < carBody.materialCount; i++) {
        carBody.materials[i].shader = carShader;
        Color *c = &carBody.materials[i].maps[MATERIAL_MAP_DIFFUSE].color;
        if (c->r > 40 && c->r < 64 && c->g < 12 && c->b < 12) { tailLight = c; tailLightOff = *c; }
    }
    for (int i = 0; i < carWheel.materialCount; i++) carWheel.materials[i].shader = carShader;
}

static void DrawCarShadow(void)
{
    // Soft-ish contact shadow: two stacked translucent quads under the body
    double c = cos(sim.psi), s = sin(sim.psi);
    for (int k = 0; k < 2; k++) {
        double hl = 2.25 + 0.15*k, hw = 0.92 + 0.10*k, cx = -0.1;
        double px[4] = { cx + hl, cx + hl, cx - hl, cx - hl }, py[4] = { hw, -hw, -hw, hw };
        Vector3 p[4];
        for (int i = 0; i < 4; i++)
            p[i] = W(sim.x + px[i]*c - py[i]*s, sim.y + px[i]*s + py[i]*c, 0.04 + 0.002*k);
        Color col = { 0, 0, 0, (unsigned char)(k ? 35 : 55) };
        DrawTriangle3D(p[0], p[2], p[1], col); DrawTriangle3D(p[0], p[3], p[2], col);
    }
}

static void DrawCar(void)
{
    const double WX[4] = { CAR_A, CAR_A, -CAR_B, -CAR_B };
    const double WY[4] = { CAR_TRACK/2, -CAR_TRACK/2, CAR_TRACK/2, -CAR_TRACK/2 };

    DrawCarShadow();
    rlDrawRenderBatchActive();      // flush the shadow before switching shaders

    float vp[3] = { camera.position.x, camera.position.y, camera.position.z };
    SetShaderValue(carShader, viewPosLoc, vp, SHADER_UNIFORM_VEC3);
    if (tailLight) *tailLight = sim.pedal > 0.05 ? (Color){ 255, 20, 12, 255 } : tailLightOff;

    rlPushMatrix();
    rlTranslatef((float)sim.x, 0, (float)-sim.y);
    rlRotatef((float)(sim.psi*RAD2DEG), 0, 1, 0);

    // Local frame: X forward, Y up, Z right (matches the glTF export)
    DrawModel(carBody, (Vector3){ 0, 0, 0 }, 1.0f, WHITE);

    for (int i = 0; i < 4; i++) {
        rlPushMatrix();
        rlTranslatef((float)WX[i], (float)CAR_R, (float)-WY[i]);
        if (i < 2) rlRotatef((float)(sim.steer*RAD2DEG), 0, 1, 0);
        rlRotatef((float)(-sim.spin[i]*RAD2DEG), 0, 0, 1);
        if (WY[i] < 0) rlRotatef(180, 0, 1, 0);    // rim faces outward on the right side too
        DrawModel(carWheel, (Vector3){ 0, 0, 0 }, 1.0f, WHITE);
        rlPopMatrix();
    }
    rlPopMatrix();
}

static void UpdateCam(float dt)
{
    Vector3 car = W(sim.x, sim.y, 0.6);
    // Chase camera: behind, above and slightly right of the car, looking ahead of it
    Vector3 pos = { car.x - 11.5f, 5.0f, car.z + 4.0f };
    Vector3 tgt = { car.x + 8, 0.3f, car.z };
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
    for (int m = (int)fmax(10, ceil((sim.x - 70)/10)*10); m <= sim.x + 70; m += 10) {
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
        double sl = fabs(sim.slip[i]); if (sl > 1) sl = 1;
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

    LoadCar();
    SimReset(&sim, 0, 1, DRIVER_STRAIGHT, SCENARIOS[0].v0Kmh, BridgeAbs, NULL);
    UpdateCam(0);

    emscripten_set_main_loop(Frame, 0, 1);
    CloseWindow();
    return 0;
}
