// Simple raylib 3D scene: orbiting camera over a grid with a few shapes.
#include "raylib.h"
#include <math.h>

#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#endif

static const int screenWidth = 960;
static const int screenHeight = 540;

static Camera3D camera = { 0 };

static void UpdateDrawFrame(void)
{
    UpdateCamera(&camera, CAMERA_ORBITAL);

    float t = (float)GetTime();
    float bob = sinf(t*2.0f)*0.5f;

    BeginDrawing();
        ClearBackground((Color){ 24, 26, 33, 255 });

        BeginMode3D(camera);
            DrawPlane((Vector3){ 0, 0, 0 }, (Vector2){ 20, 20 }, (Color){ 40, 44, 56, 255 });
            DrawGrid(20, 1.0f);

            DrawCube((Vector3){ 0, 1.0f, 0 }, 2, 2, 2, (Color){ 230, 41, 55, 255 });
            DrawCubeWires((Vector3){ 0, 1.0f, 0 }, 2, 2, 2, MAROON);

            DrawSphere((Vector3){ 4, 1.5f + bob, 0 }, 1.0f, GOLD);
            DrawSphereWires((Vector3){ 4, 1.5f + bob, 0 }, 1.0f, 12, 12, ORANGE);

            DrawCylinder((Vector3){ -4, 0, 0 }, 0.0f, 1.2f, 3.0f, 16, SKYBLUE);
            DrawCylinderWires((Vector3){ -4, 0, 0 }, 0.0f, 1.2f, 3.0f, 16, DARKBLUE);

            for (int i = 0; i < 8; i++)
            {
                float a = t*0.6f + i*(2*PI/8);
                Vector3 p = { cosf(a)*7.0f, 0.5f, sinf(a)*7.0f };
                DrawCube(p, 0.8f, 0.8f, 0.8f, ColorFromHSV(i*45.0f, 0.7f, 0.9f));
            }
        EndMode3D();

        DrawText("raylib " RAYLIB_VERSION " + WebAssembly", 12, 12, 20, RAYWHITE);
        DrawFPS(screenWidth - 90, 12);
    EndDrawing();
}

int main(void)
{
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "wasm-abs");

    camera.position = (Vector3){ 10.0f, 7.0f, 10.0f };
    camera.target = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

#if defined(PLATFORM_WEB)
    emscripten_set_main_loop(UpdateDrawFrame, 0, 1);
#else
    SetTargetFPS(60);
    while (!WindowShouldClose()) UpdateDrawFrame();
#endif

    CloseWindow();
    return 0;
}
