# ABS Braking Lab

A vehicle braking simulator built with [raylib](https://github.com/raysan5/raylib) and compiled to WebAssembly.
Write your own ABS controller in JavaScript, right in the page, and test it on five road surfaces.

**Live:** https://r3g492.github.io/wasm-abs/

## Scenarios

| # | Scene | Speed | Left wheels | Right wheels |
|---|-------|-------|-------------|--------------|
| 1 | Dry asphalt | 100 km/h | dry | dry |
| 2 | Rain | 100 km/h | wet | wet |
| 3 | Split-mu: water right | 80 km/h | dry | standing water |
| 4 | Ice | 50 km/h | ice | ice |
| 5 | Split-mu: ice right | 80 km/h | dry | ice |

The driver cruises to the brake line, then stamps on the pedal (160 bar in ~0.15 s). The driver either
holds the wheel straight or counter-steers like a human would.

## Writing an ABS controller

Your code defines `function abs(s, mem)`. It runs every 5 ms (200 Hz) while braking and returns
`[FL, FR, RL, RR]` brake pressure commands in bar. The valves can only *reduce* pressure: commands are
clamped to `0..s.masterPressure`, and the hydraulics follow them with realistic build/dump rates.

| Input | Meaning |
|-------|---------|
| `s.wheelSpeed[i]` | wheel speed ω·R (m/s), i = 0 FL, 1 FR, 2 RL, 3 RR |
| `s.wheelPressure[i]` | measured wheel pressure (bar) |
| `s.masterPressure`, `s.pedal` | driver's demand |
| `s.ax`, `s.ay`, `s.yawRate`, `s.steer` | body sensors |
| `s.t`, `s.dt` | time since brake onset, controller period |
| `s.truth.vx` | true vehicle speed (a real ECU doesn't have this) |
| `mem` | your persistent state, cleared each run |

If your code throws or returns something invalid, the run continues with unmodulated brakes and the
error is shown under the editor. Your code is saved in the browser's local storage.

The example controller ([web/abs_example.js](web/abs_example.js)) uses a reference-speed estimate,
three-phase dump/hold/build control, rear select-low, yaw-moment build-up delay and yaw-rate feedback.

## Model

- 3-DOF body (longitudinal, lateral, yaw) + 4 wheel spin DOFs, 2 kHz fixed step
- Quasi-static longitudinal and lateral load transfer
- Burckhardt friction curves per surface; combined slip via the slip-velocity vector
- Brake hydraulics: first-order lag with build (1500 bar/s) and dump (3000 bar/s) rate limits
- Wheel spin integrated with a linearised implicit step and Coulomb brake locking, so it stays stable down to a stop

## Layout

- `src/sim.c`, `src/sim.h`: simulation core (no rendering)
- `src/main.c`: raylib renderer, HUD, plots, page bridge exports
- `src/bridge.c`: calls the JS controller from C
- `web/shell.html`: page UI; `web/abs_runtime.js`: compiles/validates user code; `web/abs_example.js`: example controller
- `tests/`: headless run of every scenario in Node
- `tools/make_car.py`: Blender script that builds the sedan and exports `assets/car_body.glb` and
  `assets/car_wheel.glb` (`make car`, or run it from Blender's Python console). Its dimensions match `src/sim.h`.
- `external/raylib`: raylib 6.2 (git submodule)
- `docs/`: build output served by GitHub Pages

## Build

Requires [Emscripten](https://emscripten.org) (`brew install emscripten`).

```sh
git clone --recurse-submodules https://github.com/r3g492/wasm-abs.git
cd wasm-abs
make            # builds docs/
make serve      # preview at http://localhost:8000
make test       # all scenarios, ABS off vs example, headless in Node
ABS_FILE=my_abs.js make test   # same with your controller
```

Commit the updated `docs/` and push to publish.
