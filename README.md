# wasm-abs

A simple [raylib](https://github.com/raysan5/raylib) 3D scene compiled to WebAssembly and hosted on GitHub Pages.

**Live:** https://r3g492.github.io/wasm-abs/

## Layout

- `src/main.c` — the scene
- `web/shell.html` — HTML template for the Emscripten output
- `external/raylib` — raylib 6.2 (git submodule, pinned to tag `6.2`)
- `docs/` — build output, served by GitHub Pages

## Build

Requires [Emscripten](https://emscripten.org) (`brew install emscripten`).

```sh
git clone --recurse-submodules https://github.com/r3g492/wasm-abs.git
cd wasm-abs
make          # builds docs/index.{html,js,wasm}
make serve    # preview at http://localhost:8000
```

Commit the updated `docs/` and push to publish.
