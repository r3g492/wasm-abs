# Build raylib + the ABS simulator for the web with Emscripten.
#   make        -> docs/ (index.html/.js/.wasm + controller JS), ready for GitHub Pages
#   make serve  -> local preview on http://localhost:8000
#   make test   -> headless run of all scenarios in Node with web/abs_example.js
#                  (ABS_FILE=path/to/your_abs.js make test to try another controller)
#   make clean

CC          := emcc
AR          := emar
RAYLIB_SRC  := external/raylib/src
BUILD       := build
OUT         := docs

RAYLIB_MODULES := rcore rshapes rtextures rtext rmodels raudio
RAYLIB_OBJS    := $(addprefix $(BUILD)/raylib/,$(addsuffix .o,$(RAYLIB_MODULES)))
RAYLIB_LIB     := $(BUILD)/libraylib.web.a

SIM_SRC   := src/sim.c src/bridge.c
APP_SRC   := src/main.c $(SIM_SRC)
HEADERS   := src/sim.h src/bridge.h
WEB_JS    := $(OUT)/abs_runtime.js $(OUT)/abs_example.js

RAYLIB_CFLAGS := -std=gnu99 -Os -Wall -DPLATFORM_WEB -DGRAPHICS_API_OPENGL_ES2
APP_CFLAGS    := -std=gnu99 -O2 -Wall -I$(RAYLIB_SRC) -DPLATFORM_WEB
ASSETS        := $(wildcard assets/*.glb)
LDFLAGS       := -sUSE_GLFW=3 -sALLOW_MEMORY_GROWTH=1 -sEXPORTED_RUNTIME_METHODS=ccall \
                 --shell-file web/shell.html --preload-file assets

.PHONY: all clean serve test
all: $(OUT)/index.html $(WEB_JS)

$(BUILD)/raylib/%.o: $(RAYLIB_SRC)/%.c | $(BUILD)/raylib
	$(CC) $(RAYLIB_CFLAGS) -c $< -o $@

$(RAYLIB_LIB): $(RAYLIB_OBJS)
	$(AR) rcs $@ $^

$(OUT)/index.html: $(APP_SRC) $(HEADERS) $(RAYLIB_LIB) web/shell.html $(ASSETS) | $(OUT)
	$(CC) $(APP_CFLAGS) $(APP_SRC) $(RAYLIB_LIB) $(LDFLAGS) -o $@

$(OUT)/%.js: web/%.js | $(OUT)
	cp $< $@

$(BUILD)/headless.js: tests/headless.c tests/node_pre.js $(SIM_SRC) $(HEADERS) | $(BUILD)
	$(CC) -O2 -Wall -sENVIRONMENT=node --pre-js tests/node_pre.js tests/headless.c $(SIM_SRC) -o $@

test: $(BUILD)/headless.js
	node $(BUILD)/headless.js 0
	node $(BUILD)/headless.js 1

$(BUILD) $(BUILD)/raylib $(OUT):
	mkdir -p $@

serve: all
	python3 -m http.server 8000 -d $(OUT)

clean:
	rm -rf $(BUILD) $(OUT)/index.html $(OUT)/index.js $(OUT)/index.wasm $(OUT)/index.data $(WEB_JS)
