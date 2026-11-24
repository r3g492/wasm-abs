# Build raylib + the scene for the web with Emscripten.
#   make        -> docs/index.html (+ .js, .wasm), ready for GitHub Pages
#   make serve  -> local preview on http://localhost:8000
#   make clean

CC          := emcc
AR          := emar
RAYLIB_SRC  := external/raylib/src
BUILD       := build
OUT         := docs

RAYLIB_MODULES := rcore rshapes rtextures rtext rmodels raudio
RAYLIB_OBJS    := $(addprefix $(BUILD)/raylib/,$(addsuffix .o,$(RAYLIB_MODULES)))
RAYLIB_LIB     := $(BUILD)/libraylib.web.a

RAYLIB_CFLAGS := -std=gnu99 -Os -Wall -DPLATFORM_WEB -DGRAPHICS_API_OPENGL_ES2
APP_CFLAGS    := -std=c99 -Os -Wall -I$(RAYLIB_SRC) -DPLATFORM_WEB
LDFLAGS       := -sUSE_GLFW=3 -sALLOW_MEMORY_GROWTH=1 -sEXPORTED_RUNTIME_METHODS=ccall \
                 --shell-file web/shell.html

.PHONY: all clean serve
all: $(OUT)/index.html

$(BUILD)/raylib/%.o: $(RAYLIB_SRC)/%.c | $(BUILD)/raylib
	$(CC) $(RAYLIB_CFLAGS) -c $< -o $@

$(RAYLIB_LIB): $(RAYLIB_OBJS)
	$(AR) rcs $@ $^

$(OUT)/index.html: src/main.c $(RAYLIB_LIB) web/shell.html | $(OUT)
	$(CC) $(APP_CFLAGS) src/main.c $(RAYLIB_LIB) $(LDFLAGS) -o $@

$(BUILD)/raylib $(OUT):
	mkdir -p $@

serve: all
	python3 -m http.server 8000 -d $(OUT)

clean:
	rm -rf $(BUILD) $(OUT)/index.html $(OUT)/index.js $(OUT)/index.wasm
