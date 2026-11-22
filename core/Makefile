# ============================================================
# core/Makefile — build library C (Linux).
#
# Pemakaian:
#   make -C core linux     # -> core/libaterkia.so
#   make -C core check     # kompilasi ketat -Wall -Wextra -Werror (tanpa output)
#   make -C core clean
#
# Catatan:
# - gamepad_input.cpp TIDAK masuk library ini (dipakai via CMake).
# - Loader Python: app/aterkia_core.py memuat core/libaterkia.so.
# ============================================================

CXX       = g++
CFLAGS    ?= -O2 -Wall -Wextra -Werror
CPPFLAGS  += -Iinclude
SRC_C     := $(wildcard src/*.c)
SRC_CPP   := src/gamepad_input.cpp
LM        := -lm

.PHONY: linux windows rust check clean

# --- Rust: core (via cargo) ---
rust:
	cd ../rust/asv-core && cargo build --release
	cp ../rust/asv-core/target/release/libaterkia_core.so libaterkia_rust.so
	@echo "Rust core build: OK -> libaterkia_rust.so"


# --- Linux: shared object ELF ---
linux: libaterkia.so

libaterkia.so: $(SRC_C) $(SRC_CPP) $(wildcard include/*.h)
	$(CXX) -shared -fPIC $(CFLAGS) $(CPPFLAGS) $(SRC_C) $(SRC_CPP) -o $@ $(LM)

# --- Windows (MSYS2/MinGW): satu sumber C, output DLL ---
.PHONY: windows
windows: libaterkia.dll

libaterkia.dll: $(SRC_C) $(SRC_CPP) $(wildcard include/*.h)
	$(CXX) -shared -fPIC $(CFLAGS) $(CPPFLAGS) $(SRC_C) $(SRC_CPP) -o $@ $(LM)

# --- Verifikasi ketat tanpa menghasilkan file ---
check:
	$(CC) -fsyntax-only $(CFLAGS) $(CPPFLAGS) src/*.c
	$(CXX) -fsyntax-only $(CFLAGS) $(CPPFLAGS) src/gamepad_input.cpp
	@echo "check: lolos -Wall -Wextra -Werror"

clean:
	rm -f libaterkia.so libaterkia.dll
