# ============================================================
# core/Makefile — build library C lintas-OS (1 perintah per OS).
#
# Pemakaian:
#   make -C core linux     # Linux  -> core/libaterkia.so
#   make -C core windows   # Windows MinGW (native) -> core/aterkia_core.dll
#   make -C core cross     # Linux  -> .dll via MinGW cross (x86_64-w64-mingw32-gcc)
#   make -C core check     # kompilasi ketat -Wall -Wextra -Werror (tanpa output)
#   make -C core clean
#
# Catatan:
# - Sumber yang sama untuk kedua OS; cabang Windows sudah di-guard
#   (#ifdef _WIN32) di mavlink_bridge.c dan gamepad_input.cpp.
# - gamepad_input.cpp TIDAK masuk library ini (hanya via CMake static
#   lib, lihat CMakeLists.txt) — di Windows ia berupa stub kompatibel.
# - Loader Python: app/aterkia_core.py memilih nama lib per OS.
# ============================================================

CC        = gcc
CXX       = g++
CFLAGS    ?= -O2 -Wall -Wextra -Werror
CPPFLAGS  += -Iinclude
SRC_C     := $(wildcard src/*.c)
LM        := -lm

# Compiler MinGW: native Windows pakai "gcc" bawaan MinGW;
# cross dari Linux pakai x86_64-w64-mingw32-gcc (override: CC_WIN=...)
CC_WIN    ?= x86_64-w64-mingw32-gcc

.PHONY: linux windows cross check clean

# --- Linux: shared object ELF (dipakai app/aterkia_core.py) ---
linux: libaterkia.so

libaterkia.so: $(SRC_C) $(wildcard include/*.h)
	$(CC) -shared -fPIC $(CFLAGS) $(CPPFLAGS) $(SRC_C) -o $@ $(LM)

# --- Windows native (MinGW shell): DLL (dipakai app/aterkia_core.py) ---
windows:
	$(CC_WIN) -shared $(CFLAGS) $(CPPFLAGS) $(SRC_C) -o aterkia_core.dll $(LM)

# --- Cross-build .dll dari Linux (butuh mingw-w64 terpasang) ---
cross:
	$(CC_WIN) -shared $(CFLAGS) $(CPPFLAGS) $(SRC_C) -o aterkia_core.dll $(LM)

# --- Verifikasi ketat tanpa menghasilkan file (jalur POSIX + jalur
#     non-POSIX/Windows dipaksa lewat -U__unix__ supaya #else ikut dicek) ---
check:
	$(CC) -fsyntax-only $(CFLAGS) $(CPPFLAGS) src/*.c
	$(CC) -fsyntax-only $(CFLAGS) -U__unix__ -U__linux__ $(CPPFLAGS) src/mavlink_bridge.c
	$(CXX) -fsyntax-only $(CFLAGS) $(CPPFLAGS) src/gamepad_input.cpp
	$(CXX) -fsyntax-only $(CFLAGS) -D_WIN32 $(CPPFLAGS) src/gamepad_input.cpp
	@echo "check: semua jalur (POSIX + Windows) lolos -Wall -Wextra -Werror"

clean:
	rm -f libaterkia.so aterkia_core.dll
