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

.PHONY: linux check clean

# --- Linux: shared object ELF ---
linux: libaterkia.so

libaterkia.so: $(SRC_C) $(SRC_CPP) $(wildcard include/*.h)
	$(CXX) -shared -fPIC $(CFLAGS) $(CPPFLAGS) $(SRC_C) $(SRC_CPP) -o $@ $(LM)

# --- Verifikasi ketat tanpa menghasilkan file ---
check:
	$(CC) -fsyntax-only $(CFLAGS) $(CPPFLAGS) src/*.c
	$(CXX) -fsyntax-only $(CFLAGS) $(CPPFLAGS) src/gamepad_input.cpp
	@echo "check: lolos -Wall -Wextra -Werror"

clean:
	rm -f libaterkia.so
