# ============================================================
# core/Makefile — build library inti Rust (libaterkia_rust.so)
#
# Pemakaian:
#   make -C core rust     # -> core/libaterkia_rust.so
#   make -C core check    # cargo check
#   make -C core clean
#
# Loader Python: app/aterkia_core.py memuat core/libaterkia_rust.so.
# ============================================================

.PHONY: rust check clean

rust:
	cargo build --release
	cp target/release/libaterkia_core.so libaterkia_rust.so
	@echo "Rust core build: OK -> libaterkia_rust.so"

check:
	cargo check

clean:
	cargo clean
	rm -f libaterkia_rust.so
