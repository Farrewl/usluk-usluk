# core — kerangka modul C/C++ (Tahap 2)
#
# Tahap 1 hanya menyiapkan kerangka proyek. CMake ini akan diisi bersamaan
# dengan modul pertama yang dikerjakan di Tahap 2 (lihat docs/ARCHITECTURE.md
# dan core/README.md) — modul wajib hadir bersama `add_library`-nya, tidak
# boleh add_library menunjuk file yang belum ditulis.
#
# Rencana:
#   add_library(asv_nav_math STATIC src/nav_math.c)
#   target_include_directories(asv_nav_math PUBLIC include)
#   ... dst (mavlink_bridge.c, fuzzy.c, state_machine.c, camera.cpp, ...)
cmake_minimum_required(VERSION 3.16)
project(asv_core C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)