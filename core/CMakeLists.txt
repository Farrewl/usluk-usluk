# core — CMake inti C/C++ ASV
#
# Modul berjalan: nav_math (C) — port app/geo.py, sudah tervalidasi
# terhadap Python lewat tests/test_core_nav_math.py.
# Modul menyusul: mavlink_bridge.c, fuzzy.c, state_machine.c (C),
#                 camera.cpp, vision_detector.cpp (C++).
# Aturan: add_library hanya menunjuk file yang sudah benar-benar ada.
cmake_minimum_required(VERSION 3.16)
project(asv_core C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(asv_nav_math STATIC src/nav_math.c)
target_include_directories(asv_nav_math PUBLIC include)

enable_testing()
add_subdirectory(tests)