# core — CMake inti C/C++ ASV
#
# Modul berjalan: nav_math (C) — port app/geo.py; fuzzy (C) — port
# app/fuzzy.py (Sugeno singleton gate & docking); state_machine (C) — port
# app/state_machine.py (16 state misi); mavlink_bridge (C) — framing MAVLink
# v1 + serial (byte-identik pymavlink). Semua tervalidasi identik terhadap
# Python lewat tests/test_core_nav_math.py, test_core_fuzzy.py,
# test_core_state_machine.py & test_core_mavlink_bridge.py.
# Modul menyusul: camera.cpp, vision_detector.cpp (C++).
# Aturan: add_library hanya menunjuk file yang sudah benar-benar ada.
cmake_minimum_required(VERSION 3.16)
project(asv_core C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(asv_nav_math STATIC src/nav_math.c)
target_include_directories(asv_nav_math PUBLIC include)

add_library(asv_fuzzy STATIC src/fuzzy.c)
target_include_directories(asv_fuzzy PUBLIC include)

add_library(asv_state_machine STATIC src/state_machine.c)
target_include_directories(asv_state_machine PUBLIC include)

add_library(asv_mavlink_bridge STATIC src/mavlink_bridge.c)
target_include_directories(asv_mavlink_bridge PUBLIC include)

add_library(asv_control_filters STATIC src/control_filters.c)
target_include_directories(asv_control_filters PUBLIC include)

add_library(asv_thruster_mixer STATIC src/thruster_mixer.c)
target_include_directories(asv_thruster_mixer PUBLIC include)

add_library(asv_ecu_link STATIC src/ecu_link.c)
target_include_directories(asv_ecu_link PUBLIC include)

add_library(asv_arbitrator STATIC src/arbitrator.c)
target_include_directories(asv_arbitrator PUBLIC include)

enable_testing()
add_subdirectory(tests)