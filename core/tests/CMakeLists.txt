# core/tests — unit test C (dijalankan ctest / langsung gcc)
add_executable(test_nav_math test_nav_math.c)
target_link_libraries(test_nav_math PRIVATE asv_nav_math m)
add_test(NAME nav_math COMMAND test_nav_math)