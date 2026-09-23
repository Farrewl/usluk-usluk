# core/tests — unit test C (dijalankan ctest / langsung gcc)
add_executable(test_nav_math test_nav_math.c)
target_link_libraries(test_nav_math PRIVATE asv_nav_math m)
add_test(NAME nav_math COMMAND test_nav_math)

add_executable(test_fuzzy test_fuzzy.c)
target_link_libraries(test_fuzzy PRIVATE asv_fuzzy m)
add_test(NAME fuzzy COMMAND test_fuzzy)

add_executable(test_state_machine test_state_machine.c)
target_link_libraries(test_state_machine PRIVATE asv_state_machine)
add_test(NAME state_machine COMMAND test_state_machine)