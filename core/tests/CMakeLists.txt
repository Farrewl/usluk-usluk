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

add_executable(test_mavlink_bridge test_mavlink_bridge.c)
target_link_libraries(test_mavlink_bridge PRIVATE asv_mavlink_bridge)
add_test(NAME mavlink_bridge COMMAND test_mavlink_bridge)

add_executable(test_rc_decode test_rc_decode.c)
target_link_libraries(test_rc_decode PRIVATE asv_rc_decode m)
add_test(NAME rc_decode COMMAND test_rc_decode)

add_executable(test_manual_control test_manual_control.c)
target_link_libraries(test_manual_control PRIVATE asv_manual_control m)
add_test(NAME manual_control COMMAND test_manual_control)

add_executable(test_mode_manager test_mode_manager.c)
target_link_libraries(test_mode_manager PRIVATE asv_mode_manager)
add_test(NAME mode_manager COMMAND test_mode_manager)