add_executable(test_types test_types.cpp)
target_link_libraries(test_types hzmir_core hzmir_utils)
add_test(NAME test_types COMMAND test_types)

add_executable(test_filter test_filter.cpp)
target_link_libraries(test_filter hzmir_auto_aim hzmir_utils)
add_test(NAME test_filter COMMAND test_filter)

add_executable(test_debug test_debug.cpp)
target_link_libraries(test_debug hzmir_core hzmir_utils)
add_test(NAME test_debug COMMAND test_debug)

add_executable(test_ekf_nis test_ekf_nis.cpp)
target_link_libraries(test_ekf_nis hzmir_auto_aim hzmir_utils)
add_test(NAME test_ekf_nis COMMAND test_ekf_nis)

add_executable(test_queue test_queue.cpp)
target_link_libraries(test_queue hzmir_auto_aim hzmir_utils)
add_test(NAME test_queue COMMAND test_queue)

add_executable(test_ballistic test_ballistic.cpp)
target_link_libraries(test_ballistic hzmir_auto_aim hzmir_utils)
add_test(NAME test_ballistic COMMAND test_ballistic)

add_executable(test_ballistic_table test_ballistic_table.cpp)
target_link_libraries(test_ballistic_table hzmir_auto_aim hzmir_utils)
add_test(NAME test_ballistic_table COMMAND test_ballistic_table)

add_executable(test_ballistic_tongji test_ballistic_tongji.cpp)
target_link_libraries(test_ballistic_tongji hzmir_auto_aim hzmir_utils)
add_test(NAME test_ballistic_tongji COMMAND test_ballistic_tongji)

add_executable(test_tgd test_tgd.cpp)
target_link_libraries(test_tgd hzmir_utils ${OpenCV_LIBS})
add_test(NAME test_tgd COMMAND test_tgd)

add_executable(test_temporal test_temporal.cpp)
target_link_libraries(test_temporal hzmir_auto_aim hzmir_utils ${OpenCV_LIBS})
add_test(NAME test_temporal COMMAND test_temporal)

add_executable(test_hotkeys test_hotkeys.cpp)
target_link_libraries(test_hotkeys hzmir_utils)
add_test(NAME test_hotkeys COMMAND test_hotkeys)

add_executable(test_l3 test_l3.cpp)
target_link_libraries(test_l3 hzmir_utils ${OpenCV_LIBS})
add_test(NAME test_l3 COMMAND test_l3)

add_executable(test_rls test_rls.cpp)
target_link_libraries(test_rls hzmir_utils)
add_test(NAME test_rls COMMAND test_rls)

add_executable(test_hot_reload test_hot_reload.cpp)
target_link_libraries(test_hot_reload hzmir_utils)
add_test(NAME test_hot_reload COMMAND test_hot_reload)

add_executable(test_target_plugins test_target_plugins.cpp)
target_link_libraries(test_target_plugins hzmir_auto_aim hzmir_utils)
add_test(NAME test_target_plugins COMMAND test_target_plugins)

add_executable(test_chi2 test_chi2.cpp)
target_link_libraries(test_chi2 hzmir_utils)
add_test(NAME test_chi2 COMMAND test_chi2)

add_executable(test_nav_bridge test_nav_bridge.cpp)
target_link_libraries(test_nav_bridge hzmir_auto_aim hzmir_utils)
add_test(NAME test_nav_bridge COMMAND test_nav_bridge)

# ⭐⭐⭐ W36：构建类型自检 —— 保证 assert 真的会触发
#   （本项目已三次栽在「assert 在 Release 是空操作」；W35 又因漏传
#    -DCMAKE_BUILD_TYPE=Debug 而让 exp/hzmir_dbg 静默变成 Release）
add_executable(test_assert_live test_assert_live.cpp)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_definitions(test_assert_live PRIVATE HZMIR_EXPECTED_DEBUG=1)
else()
  target_compile_definitions(test_assert_live PRIVATE HZMIR_EXPECTED_DEBUG=0)
endif()
add_test(NAME test_assert_live COMMAND test_assert_live)

add_executable(test_target_copy test_target_copy.cpp)
target_link_libraries(test_target_copy hzmir_auto_aim hzmir_utils)
add_test(NAME test_target_copy COMMAND test_target_copy)
