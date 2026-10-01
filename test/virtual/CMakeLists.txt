# test/virtual/ —— 离线回放（doc 09 §14.8.4：★ 最重要的调参手段）
add_executable(test_video_replay test_video_replay.cpp)
target_link_libraries(test_video_replay hzmir_io hzmir_utils ${OpenCV_LIBS})
add_test(NAME test_video_replay COMMAND test_video_replay)
