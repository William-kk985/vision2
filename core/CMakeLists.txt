# core/ —— 契约层（types）+ 功能组（auto_aim / auto_buff，Stage 3 起加入）
add_library(hzmir_core STATIC
  types.cpp
)
target_link_libraries(hzmir_core PUBLIC ${OpenCV_LIBS} Eigen3::Eigen)

add_subdirectory(auto_aim)
add_subdirectory(auto_buff)
