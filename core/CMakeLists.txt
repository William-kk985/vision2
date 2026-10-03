# core/ —— 契约层（types）+ 功能组（auto_aim / auto_buff，Stage 3 起加入）
add_library(hzmir_core STATIC
  types.cpp
)
target_link_libraries(hzmir_core PUBLIC ${OpenCV_LIBS} Eigen3::Eigen)

# ⭐⭐⭐ W104：**实验源码**（`core/debug.hpp` §二 开启的实验）
#   ⚠️ 只加子目录【之后】才生效（实验可能要 include 角色头）—— 见文件末尾
add_subdirectory(auto_aim)
add_subdirectory(auto_buff)

# ⭐⭐⭐ W104：把【已开启的实验】挂进 `hzmir_core`
#   `HZMIR_EXP_SRCS` / `HZMIR_EXP_DEFS` 由顶层 `CMakeLists.txt` 从
#   `core/debug.hpp` 自动解析出来（定义了哪个 `HZMIR_EXP_*` 就有哪些）。
#   ⚠️ 放在最后：实验实现常要 include 角色头，要在子目录都加完之后。
if(HZMIR_EXP_SRCS)
  target_sources(hzmir_core PRIVATE ${HZMIR_EXP_SRCS})
  target_include_directories(hzmir_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)   # 让 exp_*.cpp 能 include 项目头
endif()
