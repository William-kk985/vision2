# src/ —— 应用层：各兵种独立完整程序（doc 09 §7.2：13 main -> 4 兵种程序）

# ── 步兵（基形态来自同济 standard_mpc.cpp，即 MPC + plan 线程）──
add_executable(infantry infantry.cpp)
target_link_libraries(infantry hzmir_auto_aim hzmir_auto_buff hzmir_io hzmir_utils ${OpenCV_LIBS})

# ── 英雄（42mm + 绑核 + 优先级；⭐ 参考同济 auto_aim_hero 分支）──
add_executable(hero hero.cpp)
target_link_libraries(hero hzmir_auto_aim hzmir_auto_buff hzmir_io hzmir_utils ${OpenCV_LIBS})

# ── 无人机 ──
add_executable(uav uav.cpp)
target_link_libraries(uav hzmir_auto_aim hzmir_auto_buff hzmir_io hzmir_utils ${OpenCV_LIBS})

# ── 哨兵 ──
# ⚠️ 同济的 src/sentry.cpp 依赖 omniperception（4 相机全向感知）—— 已归档。
#    按 doc 09 §7.5，单相机哨兵应改为「plain auto_aim + 捞出的横切能力（无敌过滤/优先级/射击过滤）」，
#    见 doc 10 W8/W14。**暂不编入**，避免假装能跑。
# ⭐ W33：单相机哨兵 —— 不再依赖已归档的 omniperception
add_executable(sentry sentry.cpp)
target_link_libraries(sentry hzmir_auto_aim hzmir_auto_buff hzmir_io hzmir_utils ${OpenCV_LIBS})
