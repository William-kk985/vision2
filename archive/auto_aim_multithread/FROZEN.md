# auto_aim_multithread —— 已冻结（2026-09-30）

**来源**：同济 `tasks/auto_aim/multithread/`（`multiThreadDetector` + `CommandGener`）

**冻结原因**（见 `docs/autoaim_compare/09` §7.3.2）：

1. ⚠️ **队列容量 16**（`mt_detector.hpp:37`）→ 检测慢于取图时堆积，**延迟上界 = 16 × 帧周期 = 160 ms**
   —— 违反「容量 1」原则（`07` Q12.6）
2. ⚠️ **`ov::Tensor` 悬垂指针**（`mt_detector.cpp:58`，指向局部 `input.data`）
3. ⭐ **有了 plan 线程后冗余**：检测本来就跑在 image_thread 上，不在 plan 线程上
4. header 自述 `//暂时不支持yolov8`

**复活步骤**（若将来要做「单线程 + 检测异步化」）：
 1. 搬到 `core/auto_aim/detector/yolo_async.cpp`
 2. **队列容量改 1**
 3. 修 `ov::Tensor` 生命周期（让 preprocess 的 `cv::Mat` 比 InferRequest 活得久）
 4. 在 `detector/` 注册为 `IArmorDetector` 的一个实现

**已知缺陷**：见上 1/2/4

**验证记录**：未验证（本仓库只在 `archive/` 保留源码，**不参与构建**）
