#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include "utils/log/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
HikRobot::HikRobot(
  double exposure_ms, double gain, const std::string & vid_pid, double fps,
  const std::vector<CameraParam> & extra, bool dump_params)
: exposure_us_(exposure_ms * 1e3), gain_(gain), fps_(fps), queue_(1), handle_(nullptr),
  daemon_quit_(false), vid_(-1), pid_(-1), extra_(extra), dump_params_(dump_params)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's daemon thread started.");

    capture_start();

    // ⭐⭐⭐ W59：**重连要有上限 + 退避** —— 原来是无脑无限「关设备 → USB reset → 重开」
    //
    // 实测后果：`reset_usb()`（`libusb_reset_device`）反复重置 USB，
    //   ⚠️ **相机直接掉线**（`MV_CC_EnumDevices` 返回 0，连续 40+ 次 `Not found camera!`），
    //   而且**红灯不灭**（设备被留在异常状态）。海康相机 USB reset 后回不来是**已知坑**。
    //
    // 新策略：
    //   · USB reset 只在**前 3 次**尝试（reset 是"重锤"，多数情况不需要）
    //   · 之后只做 `capture_start()`（不带 reset）
    //   · 连续失败 `kMaxRetry` 次后**停止重连**，明确告知用户怎么办
    //   · 每次重连**退避**（100ms → 2s），别刷屏
    int retry = 0;
    constexpr int kMaxRetry = 30;

    while (!daemon_quit_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(std::min(100 * (retry + 1), 2000)));

      if (capturing_) { retry = 0; continue; }   // ⭐ 正常取流 → 重置计数

      if (retry >= kMaxRetry) {
        tools::logger()->error(
            "[HikRobot] 连续重连 {} 次仍失败 → 停止重连（避免反复 USB reset 把相机搞掉线）\n"
            "  怎么办：\n"
            "   ① 先确认没有别的程序占着相机（MVS 客户端 / 上一个没退干净的进程）\n"
            "     `pkill -f MVS` 然后重跑\n"
            "   ② 复位: `tools/scripts/camera-reset.sh`（或物理拔插）\n"
            "   ③ 若报 `0x80000007`（无数据）→ 先查 TriggerMode / 是否被 MVS 占用",
            retry);
        break;
      }

      ++retry;
      capture_stop();
      if (retry <= 3) {
        tools::logger()->warn("[HikRobot] 重连第 {}/{} 次（带 USB reset）", retry, kMaxRetry);
        reset_usb();
      } else {
        tools::logger()->warn(
          "[HikRobot] 重连第 {}/{} 次（不再 USB reset —— reset 会把相机搞掉线）", retry,
          kMaxRetry);
      }
      capture_start();
    }

    capture_stop();

    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();

  // ⭐⭐ W56：**析构里显式清理** —— 原来只靠 daemon 线程收尾，
  //   一旦 daemon 在 `capture_start()` 里失败/提前返回，**相机就会被留在 grabbing → 红灯**。
  //   `capture_stop()` 现在是幂等的（handle_ == nullptr 直接返回）。
  capture_stop();

  if (handle_ != nullptr) {   // 兜底：万一 capture_stop 里 DestroyHandle 失败
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
  }
  tools::logger()->info("HikRobot destructed（已 StopGrabbing + CloseDevice + DestroyHandle）");
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  // ⭐⭐ W55：**带超时** —— 相机掉线时不再让主循环无限卡死
  //   （原来 `queue_.pop()` 无限阻塞 → 主循环卡住 → **Ctrl-C 不退出、热键全失效**）
  //   超时后返回空 `img` → 主循环的 `if (img.empty()) break;` 会**干净退出**
  if (!queue_.pop_for(data, std::chrono::milliseconds(500))) {
    img = cv::Mat{};
    timestamp = std::chrono::steady_clock::now();
    return;
  }

  img = data.img;
  timestamp = data.timestamp;
}

void HikRobot::capture_start()
{
  // ⭐⭐ W56：**幂等** —— 若上一轮的 handle 还在（重连路径会走到这里），
  //   先彻底清理，否则会 **句柄泄漏 + 同一台相机被重复 OpenDevice**。
  if (handle_ != nullptr) {
    tools::logger()->debug("[HikRobot] capture_start 前发现残留 handle → 先清理");
    capture_quit_ = true;
    if (capture_thread_.joinable()) capture_thread_.join();
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    if (MV_CC_DestroyHandle(handle_) == MV_OK) handle_ = nullptr;
  }

  capturing_ = false;
  capture_quit_ = false;

  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list;
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  // ⭐⭐⭐ W93：**按 yaml 的 `vid_pid` 选相机** —— 原来永远取 `pDeviceInfo[0]`，
  //   ⚠️ 插了两个海康时**随机选一个**（取决于枚举顺序，不可控）。
  //   ⇒ 现在：遍历所有设备，匹配 VID:PID；没配或匹配不上则退回第一个（并明确告警）。
  int chosen = 0;
  bool matched = false;
  if (vid_ >= 0 && pid_ >= 0 && device_list.nDeviceNum > 1) {
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
      const auto * info = device_list.pDeviceInfo[i];
      if (!info || info->nTLayerType != MV_USB_DEVICE) continue;
      const auto & usb = info->SpecialInfo.stUsb3VInfo;
      if (usb.idVendor == static_cast<unsigned int>(vid_) &&
          usb.idProduct == static_cast<unsigned int>(pid_)) {
        chosen = static_cast<int>(i);
        matched = true;
        break;
      }
    }
    if (!matched) {
      tools::logger()->warn(
        "[HikRobot] ⚠️ 没有设备的 VID:PID 匹配 yaml 的 {:04x}:{:04x} → 退回第一个\n"
        "    （共枚举到 {} 个设备；用 `--dump-camera-params` 或 MVS 确认 VID:PID）",
        vid_, pid_, device_list.nDeviceNum);
    }
  }

  // ⭐ 打印选中的设备（换相机时能立刻看出选没选对）
  {
    const auto * info = device_list.pDeviceInfo[chosen];
    if (info && info->nTLayerType == MV_USB_DEVICE) {
      const auto & usb = info->SpecialInfo.stUsb3VInfo;
      tools::logger()->info(
        "[HikRobot] 选中设备 #{}/{}：{:04x}:{:04x}  型号 {}  序列号 {}", chosen + 1,
        device_list.nDeviceNum, usb.idVendor, usb.idProduct,
        reinterpret_cast<const char *>(usb.chModelName),
        reinterpret_cast<const char *>(usb.chSerialNumber));
    }
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[chosen]);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    return;
  }

  // ⭐⭐⭐ W91：**必须在【覆盖之前】dump** —— 否则打印的是我们刚设的值（如强制 2 ms），
  //   而用户想看的是【MVS 里调好的、相机上电后的当前值】。
  //   ⇒ 这是「MVS 调好 → --dump-camera-params → 抄进 yaml」流程能成立的前提。
  if (dump_params_) dump_camera_params();

  // ⚠️ 默认开【连续自动白平衡】—— 光照/色温变化时白平衡会漂，可能影响颜色判定。
  //   ⭐ 想关掉/锁死：在 yaml 的 `camera_params.enum` 里写 `BalanceWhiteAuto: 0`
  //     （0=Off 1=Once 2=Continuous）—— 见 W90 的通用参数通道。
  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);



  // ⭐⭐⭐ W57：**必须显式关掉触发模式** —— 这是 `0x80000007`（取图超时）的头号嫌疑
  //
  // 改前：代码里**从来没碰过 `TriggerMode`**（`grep -rn TriggerMode` = 0 命中）。
  //   若相机被设成「外部触发」（MVS 客户端改过 / 某些固件默认），
  //   `StartGrabbing` **会成功**，但**永远等不到帧** → `MV_CC_GetImageBuffer` 超时。
  //   实测现象：等 43 秒才报 `0x80000007`，一帧都没有。
  set_enum_value("TriggerMode", MV_TRIGGER_MODE_OFF);          // 0 = Off
  set_enum_value("TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);  // 关掉后此项无影响，保险

  // ⭐ 帧率：**必须检查返回值**（原来 `MV_CC_SetFrameRate(handle_, 150)` 的返回值被丢弃）
  //   150 fps @ 160 万像素 ≈ 233 MB/s，接近 USB3 实际上限 → 可能设置失败
  const double kTargetFps = fps_;   // ⭐ W58：可配（yaml 的 `fps`，默认 30）
  ret = MV_CC_SetFrameRate(handle_, kTargetFps);
  if (ret != MV_OK)
    tools::logger()->warn(
      "MV_CC_SetFrameRate({:.0f}) failed: {:#x} → 用相机默认帧率", kTargetFps, ret);

  // ⭐ 回读实际生效值（下一次出问题时能一眼看出配置对不对）
  {
    MVCC_FLOATVALUE fv{};
    if (MV_CC_GetFloatValue(handle_, "ExposureTime", &fv) == MV_OK)
      tools::logger()->info("[HikRobot] 实际曝光 = {:.0f} µs", static_cast<double>(fv.fCurValue));
    if (MV_CC_GetFloatValue(handle_, "Gain", &fv) == MV_OK)
      tools::logger()->info("[HikRobot] 实际增益 = {:.1f} dB", static_cast<double>(fv.fCurValue));
    if (MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &fv) == MV_OK)
      tools::logger()->info("[HikRobot] 实际帧率 = {:.1f} fps", static_cast<double>(fv.fCurValue));
    MVCC_ENUMVALUE ev{};
    if (MV_CC_GetEnumValue(handle_, "TriggerMode", &ev) == MV_OK)
      tools::logger()->info(
        "[HikRobot] TriggerMode = {}（0=Off 才是自由运行）", ev.nCurValue);

    // ⭐⭐ W58：**打印分辨率 + 估算带宽** —— `0x80000007` 十有八九是带宽不够
    MVCC_INTVALUE iv{};
    int64_t w = 0, h = 0;
    if (MV_CC_GetIntValue(handle_, "Width", &iv) == MV_OK) w = static_cast<int64_t>(iv.nCurValue);
    if (MV_CC_GetIntValue(handle_, "Height", &iv) == MV_OK) h = static_cast<int64_t>(iv.nCurValue);
    MVCC_FLOATVALUE fr{};
    double fps_now = kTargetFps;
    if (MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &fr) == MV_OK)
      fps_now = static_cast<double>(fr.fCurValue);
    if (w > 0 && h > 0) {
      const double mbps = static_cast<double>(w * h) * fps_now / 1048576.0;   // Bayer8 = 1 B/px
      tools::logger()->info(
        "[HikRobot] 分辨率 = {}×{}，帧率 {:.1f} fps → 需带宽 ≈ {:.0f} MB/s", w, h, fps_now,
        mbps);
      if (mbps > 150.0)
        tools::logger()->warn(
            "  >150 MB/s 已接近/超过 USB3 可用带宽（若协商成 USB2 只有 ~40 MB/s）\n"
            "    若报 `0x80000007`（无数据）→ 先把 yaml 里的 `fps` 降到 30 试试");
    }
  }

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's capture thread started.");

    capturing_ = true;

    MV_FRAME_OUT raw;
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;

    while (!capture_quit_) {
      std::this_thread::sleep_for(1ms);

      unsigned int ret;
      unsigned int nMsec = 100;

      // ⭐⭐ W57：**给 SDK 调用计时** —— 实测遇到「传 100ms 超时却阻塞 43 秒」
      //   （海康 SDK 内部 USB 传输超时远长于 nMsec）。计时后能一眼分辨：
      //   是"相机没帧"还是"USB/固件卡住"。
      auto t_get0 = std::chrono::steady_clock::now();
      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      const double get_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_get0)
          .count();
      if (ret != MV_OK) {
        tools::logger()->warn(
          "MV_CC_GetImageBuffer failed: {:#x}（本次阻塞 {:.0f} ms，请求超时 {} ms）", ret, get_ms,
          nMsec);
        // ⭐⭐ W59：`0x80000007 = MV_E_NODATA（无数据）` —— 按概率给出排查方向
        if (ret == 0x80000007)
          tools::logger()->warn(
              "  `0x80000007` = MV_E_NODATA（无数据） —— 相机没给帧。按概率排查：\n"
              "    ① 别的程序占着相机（USB 相机不独占，MVS 客户端能同时打开但抢流）\n"
              "     → `pkill -f MVS` 后重跑\n"
              "    ② `TriggerMode` 是否为 Off（本程序已强制 Off）\n"
              "    ③ USB3 线材 / 端口（用 USB2 线会拿不到流）\n"
              "    ④ 相机固件卡住 → `tools/scripts/camera-reset.sh` 或物理拔插");
        else if (get_ms > 1000.0)
          tools::logger()->warn(
            "  SDK 内部阻塞远长于请求超时 → 多半是 USB 传输卡住 / 相机固件无响应");
        break;
      }
      if (get_ms > 200.0)
        tools::logger()->debug("[HikRobot] GetImageBuffer 耗时 {:.0f} ms（偏慢）", get_ms);

      auto timestamp = std::chrono::steady_clock::now();
      cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

      cvt_param.nWidth = raw.stFrameInfo.nWidth;
      cvt_param.nHeight = raw.stFrameInfo.nHeight;

      cvt_param.pSrcData = raw.pBufAddr;
      cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
      cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

      cvt_param.pDstBuffer = img.data;
      cvt_param.nDstBufferSize = img.total() * img.elemSize();
      cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

      // ret = MV_CC_ConvertPixelType(handle_, &cvt_param);
      const auto & frame_info = raw.stFrameInfo;
      auto pixel_type = frame_info.enPixelType;
      cv::Mat dst_image;
      // ⭐⭐⭐ W93：**不再用 `.at()`** —— 原来像素格式不在表里就抛 `std::out_of_range`，
      //   ⚠️ **直接崩**。换一个输出 `BayerRG10` / `Mono8` / `RGB8_Packed` 的海康相机
      //   就会触发（用户问"换另一个海康会不会出问题"，这是真隐患）。
      //   ⇒ 现在：查表 → 找不到就【明确报错 + 给出解法】，并**跳过该帧**（不崩）。
      //
      //   ⚠️ 注意 OpenCV 的命名：`COLOR_BayerXX2RGB` 的 `XX` 指的是**输入 Bayer 图案**，
      //      输出的字节序仍是 **BGR**（与我们其他地方一致）—— 实测验证过，别改成 2BGR。
      const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};

      auto it = type_map.find(pixel_type);
      if (it == type_map.end()) {
        static bool warned = false;
        if (!warned) {   // ⭐ 只报一次，不刷屏
          warned = true;
          tools::logger()->error(
            "[HikRobot] ⚠️⚠️ 不支持的像素格式 {:#x} → **跳过该帧**（不崩）。\n"
            "   解法（任选其一）：\n"
            "     ① 在 `params/camera.yaml` 的 `camera_params.enum` 里指定 8 位 Bayer：\n"
            "          PixelFormat: {}   # BayerRG8，按你的相机实际排列改\n"
            "     ② 或在 MVS 里把 PixelFormat 改成 BayerRG8/GR8/GB8/BG8 之一\n"
            "   （目前只支持这 4 种 8 位 Bayer；10/12 位或 Mono/RGB/YUV 需扩展本表）",
            static_cast<unsigned int>(pixel_type),
            static_cast<unsigned int>(PixelType_Gvsp_BayerRG8));
        }
        MV_CC_FreeImageBuffer(handle_, &raw);
        continue;
      }
      cv::cvtColor(img, dst_image, it->second);
      img = dst_image;

      queue_.push({img, timestamp});

      ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
        break;
      }
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::capture_stop()
{
  // ⭐⭐⭐ W56：**尽力清理，绝不中途 return**（这是"相机红灯不灭"的根因）
  //
  // 改前：`StopGrabbing` 失败就 `return` → `CloseDevice`/`DestroyHandle` 都不做
  //   → ⚠️ **相机永远留在 grabbing/被占用状态 → 红灯常亮**，且句柄泄漏。
  //   日志实测：`MV_CC_StopGrabbing failed: 0x80000300`（MV_E_CALLORDER）后设备没关。
  //
  // 现在：每一步都 **尝试 + 记录**，无论成败都继续下一步。
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();

  if (handle_ == nullptr) { capturing_ = false; return; }   // ⭐ 没开过就别瞎调

  unsigned int ret;

  ret = MV_CC_StopGrabbing(handle_);
  // ⭐⭐ W59 纠正：`0x80000300` 不是 `MV_E_CALLORDER`（那是 `0x80000003`），
  //   而是 **`MV_E_USB_READ`（读 USB 错误）** —— ⚠️ **这是真问题**，不是"本来就没取流"！
  //   含义：**USB 读取层面出错** → 线材 / 端口 / 供电 / 带宽 / 相机固件。
  if (ret == 0x80000300)
    tools::logger()->warn(
      "MV_CC_StopGrabbing 报 0x80000300 = MV_E_USB_READ（读 USB 错误）\n"
      "  这是USB 通信层面的问题，不是「没在取流」：\n"
      "    检查 ① USB3 线材（必须是 USB3 线，不能用 USB2 线）\n"
      "      ② 换个 USB3 口（直连主板，别用 Hub）\n"
      "      ③ 相机供电是否足够（工业相机耗电大）\n"
      "      ④ 是否被 MVS 客户端等其它程序同时占用（USB 相机不独占，会抢流）");
  else if (ret != MV_OK)
    tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
  else handle_ = nullptr;   // ⭐ 只在这里置空（DestroyHandle 成功）

  capturing_ = false;
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

// ⭐ W90：整型参数（如 Width / Height）
void HikRobot::set_int_value(const std::string & name, int64_t value)
{
  if (!handle_) return;
  const auto ret = MV_CC_SetIntValue(handle_, name.c_str(), value);
  if (ret != MV_OK)
    tools::logger()->warn("MV_CC_SetIntValue(\"{}\", {}) failed: {:#x}", name, value, ret);
  else
    tools::logger()->debug("[HikRobot] set int  {} = {}", name, value);
}

// ⭐⭐ W90：应用 yaml 的 `camera_params`（**在默认参数之后**，可覆盖曝光/增益等）
void HikRobot::apply_extra_params(const std::vector<CameraParam> & extra)
{
  if (extra.empty()) return;
  tools::logger()->info("[HikRobot] 应用 yaml 的 camera_params（{} 项）", extra.size());
  for (const auto & p : extra) {
    switch (p.kind) {
      case CameraParam::Float: set_float_value(p.name, p.fval); break;
      case CameraParam::Enum:  set_enum_value(p.name, static_cast<unsigned int>(p.ival)); break;
      case CameraParam::Int:   set_int_value(p.name, p.ival); break;
    }
  }
}

// ⭐⭐ W90：打印常用参数的当前值 —— 先在 MVS 里调好，再把值抄进 yaml
void HikRobot::dump_camera_params() const
{
  if (!handle_) {
    tools::logger()->error("[HikRobot] --dump-camera-params 需要先打开相机（handle 为空）");
    return;
  }
  tools::logger()->info("[HikRobot] ════ 相机参数当前值（抄进 yaml 的 camera_params 即可）════");

  // 浮点型
  for (const char * n : {"ExposureTime", "Gain", "Gamma", "Sharpness", "Contrast",
                         "Saturation", "AcquisitionFrameRate", "ResultingFrameRate",
                         "BalanceRatio", "BalanceRatioSelector"}) {
    MVCC_FLOATVALUE fv{};
    if (MV_CC_GetFloatValue(handle_, n, &fv) == MV_OK)
      tools::logger()->info("  float {} = {:.3f}   (范围 {:.1f}~{:.1f})",
                            n, static_cast<double>(fv.fCurValue),
                            static_cast<double>(fv.fMin), static_cast<double>(fv.fMax));
  }
  // 枚举型
  for (const char * n : {"PixelFormat", "TriggerMode", "ExposureAuto", "GainAuto",
                         "BalanceWhiteAuto", "AcquisitionMode", "GammaSelector",
                         "ExposureTimeMode", "LineSelector"}) {
    MVCC_ENUMVALUE ev{};
    if (MV_CC_GetEnumValue(handle_, n, &ev) == MV_OK)
      tools::logger()->info("  enum  {} = {}   (支持 {} 种)", n, ev.nCurValue, ev.nSupportedNum);
  }
  // 整型
  for (const char * n : {"Width", "Height", "OffsetX", "OffsetY", "PayloadSize"}) {
    MVCC_INTVALUE iv{};
    if (MV_CC_GetIntValue(handle_, n, &iv) == MV_OK)
      tools::logger()->info("  int   {} = {}   (范围 {}~{})", n, iv.nCurValue, iv.nMin, iv.nMax);
  }
  tools::logger()->info("[HikRobot] ════ 结束（yaml 里按 float/enum/int 分组填写）════");
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io