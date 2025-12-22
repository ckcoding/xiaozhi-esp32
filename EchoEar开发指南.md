# EchoEar (喵伴) 开发与测试指南

本文档将指导您如何针对 **EchoEar (喵伴)** 开发板进行功能开发、编译、烧录和测试。

## 1. 环境准备

确保您已经按照 [开发环境搭建指南](开发环境搭建指南.md) 完成了基础环境的安装。

### 关键信息
- **项目路径**: `main/boards/echoear/`
- **核心代码**: `EchoEar.cc`
- **目标芯片**: ESP32-S3

---

## 2. 编译配置

针对 EchoEar 有两种配置方式，**推荐使用方式 A**，因为它会自动处理复杂的资源配置。

### 方式 A：使用自动化脚本（推荐）

这是最简单的方法，会自动应用 `config.json` 中的所有推荐设置（包括表情包资源配置）。

```bash
# 在项目根目录下执行
python scripts/release.py echoear
```

脚本执行完毕后，固件会生成在 `build/` 目录下。

### 方式 B：手动配置 (idf.py)

如果您需要深度定制或调试，可以使用手动步骤：

1.  **设置目标芯片**
    ```bash
    idf.py set-target esp32s3
    ```

2.  **进入菜单配置**
    ```bash
    idf.py menuconfig
    ```

3.  **配置开发板**
    - `Xiaozhi Assistant` -> `Board Type` -> 选择 `EchoEar`

4.  **配置 UI 风格与资源 (重要)**
    EchoEar 默认推荐使用 **表情动画风格 (Emote)**，需要配置专门的资源文件：
    - `Xiaozhi Assistant` -> `Select display style` -> 选择 `Emote animation style`
    - `Xiaozhi Assistant` -> 勾选 `Flash Assets` -> `Flash Custom Assets`
    - `Xiaozhi Assistant` -> `Custom Assets File` 填入 URL:
      `https://dl.espressif.com/AE/wn9_nihaoxiaozhi_tts-font_puhui_common_20_4-echoear.bin`
    - 分区表确保选择 `partitions/v2/16m.csv` (通常脚本会自动处理，手动需检查 `Partition Table` 设置)

5.  **保存并退出** (按 `S` 保存，`Q` 退出)

6.  **编译**
    ```bash
    idf.py build
    ```

---

## 3. 烧录与日志监控

### 连接设备
1. 使用 USB 数据线连接 EchoEar 到电脑。
2. **⚠️ 注意**: 确保打开开发板上的 **电源开关**。

### 烧录并监控
执行以下命令（将 `/dev/ttyUSB0` 替换为您的实际端口，Windows下通常为 `COMx`）：

```bash
# 烧录并打开串口监视器
idf.py -p /dev/ttyUSB0 flash monitor
```

如果遇到权限问题（Linux）：
```bash
sudo chmod 777 /dev/ttyUSB0
```

---

## 4. 功能开发与测试示例

EchoEar 的核心逻辑位于 `main/boards/echoear/EchoEar.cc`。以下是几个常见的开发测试场景：

### 场景一：测试触摸功能

EchoEar 使用 `CST816S` 触摸芯片。您可以通过修改 `touch_event_task` 来调试触摸坐标。

**修改位置**: `main/boards/echoear/EchoEar.cc` 约 474 行

```cpp
// 原始代码
ESP_LOGI(TAG, "Touch event, TP_PIN_NUM_INT: %d", gpio_get_level(TP_PIN_NUM_INT));

// 修改为：打印更详细的坐标信息
ESP_LOGI(TAG, "👆 触摸检测: X=%d, Y=%d, 按压次数=%d", touchpad->GetTouchPoint().x, touchpad->GetTouchPoint().y, touchpad->GetPressCount());
```

**测试方法**:
1. 修改代码。
2. 重新编译并烧录: `idf.py app-flash monitor` (使用 `app-flash` 只烧录应用部分，速度更快)。
3. 在屏幕上滑动，观察串口日志输出。

### 场景二：开发自定义按键逻辑

除了默认的 Boot 键配网功能，您可以添加自定义逻辑。

**修改位置**: `InitializeButtons` 函数

```cpp
boot_button_.OnClick([this]() {
    auto &app = Application::GetInstance();
    ESP_LOGI(TAG, "按键被点击了！当前状态: %s", app.GetDeviceState() == kDeviceStateStarting ? "启动中" : "运行中");
    
    // 在这里添加您的测试代码
    // 例如：手动触发一个表情
    // display_->ShowIcon("cool"); 
    
    if (app.GetDeviceState() == kDeviceStateStarting) {
        EnterWifiConfigMode();
        return;
    }
    app.ToggleChatState();
});
```

### 场景三：硬件版本检测调试

EchoEar 有 V1.0 和 V1.2 两个版本，代码会自动检测。如果你想强制测试特定版本的逻辑：

**修改位置**: `DetectPcbVersion` 函数

```cpp
// 可以在函数开头强制返回特定版本进行测试
return 1; // 强制模拟 V1.2 版本
```

---

## 5. 常见问题排查

1.  **烧录失败 "Timed out waiting for packet header"**
    - 检查电源开关是否打开。
    - 尝试按住 `BOOT` 键再插入 USB，进入下载模式。

2.  **屏幕显示花屏或颜色不对**
    - 检查 `Initializest77916Display` 中的 `esp_lcd_panel_swap_xy` 和 `esp_lcd_panel_mirror` 设置是否与当前固件版本匹配。
    - V1.0 和 V1.2 版本的引脚定义不同，确保 `DetectPcbVersion` 检测正确（查看启动日志 `PCB verison ...`）。

3.  **无法检测到触摸**
    - 检查 `touch_event_task` 是否在运行。
    - 确保 `TP_PIN_NUM_INT` 中断引脚配置正确。

## 6. 添加新功能流程

1.  **编写**: 在 `EchoEar.cc` 中添加新的驱动类或逻辑方法。
2.  **注册**: 如果是新的后台任务，在 `EchoEar` 构造函数中使用 `xTaskCreate` 启动。
3.  **验证**: 使用 `ESP_LOGI` 打印调试信息，通过串口观察。

祝您开发愉快！🚀
