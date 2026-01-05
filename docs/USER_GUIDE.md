# 猪体尺测量系统用户指南

## 概述

猪体尺测量系统是一个工业级应用程序，用于自动测量猪的各项体尺参数。系统使用两台图达通灵雀w激光雷达同步收集猪的左右两边点云信息，经过降噪、配准、分割和计算等步骤，最终输出准确的体尺测量结果。

## 系统架构

```
    ┌─────────────────┐    ┌─────────────────┐
    │  左侧激光雷达     │    │  右侧激光雷达     │
    │  (图达通灵雀w)   │    │  (图达通灵雀w)   │
    └─────────┬───────┘    └─────────┬───────┘
              │                      │
              └────────┬─────────────┘
                       │
              ┌────────▼─────────────┐
              │   数据同步与融合      │
              └────────┬─────────────┘
                       │
              ┌────────▼─────────────┐
              │   点云预处理模块      │
              │  (降噪、配准)        │
              └────────┬─────────────┘
                       │
              ┌────────▼─────────────┐
              │   点云分割模块        │
              │  (ONNX模型推理)      │
              └────────┬─────────────┘
                       │
              ┌────────▼─────────────┐
              │   体尺计算模块        │
              └────────┬─────────────┘
                       │
              ┌────────▼─────────────┐
              │   可视化与数据管理    │
              └───────────────────────┘
```

## 安装和配置

### 系统要求

- **操作系统**: Windows 10/11 或 Linux Ubuntu 20.04+
- **处理器**: Intel i7 / AMD Ryzen 7 或更高
- **内存**: 16GB 或更高
- **显卡**: 支持CUDA的NVIDIA显卡（用于模型推理）
- **存储**: 500GB SSD或更高

### 依赖库

- PCL (Point Cloud Library) 1.10+
- ONNX Runtime
- OpenCV
- Livox SDK
- Qt5 或 ImGui (用于可视化)

### 编译和安装

1. 克隆仓库：
```bash
git clone <repository_url>
cd IsoConstraints-main
```

2. 创建构建目录：
```bash
mkdir build
cd build
```

3. 配置和编译：
```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## 使用方法

### 启动应用程序

```cpp
#include "application/PigMeasurementApp.h"

int main() {
    // 创建应用程序实例
    PigMeasurementApp app;

    // 配置应用程序参数
    PigMeasurementApp::AppConfig config;
    config.output_directory = "results/";
    config.model_path = "models/pig_segmentation.onnx";
    config.enable_visualization = true;
    config.enable_real_time = true;
    config.processing_interval_ms = 1000;

    // 初始化应用程序
    if (!app.initialize(config)) {
        std::cerr << "Failed to initialize application" << std::endl;
        return -1;
    }

    // 设置回调函数（可选）
    app.setProcessingCallback([](double progress, const std::string& status) {
        std::cout << "Progress: " << progress * 100 << "%, Status: " << status << std::endl;
    });

    app.setMeasurementCallback([](const MeasurementResult& result) {
        std::cout << "Measurement completed - Body length: "
                  << result.body_length << " m" << std::endl;
    });

    // 启动应用程序
    if (!app.start()) {
        std::cerr << "Failed to start application" << std::endl;
        return -1;
    }

    // 让应用程序运行
    while (app.isRunning()) {
        auto status = app.getStatus();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 停止应用程序
    app.stop();
    return 0;
}
```

### 激光雷达配置

系统支持配置多台图达通灵雀w激光雷达：

```cpp
// 在LivoxLidarInterface中配置设备
std::vector<LivoxLidarInterface::DeviceConfig> configs;

LivoxLidarInterface::DeviceConfig left_config;
left_config.ip_address = "192.168.1.10";  // 左侧雷达IP
left_config.port = 65000;                   // 端口
left_config.device_type = "left";           // 设备类型
configs.push_back(left_config);

LivoxLidarInterface::DeviceConfig right_config;
right_config.ip_address = "192.168.1.11";  // 右侧雷达IP
right_config.port = 65000;
right_config.device_type = "right";
configs.push_back(right_config);
```

## 测量参数

系统可测量以下猪的体尺参数：

1. **体长 (Body Length)**: 从耳根到尾根的曲线距离 (m)
2. **肩高 (Withers Height)**: 从肩部最高点到地面的垂直距离 (m)
3. **胸深 (Chest Depth)**: 胸部前后方向的最大距离 (m)
4. **胸宽 (Chest Width)**: 胸部左右方向的最大距离 (m)
5. **髋高 (Hip Height)**: 从髋部最高点到地面的垂直距离 (m)
6. **髋宽 (Hip Width)**: 髋部左右方向的最大距离 (m)
7. **腹围 (Abdominal Girth)**: 腹部周长 (m)
8. **腹宽 (Abdominal Width)**: 腹部左右方向的最大距离 (m)
9. **腹高 (Abdominal Height)**: 腹部前后方向的最大距离 (m)
10. **头长 (Head Length)**: 头部长度 (m)
11. **头宽 (Head Width)**: 头部宽度 (m)
12. **估算重量 (Estimated Weight)**: 基于体尺参数估算的重量 (kg)

## 性能优化

系统包含多个性能优化功能：

1. **并行处理**: 利用多核CPU并行处理点云数据
2. **内存管理**: 使用内存池减少内存分配/释放开销
3. **参数自适应**: 根据输入数据大小自动调整处理参数
4. **批处理**: 支持批量处理多个点云以提高效率

## 数据输出

测量结果保存在指定的输出目录中，包含：

- `measurements_*.txt`: 测量参数文本文件
- `segmentation_*.ply`: 分割结果点云文件
- `visualization_*.png`: 可视化截图（如果启用）

## 故障排除

### 常见问题

1. **激光雷达连接失败**:
   - 检查网络连接和IP配置
   - 确认Livox SDK正确安装
   - 检查防火墙设置

2. **内存不足**:
   - 增加系统内存或优化处理参数
   - 降低点云分辨率
   - 调整批处理大小

3. **处理速度慢**:
   - 检查系统资源使用情况
   - 调整线程池大小
   - 优化模型推理参数

### 调试信息

启用性能分析来监控系统运行：

```cpp
PerformanceOptimizer optimizer;
optimizer.enableProfiling(true);
optimizer.setPerformanceCallback([](const PerformanceStats& stats) {
    std::cout << "Processing time: " << stats.processing_time_ms << " ms" << std::endl;
});
```

## 开发和扩展

### 添加新的分割类别

1. 修改ONNX模型以支持新类别
2. 更新`ONNXSegmenter`中的类别名称列表
3. 在`BodyMeasurementCalculator`中添加对应处理逻辑

### 集成其他激光雷达

1. 实现新的激光雷达接口类
2. 继承或适配现有的`LivoxLidarInterface`接口
3. 更新主应用程序以支持新设备类型

## 技术支持

如需技术支持或报告问题，请联系开发团队或在项目仓库中提交issue。

## 许可证

本项目遵循[许可证类型]，详见LICENSE文件。