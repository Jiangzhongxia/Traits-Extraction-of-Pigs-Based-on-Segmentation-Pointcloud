# 猪体尺测量主应用程序

## 概述
该模块是整个猪体尺测量系统的主控制器，协调各模块工作，管理应用程序生命周期。

## 功能
- 协调各功能模块工作流程
- 管理应用程序生命周期（初始化、启动、停止）
- 处理实时和离线点云数据
- 提供用户交互接口
- 管理配置和结果保存

## 文件结构
- `PigMeasurementApp.h` - 接口头文件
- `PigMeasurementApp.cpp` - 实现文件

## 使用方法
```cpp
// 创建应用程序实例
PigMeasurementApp app;

// 配置应用程序
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
    std::cout << "Measurement completed - Body length: " << result.body_length << " m" << std::endl;
});

app.setErrorCallback([](const std::string& error) {
    std::cerr << "Error: " << error << std::endl;
});

// 启动应用程序
if (!app.start()) {
    std::cerr << "Failed to start application" << std::endl;
    return -1;
}

// 实时处理模式：应用程序会持续运行并处理数据
// 或者离线处理单个文件
// app.processFromFile("input_pointcloud.ply");

// 检查应用程序状态
while (app.isRunning()) {
    auto status = app.getStatus();
    std::cout << "Current stage: " << status.current_stage << std::endl;

    std::this->sleep_for(std::chrono::seconds(1));
}

// 停止应用程序
app.stop();
```

## 应用程序配置说明
- `config_file`: 配置文件路径
- `output_directory`: 输出目录
- `model_path`: 分割模型路径
- `enable_visualization`: 是否启用可视化
- `save_raw_data`: 是否保存原始数据
- `save_intermediate`: 是否保存中间结果
- `save_final`: 是否保存最终结果
- `enable_real_time`: 是否启用实时处理
- `processing_interval_ms`: 处理间隔(毫秒)

## 应用程序状态
- `initialized`: 是否已初始化
- `running`: 是否正在运行
- `capturing`: 是否正在采集数据
- `processing`: 是否正在处理数据
- `current_stage`: 当前阶段
- `progress`: 进度
- `error_message`: 错误信息

## 主要工作流程
1. 初始化各功能模块（激光雷达接口、点云处理、分割、测量、可视化、优化）
2. 启动激光雷达数据采集（实时模式）
3. 获取点云数据
4. 预处理点云（降噪、降采样、配准）
5. 分割点云到不同部位
6. 计算体尺参数
7. 保存结果和可视化
8. 循环执行以上步骤（实时模式）

## 注意事项
- 确保所有依赖的模型文件和配置文件存在
- 在生产环境中根据硬件性能调整处理间隔
- 实时处理模式需要稳定的激光雷达数据流
- 监控内存使用情况，避免长时间运行导致内存泄漏