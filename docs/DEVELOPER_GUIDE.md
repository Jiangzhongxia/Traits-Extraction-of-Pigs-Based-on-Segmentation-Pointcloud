# 猪体尺测量系统开发者指南

## 概述

本指南面向参与猪体尺测量系统开发的开发者，详细介绍系统的架构设计、模块接口、扩展方法等内容。

## 系统架构详解

### 模块划分

系统按功能划分为以下核心模块：

#### 1. 激光雷达接口模块 (lidar_interface)
- **功能**: 管理图达通灵雀w激光雷达设备
- **主要类**: `LivoxLidarInterface`
- **职责**:
  - 管理设备连接和配置
  - 同步采集左右两侧点云数据
  - 数据流管理

#### 2. 点云处理模块 (pointcloud_processing)
- **功能**: 点云预处理功能
- **主要类**: `PointCloudProcessor`
- **职责**:
  - 点云降噪和滤波
  - 点云降采样
  - 多视角点云配准与融合

#### 3. 点云分割模块 (segmentation)
- **功能**: 使用ONNX模型进行点云分割
- **主要类**: `ONNXSegmenter`
- **职责**:
  - 加载和运行ONNX模型
  - 点云分割为不同部位
  - 后处理和结果组织

#### 4. 体尺计算模块 (measurement)
- **功能**: 基于分割结果计算体尺参数
- **主要类**: `BodyMeasurementCalculator`
- **职责**:
  - 实现各种体尺测量算法
  - 重量估算
  - 结果验证和保存

#### 5. 可视化模块 (visualization)
- **功能**: 数据可视化和结果展示
- **主要类**: `VisualizationManager`
- **职责**:
  - 点云渲染和显示
  - 测量结果标注
  - 交互式操作

#### 6. 性能优化模块 (optimization)
- **功能**: 系统性能优化
- **主要类**: `PerformanceOptimizer`
- **职责**:
  - 并行处理管理
  - 内存池管理
  - 参数自适应调整

#### 7. 应用程序主控 (application)
- **功能**: 协调各模块工作
- **主要类**: `PigMeasurementApp`
- **职责**:
  - 应用程序生命周期管理
  - 模块间协调
  - 配置管理

## 核心数据结构

### MeasurementResult 结构

```cpp
struct MeasurementResult {
    double body_length = 0.0;      // 体长 (m)
    double withers_height = 0.0;   // 肩高 (m)
    double chest_depth = 0.0;      // 胸深 (m)
    double chest_width = 0.0;      // 胸宽 (m)
    // ... 其他测量参数
    std::string timestamp;         // 测量时间戳
};
```

### 点云数据格式

- 使用PCL的`pcl::PointXYZ`格式
- 确保点云数据坐标系一致
- 注意处理NaN值和无效点

## 模块开发指南

### 添加新功能模块

1. 创建新的模块目录:
```
src/
 └── new_module/
     ├── NewModule.h
     ├── NewModule.cpp
     ├── CLAUDE.md
     └── CMakeLists.txt
```

2. 实现模块接口:
```cpp
// NewModule.h
#pragma once
#include <pcl/point_cloud.h>

class NewModule {
public:
    struct Config {
        // 配置参数
    };

    NewModule();
    ~NewModule();

    bool process(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr& output);

    void setConfig(const Config& config) { config_ = config; }

private:
    Config config_;
    // 实现细节
};
```

3. 更新CMakeLists.txt:
```cmake
add_library(new_module
    NewModule.cpp
)

target_link_libraries(new_module
    ${PCL_LIBRARIES}
)
```

### 扩展现有模块

#### 扩展体尺计算模块

如果需要添加新的体尺测量参数:

```cpp
// 在BodyMeasurementCalculator.h中添加新方法
double calculateNewMeasurement(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

// 在实现文件中实现算法
double BodyMeasurementCalculator::calculateNewMeasurement(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // 实现具体的测量算法
    double result = 0.0;
    // ... 算法实现
    return result;
}
```

#### 扩展分割模块

如果需要添加新的分割类别:

```cpp
// 更新ONNX模型配置
ONNXSegmenter::ONNXModelConfig config;
config.class_names = {
    "background", "head", "torso", "front_left_leg",
    "front_right_leg", "hind_left_leg", "hind_right_leg",
    "tail", "ear", "neck", "new_part"  // 添加新类别
};
```

## 性能优化策略

### 并行处理

系统支持多层次的并行处理:

1. **数据级并行**: 同时处理多个点云
2. **任务级并行**: 并行执行不同处理步骤
3. **算法级并行**: 算法内部的并行化

### 内存管理

- 使用对象池减少内存分配开销
- 及时释放不需要的点云数据
- 避免点云数据的不必要复制

### 缓存策略

- 缓存中间处理结果
- 预加载常用的模型文件
- 使用LRU缓存管理历史数据

## 测试策略

### 单元测试

为每个模块编写单元测试:

```cpp
// test_new_module.cpp
bool testNewModule() {
    NewModule module;
    NewModule::Config config;
    module.setConfig(config);

    // 创建测试数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    // ... 填充测试数据

    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    bool success = module.process(input, output);

    // 验证结果
    return success && output->size() > 0;
}
```

### 集成测试

测试模块间的交互:

```cpp
bool testModuleIntegration() {
    // 测试多个模块的协作
    PointCloudProcessor processor;
    ONNXSegmenter segmenter;
    BodyMeasurementCalculator calculator;

    // ... 配置和测试数据流
    // 从点云处理到分割再到测量的完整流程
}
```

## 配置和参数调优

### 动态参数调整

系统支持运行时参数调整:

```cpp
// 参数调优示例
auto tuned_params = optimizer.autoTuneParameters(input_cloud->size());
processor.setParams(tuned_params["processor_config"]);
segmenter.setParams(tuned_params["segmenter_config"]);
```

### 配置文件管理

使用JSON格式管理配置:

```json
{
  "lidar": {
    "left_ip": "192.168.1.10",
    "right_ip": "192.168.1.11",
    "port": 65000
  },
  "processing": {
    "voxel_leaf_size": 0.01,
    "outlier_radius": 0.1
  },
  "segmentation": {
    "model_path": "models/pig_segmentation.onnx",
    "input_size": 8192
  }
}
```

## 错误处理和日志

### 错误处理策略

系统采用分层错误处理:

```cpp
// 在模块内部处理错误
bool SomeModule::process(...) {
    try {
        // 主要处理逻辑
        return true;
    } catch (const std::exception& e) {
        // 记录错误信息
        handleError(std::string("Processing error: ") + e.what());
        return false;
    }
}
```

### 日志系统

集成日志系统来追踪运行状态:

```cpp
// 使用回调函数传递状态信息
void SomeModule::setProgressCallback(
    std::function<void(double progress, const std::string& stage)> callback) {
    progress_callback_ = callback;
}
```

## 部署和发布

### 构建配置

创建不同的构建配置:

```cmake
# Release配置
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
endif()

# Debug配置
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -DDEBUG")
endif()
```

### 依赖管理

使用包管理器管理依赖:

```cmake
find_package(PCL 1.10 REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(ONNXRuntime REQUIRED QUIET)
```

## 版本控制和发布

### Git工作流

使用Git进行版本控制:

```bash
# 创建功能分支
git checkout -b feature/new-measurement-algorithm

# 开发完成后
git add .
git commit -m "Add new measurement algorithm for pig weight estimation"
git push origin feature/new-measurement-algorithm

# 创建PR进行代码审查
```

### 发布流程

1. 完成功能开发和测试
2. 更新版本号
3. 创建发布分支
4. 生成发布包
5. 部署到目标环境

## 质量保证

### 代码规范

遵循统一的代码规范:

- 使用一致的命名约定
- 正确的注释格式
- 合理的函数长度
- 适当的错误处理

### 代码审查

实施代码审查流程:

- 所有代码变更需经过审查
- 使用静态分析工具
- 自动化测试验证

### 性能监控

持续监控系统性能:

- 处理时间统计
- 内存使用跟踪
- 精度指标评估

## 技术支持

如需进一步的技术支持或对系统架构有疑问，请参考以下资源:

- 源代码注释和文档
- 模块级CLAUDE.md文件
- 用户指南和开发者指南
- 项目仓库中的issue讨论

## 未来发展方向

### 技术改进

1. 深度学习模型优化
2. 实时处理性能提升
3. 多设备协同处理
4. 云端数据处理集成

### 功能扩展

1. 支持更多动物种类
2. 健康状况评估
3. 生长趋势分析
4. 预测性分析功能

本指南持续更新以反映系统发展的最新情况。