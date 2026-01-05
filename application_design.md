# 工业级猪体尺测量应用程序架构设计

## 1. 总体架构

### 1.1 系统概览
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

### 1.2 架构特点
- 模块化设计，各模块职责清晰
- 支持实时数据处理
- 可扩展性好，便于功能升级
- 工业级稳定性与性能优化

## 2. 核心模块设计

### 2.1 激光雷达数据接口模块
**职责：**
- 与图达通灵雀w激光雷达通信
- 同步收集左右两侧点云数据
- 数据格式转换与校验

**接口设计：**
```cpp
class LivoxLidarInterface {
public:
    struct DeviceConfig {
        std::string ip_address;
        int port;
        std::string device_type; // "left" or "right"
    };

    bool initialize(const std::vector<DeviceConfig>& configs);
    bool startCapture();
    bool stopCapture();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getLeftPointCloud();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getRightPointCloud();
    bool syncPointClouds(pcl::PointCloud<pcl::PointXYZI>::Ptr& combined);
    bool isDeviceConnected();
};
```

### 2.2 点云预处理模块
**职责：**
- 点云降噪
- 点云配准与融合
- 基于现有项目的法向量定向功能

**接口设计：**
```cpp
class PointCloudPreprocessor {
public:
    struct PreprocessParams {
        double voxel_leaf_size = 0.01;
        double outlier_radius = 0.1;
        int outlier_min_neighbors = 10;
        double registration_max_distance = 0.05;
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr denoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                                  double leaf_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr registerClouds(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& left_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& right_cloud,
        Eigen::Matrix4f& transform);
    pcl::PointCloud<pcl::PointXYZ>::Ptr orientNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        bool is_noisy = true);
};
```

### 2.3 点云分割模块
**职责：**
- 使用ONNX模型进行点云分割
- 将完整的猪点云分割为不同部位（头、四肢、尾巴等）

**接口设计：**
```cpp
class PointCloudSegmenter {
public:
    struct ONNXModelConfig {
        std::string model_path;
        int input_size = 8192; // 默认输入点数
        float confidence_threshold = 0.7;
    };

    bool loadModel(const ONNXModelConfig& config);
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> segment(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input);
    std::vector<std::string> getAvailableClasses();
    bool isModelLoaded();
};
```

### 2.4 体尺计算模块
**职责：**
- 根据分割后的不同部位计算体尺信息
- 继承并扩展现有体尺计算功能

**接口设计：**
```cpp
class BodyMeasurementCalculator {
public:
    struct MeasurementResult {
        double body_length = 0.0;      // 体长
        double withers_height = 0.0;   // 肩高
        double chest_depth = 0.0;      // 胸深
        double chest_width = 0.0;      // 胸宽
        double hip_height = 0.0;       // 髋高
        double hip_width = 0.0;        // 髋宽
        double abdominal_girth = 0.0;  // 腹围
        double abdominal_width = 0.0;  // 腹宽
        double abdominal_height = 0.0; // 腹高
    };

    MeasurementResult calculate(const std::map<std::string,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr>& segments);
    void saveResults(const MeasurementResult& result,
                     const std::string& output_path);
    std::vector<std::string> getAvailableMeasurements();
};
```

### 2.5 可视化与数据管理模块
**职责：**
- 点云数据可视化
- 测量结果展示
- 数据存储与管理

**接口设计：**
```cpp
class VisualizationManager {
public:
    struct VisualizationConfig {
        bool show_original_cloud = true;
        bool show_segmented_cloud = true;
        bool show_measurements = true;
        bool show_bounding_boxes = true;
    };

    bool initialize();
    bool displayPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool displaySegmentedCloud(const std::map<std::string,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr>& segments);
    bool displayMeasurements(const BodyMeasurementCalculator::MeasurementResult& result);
    bool updateConfig(const VisualizationConfig& config);
    bool saveScreenshot(const std::string& filename);
    bool runVisualizationLoop();
};
```

### 2.6 主应用程序控制器
**职责：**
- 协调各模块工作
- 管理应用程序生命周期
- 处理用户输入和配置

**接口设计：**
```cpp
class PigMeasurementApp {
public:
    struct AppConfig {
        std::string config_file = "config/app_config.json";
        std::string output_directory = "results/";
        bool enable_visualization = true;
        bool save_raw_data = true;
        bool save_intermediate = true;
        bool save_final = true;
    };

    bool initialize(const AppConfig& config);
    bool runMeasurementCycle();
    bool processSinglePig();
    void stop();
    bool isRunning();
    void setProcessingCallback(std::function<void(double progress,
                                                 const std::string& status)> callback);
};
```

## 3. 数据流设计

### 3.1 实时数据处理流程
```
激光雷达数据 → 数据同步 → 点云预处理 → 点云分割 → 体尺计算 → 结果保存/可视化
```

### 3.2 批量处理流程
```
点云文件 → 数据加载 → 预处理 → 分割 → 计算 → 结果保存
```

## 4. 性能与稳定性考虑

### 4.1 实时性能
- 异步数据处理，避免阻塞
- 多线程并行处理
- 数据缓冲池管理

### 4.2 内存管理
- 点云数据流式处理
- 智能内存回收
- 大数据量分块处理

### 4.3 错误处理
- 设备连接异常处理
- 数据丢失恢复机制
- 计算错误容错

## 5. 扩展性设计

### 5.1 插件化架构
- 激光雷达类型可扩展
- 分割模型可替换
- 测量算法可更新

### 5.2 配置管理
- JSON配置文件
- 运行时参数调整
- 远程配置更新

## 6. 技术栈

### 6.1 核心技术
- C++17 (主程序)
- PCL 1.10+ (点云处理)
- OpenCV (图像处理)
- ONNX Runtime (模型推理)
- Qt/ImGui (GUI)

### 6.2 依赖库
- PCL (点云库)
- Eigen3 (线性代数)
- OpenMP (并行计算)
- Boost (系统库)
- Livox SDK (激光雷达接口)

## 7. 部署方案

### 7.1 硬件要求
- CPU: Intel i7 / AMD Ryzen 7 或更高
- GPU: 支持CUDA的显卡(推理用)
- RAM: 16GB 或更高
- 存储: SSD，500GB以上

### 7.2 软件环境
- 操作系统: Windows 10/11 或 Linux Ubuntu 20.04+
- 编译器: GCC 9+, Visual Studio 2019+
- CUDA: 适合的GPU驱动