# ONNX点云分割模块

## 概述
该模块使用ONNX模型对点云进行分割，能够识别猪的不同部位（头部、躯干、四肢、尾巴等）。

## 功能
- 加载和运行ONNX模型进行点云分割
- 重采样点云到固定大小以适应模型输入
- 根据预测标签分割点云为不同部位
- 提供进度和统计信息回调
- 保存分割结果到文件

## 文件结构
- `ONNXSegmenter.h` - 接口头文件
- `ONNXSegmenter.cpp` - 实现文件

## 使用方法
```cpp
// 创建分割器实例
ONNXSegmenter segmenter;

// 配置模型
ONNXSegmenter::ONNXModelConfig config;
config.model_path = "path/to/your/model.onnx";
config.input_size = 8192;
config.confidence_threshold = 0.7f;
config.class_names = {"background", "head", "torso", "leg", "tail"};

// 加载模型
if (!segmenter.loadModel(config)) {
    std::cerr << "Failed to load model" << std::endl;
    return -1;
}

// 设置回调函数（可选）
segmenter.setProgressCallback([](double progress, const std::string& stage) {
    std::cout << "Progress: " << progress * 100 << "% at stage: " << stage << std::endl;
});

segmenter.setStatsCallback([](const std::string& stage, const std::string& stats) {
    std::cout << "Stats for " << stage << ": " << stats << std::endl;
});

// 执行分割
ONNXSegmenter::SegmentationResult result = segmenter.segment(input_cloud);

// 访问分割结果
for (const auto& segment : result.segments) {
    std::string class_name = segment.first;
    pcl::PointCloud<pcl::PointXYZ>::Ptr part_cloud = segment.second;
    std::cout << class_name << " has " << part_cloud->size() << " points" << std::endl;
}

// 保存结果
segmenter.saveSegmentationResult(result, "output_base", "ply");
```

## 模型配置说明
- `model_path`: ONNX模型文件路径
- `input_size`: 模型期望的输入点数（默认8192）
- `confidence_threshold`: 置信度阈值
- `class_names`: 类别名称列表

## 分割类别
默认类别包括：
- background (背景)
- head (头部)
- torso (躯干)
- front_left_leg (前左腿)
- front_right_leg (前右腿)
- hind_left_leg (后左腿)
- hind_right_leg (后右腿)
- tail (尾巴)
- ear (耳朵)
- neck (脖子)

## 注意事项
- 需要安装ONNX Runtime库
- 输入点云会被重采样到固定大小以适应模型输入要求
- 分割结果包含每部分的点云和整体置信度评分
- 确保ONNX模型路径正确且模型文件存在