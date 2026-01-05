# 体尺计算模块

## 概述
该模块根据分割后的点云数据计算猪的各项体尺参数，包括体长、肩高、胸围、腹围、髋宽高等。

## 功能
- 计算猪的各项体尺参数
- 基于体尺数据估算重量
- 保存测量结果到文件
- 提供进度和统计信息回调

## 文件结构
- `BodyMeasurementCalculator.h` - 接口头文件
- `BodyMeasurementCalculator.cpp` - 实现文件

## 使用方法
```cpp
// 创建计算器实例
BodyMeasurementCalculator calculator;

// 设置测量参数（可选）
BodyMeasurementCalculator::MeasurementParams params;
params.ground_height_offset = 0.0;
params.min_confidence = 0.7;
params.body_length_search_radius = 0.1;
calculator.setParams(params);

// 设置回调函数（可选）
calculator.setProgressCallback([](double progress, const std::string& stage) {
    std::cout << "Progress: " << progress * 100 << "% at stage: " << stage << std::endl;
});

calculator.setStatsCallback([](const std::string& stage, const std::string& stats) {
    std::cout << "Stats for " << stage << ": " << stats << std::endl;
});

// 准备分割后的点云数据
std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> segments;
segments["head"] = head_cloud;
segments["torso"] = torso_cloud;
segments["tail"] = tail_cloud;
// ... 添加其他部位的点云

// 执行测量
MeasurementResult result = calculator.calculate(segments);

// 访问测量结果
std::cout << "Body length: " << result.body_length << " m" << std::endl;
std::cout << "Withers height: " << result.withers_height << " m" << std::endl;
std::cout << "Estimated weight: " << result.estimated_weight << " kg" << std::endl;

// 保存结果
calculator.saveResults(result, "measurements.txt");
```

## 测量参数说明
- `ground_height_offset`: 地面高度偏移
- `min_confidence`: 最小置信度
- `body_length_search_radius`: 体长搜索半径
- `height_search_radius`: 高度测量搜索半径
- `volume_voxel_size`: 体积估算体素大小
- `weight_coefficient`: 重量估算系数

## 可测量的参数
- `body_length`: 体长 (m) - 从耳根到尾根的曲线距离
- `withers_height`: 肩高 (m) - 从肩部最高点到地面的垂直距离
- `chest_depth`: 胸深 (m) - 胸部前后方向的最大距离
- `chest_width`: 胸宽 (m) - 胸部左右方向的最大距离
- `hip_height`: 髋高 (m) - 从髋部最高点到地面的垂直距离
- `hip_width`: 髋宽 (m) - 髋部左右方向的最大距离
- `abdominal_girth`: 腹围 (m) - 腹部周长
- `abdominal_width`: 腹宽 (m) - 腹部左右方向的最大距离
- `abdominal_height`: 腹高 (m) - 腹部前后方向的最大距离
- `head_length`: 头长 (m)
- `head_width`: 头宽 (m)
- `ear_length`: 耳长 (m)
- `neck_length`: 颈长 (m)
- `tail_length`: 尾长 (m)
- `front_leg_girth`: 前腿围 (m)
- `hind_leg_girth`: 后腿围 (m)
- `estimated_weight`: 估算重量 (kg)

## 注意事项
- 需要提供分割后的各部位点云数据
- 测量结果包含时间戳信息
- 重量估算是基于体尺参数的估算值，可能与实际重量有差异
- 确保输入的点云数据坐标系一致