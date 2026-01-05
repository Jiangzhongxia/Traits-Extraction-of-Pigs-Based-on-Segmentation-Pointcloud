# 实时点云处理流水线模块

## 概述
该模块实现了点云的实时处理流水线，包括降噪、配准、法向量定向等预处理功能。

## 功能
- 点云降噪处理（使用统计离群点移除）
- 点云降采样（使用体素网格滤波）
- 点云配准与融合（使用ICP算法）
- 法向量定向（集成IsoConstraints算法）
- 批量处理功能
- 进度和统计信息回调

## 文件结构
- `PointCloudProcessor.h` - 接口头文件
- `PointCloudProcessor.cpp` - 实现文件

## 使用方法
```cpp
// 创建处理器实例
PointCloudProcessor processor;

// 设置处理参数
PointCloudProcessor::ProcessingParams params;
params.voxel_leaf_size = 0.01;
params.outlier_radius = 0.1;
params.registration_max_distance = 0.05;
processor.setParams(params);

// 设置回调函数（可选）
processor.setProgressCallback([](double progress, const std::string& stage) {
    std::cout << "Progress: " << progress * 100 << "% at stage: " << stage << std::endl;
});

processor.setStatsCallback([](const std::string& stage, const std::string& stats) {
    std::cout << "Stats for " << stage << ": " << stats << std::endl;
});

// 处理左右点云
auto result_cloud = processor.processClouds(left_cloud, right_cloud);

// 或者单独处理每个步骤
auto downsampled_left = processor.downsample(left_cloud);
auto denoised_left = processor.denoise(downsampled_left);
// ...
```

## 处理参数说明
- `voxel_leaf_size`: 体素网格滤波器的叶大小，用于降采样
- `outlier_radius`: 离群点移除的搜索半径
- `outlier_min_neighbors`: 离群点移除的最小邻居数
- `registration_max_distance`: ICP配准的最大对应点距离
- `registration_max_iterations`: ICP配准的最大迭代次数
- `normal_search_radius`: 法线估计的搜索半径
- `is_noisy_input`: 是否为噪声输入
- `use_implicit_orient`: 是否使用隐式定向
- `oriented_optimized`: 是否使用定向优化

## 注意事项
- 该模块与现有IsoConstraints项目集成，保留了法向量定向功能
- 处理大量点云数据时，注意内存使用情况
- 配准过程可能耗时较长，建议在性能要求高的场景下调参优化