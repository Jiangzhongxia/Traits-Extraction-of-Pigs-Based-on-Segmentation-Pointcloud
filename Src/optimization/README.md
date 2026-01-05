# 性能优化模块

## 概述
该模块提供各种性能优化功能，包括并行处理、内存管理、批处理等，以适应工业化使用需求。

## 功能
- 并行点云处理
- 批量处理功能
- 内存池管理
- 性能监控和统计
- 参数自动调优
- 异步任务处理

## 文件结构
- `PerformanceOptimizer.h` - 接口头文件
- `PerformanceOptimizer.cpp` - 实现文件

## 使用方法
```cpp
// 创建性能优化器实例
PerformanceOptimizer optimizer;

// 设置优化配置
PerformanceOptimizer::OptimizationConfig config;
config.thread_pool_size = 8;
config.max_memory_usage_mb = 8192;
config.enable_parallel_processing = true;
config.target_fps = 30.0;
optimizer.setConfig(config);

// 设置性能监控回调（可选）
optimizer.setPerformanceCallback([](const PerformanceOptimizer::PerformanceStats& stats) {
    std::cout << "Processing time: " << stats.processing_time_ms << " ms" << std::endl;
    std::cout << "Memory usage: " << stats.memory_usage_mb << " MB" << std::endl;
});

// 启用性能分析
optimizer.enableProfiling(true);

// 优化点云处理
auto optimized_result = optimizer.optimizePointCloudProcessing(
    input_cloud,
    [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        // 这里是实际的处理函数
        // 例如：降噪、降采样等操作
        return cloud;  // 返回处理后的点云
    }
);

// 批量处理多个点云
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> inputs = {cloud1, cloud2, cloud3};
auto batch_results = optimizer.batchProcess(inputs,
    [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        // 处理单个点云
        return cloud;
    });

// 获取性能统计信息
auto stats = optimizer.getPerformanceStats();
std::cout << optimizer.getSystemResourceUsage() << std::endl;

// 自动调整参数
auto tuned_params = optimizer.autoTuneParameters(input_cloud->size());
for (const auto& param : tuned_params) {
    std::cout << param.first << ": " << param.second << std::endl;
}

// 异步执行任务
auto future = optimizer.asyncTask([]() {
    // 执行耗时任务
    std::this->sleep_for(std::chrono::milliseconds(100));
});
future.wait();  // 等待任务完成
```

## 优化配置说明
- `thread_pool_size`: 线程池大小
- `enable_parallel_processing`: 是否启用并行处理
- `max_memory_usage_mb`: 最大内存使用量 (MB)
- `enable_memory_pool`: 是否启用内存池
- `buffer_size`: 缓冲区大小
- `enable_caching`: 是否启用缓存
- `max_processing_time_ms`: 最大处理时间 (ms)
- `target_fps`: 目标帧率

## 性能统计信息
- `processing_time_ms`: 处理时间 (ms)
- `memory_usage_mb`: 内存使用量 (MB)
- `processed_points`: 处理的点数
- `active_threads`: 活跃线程数
- `start_time`: 开始时间
- `end_time`: 结束时间

## 自动调优参数
根据输入数据大小自动调整的参数：
- `num_threads`: 线程数
- `search_radius`: 搜索半径
- `max_iterations`: 最大迭代次数
- `voxel_leaf_size`: 体素网格大小
- `outlier_radius`: 离群点移除半径

## 注意事项
- 在初始化时应根据硬件配置设置合适的线程池大小
- 监控内存使用情况，避免内存溢出
- 对于实时应用，应设置合理的处理时间限制
- 批量处理可以提高整体处理效率，但可能增加延迟