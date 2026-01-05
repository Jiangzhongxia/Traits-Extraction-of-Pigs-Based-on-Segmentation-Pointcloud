#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>
#include <functional>
#include <thread>
#include <mutex>
#include <chrono>
#include <memory>

/**
 * @brief 性能优化器类
 * 提供各种性能优化功能以适应工业化使用需求
 */
class PerformanceOptimizer {
public:
    struct OptimizationConfig {
        // 线程池配置
        int thread_pool_size = 4;
        bool enable_parallel_processing = true;

        // 内存管理配置
        size_t max_memory_usage_mb = 8192;  // 最大内存使用量 (MB)
        bool enable_memory_pool = true;

        // 缓冲区配置
        size_t buffer_size = 1024;  // 缓冲区大小
        bool enable_caching = true;

        // 实时性能配置
        double max_processing_time_ms = 1000.0;  // 最大处理时间 (ms)
        double target_fps = 30.0;  // 目标帧率
    };

    struct PerformanceStats {
        double processing_time_ms = 0.0;
        double memory_usage_mb = 0.0;
        size_t processed_points = 0;
        int active_threads = 0;
        std::chrono::steady_clock::time_point start_time;
        std::chrono::steady_clock::time_point end_time;
    };

    PerformanceOptimizer();
    ~PerformanceOptimizer();

    /**
     * @brief 设置优化配置
     * @param config 优化配置
     */
    void setConfig(const OptimizationConfig& config);

    /**
     * @brief 获取当前优化配置
     * @return 当前优化配置
     */
    OptimizationConfig getConfig() const;

    /**
     * @brief 优化点云处理 - 并行处理
     * @param input 输入点云
     * @param process_func 处理函数
     * @return 处理后的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr optimizePointCloudProcessing(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        std::function<pcl::PointCloud<pcl::PointXYZ>::Ptr(const pcl::PointCloud<pcl::PointXYZ>::Ptr&)> process_func);

    /**
     * @brief 批量处理点云以提高效率
     * @param inputs 输入点云列表
     * @param process_func 处理函数
     * @return 处理后的点云列表
     */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> batchProcess(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& inputs,
        std::function<pcl::PointCloud<pcl::PointXYZ>::Ptr(const pcl::PointCloud<pcl::PointXYZ>::Ptr&)> process_func);

    /**
     * @brief 内存池管理 - 分配内存
     * @param size 需要分配的大小
     * @return 分配的内存指针
     */
    void* allocateMemory(size_t size);

    /**
     * @brief 内存池管理 - 释放内存
     * @param ptr 内存指针
     */
    void releaseMemory(void* ptr);

    /**
     * @brief 获取性能统计信息
     * @return 性能统计信息
     */
    PerformanceStats getPerformanceStats() const;

    /**
     * @brief 重置性能统计信息
     */
    void resetPerformanceStats();

    /**
     * @brief 设置性能监控回调函数
     * @param callback 回调函数
     */
    void setPerformanceCallback(std::function<void(const PerformanceStats& stats)> callback);

    /**
     * @brief 启用/禁用性能分析
     * @param enable 是否启用
     */
    void enableProfiling(bool enable);

    /**
     * @brief 获取当前系统资源使用情况
     * @return 资源使用情况字符串
     */
    std::string getSystemResourceUsage() const;

    /**
     * @brief 优化算法参数自动调整
     * @param input_size 输入数据大小
     * @return 优化后的参数集合
     */
    std::map<std::string, double> autoTuneParameters(size_t input_size);

    /**
     * @brief 异步处理任务
     * @param task 任务函数
     * @return 异步结果
     */
    std::future<void> asyncTask(std::function<void()> task);

private:
    OptimizationConfig config_;
    mutable std::mutex stats_mutex_;
    PerformanceStats stats_;
    std::function<void(const PerformanceStats& stats)> performance_callback_;
    bool profiling_enabled_;
    std::vector<std::thread> thread_pool_;
    std::mutex thread_pool_mutex_;
    bool should_stop_thread_pool_;

    /**
     * @brief 初始化线程池
     */
    void initializeThreadPool();

    /**
     * @brief 关闭线程池
     */
    void shutdownThreadPool();

    /**
     * @brief 更新性能统计信息
     * @param start_time 开始时间
     * @param end_time 结束时间
     * @param processed_points 处理的点数
     */
    void updatePerformanceStats(
        const std::chrono::steady_clock::time_point& start_time,
        const std::chrono::steady_clock::time_point& end_time,
        size_t processed_points);

    /**
     * @brief 检查内存使用情况
     * @return 是否超过内存限制
     */
    bool isMemoryUsageExceeded() const;

    /**
     * @brief 执行性能分析
     * @param func 要分析的函数
     * @param description 函数描述
     */
    template<typename Func>
    auto profileFunction(Func&& func, const std::string& description) -> decltype(func());
};