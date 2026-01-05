#include "PerformanceOptimizer.h"
#include <iostream>
#include <thread>
#include <future>
#include <queue>
#include <condition_variable>
#include <algorithm>
#include <numeric>

PerformanceOptimizer::PerformanceOptimizer()
    : profiling_enabled_(false), should_stop_thread_pool_(false) {
    // 初始化默认配置
    config_.thread_pool_size = std::min(8, (int)std::thread::hardware_concurrency());
    config_.max_memory_usage_mb = 8192;
    config_.enable_memory_pool = true;
    config_.buffer_size = 1024;
    config_.enable_caching = true;
    config_.max_processing_time_ms = 1000.0;
    config_.target_fps = 30.0;

    // 初始化性能统计
    resetPerformanceStats();

    // 初始化线程池
    initializeThreadPool();
}

PerformanceOptimizer::~PerformanceOptimizer() {
    shutdownThreadPool();
}

void PerformanceOptimizer::setConfig(const OptimizationConfig& config) {
    config_ = config;

    // 根据新配置重新初始化线程池
    shutdownThreadPool();
    initializeThreadPool();
}

PerformanceOptimizer::OptimizationConfig PerformanceOptimizer::getConfig() const {
    return config_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PerformanceOptimizer::optimizePointCloudProcessing(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    std::function<pcl::PointCloud<pcl::PointXYZ>::Ptr(const pcl::PointCloud<pcl::PointXYZ>::Ptr&)> process_func) {

    if (!input || input->empty()) {
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    // 检查内存使用情况
    if (isMemoryUsageExceeded()) {
        std::cerr << "Warning: Memory usage exceeded limit" << std::endl;
        // 可以在此处实现内存清理逻辑
    }

    auto start_time = std::chrono::steady_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr result;

    if (config_.enable_parallel_processing && input->size() > 10000) {
        // 对于大点云，使用并行处理
        result = profileFunction(
            [&process_func, &input]() {
                return process_func(input);
            },
            "Parallel Point Cloud Processing"
        );
    } else {
        // 对于小点云，直接处理
        result = profileFunction(
            [&process_func, &input]() {
                return process_func(input);
            },
            "Sequential Point Cloud Processing"
        );
    }

    auto end_time = std::chrono::steady_clock::now();

    // 更新性能统计
    updatePerformanceStats(start_time, end_time, input->size());

    return result;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PerformanceOptimizer::batchProcess(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& inputs,
    std::function<pcl::PointCloud<pcl::PointXYZ>::Ptr(const pcl::PointCloud<pcl::PointXYZ>::Ptr&)> process_func) {

    if (inputs.empty()) {
        return {};
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> results(inputs.size());

    if (config_.enable_parallel_processing && config_.thread_pool_size > 1) {
        // 使用线程池并行处理
        std::vector<std::future<void>> futures;

        for (size_t i = 0; i < inputs.size(); ++i) {
            futures.push_back(asyncTask([&, i]() {
                results[i] = optimizePointCloudProcessing(inputs[i], process_func);
            }));
        }

        // 等待所有任务完成
        for (auto& future : futures) {
            future.wait();
        }
    } else {
        // 顺序处理
        for (size_t i = 0; i < inputs.size(); ++i) {
            results[i] = optimizePointCloudProcessing(inputs[i], process_func);
        }
    }

    return results;
}

void* PerformanceOptimizer::allocateMemory(size_t size) {
    // 在实际实现中，这里会使用内存池管理
    // 为了简化，这里直接使用new，但在工业应用中应该实现内存池
    return new char[size];
}

void PerformanceOptimizer::releaseMemory(void* ptr) {
    // 在实际实现中，这里会将内存返回到内存池
    // 为了简化，这里直接使用delete
    if (ptr) {
        delete[] static_cast<char*>(ptr);
    }
}

PerformanceOptimizer::PerformanceStats PerformanceOptimizer::getPerformanceStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void PerformanceOptimizer::resetPerformanceStats() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_ = PerformanceStats();
    stats_.start_time = std::chrono::steady_clock::now();
}

void PerformanceOptimizer::setPerformanceCallback(std::function<void(const PerformanceStats& stats)> callback) {
    performance_callback_ = callback;
}

void PerformanceOptimizer::enableProfiling(bool enable) {
    profiling_enabled_ = enable;
}

std::string PerformanceOptimizer::getSystemResourceUsage() const {
    std::stringstream ss;

    // 获取当前统计信息
    auto current_stats = getPerformanceStats();

    ss << "Performance Stats:\n";
    ss << "  Processing Time: " << current_stats.processing_time_ms << " ms\n";
    ss << "  Memory Usage: " << current_stats.memory_usage_mb << " MB\n";
    ss << "  Processed Points: " << current_stats.processed_points << "\n";
    ss << "  Active Threads: " << current_stats.active_threads << "\n";
    ss << "  Target FPS: " << config_.target_fps << "\n";
    ss << "  Max Processing Time: " << config_.max_processing_time_ms << " ms\n";
    ss << "  Thread Pool Size: " << config_.thread_pool_size << "\n";

    return ss.str();
}

std::map<std::string, double> PerformanceOptimizer::autoTuneParameters(size_t input_size) {
    std::map<std::string, double> tuned_params;

    // 根据输入数据大小自动调整参数
    if (input_size < 10000) {
        // 小数据集：使用较少的线程，更小的搜索半径
        tuned_params["num_threads"] = std::max(1.0, config_.thread_pool_size / 4.0);
        tuned_params["search_radius"] = 0.01;
        tuned_params["max_iterations"] = 50.0;
    } else if (input_size < 100000) {
        // 中等数据集：使用中等数量的线程
        tuned_params["num_threads"] = std::max(1.0, config_.thread_pool_size / 2.0);
        tuned_params["search_radius"] = 0.02;
        tuned_params["max_iterations"] = 100.0;
    } else {
        // 大数据集：使用更多线程，但要注意内存使用
        tuned_params["num_threads"] = config_.thread_pool_size;
        tuned_params["search_radius"] = 0.03;
        tuned_params["max_iterations"] = 200.0;
    }

    // 调整其他可能影响性能的参数
    tuned_params["voxel_leaf_size"] = 0.001 * std::sqrt(input_size / 100000.0) + 0.005;
    tuned_params["outlier_radius"] = 0.01 * std::sqrt(input_size / 100000.0) + 0.05;

    return tuned_params;
}

std::future<void> PerformanceOptimizer::asyncTask(std::function<void()> task) {
    return std::async(std::launch::async, task);
}

void PerformanceOptimizer::initializeThreadPool() {
    shutdownThreadPool();  // 先清理现有线程池

    thread_pool_.clear();
    should_stop_thread_pool_ = false;

    for (int i = 0; i < config_.thread_pool_size; ++i) {
        thread_pool_.emplace_back([this]() {
            while (!should_stop_thread_pool_) {
                // 线程池工作逻辑
                std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 避免空转
            }
        });
    }
}

void PerformanceOptimizer::shutdownThreadPool() {
    should_stop_thread_pool_ = true;

    for (auto& thread : thread_pool_) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    thread_pool_.clear();
}

void PerformanceOptimizer::updatePerformanceStats(
    const std::chrono::steady_clock::time_point& start_time,
    const std::chrono::steady_clock::time_point& end_time,
    size_t processed_points) {

    std::lock_guard<std::mutex> lock(stats_mutex_);

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    stats_.processing_time_ms = duration.count() / 1000.0;
    stats_.processed_points += processed_points;
    stats_.active_threads = config_.enable_parallel_processing ? config_.thread_pool_size : 1;

    // 更新结束时间
    stats_.end_time = end_time;

    // 调用性能回调（如果设置）
    if (performance_callback_) {
        performance_callback_(stats_);
    }
}

bool PerformanceOptimizer::isMemoryUsageExceeded() const {
    // 这里应该实现实际的内存使用监测
    // 简化实现：假设每个点占用大约50字节
    size_t estimated_memory = stats_.processed_points * 50;  // 50 bytes per point (approx)
    size_t memory_limit = config_.max_memory_usage_mb * 1024 * 1024;  // 转换为字节

    return estimated_memory > memory_limit;
}

template<typename Func>
auto PerformanceOptimizer::profileFunction(Func&& func, const std::string& description) -> decltype(func()) {
    if (!profiling_enabled_) {
        return func();
    }

    auto start = std::chrono::high_resolution_clock::now();
    auto result = func();
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    double ms = duration.count() / 1000.0;

    std::cout << "Performance: " << description << " took " << ms << " ms" << std::endl;

    return result;
}