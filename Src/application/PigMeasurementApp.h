#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>

#include "LivoxLidarInterface.h"
#include "PointCloudProcessor.h"
#include "ONNXSegmenter.h"
#include "BodyMeasurementCalculator.h"
#include "VisualizationManager.h"
#include "PerformanceOptimizer.h"

/**
 * @brief 猪体尺测量主应用程序类
 * 协调各模块工作，管理应用程序生命周期
 */
class PigMeasurementApp {
public:
    struct AppConfig {
        std::string config_file = "config/app_config.json";
        std::string output_directory = "results/";
        std::string model_path = "models/pig_segmentation.onnx";
        bool enable_visualization = true;
        bool save_raw_data = true;
        bool save_intermediate = true;
        bool save_final = true;
        bool enable_real_time = true;
        int processing_interval_ms = 1000;  // 处理间隔(毫秒)
    };

    struct AppStatus {
        bool initialized = false;
        bool running = false;
        bool capturing = false;
        bool processing = false;
        std::string current_stage = "idle";
        double progress = 0.0;
        std::string error_message = "";
    };

    PigMeasurementApp();
    ~PigMeasurementApp();

    /**
     * @brief 初始化应用程序
     * @param config 应用程序配置
     * @return 是否初始化成功
     */
    bool initialize(const AppConfig& config);

    /**
     * @brief 启动应用程序
     * @return 是否启动成功
     */
    bool start();

    /**
     * @brief 运行单个测量周期
     * @return 是否成功完成测量周期
     */
    bool runMeasurementCycle();

    /**
     * @brief 处理单个猪的测量
     * @return 是否成功处理
     */
    bool processSinglePig();

    /**
     * @brief 停止应用程序
     */
    void stop();

    /**
     * @brief 检查应用程序是否正在运行
     * @return 是否正在运行
     */
    bool isRunning() const;

    /**
     * @brief 获取当前应用程序状态
     * @return 应用程序状态
     */
    AppStatus getStatus() const;

    /**
     * @brief 设置处理进度回调
     * @param callback 回调函数
     */
    void setProcessingCallback(std::function<void(double progress, const std::string& status)> callback);

    /**
     * @brief 设置测量结果回调
     * @param callback 回调函数
     */
    void setMeasurementCallback(std::function<void(const MeasurementResult& result)> callback);

    /**
     * @brief 设置错误回调
     * @param callback 回调函数
     */
    void setErrorCallback(std::function<void(const std::string& error)> callback);

    /**
     * @brief 从文件加载点云进行处理（非实时模式）
     * @param file_path 点云文件路径
     * @return 是否成功加载和处理
     */
    bool processFromFile(const std::string& file_path);

    /**
     * @brief 保存配置到文件
     * @param config_file 配置文件路径
     */
    void saveConfig(const std::string& config_file);

    /**
     * @brief 从文件加载配置
     * @param config_file 配置文件路径
     */
    void loadConfig(const std::string& config_file);

private:
    AppConfig config_;
    AppStatus status_;
    std::thread main_thread_;
    std::mutex status_mutex_;
    std::mutex config_mutex_;

    // 各模块实例
    std::unique_ptr<LivoxLidarInterface> lidar_interface_;
    std::unique_ptr<PointCloudProcessor> pointcloud_processor_;
    std::unique_ptr<ONNXSegmenter> segmenter_;
    std::unique_ptr<BodyMeasurementCalculator> calculator_;
    std::unique_ptr<VisualizationManager> visualizer_;
    std::unique_ptr<PerformanceOptimizer> optimizer_;

    // 回调函数
    std::function<void(double progress, const std::string& status)> processing_callback_;
    std::function<void(const MeasurementResult& result)> measurement_callback_;
    std::function<void(const std::string& error)> error_callback_;

    /**
     * @brief 初始化各模块
     * @return 是否成功初始化所有模块
     */
    bool initializeModules();

    /**
     * @brief 主处理循环
     */
    void mainProcessingLoop();

    /**
     * @brief 更新应用程序状态
     * @param running 是否运行
     * @param stage 当前阶段
     * @param progress 进度
     */
    void updateStatus(bool running, const std::string& stage, double progress = 0.0);

    /**
     * @brief 处理错误
     * @param error 错误信息
     */
    void handleError(const std::string& error);

    /**
     * @brief 创建输出目录
     */
    void createOutputDirectories();

    /**
     * @brief 保存测量结果到文件
     * @param result 测量结果
     * @param base_filename 基础文件名
     */
    void saveMeasurementResult(const MeasurementResult& result, const std::string& base_filename);
};