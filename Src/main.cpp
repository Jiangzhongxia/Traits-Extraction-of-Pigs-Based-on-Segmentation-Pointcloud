/**
 * @file main.cpp
 * @brief 猪体尺测量系统主程序
 *
 * 该程序实现了完整的猪体尺测量流程，包括：
 * - 激光雷达数据采集
 * - 点云预处理
 * - 点云分割
 * - 体尺参数计算
 * - 结果可视化
 */

#include <iostream>
#include <signal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "application/PigMeasurementApp.h"

// 全局变量用于优雅退出
static bool should_exit = false;

// 信号处理函数
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down gracefully..." << std::endl;
    should_exit = true;
}

int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "    猪体尺测量系统 V1.0" << std::endl;
    std::cout << "    基于双目激光雷达的自动化测量" << std::endl;
    std::cout << "========================================" << std::endl;

    // 注册信号处理函数，用于优雅退出
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        // 创建应用程序实例
        PigMeasurementApp app;

        // 配置应用程序参数
        PigMeasurementApp::AppConfig config;
        config.output_directory = "results/";
        config.model_path = "models/pig_segmentation.onnx";
        config.enable_visualization = true;  // 启用可视化
        config.enable_real_time = true;      // 启用实时处理
        config.processing_interval_ms = 1000; // 处理间隔1秒

        std::cout << "初始化应用程序..." << std::endl;

        // 初始化应用程序
        if (!app.initialize(config)) {
            std::cerr << "错误: 应用程序初始化失败" << std::endl;
            return -1;
        }

        // 设置回调函数
        app.setProcessingCallback([](double progress, const std::string& status) {
            std::cout << "进度: " << static_cast<int>(progress * 100) << "% - " << status << std::endl;
        });

        app.setMeasurementCallback([](const MeasurementResult& result) {
            std::cout << "\n测量完成!" << std::endl;
            std::cout << "  体长: " << result.body_length << " m" << std::endl;
            std::cout << "  肩高: " << result.withers_height << " m" << std::endl;
            std::cout << "  胸深: " << result.chest_depth << " m" << std::endl;
            std::cout << "  胸宽: " << result.chest_width << " m" << std::endl;
            std::cout << "  髋高: " << result.hip_height << " m" << std::endl;
            std::cout << "  髋宽: " << result.hip_width << " m" << std::endl;
            std::cout << "  腹围: " << result.abdominal_girth << " m" << std::endl;
            std::cout << "  估算重量: " << result.estimated_weight << " kg" << std::endl;
            std::cout << "  时间戳: " << result.timestamp << std::endl;
            std::cout << "----------------------------------------" << std::endl;
        });

        app.setErrorCallback([](const std::string& error) {
            std::cerr << "错误: " << error << std::endl;
        });

        std::cout << "启动应用程序..." << std::endl;

        // 启动应用程序
        if (!app.start()) {
            std::cerr << "错误: 应用程序启动失败" << std::endl;
            return -1;
        }

        std::cout << "应用程序已启动，按 Ctrl+C 退出" << std::endl;

        // 主循环 - 让应用程序运行
        while (!should_exit && app.isRunning()) {
            auto status = app.getStatus();
            std::cout << "状态: " << status.current_stage
                      << " | 进度: " << static_cast<int>(status.progress) << "%" << std::endl;

            // 每隔5秒输出一次状态
            for (int i = 0; i < 5 && !should_exit; ++i) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        std::cout << "\n正在停止应用程序..." << std::endl;

        // 停止应用程序
        app.stop();

        std::cout << "应用程序已安全退出" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "程序异常: " << e.what() << std::endl;
        return -1;
    }

    std::cout << "谢谢使用猪体尺测量系统!" << std::endl;
    return 0;
}