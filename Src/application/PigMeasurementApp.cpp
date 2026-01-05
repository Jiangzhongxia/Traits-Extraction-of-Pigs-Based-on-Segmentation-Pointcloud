#include "PigMeasurementApp.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

PigMeasurementApp::PigMeasurementApp() {
    // 初始化默认配置
    config_.config_file = "config/app_config.json";
    config_.output_directory = "results/";
    config_.model_path = "models/pig_segmentation.onnx";
    config_.enable_visualization = true;
    config_.save_raw_data = true;
    config_.save_intermediate = true;
    config_.save_final = true;
    config_.enable_real_time = true;
    config_.processing_interval_ms = 1000;
}

PigMeasurementApp::~PigMeasurementApp() {
    stop();
}

bool PigMeasurementApp::initialize(const AppConfig& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = config;

    updateStatus(false, "Initializing", 0.0);

    if (!initializeModules()) {
        handleError("Failed to initialize modules");
        return false;
    }

    // 创建输出目录
    createOutputDirectories();

    status_.initialized = true;
    updateStatus(false, "Initialized", 100.0);

    std::cout << "PigMeasurementApp initialized successfully" << std::endl;
    return true;
}

bool PigMeasurementApp::start() {
    if (!status_.initialized) {
        handleError("Application not initialized");
        return false;
    }

    updateStatus(true, "Starting", 0.0);

    // 启动主处理线程
    main_thread_ = std::thread(&PigMeasurementApp::mainProcessingLoop, this);

    updateStatus(true, "Started", 100.0);

    std::cout << "PigMeasurementApp started" << std::endl;
    return true;
}

bool PigMeasurementApp::runMeasurementCycle() {
    if (!status_.initialized) {
        handleError("Application not initialized");
        return false;
    }

    updateStatus(true, "Measurement cycle start", 0.0);

    try {
        // 1. 获取点云数据
        pcl::PointCloud<pcl::PointXYZI>::Ptr left_cloud, right_cloud;
        if (config_.enable_real_time) {
            // 实时模式：从激光雷达获取数据
            left_cloud = lidar_interface_->getLeftPointCloud();
            right_cloud = lidar_interface_->getRightPointCloud();

            // 同步融合点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            if (!lidar_interface_->syncPointClouds(combined_cloud) || combined_cloud->empty()) {
                handleError("Failed to get combined point cloud from lidar");
                return false;
            }

            // 转换为PointXYZ格式（移除强度信息）
            pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& point : combined_cloud->points) {
                pcl::PointXYZ xyz_point;
                xyz_point.x = point.x;
                xyz_point.y = point.y;
                xyz_point.z = point.z;
                processed_cloud->push_back(xyz_point);
            }
        } else {
            // 文件模式：需要通过processFromFile方法处理
            handleError("Real-time mode disabled, use processFromFile instead");
            return false;
        }

        updateStatus(true, "Point cloud acquired", 20.0);

        // 2. 预处理点云
        auto preprocessed_cloud = pointcloud_processor_->processClouds(
            processed_cloud, processed_cloud); // 简化处理，实际应用中需要合适的输入

        updateStatus(true, "Point cloud preprocessed", 40.0);

        // 3. 分割点云
        auto segmentation_result = segmenter_->segment(preprocessed_cloud);

        updateStatus(true, "Point cloud segmented", 60.0);

        // 4. 计算体尺
        auto measurement_result = calculator_->calculate(segmentation_result.segments);

        updateStatus(true, "Measurements calculated", 80.0);

        // 保存结果
        std::string timestamp = std::to_string(std::time(nullptr));
        saveMeasurementResult(measurement_result, config_.output_directory + "measurement_" + timestamp);

        // 调用测量结果回调
        if (measurement_callback_) {
            measurement_callback_(measurement_result);
        }

        // 可视化结果（如果启用）
        if (config_.enable_visualization && visualizer_) {
            visualizer_->displaySegmentedCloud(segmentation_result.segments);
            visualizer_->displayMeasurements(measurement_result);
        }

        updateStatus(true, "Measurement cycle completed", 100.0);

        return true;
    } catch (const std::exception& e) {
        handleError(std::string("Exception in measurement cycle: ") + e.what());
        return false;
    }
}

bool PigMeasurementApp::processSinglePig() {
    return runMeasurementCycle();
}

void PigMeasurementApp::stop() {
    updateStatus(false, "Stopping", 0.0);

    status_.running = false;
    status_.capturing = false;
    status_.processing = false;

    if (main_thread_.joinable()) {
        main_thread_.join();
    }

    // 停止激光雷达接口
    if (lidar_interface_) {
        lidar_interface_->stopCapture();
    }

    updateStatus(false, "Stopped", 100.0);

    std::cout << "PigMeasurementApp stopped" << std::endl;
}

bool PigMeasurementApp::isRunning() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_.running;
}

PigMeasurementApp::AppStatus PigMeasurementApp::getStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

void PigMeasurementApp::setProcessingCallback(std::function<void(double progress, const std::string& status)> callback) {
    processing_callback_ = callback;
}

void PigMeasurementApp::setMeasurementCallback(std::function<void(const MeasurementResult& result)> callback) {
    measurement_callback_ = callback;
}

void PigMeasurementApp::setErrorCallback(std::function<void(const std::string& error)> callback) {
    error_callback_ = callback;
}

bool PigMeasurementApp::processFromFile(const std::string& file_path) {
    // 加载点云文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPLYFile(file_path, *cloud) == -1) {
        if (pcl::io::loadPCDFile(file_path, *cloud) == -1) {
            handleError("Failed to load point cloud file: " + file_path);
            return false;
        }
    }

    if (cloud->empty()) {
        handleError("Loaded point cloud is empty: " + file_path);
        return false;
    }

    updateStatus(true, "File loaded", 10.0);

    // 预处理点云
    auto preprocessed_cloud = pointcloud_processor_->processClouds(cloud, cloud);

    updateStatus(true, "Point cloud preprocessed", 40.0);

    // 分割点云
    auto segmentation_result = segmenter_->segment(preprocessed_cloud);

    updateStatus(true, "Point cloud segmented", 70.0);

    // 计算体尺
    auto measurement_result = calculator_->calculate(segmentation_result.segments);

    updateStatus(true, "Measurements calculated", 90.0);

    // 保存结果
    std::string filename = config_.output_directory + "measurement_" +
                          file_path.substr(file_path.find_last_of("/\\") + 1) + "_" +
                          std::to_string(std::time(nullptr));
    saveMeasurementResult(measurement_result, filename);

    // 调用测量结果回调
    if (measurement_callback_) {
        measurement_callback_(measurement_result);
    }

    // 可视化结果（如果启用）
    if (config_.enable_visualization && visualizer_) {
        visualizer_->displaySegmentedCloud(segmentation_result.segments);
        visualizer_->displayMeasurements(measurement_result);
    }

    updateStatus(true, "File processing completed", 100.0);

    return true;
}

void PigMeasurementApp::saveConfig(const std::string& config_file) {
    std::ofstream file(config_file);
    if (file.is_open()) {
        file << "{\n";
        file << "  \"output_directory\": \"" << config_.output_directory << "\",\n";
        file << "  \"model_path\": \"" << config_.model_path << "\",\n";
        file << "  \"enable_visualization\": " << (config_.enable_visualization ? "true" : "false") << ",\n";
        file << "  \"save_raw_data\": " << (config_.save_raw_data ? "true" : "false") << ",\n";
        file << "  \"save_intermediate\": " << (config_.save_intermediate ? "true" : "false") << ",\n";
        file << "  \"save_final\": " << (config_.save_final ? "true" : "false") << ",\n";
        file << "  \"enable_real_time\": " << (config_.enable_real_time ? "true" : "false") << ",\n";
        file << "  \"processing_interval_ms\": " << config_.processing_interval_ms << "\n";
        file << "}\n";
        file.close();
    }
}

void PigMeasurementApp::loadConfig(const std::string& config_file) {
    std::ifstream file(config_file);
    if (file.is_open()) {
        // 简单的JSON解析（在实际应用中应该使用专门的JSON库）
        // 这里只是示例，实际实现需要更健壮的JSON解析
        std::string line;
        while (std::getline(file, line)) {
            // 实现简单的配置加载逻辑
            // 在实际应用中，应该使用JSON解析库如nlohmann/json
        }
        file.close();
    }
}

bool PigMeasurementApp::initializeModules() {
    // 初始化性能优化器
    optimizer_ = std::make_unique<PerformanceOptimizer>();

    // 初始化激光雷达接口
    lidar_interface_ = std::make_unique<LivoxLidarInterface>();

    // 配置激光雷达参数
    std::vector<LivoxLidarInterface::DeviceConfig> lidar_configs;
    LivoxLidarInterface::DeviceConfig left_config;
    left_config.ip_address = "192.168.1.10";
    left_config.port = 65000;
    left_config.device_type = "left";
    lidar_configs.push_back(left_config);

    LivoxLidarInterface::DeviceConfig right_config;
    right_config.ip_address = "192.168.1.11";
    right_config.port = 65000;
    right_config.device_type = "right";
    lidar_configs.push_back(right_config);

    if (!lidar_interface_->initialize(lidar_configs)) {
        std::cerr << "Failed to initialize lidar interface" << std::endl;
        return false;
    }

    // 初始化点云处理器
    pointcloud_processor_ = std::make_unique<PointCloudProcessor>();
    PointCloudProcessor::ProcessingParams proc_params;
    proc_params.voxel_leaf_size = 0.01;
    proc_params.outlier_radius = 0.1;
    proc_params.registration_max_distance = 0.05;
    pointcloud_processor_->setParams(proc_params);

    // 设置回调函数
    pointcloud_processor_->setProgressCallback([this](double progress, const std::string& stage) {
        if (processing_callback_) {
            processing_callback_(progress * 0.3 + 0.4, "Processing: " + stage);
        }
    });

    // 初始化分割器
    segmenter_ = std::make_unique<ONNXSegmenter>();
    ONNXSegmenter::ONNXModelConfig model_config;
    model_config.model_path = config_.model_path;
    model_config.input_size = 8192;
    model_config.confidence_threshold = 0.7f;

    if (!segmenter_->loadModel(model_config)) {
        std::cerr << "Failed to load segmentation model: " << config_.model_path << std::endl;
        return false;
    }

    // 初始化体尺计算器
    calculator_ = std::make_unique<BodyMeasurementCalculator>();
    BodyMeasurementCalculator::MeasurementParams calc_params;
    calc_params.ground_height_offset = 0.0;
    calc_params.min_confidence = 0.7;
    calculator_->setParams(calc_params);

    // 设置回调函数
    calculator_->setProgressCallback([this](double progress, const std::string& stage) {
        if (processing_callback_) {
            processing_callback_(progress * 0.2 + 0.7, "Calculating: " + stage);
        }
    });

    // 初始化可视化管理器（如果启用）
    if (config_.enable_visualization) {
        visualizer_ = std::make_unique<VisualizationManager>();
        VisualizationManager::VisualizationConfig viz_config;
        viz_config.window_width = 1200;
        viz_config.window_height = 800;
        viz_config.show_segmented_cloud = true;
        viz_config.show_measurements = true;
        visualizer_->setConfig(viz_config);

        if (!visualizer_->initialize()) {
            std::cerr << "Warning: Failed to initialize visualizer" << std::endl;
            // 可视化失败不是致命错误，继续初始化
        }
    }

    return true;
}

void PigMeasurementApp::mainProcessingLoop() {
    status_.running = true;

    // 启动激光雷达数据采集
    if (lidar_interface_) {
        lidar_interface_->startCapture();
    }

    while (status_.running) {
        auto start_time = std::chrono::steady_clock::now();

        // 运行测量周期
        if (!runMeasurementCycle()) {
            std::cerr << "Measurement cycle failed" << std::endl;
        }

        // 控制处理频率
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        auto sleep_time = std::max(0L, (long)config_.processing_interval_ms - elapsed_ms);

        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
        }
    }

    // 停止激光雷达采集
    if (lidar_interface_) {
        lidar_interface_->stopCapture();
    }
}

void PigMeasurementApp::updateStatus(bool running, const std::string& stage, double progress) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.running = running;
    status_.current_stage = stage;
    status_.progress = progress;

    // 调用进度回调
    if (processing_callback_) {
        processing_callback_(progress / 100.0, stage);
    }
}

void PigMeasurementApp::handleError(const std::string& error) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.error_message = error;

    std::cerr << "Error: " << error << std::endl;

    // 调用错误回调
    if (error_callback_) {
        error_callback_(error);
    }
}

void PigMeasurementApp::createOutputDirectories() {
    // 在实际实现中，应该使用适当的文件系统操作
    // 这里简化为打印信息
    std::cout << "Creating output directory: " << config_.output_directory << std::endl;
}

void PigMeasurementApp::saveMeasurementResult(const MeasurementResult& result, const std::string& base_filename) {
    if (config_.save_final) {
        calculator_->saveResults(result, base_filename + ".txt");
    }
}