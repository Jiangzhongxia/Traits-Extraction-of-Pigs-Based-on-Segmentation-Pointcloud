#include "VisualizationManager.h"
#include <iostream>
#include <random>

VisualizationManager::VisualizationManager()
    : initialized_(false) {
    // 初始化默认配置
    config_.show_original_cloud = true;
    config_.show_segmented_cloud = true;
    config_.show_measurements = true;
    config_.show_bounding_boxes = true;
    config_.show_coordinate_system = true;
    config_.coordinate_system_scale = 0.1;
    config_.window_width = 1200;
    config_.window_height = 800;
    config_.window_title = "Pig Measurement Visualization";
    config_.point_size = 1.0;
    config_.show_axes = true;
}

VisualizationManager::~VisualizationManager() {
    closeWindow();
}

bool VisualizationManager::initialize() {
    initializeViewer();
    return initialized_;
}

void VisualizationManager::setConfig(const VisualizationConfig& config) {
    config_ = config;
}

VisualizationManager::VisualizationConfig VisualizationManager::getConfig() const {
    return config_;
}

bool VisualizationManager::displayPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!initialized_ || !viewer_ || !cloud || cloud->empty()) {
        return false;
    }

    // 将点云添加到查看器
    std::string cloud_id = "original_cloud";
    viewer_->removePointCloud(cloud_id);

    // 创建带颜色的点云
    auto colored_cloud = addColorToCloud(cloud, 255, 255, 255); // 白色

    viewer_->addPointCloud(colored_cloud, cloud_id);
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        config_.point_size,
        cloud_id
    );

    return true;
}

bool VisualizationManager::displaySegmentedCloud(const std::map<std::string,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr>& segments) {
    if (!initialized_ || !viewer_ || segments.empty()) {
        return false;
    }

    // 清除之前的分割点云
    for (auto it = segments.begin(); it != segments.end(); ++it) {
        std::string cloud_id = "segment_" + it->first;
        viewer_->removePointCloud(cloud_id);
    }

    // 显示每个分割部分
    for (auto it = segments.begin(); it != segments.end(); ++it) {
        if (!it->second || it->second->empty()) {
            continue;
        }

        std::string cloud_id = "segment_" + it->first;
        auto color = getColorForClass(it->first);

        auto colored_cloud = addColorToCloud(it->second, color[0], color[1], color[2]);

        viewer_->addPointCloud(colored_cloud, cloud_id);
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            config_.point_size,
            cloud_id
        );
    }

    return true;
}

bool VisualizationManager::displayMeasurements(const MeasurementResult& result) {
    if (!initialized_ || !viewer_) {
        return false;
    }

    // 移除之前的测量文本
    viewer_->removeShape("measurements_text");

    // 创建测量结果文本
    std::stringstream ss;
    ss << "Measurements (Timestamp: " << result.timestamp << "):\n"
       << "Body Length: " << result.body_length << " m\n"
       << "Withers Height: " << result.withers_height << " m\n"
       << "Chest Depth: " << result.chest_depth << " m\n"
       << "Chest Width: " << result.chest_width << " m\n"
       << "Hip Height: " << result.hip_height << " m\n"
       << "Hip Width: " << result.hip_width << " m\n"
       << "Abdominal Girth: " << result.abdominal_girth << " m\n"
       << "Abdominal Width: " << result.abdominal_width << " m\n"
       << "Abdominal Height: " << result.abdominal_height << " m\n"
       << "Estimated Weight: " << result.estimated_weight << " kg";

    // 添加文本到查看器
    viewer_->addText(ss.str(), 10, 250, 12, 0, 1.0, 0, "measurements_text");

    return true;
}

bool VisualizationManager::displayBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                             const std::vector<uint8_t>& color) {
    if (!initialized_ || !viewer_ || !cloud || cloud->empty()) {
        return false;
    }

    // 计算边界框
    auto bbox = calculateBoundingBox(cloud);
    if (bbox.size() != 6) {
        return false;
    }

    // 移除旧的边界框
    viewer_->removeShape("bounding_box");

    // 添加新的边界框
    viewer_->addCube(bbox[0], bbox[1], bbox[2], bbox[3], bbox[4], bbox[5]);  // min_x, max_x, min_y, max_y, min_z, max_z
    viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        color[0]/255.0, color[1]/255.0, color[2]/255.0,
        "bounding_box"
    );
    viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
        "bounding_box"
    );

    return true;
}

bool VisualizationManager::updateConfig(const VisualizationConfig& config) {
    bool was_initialized = initialized_;
    if (was_initialized) {
        closeWindow();
    }

    setConfig(config);

    if (was_initialized) {
        return initialize();
    }

    return true;
}

bool VisualizationManager::saveScreenshot(const std::string& filename) {
    if (!initialized_ || !viewer_) {
        return false;
    }

    viewer_->saveScreenshot(filename);
    std::cout << "Screenshot saved to " << filename << std::endl;
    return true;
}

bool VisualizationManager::runVisualizationLoop() {
    if (!initialized_ || !viewer_) {
        std::cerr << "Error: Visualization not initialized" << std::endl;
        return false;
    }

    std::cout << "Visualization running. Press 'q' to quit." << std::endl;

    // 持续显示直到用户关闭窗口或按'q'
    while (!viewer_->wasStopped()) {
        viewer_->spinOnce(100);  // 等待100ms

        // 调用渲染回调（如果设置）
        if (render_callback_) {
            render_callback_();
        }
    }

    return true;
}

bool VisualizationManager::isWindowOpen() const {
    return initialized_ && viewer_ && !viewer_->wasStopped();
}

void VisualizationManager::closeWindow() {
    if (viewer_) {
        viewer_->close();
        initialized_ = false;
    }
}

void VisualizationManager::setRenderCallback(std::function<void()> callback) {
    render_callback_ = callback;
}

void VisualizationManager::setUserInteractionCallback(std::function<void(const pcl::visualization::KeyboardEvent&)> callback) {
    interaction_callback_ = callback;

    // 如果查看器已初始化，设置键盘回调
    if (viewer_) {
        viewer_->registerKeyboardCallback(
            [this](const pcl::visualization::KeyboardEvent& event) {
                if (interaction_callback_) {
                    interaction_callback_(event);
                }
            }
        );
    }
}

std::vector<uint8_t> VisualizationManager::getColorForClass(const std::string& class_name) const {
    // 为不同部位定义特定颜色
    static std::map<std::string, std::vector<uint8_t>> color_map = {
        {"background", {128, 128, 128}},   // 灰色
        {"head", {255, 0, 0}},            // 红色
        {"torso", {0, 255, 0}},           // 绿色
        {"front_left_leg", {0, 0, 255}},  // 蓝色
        {"front_right_leg", {255, 255, 0}}, // 青色
        {"hind_left_leg", {255, 0, 255}},   // 洋红色
        {"hind_right_leg", {0, 255, 255}},  // 黄色
        {"tail", {128, 0, 128}},          // 紫色
        {"ear", {255, 165, 0}},           // 橙色
        {"neck", {139, 69, 19}}           // 棕色
    };

    auto it = color_map.find(class_name);
    if (it != color_map.end()) {
        return it->second;
    }

    // 如果类别未定义，生成随机颜色
    return generateRandomColor();
}

void VisualizationManager::initializeViewer() {
    std::string viewer_name = config_.window_title;
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>(viewer_name);
    viewer_->setSize(config_.window_width, config_.window_height);
    viewer_->setBackgroundColor(0.1, 0.1, 0.1); // 深灰色背景

    // 添加坐标系
    if (config_.show_coordinate_system) {
        viewer_->addCoordinateSystem(config_.coordinate_system_scale);
    }

    // 设置查看器属性
    viewer_->initCameraParameters();
    viewer_->setShowFPS(true);

    initialized_ = true;

    // 设置键盘回调（如果已定义）
    if (interaction_callback_) {
        viewer_->registerKeyboardCallback(
            [this](const pcl::visualization::KeyboardEvent& event) {
                if (interaction_callback_) {
                    interaction_callback_(event);
                }
            }
        );
    }
}

std::vector<uint8_t> VisualizationManager::generateRandomColor() const {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(50, 255);  // 从50到255之间生成，避免太暗的颜色

    return {static_cast<uint8_t>(dis(gen)),
            static_cast<uint8_t>(dis(gen)),
            static_cast<uint8_t>(dis(gen))};
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr VisualizationManager::addColorToCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    uint8_t r, uint8_t g, uint8_t b) const {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->resize(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i) {
        pcl::PointXYZRGB point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        point.r = r;
        point.g = g;
        point.b = b;
        colored_cloud->points[i] = point;
    }

    return colored_cloud;
}

std::vector<double> VisualizationManager::calculateBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
    if (cloud->empty()) {
        return {};
    }

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    return {min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z};
}