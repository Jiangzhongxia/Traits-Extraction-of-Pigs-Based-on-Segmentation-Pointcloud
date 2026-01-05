#include "LivoxLidarInterface.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

// 模拟Livox SDK头文件 - 在实际项目中需要替换为真实的SDK
// #include "livox_sdk.h"

LivoxLidarInterface::LivoxLidarInterface()
    : initialized_(false),
      capturing_(false),
      should_stop_(false),
      left_cloud_(new pcl::PointCloud<pcl::PointXYZI>),
      right_cloud_(new pcl::PointCloud<pcl::PointXYZI>),
      combined_cloud_(new pcl::PointCloud<pcl::PointXYZI>) {
}

LivoxLidarInterface::~LivoxLidarInterface() {
    stopCapture();
    disconnectFromDevice();
}

bool LivoxLidarInterface::initialize(const std::vector<DeviceConfig>& configs) {
    if (configs.size() != 2) {
        std::cerr << "Error: Expected exactly 2 devices (left and right)" << std::endl;
        return false;
    }

    // 验证设备配置
    bool has_left = false, has_right = false;
    for (const auto& config : configs) {
        if (config.device_type == "left") {
            has_left = true;
        } else if (config.device_type == "right") {
            has_right = true;
        }
    }

    if (!has_left || !has_right) {
        std::cerr << "Error: Need both left and right device configurations" << std::endl;
        return false;
    }

    device_configs_ = configs;

    // 模拟连接设备
    for (const auto& config : configs) {
        if (!connectToDevice(config)) {
            std::cerr << "Failed to connect to " << config.device_type << " device" << std::endl;
            return false;
        }
    }

    initialized_ = true;
    std::cout << "LivoxLidarInterface initialized successfully" << std::endl;
    return true;
}

bool LivoxLidarInterface::startCapture() {
    if (!initialized_) {
        std::cerr << "Error: Interface not initialized" << std::endl;
        return false;
    }

    if (capturing_) {
        std::cout << "Capture already running" << std::endl;
        return true;
    }

    capturing_ = true;
    should_stop_ = false;

    // 启动数据采集线程
    capture_thread_ = std::thread(&LivoxLidarInterface::captureThreadFunction, this);

    std::cout << "Started point cloud capture" << std::endl;
    return true;
}

bool LivoxLidarInterface::stopCapture() {
    if (!capturing_) {
        return true; // Already stopped
    }

    should_stop_ = true;

    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    capturing_ = false;
    std::cout << "Stopped point cloud capture" << std::endl;
    return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LivoxLidarInterface::getLeftPointCloud() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto cloud_copy = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*left_cloud_);
    return cloud_copy;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LivoxLidarInterface::getRightPointCloud() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto cloud_copy = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*right_cloud_);
    return cloud_copy;
}

bool LivoxLidarInterface::syncPointClouds(pcl::PointCloud<pcl::PointXYZI>::Ptr& combined) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (left_cloud_->empty() && right_cloud_->empty()) {
        return false;
    }

    // 创建组合点云
    combined->clear();

    // 添加左侧点云
    for (const auto& point : left_cloud_->points) {
        combined->push_back(point);
    }

    // 添加右侧点云
    for (const auto& point : right_cloud_->points) {
        combined->push_back(point);
    }

    return !combined->empty();
}

LivoxLidarInterface::ConnectionStatus LivoxLidarInterface::getConnectionStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

void LivoxLidarInterface::setDataCallback(std::function<void(const pcl::PointCloud<pcl::PointXYZI>::Ptr&,
                                                            const std::string& side)> callback) {
    data_callback_ = callback;
}

bool LivoxLidarInterface::isDeviceConnected() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_.left_connected && status_.right_connected;
}

void LivoxLidarInterface::captureThreadFunction() {
    while (!should_stop_) {
        // 模拟数据采集 - 在实际实现中这里会从Livox SDK获取数据
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟数据采集间隔

        // 模拟生成一些点云数据
        {
            std::lock_guard<std::mutex> lock(data_mutex_);

            // 清空之前的点云数据
            left_cloud_->clear();
            right_cloud_->clear();

            // 生成模拟的左侧点云数据
            for (int i = 0; i < 1000; ++i) {
                pcl::PointXYZI point;
                point.x = static_cast<float>(-1.0 + (rand() % 1000) / 100.0); // 范围 -1.0 到 9.0
                point.y = static_cast<float>(-0.5 + (rand() % 1000) / 1000.0); // 范围 -0.5 到 0.5
                point.z = static_cast<float>((rand() % 1000) / 100.0); // 范围 0.0 到 10.0
                point.intensity = static_cast<float>(rand() % 255); // 模拟强度值
                left_cloud_->push_back(point);
            }

            // 生成模拟的右侧点云数据
            for (int i = 0; i < 1000; ++i) {
                pcl::PointXYZI point;
                point.x = static_cast<float>(1.0 + (rand() % 1000) / 100.0); // 范围 1.0 到 11.0
                point.y = static_cast<float>(-0.5 + (rand() % 1000) / 1000.0); // 范围 -0.5 到 0.5
                point.z = static_cast<float>((rand() % 1000) / 100.0); // 范围 0.0 到 10.0
                point.intensity = static_cast<float>(rand() % 255); // 模拟强度值
                right_cloud_->push_back(point);
            }

            // 更新状态信息
            {
                std::lock_guard<std::mutex> lock_status(status_mutex_);
                status_.left_point_count = left_cloud_->size();
                status_.right_point_count = right_cloud_->size();
            }

            // 如果设置了回调函数，则调用
            if (data_callback_) {
                data_callback_(left_cloud_, "left");
                data_callback_(right_cloud_, "right");
            }
        }
    }
}

bool LivoxLidarInterface::connectToDevice(const DeviceConfig& config) {
    // 模拟连接设备
    std::cout << "Connecting to " << config.device_type << " device at "
              << config.ip_address << ":" << config.port << std::endl;

    // 在实际实现中，这里会使用Livox SDK连接到设备
    // Livox SDK连接代码示例（伪代码）：
    /*
    livox_status status = livox_sdk_init();
    if (status != kLivoxSdkSuccess) {
        return false;
    }

    // 设置设备连接参数
    DeviceConfig livox_config;
    // ... 配置参数

    // 连接设备
    status = livox_add_device_by_ip(config.ip_address.c_str());
    if (status != kLivoxSdkSuccess) {
        return false;
    }
    */

    // 模拟连接成功
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟连接时间

    // 更新连接状态
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (config.device_type == "left") {
            status_.left_connected = true;
            status_.left_status = "Connected";
        } else if (config.device_type == "right") {
            status_.right_connected = true;
            status_.right_status = "Connected";
        }
    }

    std::cout << config.device_type << " device connected successfully" << std::endl;
    return true;
}

void LivoxLidarInterface::disconnectFromDevice() {
    std::cout << "Disconnecting from devices..." << std::endl;

    // 在实际实现中，这里会断开Livox SDK连接
    // Livox SDK断开连接代码示例（伪代码）：
    /*
    livox_disconnect_all_device();
    livox_sdk_uninit();
    */

    // 更新连接状态
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.left_connected = false;
        status_.right_connected = false;
        status_.left_status = "Disconnected";
        status_.right_status = "Disconnected";
    }

    std::cout << "Disconnected from all devices" << std::endl;
}