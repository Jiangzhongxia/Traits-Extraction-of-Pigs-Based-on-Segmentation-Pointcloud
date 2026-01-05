#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>

/**
 * @brief 图达通灵雀w激光雷达接口类
 * 用于同步收集猪的左右两边点云信息
 */
class LivoxLidarInterface {
public:
    struct DeviceConfig {
        std::string ip_address;
        int port;
        std::string device_type; // "left" or "right"
    };

    struct ConnectionStatus {
        bool left_connected = false;
        bool right_connected = false;
        std::string left_status = "Disconnected";
        std::string right_status = "Disconnected";
        int left_point_count = 0;
        int right_point_count = 0;
    };

    LivoxLidarInterface();
    ~LivoxLidarInterface();

    /**
     * @brief 初始化激光雷达连接
     * @param configs 设备配置列表
     * @return 是否初始化成功
     */
    bool initialize(const std::vector<DeviceConfig>& configs);

    /**
     * @brief 启动点云数据采集
     * @return 是否启动成功
     */
    bool startCapture();

    /**
     * @brief 停止点云数据采集
     * @return 是否停止成功
     */
    bool stopCapture();

    /**
     * @brief 获取左侧点云数据
     * @return 左侧点云指针
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getLeftPointCloud();

    /**
     * @brief 获取右侧点云数据
     * @return 右侧点云指针
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getRightPointCloud();

    /**
     * @brief 同步获取左右点云并融合
     * @param combined 融合后的点云
     * @return 是否成功获取同步数据
     */
    bool syncPointClouds(pcl::PointCloud<pcl::PointXYZI>::Ptr& combined);

    /**
     * @brief 获取连接状态
     * @return 连接状态结构体
     */
    ConnectionStatus getConnectionStatus() const;

    /**
     * @brief 设置数据回调函数
     * @param callback 回调函数
     */
    void setDataCallback(std::function<void(const pcl::PointCloud<pcl::PointXYZI>::Ptr&,
                                           const std::string& side)> callback);

    /**
     * @brief 检查设备是否连接
     * @return 是否所有设备已连接
     */
    bool isDeviceConnected() const;

private:
    std::vector<DeviceConfig> device_configs_;
    bool initialized_;
    bool capturing_;
    std::thread capture_thread_;
    std::mutex data_mutex_;
    std::mutex status_mutex_;
    std::condition_variable cv_;
    bool should_stop_;

    // 存储左右侧点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr left_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr combined_cloud_;

    // 连接状态
    mutable std::mutex status_mutex_;
    ConnectionStatus status_;

    // 回调函数
    std::function<void(const pcl::PointCloud<pcl::PointXYZI>::Ptr&,
                      const std::string& side)> data_callback_;

    // 模拟数据采集线程函数
    void captureThreadFunction();

    // 模拟连接函数
    bool connectToDevice(const DeviceConfig& config);

    // 模拟断开连接函数
    void disconnectFromDevice();
};