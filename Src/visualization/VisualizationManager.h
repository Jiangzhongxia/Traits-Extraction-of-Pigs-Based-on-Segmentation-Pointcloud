#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <functional>

#include "BodyMeasurementCalculator.h"  // 包含测量结果结构体

/**
 * @brief 可视化管理器类
 * 管理点云数据可视化、测量结果显示等功能
 */
class VisualizationManager {
public:
    struct VisualizationConfig {
        bool show_original_cloud = true;
        bool show_segmented_cloud = true;
        bool show_measurements = true;
        bool show_bounding_boxes = true;
        bool show_coordinate_system = true;
        double coordinate_system_scale = 0.1;

        // 窗口设置
        int window_width = 1200;
        int window_height = 800;
        std::string window_title = "Pig Measurement Visualization";

        // 点云渲染设置
        double point_size = 1.0;
        bool show_axes = true;
    };

    VisualizationManager();
    ~VisualizationManager();

    /**
     * @brief 初始化可视化窗口
     * @return 是否初始化成功
     */
    bool initialize();

    /**
     * @brief 设置可视化配置
     * @param config 可视化配置
     */
    void setConfig(const VisualizationConfig& config);

    /**
     * @brief 获取当前可视化配置
     * @return 当前可视化配置
     */
    VisualizationConfig getConfig() const;

    /**
     * @brief 显示原始点云
     * @param cloud 输入点云
     * @return 是否显示成功
     */
    bool displayPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief 显示分割后的点云（不同颜色表示不同部位）
     * @param segments 分割后的点云数据
     * @return 是否显示成功
     */
    bool displaySegmentedCloud(const std::map<std::string,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr>& segments);

    /**
     * @brief 显示测量结果
     * @param result 测量结果
     * @return 是否显示成功
     */
    bool displayMeasurements(const MeasurementResult& result);

    /**
     * @brief 显示点云边界框
     * @param cloud 输入点云
     * @param color RGB颜色值 [0-255]
     * @return 是否显示成功
     */
    bool displayBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                           const std::vector<uint8_t>& color = {255, 0, 0});

    /**
     * @brief 更新可视化配置
     * @param config 新的可视化配置
     * @return 是否更新成功
     */
    bool updateConfig(const VisualizationConfig& config);

    /**
     * @brief 保存当前视图为截图
     * @param filename 文件名
     * @return 是否保存成功
     */
    bool saveScreenshot(const std::string& filename);

    /**
     * @brief 运行可视化循环
     * @return 是否成功运行
     */
    bool runVisualizationLoop();

    /**
     * @brief 检查可视化窗口是否仍打开
     * @return 窗口是否打开
     */
    bool isWindowOpen() const;

    /**
     * @brief 关闭可视化窗口
     */
    void closeWindow();

    /**
     * @brief 设置渲染完成回调函数
     * @param callback 回调函数
     */
    void setRenderCallback(std::function<void()> callback);

    /**
     * @brief 设置用户交互回调函数
     * @param callback 回调函数
     */
    void setUserInteractionCallback(std::function<void(const pcl::visualization::KeyboardEvent&)> callback);

    /**
     * @brief 为不同部位生成颜色
     * @param class_name 部位名称
     * @return RGB颜色值 [0-255]
     */
    std::vector<uint8_t> getColorForClass(const std::string& class_name) const;

private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    VisualizationConfig config_;
    bool initialized_;
    std::function<void()> render_callback_;
    std::function<void(const pcl::visualization::KeyboardEvent&)> interaction_callback_;

    /**
     * @brief 初始化可视化查看器
     */
    void initializeViewer();

    /**
     * @brief 生成随机颜色
     * @return RGB颜色值 [0-255]
     */
    std::vector<uint8_t> generateRandomColor() const;

    /**
     * @brief 为点云添加颜色
     * @param cloud 输入点云
     * @param r, g, b 颜色值 [0-255]
     * @return 带颜色的点云
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr addColorToCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        uint8_t r, uint8_t g, uint8_t b) const;

    /**
     * @brief 计算点云的边界框
     * @param cloud 输入点云
     * @return 边界框坐标 [min_x, max_x, min_y, max_y, min_z, max_z]
     */
    std::vector<double> calculateBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;
};