#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <vector>
#include <string>
#include <functional>
#include <memory>

/**
 * @brief 点云处理器类
 * 负责点云降噪、配准、法向量定向等预处理功能
 */
class PointCloudProcessor {
public:
    struct ProcessingParams {
        // 降采样参数
        double voxel_leaf_size = 0.01;

        // 离群点移除参数
        double outlier_radius = 0.1;
        int outlier_min_neighbors = 10;

        // 配准参数
        double registration_max_distance = 0.05;
        int registration_max_iterations = 100;
        double registration_transformation_epsilon = 1e-6;
        double registration_euclidean_fitness_epsilon = 0.1;

        // 法向量估计参数
        double normal_search_radius = 0.03;

        // 是否为噪声输入
        bool is_noisy_input = true;

        // 其他参数
        bool use_implicit_orient = true;  // 是否使用隐式定向
        bool oriented_optimized = true;   // 是否使用定向优化
    };

    PointCloudProcessor();
    ~PointCloudProcessor();

    /**
     * @brief 设置处理参数
     * @param params 处理参数
     */
    void setParams(const ProcessingParams& params);

    /**
     * @brief 获取当前处理参数
     * @return 当前处理参数
     */
    ProcessingParams getParams() const;

    /**
     * @brief 点云降噪处理
     * @param input 输入点云
     * @return 降噪后的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr denoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input);

    /**
     * @brief 点云降采样
     * @param input 输入点云
     * @param leaf_size 降采样叶大小
     * @return 降采样后的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                                  double leaf_size = -1.0);

    /**
     * @brief 点云配准与融合
     * @param left_cloud 左侧点云
     * @param right_cloud 右侧点云
     * @param transform 输出变换矩阵
     * @return 配准融合后的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr registerClouds(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& left_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& right_cloud,
        Eigen::Matrix4f& transform);

    /**
     * @brief 批量处理点云（降噪、降采样、配准）
     * @param left_cloud 左侧点云
     * @param right_cloud 右侧点云
     * @return 处理后的融合点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr processClouds(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& left_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& right_cloud);

    /**
     * @brief 使用IsoConstraints算法定向法向量
     * @param input 带法线的输入点云
     * @param is_noisy 是否为噪声输入
     * @return 定向后的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr orientNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        bool is_noisy = true);

    /**
     * @brief 设置处理进度回调函数
     * @param callback 回调函数
     */
    void setProgressCallback(std::function<void(double progress, const std::string& stage)> callback);

    /**
     * @brief 设置统计信息回调函数
     * @param callback 回调函数
     */
    void setStatsCallback(std::function<void(const std::string& stage, const std::string& stats)> callback);

private:
    ProcessingParams params_;
    std::function<void(double progress, const std::string& stage)> progress_callback_;
    std::function<void(const std::string& stage, const std::string& stats)> stats_callback_;

    /**
     * @brief 应用统计回调
     * @param stage 阶段名称
     * @param stats 统计信息
     */
    void callStatsCallback(const std::string& stage, const std::string& stats) const;

    /**
     * @brief 应用进度回调
     * @param progress 进度(0-1)
     * @param stage 阶段名称
     */
    void callProgressCallback(double progress, const std::string& stage) const;

    /**
     * @brief 估计点云法线
     * @param cloud 输入点云
     * @return 带法线的点云
     */
    pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};