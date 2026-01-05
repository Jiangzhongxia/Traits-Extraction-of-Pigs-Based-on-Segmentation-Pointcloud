#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <string>
#include <map>
#include <functional>

/**
 * @brief 体尺计算结果结构体
 * 存储猪的各项体尺测量结果
 */
struct MeasurementResult {
    double body_length = 0.0;      // 体长 (m)
    double withers_height = 0.0;   // 肩高 (m)
    double chest_depth = 0.0;      // 胸深 (m)
    double chest_width = 0.0;      // 胸宽 (m)
    double hip_height = 0.0;       // 髋高 (m)
    double hip_width = 0.0;        // 髋宽 (m)
    double abdominal_girth = 0.0;  // 腹围 (m)
    double abdominal_width = 0.0;  // 腹宽 (m)
    double abdominal_height = 0.0; // 腹高 (m)
    double head_length = 0.0;      // 头长 (m)
    double head_width = 0.0;       // 头宽 (m)
    double ear_length = 0.0;       // 耳长 (m)
    double neck_length = 0.0;      // 颈长 (m)
    double neck_girth = 0.0;       // 颈围 (m)
    double tail_length = 0.0;      // 尾长 (m)
    double tail_girth = 0.0;       // 尾围 (m)
    double front_leg_girth = 0.0;  // 前腿围 (m)
    double hind_leg_girth = 0.0;   // 后腿围 (m)

    // 计算总重量的估计值 (kg) - 基于体尺的估算
    double estimated_weight = 0.0;

    // 测量时间戳
    std::string timestamp;
};

/**
 * @brief 体尺计算计算器类
 * 根据分割后的点云数据计算猪的各项体尺参数
 */
class BodyMeasurementCalculator {
public:
    struct MeasurementParams {
        // 距离计算相关参数
        double ground_height_offset = 0.0;  // 地面高度偏移
        double min_confidence = 0.7;        // 最小置信度

        // 体尺计算相关参数
        double body_length_search_radius = 0.1;  // 体长搜索半径
        double height_search_radius = 0.05;      // 高度测量搜索半径

        // 体积估算相关参数
        double volume_voxel_size = 0.01;        // 体积估算体素大小
        double weight_coefficient = 75.0;       // 重量估算系数
    };

    BodyMeasurementCalculator();
    ~BodyMeasurementCalculator();

    /**
     * @brief 设置测量参数
     * @param params 测量参数
     */
    void setParams(const MeasurementParams& params);

    /**
     * @brief 获取当前测量参数
     * @return 当前测量参数
     */
    MeasurementParams getParams() const;

    /**
     * @brief 计算体尺参数
     * @param segments 分割后的点云数据，键为部位名称
     * @return 测量结果
     */
    MeasurementResult calculate(
        const std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>& segments);

    /**
     * @brief 计算体长（从耳根到尾根的曲线距离）
     * @param head_cloud 头部点云
     * @param torso_cloud 躯干点云
     * @param tail_cloud 尾巴点云
     * @return 体长 (m)
     */
    double calculateBodyLength(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& head_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& tail_cloud);

    /**
     * @brief 计算肩高（从肩部最高点到地面的垂直距离）
     * @param torso_cloud 躯干点云
     * @param ground_height 地面高度
     * @return 肩高 (m)
     */
    double calculateWithersHeight(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud,
        double ground_height = 0.0);

    /**
     * @brief 计算胸深（胸部前后方向的最大距离）
     * @param torso_cloud 躯干点云
     * @return 胸深 (m)
     */
    double calculateChestDepth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud);

    /**
     * @brief 计算胸宽（胸部左右方向的最大距离）
     * @param torso_cloud 躯干点云
     * @return 胸宽 (m)
     */
    double calculateChestWidth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud);

    /**
     * @brief 计算髋高（从髋部最高点到地面的垂直距离）
     * @param torso_cloud 躯干点云
     * @param ground_height 地面高度
     * @return 髋高 (m)
     */
    double calculateHipHeight(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud,
        double ground_height = 0.0);

    /**
     * @brief 计算髋宽（髋部左右方向的最大距离）
     * @param torso_cloud 躯干点云
     * @return 髋宽 (m)
     */
    double calculateHipWidth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud);

    /**
     * @brief 计算腹围（腹部周长）
     * @param torso_cloud 躯干点云
     * @return 腹围 (m)
     */
    double calculateAbdominalGirth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud);

    /**
     * @brief 计算腹宽（腹部左右方向的最大距离）
     * @param torso_cloud 躯干点云
     * @return 腹宽 (m)
     */
    double calculateAbdominalWidth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud);

    /**
     * @brief 计算腹高（腹部前后方向的最大距离）
     * @param torso_cloud 躯干点云
     * @return 腹高 (m)
     */
    double calculateAbdominalHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud);

    /**
     * @brief 估算重量
     * @param measurements 测量结果
     * @return 估算重量 (kg)
     */
    double estimateWeight(const MeasurementResult& measurements) const;

    /**
     * @brief 保存测量结果到文件
     * @param result 测量结果
     * @param filename 文件名
     */
    void saveResults(const MeasurementResult& result, const std::string& filename);

    /**
     * @brief 获取可用的测量项目列表
     * @return 测量项目名称列表
     */
    std::vector<std::string> getAvailableMeasurements() const;

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
    MeasurementParams params_;
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
     * @brief 计算点云在指定轴向的极值
     * @param cloud 输入点云
     * @param axis 轴向 (0=x, 1=y, 2=z)
     * @param min_val 最小值
     * @param max_val 最大值
     */
    void getAxisExtents(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int axis,
                       double& min_val, double& max_val) const;

    /**
     * @brief 从点云中找到最高点的z坐标
     * @param cloud 输入点云
     * @return 最高点z坐标
     */
    double getHighestZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

    /**
     * @brief 从点云中找到最低点的z坐标
     * @param cloud 输入点云
     * @param ground_height 地面高度，用于修正
     * @return 最低点z坐标
     */
    double getLowestZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                     double ground_height = 0.0) const;
};