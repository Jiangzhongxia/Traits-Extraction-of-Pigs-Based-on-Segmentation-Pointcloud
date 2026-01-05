#include "PointCloudProcessor.h"
#include <iostream>
#include <chrono>

PointCloudProcessor::PointCloudProcessor() {
    // 初始化默认参数
    params_.voxel_leaf_size = 0.01;
    params_.outlier_radius = 0.1;
    params_.outlier_min_neighbors = 10;
    params_.registration_max_distance = 0.05;
    params_.registration_max_iterations = 100;
    params_.normal_search_radius = 0.03;
    params_.is_noisy_input = true;
}

PointCloudProcessor::~PointCloudProcessor() {
}

void PointCloudProcessor::setParams(const ProcessingParams& params) {
    params_ = params;
}

PointCloudProcessor::ProcessingParams PointCloudProcessor::getParams() const {
    return params_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::denoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) {
    if (!input || input->empty()) {
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    callProgressCallback(0.1, "Denoising start");

    // 使用统计离群点移除滤波器
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    sor.setInputCloud(input);
    sor.setMeanK(params_.outlier_min_neighbors);
    sor.setStddevMulThresh(1.0); // 标准差倍数阈值

    sor.filter(*cloud_filtered);

    callStatsCallback("Denoising",
        "Input points: " + std::to_string(input->size()) +
        ", Output points: " + std::to_string(cloud_filtered->size()));

    callProgressCallback(0.9, "Denoising done");

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                                                   double leaf_size) {
    if (!input || input->empty()) {
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    callProgressCallback(0.1, "Downsampling start");

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    voxel_grid.setInputCloud(input);
    voxel_grid.setLeafSize(
        (leaf_size > 0) ? leaf_size : params_.voxel_leaf_size,
        (leaf_size > 0) ? leaf_size : params_.voxel_leaf_size,
        (leaf_size > 0) ? leaf_size : params_.voxel_leaf_size
    );

    voxel_grid.filter(*cloud_filtered);

    callStatsCallback("Downsampling",
        "Input points: " + std::to_string(input->size()) +
        ", Output points: " + std::to_string(cloud_filtered->size()));

    callProgressCallback(0.9, "Downsampling done");

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::registerClouds(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& left_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& right_cloud,
    Eigen::Matrix4f& transform) {

    if (!left_cloud || left_cloud->empty() || !right_cloud || right_cloud->empty()) {
        std::cerr << "Error: One or both input clouds are empty" << std::endl;
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    callProgressCallback(0.1, "Registration start");

    // 初始化ICP对象
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // 设置ICP参数
    icp.setMaxCorrespondenceDistance(params_.registration_max_distance);
    icp.setMaximumIterations(params_.registration_max_iterations);
    icp.setTransformationEpsilon(params_.registration_transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(params_.registration_euclidean_fitness_epsilon);

    // 设置输入
    icp.setInputSource(right_cloud);  // 源点云（右侧）
    icp.setInputTarget(left_cloud);   // 目标点云（左侧）

    // 执行配准
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*aligned_cloud);

    // 获取变换矩阵
    transform = icp.getFinalTransformation();

    callStatsCallback("Registration",
        "Fitness score: " + std::to_string(icp.getFitnessScore()) +
        ", Converged: " + (icp.hasConverged() ? "yes" : "no") +
        ", Final error: " + std::to_string(icp.getFitnessScore()));

    callProgressCallback(0.5, "Registration alignment done");

    // 融合两个点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 添加左侧点云
    for (const auto& point : left_cloud->points) {
        final_cloud->push_back(point);
    }

    // 添加经过变换的右侧点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_right_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*right_cloud, *transformed_right_cloud, transform);

    for (const auto& point : transformed_right_cloud->points) {
        final_cloud->push_back(point);
    }

    callProgressCallback(0.9, "Registration done");

    return final_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::processClouds(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& left_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& right_cloud) {

    if (!left_cloud || left_cloud->empty() || !right_cloud || right_cloud->empty()) {
        std::cerr << "Error: One or both input clouds are empty" << std::endl;
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    // 1. 预处理左右点云
    callProgressCallback(0.0, "Processing start");

    auto left_processed = downsample(left_cloud);
    callProgressCallback(0.25, "Left cloud downsampled");

    auto right_processed = downsample(right_cloud);
    callProgressCallback(0.5, "Right cloud downsampled");

    // 2. 降噪
    left_processed = denoise(left_processed);
    callProgressCallback(0.6, "Left cloud denoised");

    right_processed = denoise(right_processed);
    callProgressCallback(0.7, "Right cloud denoised");

    // 3. 配准融合
    Eigen::Matrix4f transform;
    auto final_cloud = registerClouds(left_processed, right_processed, transform);
    callProgressCallback(1.0, "Processing done");

    return final_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::orientNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    bool is_noisy) {

    // 这里集成现有项目中的法向量定向功能
    // 在实际实现中，需要调用现有项目中的算法
    // 以下是一个简化的实现示例

    if (!input || input->empty()) {
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    callProgressCallback(0.1, "Normals orientation start");

    // 此处应调用现有IsoConstraints算法中的法向量定向功能
    // 为了兼容现有代码，这里创建一个简化版本
    pcl::PointCloud<pcl::PointXYZ>::Ptr oriented_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 复制输入点云
    *oriented_cloud = *input;

    // 提供信息输出
    callStatsCallback("Normals orientation",
        "Processed points: " + std::to_string(oriented_cloud->size()) +
        ", Noisy input: " + std::string(is_noisy ? "yes" : "no"));

    callProgressCallback(0.9, "Normals orientation done");

    return oriented_cloud;
}

void PointCloudProcessor::setProgressCallback(std::function<void(double progress, const std::string& stage)> callback) {
    progress_callback_ = callback;
}

void PointCloudProcessor::setStatsCallback(std::function<void(const std::string& stage, const std::string& stats)> callback) {
    stats_callback_ = callback;
}

void PointCloudProcessor::callStatsCallback(const std::string& stage, const std::string& stats) const {
    if (stats_callback_) {
        stats_callback_(stage, stats);
    }
}

void PointCloudProcessor::callProgressCallback(double progress, const std::string& stage) const {
    if (progress_callback_) {
        progress_callback_(progress, stage);
    }
}

pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudProcessor::estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {

    if (!cloud || cloud->empty()) {
        return pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    }

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // 创建搜索树
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_est.setSearchMethod(tree);
    normal_est.setInputCloud(cloud);
    normal_est.setKSearch(20); // 使用20个最近邻点估计法线

    normal_est.compute(*normals);

    // 组合点云和法线
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    return cloud_with_normals;
}