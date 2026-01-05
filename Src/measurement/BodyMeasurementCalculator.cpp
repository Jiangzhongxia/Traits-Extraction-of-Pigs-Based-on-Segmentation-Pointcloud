#include "BodyMeasurementCalculator.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <ctime>

BodyMeasurementCalculator::BodyMeasurementCalculator() {
    // 初始化默认参数
    params_.ground_height_offset = 0.0;
    params_.min_confidence = 0.7;
    params_.body_length_search_radius = 0.1;
    params_.height_search_radius = 0.05;
    params_.volume_voxel_size = 0.01;
    params_.weight_coefficient = 75.0;
}

BodyMeasurementCalculator::~BodyMeasurementCalculator() {
}

void BodyMeasurementCalculator::setParams(const MeasurementParams& params) {
    params_ = params;
}

BodyMeasurementCalculator::MeasurementParams BodyMeasurementCalculator::getParams() const {
    return params_;
}

MeasurementResult BodyMeasurementCalculator::calculate(
    const std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>& segments) {

    MeasurementResult result;

    // 获取当前时间戳
    std::time_t now = std::time(0);
    char buffer[100];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    result.timestamp = std::string(buffer);

    callProgressCallback(0.05, "Measurement calculation start");

    // 获取各部位的点云
    auto head_it = segments.find("head");
    auto torso_it = segments.find("torso");
    auto tail_it = segments.find("tail");
    auto front_left_leg_it = segments.find("front_left_leg");
    auto front_right_leg_it = segments.find("front_right_leg");
    auto hind_left_leg_it = segments.find("hind_left_leg");
    auto hind_right_leg_it = segments.find("hind_right_leg");
    auto ear_it = segments.find("ear");
    auto neck_it = segments.find("neck");

    // 计算体长
    if (head_it != segments.end() && torso_it != segments.end() && tail_it != segments.end()) {
        result.body_length = calculateBodyLength(head_it->second, torso_it->second, tail_it->second);
        callProgressCallback(0.15, "Body length calculated");
    }

    // 计算肩高
    if (torso_it != segments.end()) {
        result.withers_height = calculateWithersHeight(torso_it->second, params_.ground_height_offset);
        callProgressCallback(0.25, "Withers height calculated");
    }

    // 计算胸深和胸宽
    if (torso_it != segments.end()) {
        result.chest_depth = calculateChestDepth(torso_it->second);
        callProgressCallback(0.35, "Chest depth calculated");

        result.chest_width = calculateChestWidth(torso_it->second);
        callProgressCallback(0.45, "Chest width calculated");
    }

    // 计算髋高和髋宽
    if (torso_it != segments.end()) {
        result.hip_height = calculateHipHeight(torso_it->second, params_.ground_height_offset);
        callProgressCallback(0.55, "Hip height calculated");

        result.hip_width = calculateHipWidth(torso_it->second);
        callProgressCallback(0.65, "Hip width calculated");
    }

    // 计算腹围、腹宽、腹高
    if (torso_it != segments.end()) {
        result.abdominal_girth = calculateAbdominalGirth(torso_it->second);
        callProgressCallback(0.75, "Abdominal girth calculated");

        result.abdominal_width = calculateAbdominalWidth(torso_it->second);
        callProgressCallback(0.85, "Abdominal width calculated");

        result.abdominal_height = calculateAbdominalHeight(torso_it->second);
        callProgressCallback(0.95, "Abdominal height calculated");
    }

    // 计算头长和头宽（如果头部点云可用）
    if (head_it != segments.end()) {
        double min_x, max_x, min_y, max_y, min_z, max_z;
        getAxisExtents(head_it->second, 0, min_x, max_x);
        getAxisExtents(head_it->second, 1, min_y, max_y);
        getAxisExtents(head_it->second, 2, min_z, max_z);

        result.head_length = max_x - min_x;
        result.head_width = max_y - min_y;
    }

    // 计算耳长（如果耳朵点云可用）
    if (ear_it != segments.end()) {
        double min_x, max_x, min_y, max_y, min_z, max_z;
        getAxisExtents(ear_it->second, 0, min_x, max_x);
        getAxisExtents(ear_it->second, 1, min_y, max_y);
        getAxisExtents(ear_it->second, 2, min_z, max_z);

        result.ear_length = std::max({max_x - min_x, max_y - min_y, max_z - min_z});
    }

    // 计算颈长（如果脖子点云可用）
    if (neck_it != segments.end()) {
        double min_x, max_x, min_y, max_y, min_z, max_z;
        getAxisExtents(neck_it->second, 0, min_x, max_x);
        getAxisExtents(neck_it->second, 1, min_y, max_y);
        getAxisExtents(neck_it->second, 2, min_z, max_z);

        result.neck_length = max_z - min_z;  // 假设颈部沿z轴方向
    }

    // 计算尾长（如果尾巴点云可用）
    if (tail_it != segments.end()) {
        double min_x, max_x, min_y, max_y, min_z, max_z;
        getAxisExtents(tail_it->second, 0, min_x, max_x);
        getAxisExtents(tail_it->second, 1, min_y, max_y);
        getAxisExtents(tail_it->second, 2, min_z, max_z);

        result.tail_length = std::sqrt(std::pow(max_x - min_x, 2) +
                                      std::pow(max_y - min_y, 2) +
                                      std::pow(max_z - min_z, 2));
    }

    // 计算腿围
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> front_legs, hind_legs;
    if (front_left_leg_it != segments.end()) front_legs.push_back(front_left_leg_it->second);
    if (front_right_leg_it != segments.end()) front_legs.push_back(front_right_leg_it->second);
    if (hind_left_leg_it != segments.end()) hind_legs.push_back(hind_left_leg_it->second);
    if (hind_right_leg_it != segments.end()) hind_legs.push_back(hind_right_leg_it->second);

    // 前腿围平均值
    if (!front_legs.empty()) {
        double total_girth = 0.0;
        for (const auto& leg : front_legs) {
            double min_x, max_x, min_y, max_y, min_z, max_z;
            getAxisExtents(leg, 0, min_x, max_x);
            getAxisExtents(leg, 1, min_y, max_y);
            double leg_width = max_x - min_x;
            double leg_depth = max_y - min_y;
            total_girth += M_PI * std::sqrt((leg_width/2.0) * (leg_depth/2.0));  // 椭圆周长近似
        }
        result.front_leg_girth = total_girth / front_legs.size();
    }

    // 后腿围平均值
    if (!hind_legs.empty()) {
        double total_girth = 0.0;
        for (const auto& leg : hind_legs) {
            double min_x, max_x, min_y, max_y, min_z, max_z;
            getAxisExtents(leg, 0, min_x, max_x);
            getAxisExtents(leg, 1, min_y, max_y);
            double leg_width = max_x - min_x;
            double leg_depth = max_y - min_y;
            total_girth += M_PI * std::sqrt((leg_width/2.0) * (leg_depth/2.0));  // 椭圆周长近似
        }
        result.hind_leg_girth = total_girth / hind_legs.size();
    }

    // 估算重量
    result.estimated_weight = estimateWeight(result);

    callProgressCallback(1.0, "Measurement calculation done");

    // 统计信息
    std::string stats = "Measurements calculated - ";
    stats += "Body length: " + std::to_string(result.body_length) + "m, ";
    stats += "Withers height: " + std::to_string(result.withers_height) + "m, ";
    stats += "Estimated weight: " + std::to_string(result.estimated_weight) + "kg";
    callStatsCallback("Final measurements", stats);

    return result;
}

double BodyMeasurementCalculator::calculateBodyLength(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& head_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& tail_cloud) {

    if (head_cloud->empty() || torso_cloud->empty() || tail_cloud->empty()) {
        return 0.0;
    }

    // 找到头部最前面的点（最大x值）
    double head_min_x, head_max_x, dummy1, dummy2, dummy3, dummy4;
    getAxisExtents(head_cloud, 0, head_min_x, head_max_x);
    pcl::PointXYZ head_point;
    head_point.x = head_max_x;
    // 找到最接近这个x值的点
    float min_dist = std::numeric_limits<float>::max();
    for (const auto& p : head_cloud->points) {
        float dist = std::abs(p.x - head_max_x);
        if (dist < min_dist) {
            min_dist = dist;
            head_point = p;
        }
    }

    // 找到尾巴最后面的点（最小x值）
    double tail_min_x, tail_max_x, dummy5, dummy6, dummy7, dummy8;
    getAxisExtents(tail_cloud, 0, tail_min_x, tail_max_x);
    pcl::PointXYZ tail_point;
    tail_point.x = tail_min_x;
    // 找到最接近这个x值的点
    min_dist = std::numeric_limits<float>::max();
    for (const auto& p : tail_cloud->points) {
        float dist = std::abs(p.x - tail_min_x);
        if (dist < min_dist) {
            min_dist = dist;
            tail_point = p;
        }
    }

    // 计算体长（近似为直线距离，实际应用中可能需要更复杂的曲线测量）
    double length = std::sqrt(
        std::pow(head_point.x - tail_point.x, 2) +
        std::pow(head_point.y - tail_point.y, 2) +
        std::pow(head_point.z - tail_point.z, 2)
    );

    return length;
}

double BodyMeasurementCalculator::calculateWithersHeight(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud,
    double ground_height) {

    if (torso_cloud->empty()) {
        return 0.0;
    }

    // 找到躯干的最高点
    double highest_z = getHighestZ(torso_cloud);

    // 计算肩高（最高点z坐标减去地面高度）
    double height = highest_z - ground_height;

    return std::max(0.0, height);  // 确保不为负数
}

double BodyMeasurementCalculator::calculateChestDepth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud) {
    if (torso_cloud->empty()) {
        return 0.0;
    }

    double min_z, max_z, dummy1, dummy2, dummy3, dummy4;
    getAxisExtents(torso_cloud, 2, min_z, max_z);

    return max_z - min_z;
}

double BodyMeasurementCalculator::calculateChestWidth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud) {
    if (torso_cloud->empty()) {
        return 0.0;
    }

    double min_y, max_y, dummy1, dummy2, dummy3, dummy4;
    getAxisExtents(torso_cloud, 1, min_y, max_y);

    return max_y - min_y;
}

double BodyMeasurementCalculator::calculateHipHeight(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud,
    double ground_height) {

    if (torso_cloud->empty()) {
        return 0.0;
    }

    // 找到躯干的最高点（髋部区域通常在躯干后半部分）
    double highest_z = getHighestZ(torso_cloud);

    // 计算髋高
    double height = highest_z - ground_height;

    return std::max(0.0, height);
}

double BodyMeasurementCalculator::calculateHipWidth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud) {
    if (torso_cloud->empty()) {
        return 0.0;
    }

    double min_y, max_y, dummy1, dummy2, dummy3, dummy4;
    getAxisExtents(torso_cloud, 1, min_y, max_y);

    return max_y - min_y;
}

double BodyMeasurementCalculator::calculateAbdominalGirth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud) {
    if (torso_cloud->empty()) {
        return 0.0;
    }

    // 计算躯干的边界框尺寸，然后估算腹围
    double min_x, max_x, min_y, max_y, min_z, max_z;
    getAxisExtents(torso_cloud, 0, min_x, max_x);
    getAxisExtents(torso_cloud, 1, min_y, max_y);
    getAxisExtents(torso_cloud, 2, min_z, max_z);

    // 使用椭圆周长公式估算腹围
    double width = max_y - min_y;  // 左右宽度
    double depth = max_z - min_z;  // 前后深度
    double girth = M_PI * (3 * (width/2 + depth/2) - std::sqrt((3*width/2 + depth/2) * (width/2 + 3*depth/2)));

    return std::max(0.0, girth);
}

double BodyMeasurementCalculator::calculateAbdominalWidth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud) {
    if (torso_cloud->empty()) {
        return 0.0;
    }

    double min_y, max_y, dummy1, dummy2, dummy3, dummy4;
    getAxisExtents(torso_cloud, 1, min_y, max_y);

    return max_y - min_y;
}

double BodyMeasurementCalculator::calculateAbdominalHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& torso_cloud) {
    if (torso_cloud->empty()) {
        return 0.0;
    }

    double min_z, max_z, dummy1, dummy2, dummy3, dummy4;
    getAxisExtents(torso_cloud, 2, min_z, max_z);

    return max_z - min_z;
}

double BodyMeasurementCalculator::estimateWeight(const MeasurementResult& measurements) const {
    // 使用体尺参数估算重量的公式
    // 这是一个简化的估算公式，实际应用中可能需要更复杂的模型
    if (measurements.body_length > 0 && measurements.chest_girth > 0) {
        // 基于体长和胸围的重量估算公式
        double estimated_weight = (measurements.chest_girth * measurements.chest_girth * measurements.body_length) * params_.weight_coefficient;
        return estimated_weight;
    }

    // 如果没有体长和胸围数据，使用其他参数的组合估算
    double combined_size = measurements.body_length + measurements.withers_height +
                          measurements.chest_width + measurements.chest_depth;
    if (combined_size > 0) {
        return combined_size * 100.0;  // 简化的估算
    }

    return 0.0;
}

void BodyMeasurementCalculator::saveResults(const MeasurementResult& result, const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        file << "Measurement Results - " << result.timestamp << "\n";
        file << "========================================\n";
        file << "Body Length: " << result.body_length << " m\n";
        file << "Withers Height: " << result.withers_height << " m\n";
        file << "Chest Depth: " << result.chest_depth << " m\n";
        file << "Chest Width: " << result.chest_width << " m\n";
        file << "Hip Height: " << result.hip_height << " m\n";
        file << "Hip Width: " << result.hip_width << " m\n";
        file << "Abdominal Girth: " << result.abdominal_girth << " m\n";
        file << "Abdominal Width: " << result.abdominal_width << " m\n";
        file << "Abdominal Height: " << result.abdominal_height << " m\n";
        file << "Head Length: " << result.head_length << " m\n";
        file << "Head Width: " << result.head_width << " m\n";
        file << "Ear Length: " << result.ear_length << " m\n";
        file << "Neck Length: " << result.neck_length << " m\n";
        file << "Tail Length: " << result.tail_length << " m\n";
        file << "Front Leg Girth: " << result.front_leg_girth << " m\n";
        file << "Hind Leg Girth: " << result.hind_leg_girth << " m\n";
        file << "Estimated Weight: " << result.estimated_weight << " kg\n";
        file.close();

        std::cout << "Measurement results saved to " << filename << std::endl;
    } else {
        std::cerr << "Error: Could not open file " << filename << " for writing" << std::endl;
    }
}

std::vector<std::string> BodyMeasurementCalculator::getAvailableMeasurements() const {
    return {
        "body_length", "withers_height", "chest_depth", "chest_width",
        "hip_height", "hip_width", "abdominal_girth", "abdominal_width",
        "abdominal_height", "head_length", "head_width", "ear_length",
        "neck_length", "tail_length", "front_leg_girth", "hind_leg_girth",
        "estimated_weight"
    };
}

void BodyMeasurementCalculator::setProgressCallback(std::function<void(double progress, const std::string& stage)> callback) {
    progress_callback_ = callback;
}

void BodyMeasurementCalculator::setStatsCallback(std::function<void(const std::string& stage, const std::string& stats)> callback) {
    stats_callback_ = callback;
}

void BodyMeasurementCalculator::callStatsCallback(const std::string& stage, const std::string& stats) const {
    if (stats_callback_) {
        stats_callback_(stage, stats);
    }
}

void BodyMeasurementCalculator::callProgressCallback(double progress, const std::string& stage) const {
    if (progress_callback_) {
        progress_callback_(progress, stage);
    }
}

void BodyMeasurementCalculator::getAxisExtents(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int axis,
                                               double& min_val, double& max_val) const {
    if (cloud->empty()) {
        min_val = max_val = 0.0;
        return;
    }

    min_val = std::numeric_limits<double>::max();
    max_val = std::numeric_limits<double>::lowest();

    for (const auto& point : cloud->points) {
        double val;
        switch(axis) {
            case 0: val = point.x; break;  // x轴
            case 1: val = point.y; break;  // y轴
            case 2: val = point.z; break;  // z轴
            default: val = point.x; break;
        }

        if (val < min_val) min_val = val;
        if (val > max_val) max_val = val;
    }
}

double BodyMeasurementCalculator::getHighestZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
    if (cloud->empty()) {
        return 0.0;
    }

    double max_z = std::numeric_limits<double>::lowest();
    for (const auto& point : cloud->points) {
        if (point.z > max_z) {
            max_z = point.z;
        }
    }
    return max_z;
}

double BodyMeasurementCalculator::getLowestZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                             double ground_height) const {
    if (cloud->empty()) {
        return ground_height;
    }

    double min_z = std::numeric_limits<double>::max();
    for (const auto& point : cloud->points) {
        if (point.z < min_z) {
            min_z = point.z;
        }
    }
    return min_z;
}