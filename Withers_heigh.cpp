//#include <iostream>
//#include <vector>
//#include <cmath>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/common/centroid.h>
//#include <Eigen/Dense>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/extract_indices.h>
//#include <Eigen/Dense>
//#include <fstream>
//#include <sstream>
//#include <pcl/visualization/cloud_viewer.h>
//#include <algorithm> // For std::min_element and std::max_element
//#include <pcl/surface/on_nurbs/fitting_curve_2d_tdm.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d_sdm.h>
//#include <pcl/surface/on_nurbs/triangulation.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
//#include <pcl/common/centroid.h>  // 用于计算点云的质心
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d.h>
//#include <pcl/filters/passthrough.h>
//#include <unsupported/Eigen/Splines>
//using namespace std;
//pcl::visualization::PCLVisualizer viewer("Curve Fitting 2D");
//
//
//
//// 定义切片的厚度
//const float SLICE_THICKNESS = 0.06f;
//
//// 计算点云的质心
//pcl::PointXYZ computeCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
//    Eigen::Vector4f centroid;
//    pcl::compute3DCentroid(*cloud, centroid);
//    return pcl::PointXYZ(centroid[0], centroid[1], centroid[2]);
//}
//
//pcl::PointXYZ findZAxisExtremaCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool findMin) {
//    // Find the point with the min or max Z value
//    auto extrema_point = std::min_element(cloud->points.begin(), cloud->points.end(),
//        [findMin](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
//            return findMin ? (a.z < b.z) : (a.z > b.z);
//        });
//
//    float z_extrema = extrema_point->z;
//
//    // Gather points around the extrema Z value within a certain threshold (neighborhood)
//    pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood(new pcl::PointCloud<pcl::PointXYZ>);
//    float z_threshold = 0.01f;  // Example threshold for neighborhood, you can adjust it as needed
//
//    for (const auto& point : cloud->points) {
//        if (std::abs(point.z - z_extrema) < z_threshold) {
//            neighborhood->points.push_back(point);
//        }
//    }
//    // Calculate the centroid of the neighborhood
//    return computeCentroid(neighborhood);
//}
// 
//// 根据x和z值找到对应领域的点集并计算质心
//pcl::PointXYZ findRegionCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float x, float z, float y,float radius) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr region_points(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // 遍历点云，找到满足条件的点
//    for (const auto& point : cloud->points) {
//        if (std::abs(point.x - x) <= radius && std::abs(point.z - z) <= radius && point.y > y) {
//            region_points->points.push_back(point);
//        }
//    }
//    // 如果没有点满足条件，返回一个无效的点
//    if (region_points->points.empty()) {
//        std::cerr << "No points found in the specified region." << std::endl;
//        return pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(),
//            std::numeric_limits<float>::quiet_NaN(),
//            std::numeric_limits<float>::quiet_NaN());
//    }
//
//    // 计算点集的质心
//    Eigen::Vector4f centroid;
//    pcl::compute3DCentroid(*region_points, centroid);
//    return pcl::PointXYZ(centroid[0], centroid[1], centroid[2]);
//}
//
//// 函数：找到y轴最小值的点的领域中心点
//pcl::PointXYZ findMinYRegionCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float y_neighborhood) {
//    pcl::PointXYZ min_y_point;
//    min_y_point.y = std::numeric_limits<float>::max();
//
//    // 首先找到y轴最小值的点
//    for (const auto& point : cloud->points) {
//        if (point.y < min_y_point.y) {
//            min_y_point = point;
//        }
//    }
//
//    // 定义领域点的集合
//    std::vector<pcl::PointXYZ> region_points;
//    for (const auto& point : cloud->points) {
//        // 判断该点是否在领域内（基于y轴的范围）
//        if (std::fabs(point.y - min_y_point.y) <= y_neighborhood) {
//            region_points.push_back(point);
//        }
//    }
//
//    // 计算领域中心点
//    pcl::PointXYZ region_center;
//    region_center.x = 0.0f;
//    region_center.y = 0.0f;
//    region_center.z = 0.0f;
//
//    for (const auto& point : region_points) {
//        region_center.x += point.x;
//        region_center.y += point.y;
//        region_center.z += point.z;
//    }
//
//    region_center.x /= region_points.size();
//    region_center.y /= region_points.size();
//    region_center.z /= region_points.size();
//
//    return region_center;
//}
//
//// 函数：计算两个点的中心点
//pcl::PointXYZ calculateCenterPoint(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
//    pcl::PointXYZ center;
//    center.x = (p1.x + p2.x) / 2.0f;
//    center.y = (p1.y + p2.y) / 2.0f;
//    center.z = (p1.z + p2.z) / 2.0f;
//    return center;
//}
//
//
//// 点云加载函数
//bool loadXYZFile(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//    std::ifstream infile(filename);
//    if (!infile.is_open()) {
//        std::cerr << "Failed to open file " << filename << std::endl;
//        return false;
//    }
//
//    cloud->clear();  // 清空之前的点云数据
//
//    std::string line;
//    while (std::getline(infile, line)) {
//        std::istringstream iss(line);
//        pcl::PointXYZ point;
//        if (!(iss >> point.x >> point.y >> point.z)) {
//            std::cerr << "Failed to parse line: " << line << std::endl;
//            continue;
//        }
//        cloud->points.push_back(point);
//    }
//
//    cloud->width = cloud->points.size();
//    cloud->height = 1;
//    cloud->is_dense = true;
//
//    infile.close();
//    return true;
//}
//
//// 将点云坐标系的原点移动到指定质心
//void translateToCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointXYZ& centroid) {
//    for (auto& point : cloud->points) {
//        point.x -= centroid.x;
//        point.y -= centroid.y;
//        point.z -= centroid.z;
//    }
//}
//
//int main(int argc, char** argv) {
//
//    // 左耳、右耳和前中后三个部分的点云文件路径
//    std::string full_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\150.xyz";
//    std::string head_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\59.xyz";
//    std::string left_ear_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\57.xyz";
//    std::string right_ear_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\58.xyz";
//    std::string front_part_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\56.xyz";
//    std::string middle_part_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\55.xyz";
//    std::string back_part_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\54.xyz";
//    std::string Afront_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\50.xyz";
//    std::string Bfront_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\51.xyz";
//    std::string output_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\withers_height\\points.pcd";
//
//    // 创建点云指针
//    pcl::PointCloud<pcl::PointXYZ>::Ptr full(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr head(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr left_ear(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr right_ear(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr front_part(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr middle_part(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr back_part(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr Afront_part(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr Bfront_part(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // 加载点云
//    if (!loadXYZFile(head_path, head) || !loadXYZFile(full_path, full) || !loadXYZFile(left_ear_path, left_ear) ||
//        !loadXYZFile(right_ear_path, right_ear) ||
//        !loadXYZFile(front_part_path, front_part) ||
//        !loadXYZFile(middle_part_path, middle_part) ||
//        !loadXYZFile(back_part_path, back_part) ||
//        !loadXYZFile(Afront_path, Afront_part) ||
//        !loadXYZFile(Bfront_path, Bfront_part)) {
//        std::cerr << "Error loading one or more point cloud files." << std::endl;
//        return -1;
//    }
//    pcl::PointXYZ centroid = computeCentroid(full);
//    translateToCentroid(full, centroid);
//    translateToCentroid(front_part, centroid);
//    translateToCentroid(Afront_part, centroid);
//    translateToCentroid(Bfront_part, centroid);
//    // Step 1: Calculate front part Z-axis extrema centroids
//    pcl::PointXYZ front_min_z_centroid = findZAxisExtremaCentroid(front_part, true);
//    pcl::PointXYZ front_max_z_centroid = findZAxisExtremaCentroid(front_part, false);
//    pcl::PointXYZ front_center;
//    front_center.x = (front_min_z_centroid.x + front_max_z_centroid.x) / 2.0f;
//    front_center.y = (front_min_z_centroid.y + front_max_z_centroid.y) / 2.0f;
//    front_center.z = (front_min_z_centroid.z + front_max_z_centroid.z) / 2.0f;
//    // 设置搜索半径
//    float radius = 0.05f; // 根据需要调整这个半径值
//    
//    // 根据前中后中心找到对应领域的点集质心
//    pcl::PointXYZ p1 = findRegionCentroid(front_part, front_center.x, front_center.z, front_center.y,radius);
//
//    // 定义邻域大小（y轴方向），你可以根据需求调整
//    float y_neighborhood = 0.1f;
//
//    // 找到两个点云的最小y值的领域中心点
//    pcl::PointXYZ center1 = findMinYRegionCenter(Afront_part, y_neighborhood);
//    pcl::PointXYZ center2 = findMinYRegionCenter(Bfront_part, y_neighborhood);
//
//    // 计算两个中心点的中间点
//    pcl::PointXYZ final_center = calculateCenterPoint(center1, center2);
//
//    float distance = std::abs(final_center.y - p1.y);
//    std::cout << "withers_height =  " << distance << std::endl;
//    //std::cout << "Total length of the curve: " << withers_height << " units." << std::endl;
//    // 将曲线长度写入到txt文件
//    std::ofstream output_file("E:\\pig1-150\\xyz150\\xyz150\\014_new\\withers_height\\distance.txt");  // 打开文件（会自动创建或覆盖）
//
//    if (output_file.is_open()) {
//        // 将曲线长度写入文件
//        output_file << "withers_height: " << distance << std::endl;
//        output_file.close();  // 关闭文件
//        std::cout << "Curve Length saved to curve_length.txt" << std::endl;
//    }
//    else {
//        std::cerr << "Unable to open file for writing." << std::endl;
//    }
//    return 0;
//}