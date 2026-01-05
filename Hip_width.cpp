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
////pcl::visualization::PCLVisualizer viewer("Curve Fitting 2D");
//
//pcl::PointCloud<pcl::PointXYZ>::Ptr slicePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float min_x, float max_x) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr sliced_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//
//    // 遍历点云中的每个点，并根据 x 的范围进行切片
//    for (const auto& point : cloud->points) {
//        if (point.x >= min_x && point.x <= max_x) {
//            sliced_cloud->points.push_back(point);
//        }
//    }
//
//    return sliced_cloud;
//}
//// 拟合垂直于 XoZ 平面的平面方程的函数
//void fitPlane(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, float& A, float& B, float& C, float& D) {
//    // 由于平面垂直于 XoZ 平面，所以 B = 0
//    B = 0.0f;
//
//    // 计算法向量 n (A, B, C)
//    // 垂直于 XoZ 平面，意味着平面的法向量在 Y 轴上没有分量
//    A = p2.z - p1.z;  // z2 - z1
//    C = p1.x - p2.x;  // x1 - x2
//
//    // 法向量应满足 A * (x2 - x1) + C * (z2 - z1) = 0，这个公式得出的向量方向与 (x2-x1, z2-z1) 垂直
//    // 将 B 设为 0 后，D 通过平面点方程求解
//    D = -(A * p1.x + C * p1.z); // D = -(A*x1 + C*z1)
//}
//
//// 计算点到平面的距离
//double calculateDistanceToPlane(const pcl::PointXYZ& point, double A, double B, double C, double D) {
//    double numerator = std::abs(A * point.x + B * point.y + C * point.z + D);
//    double denominator = std::sqrt(A * A + B * B + C * C);
//    return numerator / denominator;
//}
//
//pcl::PointXYZ findMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr& back_part) {
//    // 初始化z轴最大最小值
//    pcl::PointXYZ min_z_point = back_part->points[0];
//    pcl::PointXYZ max_z_point = back_part->points[0];
//
//    // 找到z轴的最大值和最小值
//    for (const auto& point : back_part->points) {
//        if (point.z < min_z_point.z) {
//            min_z_point = point;
//        }
//        if (point.z > max_z_point.z) {
//            max_z_point = point;
//        }
//    }
//
//    // 计算z轴最大最小值的中点
//    float z_mid = (max_z_point.z + min_z_point.z) / 2.0f;
//
//    // 初始化下、上部分点云
//    pcl::PointCloud<pcl::PointXYZ>::Ptr lower_part(new pcl::PointCloud<pcl::PointXYZ>());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr upper_part(new pcl::PointCloud<pcl::PointXYZ>());
//
//    // 基于z_mid将back_part分为上下两部分
//    for (const auto& point : back_part->points) {
//        if (point.z <= z_mid) {
//            lower_part->points.push_back(point);
//        }
//        else {
//            upper_part->points.push_back(point);
//        }
//    }
//
//    // 查找上下部分中x轴最da值点
//    pcl::PointXYZ max_lower_x = lower_part->points[0];
//    pcl::PointXYZ max_upper_x = upper_part->points[0];
//
//    for (const auto& point : lower_part->points) {
//        if (point.x > max_lower_x.x) {
//            max_lower_x = point;
//        }
//    }
//
//    for (const auto& point : upper_part->points) {
//        if (point.x > max_upper_x.x) {
//            max_upper_x = point;
//        }
//    }
//    // 计算左右最大x点的中点
//    pcl::PointXYZ max_mid_point;
//    max_mid_point.x = (max_lower_x.x + max_upper_x.x) / 2.0f;
//    max_mid_point.y = (max_lower_x.y + max_upper_x.y) / 2.0f;
//    max_mid_point.z = (max_lower_x.z + max_upper_x.z) / 2.0f;
//
//    return max_mid_point;
//}
//
//pcl::PointXYZ findP1(const pcl::PointCloud<pcl::PointXYZ>::Ptr& back_part) {
//    // 初始化z轴最大最小值
//    pcl::PointXYZ min_z_point = back_part->points[0];
//    // 找到z轴的最小值
//    for (const auto& point : back_part->points) {
//        if (point.z > min_z_point.z) {
//            min_z_point = point;
//        }
//    }
//    // 搜索mid_point周围的点并计算质心作为最终尾部点
//    float radius = 0.05f; // 定义领域搜索半径，根据点云密度调整
//    pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood(new pcl::PointCloud<pcl::PointXYZ>());
//    for (const auto& point : back_part->points) {
//        if (std::sqrt(std::pow(point.x - min_z_point.x, 2) + std::pow(point.y - min_z_point.y, 2) + std::pow(point.z - min_z_point.z, 2)) <= radius) {
//            neighborhood->points.push_back(point);
//        }
//    }
//    // 计算质心
//    Eigen::Vector4f centroid;
//    pcl::compute3DCentroid(*neighborhood, centroid);
//    pcl::PointXYZ P1;
//    P1.x = centroid[0];
//    P1.y = centroid[1];
//    P1.z = centroid[2];
//    return P1;
//}
//
//pcl::PointXYZ findP2(const pcl::PointCloud<pcl::PointXYZ>::Ptr& back_part) {
//    // 初始化z轴最大最小值
//    pcl::PointXYZ max_z_point = back_part->points[0];
//    // 找到z轴的最小值
//    for (const auto& point : back_part->points) {
//        if (point.z < max_z_point.z) {
//            max_z_point = point;
//        }
//    }
//    // 搜索mid_point周围的点并计算质心作为最终尾部点
//    float radius = 0.05f; // 定义领域搜索半径，根据点云密度调整
//    pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood(new pcl::PointCloud<pcl::PointXYZ>());
//    for (const auto& point : back_part->points) {
//        if (std::sqrt(std::pow(point.x - max_z_point.x, 2) + std::pow(point.y - max_z_point.y, 2) + std::pow(point.z - max_z_point.z, 2)) <= radius) {
//            neighborhood->points.push_back(point);
//        }
//    }
//    // 计算质心
//    Eigen::Vector4f centroid;
//    pcl::compute3DCentroid(*neighborhood, centroid);
//    pcl::PointXYZ P1;
//    P1.x = centroid[0];
//    P1.y = centroid[1];
//    P1.z = centroid[2];
//    return P1;
//}
//
//pcl::PointXYZ findMin(const pcl::PointCloud<pcl::PointXYZ>::Ptr& back_part) {
//    // 初始化z轴最大最小值
//    pcl::PointXYZ min_z_point = back_part->points[0];
//    pcl::PointXYZ max_z_point = back_part->points[0];
//
//    // 找到z轴的最大值和最小值
//    for (const auto& point : back_part->points) {
//        if (point.z < min_z_point.z) {
//            min_z_point = point;
//        }
//        if (point.z > max_z_point.z) {
//            max_z_point = point;
//        }
//    }
//    // 计算z轴最大最小值的中点
//    float z_mid = (max_z_point.z + min_z_point.z) / 2.0f;
//    // 初始化下、上部分点云
//    pcl::PointCloud<pcl::PointXYZ>::Ptr lower_part(new pcl::PointCloud<pcl::PointXYZ>());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr upper_part(new pcl::PointCloud<pcl::PointXYZ>());
//    // 基于z_mid将back_part分为上下两部分
//    for (const auto& point : back_part->points) {
//        if (point.z <= z_mid) {
//            lower_part->points.push_back(point);
//        }
//        else {
//            upper_part->points.push_back(point);
//        }
//    }
//    // 查找上下部分中x轴最da值点
//    pcl::PointXYZ min_lower_x = lower_part->points[0];
//    pcl::PointXYZ min_upper_x = upper_part->points[0];
//    for (const auto& point : lower_part->points) {
//        if (point.x < min_lower_x.x) {
//            min_lower_x = point;
//        }
//    }
//    for (const auto& point : upper_part->points) {
//        if (point.x < min_upper_x.x) {
//            min_upper_x = point;
//        }
//    }
//    // 计算左右最大x点的中点
//    pcl::PointXYZ min_mid_point;
//    min_mid_point.x = (min_lower_x.x + min_upper_x.x) / 2.0f;
//    min_mid_point.y = (min_lower_x.y + min_upper_x.y) / 2.0f;
//    min_mid_point.z = (min_lower_x.z + min_upper_x.z) / 2.0f;
//
//    return min_mid_point;
//}
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
//// 计算点云的质心
//pcl::PointXYZ computeCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
//    Eigen::Vector4f centroid;
//    pcl::compute3DCentroid(*cloud, centroid);
//    return pcl::PointXYZ(centroid[0], centroid[1], centroid[2]);
//}
//// 将点云坐标系的原点移动到指定质心
//void translateToCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointXYZ& centroid) {
//    for (auto& point : cloud->points) {
//        point.x -= centroid.x;
//        point.y -= centroid.y;
//        point.z -= centroid.z;
//    }
//}
//
//void savePlaneAsXYZ(float A, float B, float C, float D, const std::string& filename,
//    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float resolution) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // 计算点云的边界范围
//    float x_min = std::numeric_limits<float>::max();
//    float x_max = std::numeric_limits<float>::lowest();
//    float y_min = std::numeric_limits<float>::max();
//    float y_max = std::numeric_limits<float>::lowest();
//
//    for (const auto& point : cloud->points) {
//        if (point.x < x_min) x_min = point.x;
//        if (point.x > x_max) x_max = point.x;
//        if (point.y < y_min) y_min = point.y;
//        if (point.y > y_max) y_max = point.y;
//    }
//
//    // 在计算出的范围内生成平面上的点
//    for (float x = x_min; x <= x_max; x += resolution) {
//        for (float y = y_min; y <= y_max; y += resolution) {
//            float z = -(A * x + B * y + D) / C;
//            plane_cloud->points.emplace_back(x, y, z);
//        }
//    }
//
//    // 保存为 XYZ 文件
//    std::ofstream ofs(filename);
//    if (!ofs.is_open()) {
//        std::cerr << "Failed to open file " << filename << " for writing." << std::endl;
//        return;
//    }
//
//    for (const auto& point : plane_cloud->points) {
//        ofs << point.x << " " << point.y << " " << point.z << std::endl;
//    }
//
//    ofs.close();
//    std::cout << "Plane point cloud saved to " << filename << std::endl;
//}
//
//// 计算点到平面的距离
//float pointToPlaneDistance(const pcl::PointXYZ& point, float A, float B, float C, float D) {
//    return std::fabs(A * point.x + B * point.y + C * point.z + D) / std::sqrt(A * A + B * B + C * C);
//}
//
//// 分割点云，找到离平面最远的点并返回其距离和
//float findMaxDistanceSum(pcl::PointCloud<pcl::PointXYZ>::Ptr& back_part, float A, float B, float C, float D) {
//    // 初始化z轴最大最小值
//    pcl::PointXYZ min_z_point = back_part->points[0];
//    pcl::PointXYZ max_z_point = back_part->points[0];
//
//    // 找到z轴的最大值和最小值
//    for (const auto& point : back_part->points) {
//        if (point.z < min_z_point.z) {
//            min_z_point = point;
//        }
//        if (point.z > max_z_point.z) {
//            max_z_point = point;
//        }
//    }
//    // 计算z轴最大最小值的中点
//    float z_mid = (max_z_point.z + min_z_point.z) / 2.0f;
//    // 初始化下、上部分点云
//    pcl::PointCloud<pcl::PointXYZ>::Ptr lower_part(new pcl::PointCloud<pcl::PointXYZ>());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr upper_part(new pcl::PointCloud<pcl::PointXYZ>());
//    // 基于z_mid将back_part分为上下两部分
//    for (const auto& point : back_part->points) {
//        if (point.z <= z_mid) {
//            lower_part->points.push_back(point);
//        }
//        else {
//            upper_part->points.push_back(point);
//        }
//    }
//    float max_positive_distance = 0.0f;
//    float max_negative_distance = 0.0f;
//
//    for (const auto& point : lower_part->points) {
//        float distance = pointToPlaneDistance(point, A, B, C, D);
//        if (distance > max_positive_distance) {
//            max_positive_distance = distance;
//        }
//    }
//    for (const auto& point : upper_part->points) {
//        float distance = pointToPlaneDistance(point, A, B, C, D);
//        if (distance > max_negative_distance) {
//            max_negative_distance = distance;
//        }
//    }
//    return max_positive_distance + max_negative_distance;
//}
//
//int main(int argc, char** argv) {
//
//    // 左耳、右耳和前中后三个部分的点云文件路径
//    std::string full_path = "E:\\pig1-150\\xyz150\\xyz150\\150\\150.xyz";
//    std::string head_path = "E:\\pig1-150\\xyz150\\xyz150\\150\\59.xyz";
//    std::string left_ear_path = "E:\\pig1-150\\xyz150\\xyz150\\150\\57.xyz";
//    std::string right_ear_path = "E:\\pig1-150\\xyz150\\xyz150\\150\\58.xyz";
//    std::string front_part_path = "E:\\pig1-150\\xyz150\\xyz150\\150\\56.xyz";
//    std::string middle_part_path = "E:\\pig1-150\\xyz150\\xyz150\\150\\55.xyz";
//    std::string back_part_path = "E:\\pig1-150\\xyz150\\xyz150\\150\\54.xyz";
//    std::string Afront_path = "E:\\pig1-150\\xyz150\\xyz150\\150\\50.xyz";
//    std::string Bfront_path = "E:\\pig1-150\\xyz150\\xyz150\\150\\51.xyz";
//    std::string Aback_path = "E:\\pig1-150\\xyz150\\xyz150\\150\\52.xyz";
//    std::string Bback_path = "E:\\pig1-150\\xyz150\\xyz150\\150\\53.xyz";
//    std::string output_path = "E:\\pig1-150\\xyz150\\xyz150\\150\\Hip_width\\points.pcd";
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
//    pcl::PointXYZ centroid = computeCentroid(back_part);
//
//    // 将其他点云的坐标系原点移动到full的质心
//    translateToCentroid(back_part, centroid);
//    // 获取点云的 x 值范围
//    float min_x = std::numeric_limits<float>::max();
//    float max_x = std::numeric_limits<float>::min();
//
//    for (const auto& point : back_part->points) {
//        if (point.x < min_x) min_x = point.x;
//        if (point.x > max_x) max_x = point.x;
//    }
//
//    // 计算前后 5cm 的范围
//    float front_slice_min_x = min_x;
//    float front_slice_max_x = min_x + 0.1f;
//
//    float back_slice_min_x = max_x - 0.1f;
//    float back_slice_max_x = max_x;
//
//    // 切出前后 5cm 的切片
//    pcl::PointCloud<pcl::PointXYZ>::Ptr front_slice = slicePointCloud(back_part, front_slice_min_x, front_slice_max_x);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr back_slice = slicePointCloud(back_part, back_slice_min_x, back_slice_max_x);
//    //pcl::io::savePCDFileASCII("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\abdominal_width\\front_slice.pcd", *front_slice);
//    //pcl::io::savePCDFileASCII("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\abdominal_width\\back_slice.pcd", *back_slice);
//    pcl::PointXYZ min = findMin(front_slice);
//    pcl::PointXYZ max = findMax(back_slice);
//    pcl::PointXYZ P1 = findP1(back_part);
//    pcl::PointXYZ P2 = findP2(back_part);
//    float A, B, C, D;
//    fitPlane(max, min, A, B, C, D);
//
//    double totalDistance = findMaxDistanceSum(back_part, A, B, C, D);
//    float resolution = 0.08;
//    std::cout << "totalDistance = " << totalDistance << endl;
//    //savePlaneAsXYZ(A, B, C, D, output_path, back_part, resolution);
//    //pcl::io::savePCDFileASCII("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\Hip_width\\54_cen.pcd", *back_part);
//    std::cout << "Hip_width =  " << totalDistance << std::endl;
//    std::ofstream output_file("E:\\pig1-150\\xyz150\\xyz150\\150\\Hip_width\\Hip_width.txt");  // 打开文件（会自动创建或覆盖）
//    
//    if (output_file.is_open()) {
//        // 将曲线长度写入文件
//        output_file << "Hip_width " << totalDistance << std::endl;
//        output_file.close();  // 关闭文件
//        std::cout << "Hip_width saved to curve_length.txt" << std::endl;
//    }
//    else {
//        std::cerr << "Unable to open file for writing." << std::endl;
//    }
//    return 0;
//}