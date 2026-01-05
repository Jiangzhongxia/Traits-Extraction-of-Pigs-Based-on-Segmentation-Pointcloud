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
//
//using namespace std;
//
//// Save points to a file
//void savePointsToFile(const std::string& filename, const std::vector<pcl::PointXYZ>& points) {
//    std::ofstream outfile(filename);
//    if (!outfile.is_open()) {
//        std::cerr << "Failed to open file " << filename << std::endl;
//        return;
//    }
//
//    for (const auto& point : points) {
//        outfile << point.x << " " << point.y << " " << point.z << std::endl;
//    }
//
//    outfile.close();
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
////
//// 计算点云的质心
//pcl::PointXYZ computeCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
//    Eigen::Vector4f centroid;
//    pcl::compute3DCentroid(*cloud, centroid);
//    return pcl::PointXYZ(centroid[0], centroid[1], centroid[2]);
//}
//
//// 对点云进行 X 轴过滤以提取尾部点云
//pcl::PointCloud<pcl::PointXYZ>::Ptr extractTailPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float x_threshold) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr tail_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // 使用 PassThrough 过滤器来提取 X 轴上超过阈值的点
//    pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setInputCloud(cloud);
//    pass.setFilterFieldName("x");
//    pass.setFilterLimits(x_threshold, std::numeric_limits<float>::max());
//    pass.filter(*tail_cloud);
//
//    return tail_cloud;
//}
//
//// Function to find the extreme points on x-axis in a point cloud
//pcl::PointCloud<pcl::PointXYZ>::Ptr extractExtremePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool findMax) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr extremePoints(new pcl::PointCloud<pcl::PointXYZ>);
//
//    if (cloud->points.empty()) {
//        return extremePoints;
//    }
//
//    // Find the extreme x point (either min or max)
//    auto extreme_it = std::min_element(cloud->points.begin(), cloud->points.end(),
//        [findMax](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
//            return findMax ? a.z > b.z : a.z < b.z;
//        });
//
//    float extreme_z = extreme_it->z;
//
//    // Collect all points within a small neighborhood around the extreme x value
//    float epsilon = 0.01f; // Adjust this value based on point cloud density
//    for (const auto& point : cloud->points) {
//        if (std::fabs(point.z - extreme_z) <= epsilon) {
//            extremePoints->points.push_back(point);
//        }
//    }
//
//    return extremePoints;
//}
//pcl::PointXYZ findZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr& back_part) {
//    // 初始化z轴最大最小值
//    pcl::PointXYZ min_z_point = back_part->points[0];
//    pcl::PointXYZ max_z_point = back_part->points[0];
//    // 找到z轴的最大值和最小值
//    for (const auto& point : back_part->points) {
//        if (point.z < min_z_point.z) {
//            min_z_point = point;
//        }
//        if (point.z > max_z_point.z) {
//            max_z_point = point;
//        }
//    }
//    pcl::PointXYZ front_center;
//    front_center.x = (min_z_point.x + max_z_point.x) / 2.0f;
//    front_center.y = (min_z_point.y + max_z_point.y) / 2.0f;
//    front_center.z = (min_z_point.z + max_z_point.z) / 2.0f;
//
//    return front_center;
//}
//
//pcl::PointXYZ findRegionCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float x, float z, float y, float initial_radius) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr region_points(new pcl::PointCloud<pcl::PointXYZ>);
//    float radius = initial_radius;
//    float max_radius = 1.0;
//    while (radius <= max_radius) {
//        region_points->points.clear(); // Clear previous points
//
//        // 遍历点云，找到满足条件的点
//        for (const auto& point : cloud->points) {
//            if (std::abs(point.x - x) <= radius && std::abs(point.z - z) <= radius && point.y > y) {
//                region_points->points.push_back(point);
//            }
//        }
//
//        // 如果找到了满足条件的点，计算质心并返回
//        if (!region_points->points.empty()) {
//            Eigen::Vector4f centroid;
//            pcl::compute3DCentroid(*region_points, centroid);
//            return pcl::PointXYZ(x, centroid[1], z);
//        }
//
//        // 增大半径
//        radius += initial_radius; // 或者增加一个固定值，比如 0.1
//    }
//
//    // 如果仍然没有找到点，返回一个无效的点
//    std::cerr << "No points found in the specified region even after increasing the radius." << std::endl;
//    return pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(),
//        std::numeric_limits<float>::quiet_NaN(),
//        std::numeric_limits<float>::quiet_NaN());
//}
//pcl::PointXYZ findEarRegionCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float x, float z, float y, float initial_radius) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr region_points(new pcl::PointCloud<pcl::PointXYZ>);
//    float radius = initial_radius;
//    float max_radius = 1.0;
//    while (radius <= max_radius) {
//        region_points->points.clear(); // Clear previous points
//
//        // 遍历点云，找到满足条件的点
//        for (const auto& point : cloud->points) {
//            if (std::abs(point.x - x) <= radius && std::abs(point.z - z) <= radius && point.y > y) {
//                region_points->points.push_back(point);
//            }
//        }
//
//        // 如果找到了满足条件的点，计算质心并返回
//        if (!region_points->points.empty()) {
//            Eigen::Vector4f centroid;
//            pcl::compute3DCentroid(*region_points, centroid);
//            return pcl::PointXYZ(centroid[0], centroid[1], centroid[2]);
//        }
//
//        // 增大半径
//        radius += initial_radius; // 或者增加一个固定值，比如 0.1
//    }
//
//    // 如果仍然没有找到点，返回一个无效的点
//    std::cerr << "No points found in the specified region even after increasing the radius." << std::endl;
//    return pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(),
//        std::numeric_limits<float>::quiet_NaN(),
//        std::numeric_limits<float>::quiet_NaN());
//}
//pcl::PointXYZ findTailRootPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr& back_part) {
//    pcl::PointXYZ min_x = back_part->points[0];
//    for (const auto& point : back_part->points) {
//        if (point.x  <  min_x.x) {
//            min_x = point;
//        }
//    }
//    return  min_x ;
//}
//
//// Helper function to find the centroid of the neighborhood around the min/max X value
//pcl::PointXYZ findXAxisExtremaCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool findMin) {
//    // Find the point with the min or max X value
//    auto extrema_point = std::min_element(cloud->points.begin(), cloud->points.end(),
//        [findMin](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
//            return findMin ? (a.x < b.x) : (a.x > b.x);
//        });
//
//    float x_extrema = extrema_point->x;
//
//    // Gather points around the extrema X value within a certain threshold (neighborhood)
//    pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood(new pcl::PointCloud<pcl::PointXYZ>);
//    float x_threshold = 0.01f;  // Example threshold for neighborhood, you can adjust it as needed
//
//    for (const auto& point : cloud->points) {
//        if (std::abs(point.x - x_extrema) < x_threshold) {
//            neighborhood->points.push_back(point);
//        }
//    }
//
//    // Calculate the centroid of the neighborhood
//    return computeCentroid(neighborhood);
//}
//
//// 检查点是否为 NaN 的辅助函数
//bool isPointValid(const pcl::PointXYZ& point) {
//    return !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z);
//}
//
//// 计算两点之间的欧几里得距离
//double euclideanDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
//    return std::sqrt(std::pow(p1.x() - p2.x(), 2) +
//        std::pow(p1.y() - p2.y(), 2) +
//        std::pow(p1.z() - p2.z(), 2));
//}
//// 生成参数值
//std::vector<double> generateParameters(const std::vector<Eigen::Vector3d>& points) {
//    std::vector<double> parameters;
//    parameters.push_back(0.0); // 第一个点的参数为 0
//
//    for (size_t i = 1; i < points.size(); ++i) {
//        double t = parameters.back() + euclideanDistance(points[i - 1], points[i]);
//        parameters.push_back(t);
//    }
//    // 归一化参数
//    double total_length = parameters.back();
//    for (auto& t : parameters) {
//        t /= total_length;
//    }
//    return parameters;
//}
//// 拟合 x, y, z 坐标分别作为函数的三次多项式
//Eigen::VectorXd fitCurve(const std::vector<double>& t_values, const std::vector<double>& coordinate_values) {
//    int n = t_values.size();
//    Eigen::MatrixXd A(n, 4);
//    Eigen::VectorXd b(n);
//
//    for (int i = 0; i < n; ++i) {
//        double t = t_values[i];
//        A(i, 0) = 1;
//        A(i, 1) = t;
//        A(i, 2) = t * t;
//        A(i, 3) = t * t * t;
//        b(i) = coordinate_values[i];
//    }
//    // 最小二乘法求解 Ax = b
//    Eigen::VectorXd coefficients = A.colPivHouseholderQr().solve(b);
//    return coefficients;
//}
//std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> fit3DCurve(const std::vector<Eigen::Vector3d>& points) {
//    std::vector<double> t_values = generateParameters(points);
//
//    // 提取各个维度的坐标
//    std::vector<double> x_values, y_values, z_values;
//    for (const auto& p : points) {
//        x_values.push_back(p.x());
//        y_values.push_back(p.y());
//        z_values.push_back(p.z());
//    }
//
//    Eigen::VectorXd x_coeff = fitCurve(t_values, x_values);
//    Eigen::VectorXd y_coeff = fitCurve(t_values, y_values);
//    Eigen::VectorXd z_coeff = fitCurve(t_values, z_values);
//
//    return std::make_tuple(x_coeff, y_coeff, z_coeff);
//}
//std::vector<Eigen::Vector3d> sample3DCurve(const Eigen::VectorXd& x_coeff,
//    const Eigen::VectorXd& y_coeff,
//    const Eigen::VectorXd& z_coeff,
//    int num_samples = 1000) {
//    std::vector<Eigen::Vector3d> curve_points;
//
//    for (int i = 0; i <= num_samples; ++i) {
//        double t = static_cast<double>(i) / num_samples;
//
//        double x = x_coeff[0] + x_coeff[1] * t + x_coeff[2] * t * t + x_coeff[3] * t * t * t;
//        double y = y_coeff[0] + y_coeff[1] * t + y_coeff[2] * t * t + y_coeff[3] * t * t * t;
//        double z = z_coeff[0] + z_coeff[1] * t + z_coeff[2] * t * t + z_coeff[3] * t * t * t;
//
//        curve_points.push_back(Eigen::Vector3d(x, y, z));
//    }
//
//    return curve_points;
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
//// 计算曲线的长度
//double calculateCurveLength(const std::vector<Eigen::Vector3d>& curve_points) {
//    double length = 0.0;
//
//    for (size_t i = 1; i < curve_points.size(); ++i) {
//        length += euclideanDistance(curve_points[i - 1], curve_points[i]);
//    }
//
//    return length;
//}
//// 输出样本点到文件
//void outputCurveToFile(const std::vector<Eigen::Vector3d>& curve_points, const std::string& filename) {
//    std::ofstream file(filename);
//
//    if (file.is_open()) {
//        for (const auto& point : curve_points) {
//            file << point.x() << " " << point.y() << " " << point.z() << "\n";
//        }
//        file.close();
//        std::cout << "Curve points have been written to " << filename << std::endl;
//    }
//    else {
//        std::cerr << "Unable to open file " << filename << std::endl;
//    }
//}
//
//
//int main(int argc, char** argv) {
//
//    // 左耳、右耳和前中后三个部分的点云文件路径
//    std::string full_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\14.xyz";
//    std::string head_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\59.xyz";
//    std::string left_ear_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\57.xyz";
//    std::string right_ear_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\58.xyz";
//    std::string front_part_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\56.xyz";
//    std::string middle_part_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\55.xyz";
//    std::string back_part_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\54.xyz";
//    std::string tail_path = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\60.xyz";
//
//    // 创建点云指针
//    pcl::PointCloud<pcl::PointXYZ>::Ptr full(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr head(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr left_ear(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr right_ear(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr front_part(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr middle_part(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr back_part(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr tail_part(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // 加载点云
//    if (!loadXYZFile(head_path, head) || !loadXYZFile(full_path, full) || !loadXYZFile(left_ear_path, left_ear) ||
//        !loadXYZFile(right_ear_path, right_ear) ||
//        !loadXYZFile(front_part_path, front_part) ||
//        !loadXYZFile(middle_part_path, middle_part) ||
//        !loadXYZFile(back_part_path, back_part)|| !loadXYZFile(tail_path, tail_part)) {
//        std::cerr << "Error loading one or more point cloud files." << std::endl;
//        return -1;
//    }
//
//    pcl::PointXYZ centroid = computeCentroid(full);
//
//    // 将其他点云的坐标系原点移动到full的质心
//    translateToCentroid(full, centroid);
//    translateToCentroid(head, centroid);
//    translateToCentroid(left_ear, centroid);
//    translateToCentroid(right_ear, centroid);
//    translateToCentroid(front_part, centroid);
//    translateToCentroid(middle_part, centroid);
//    translateToCentroid(back_part, centroid);
//    translateToCentroid(tail_part, centroid);
//    //// 保存或可视化调整后的点云
//    pcl::io::savePCDFileASCII("E:\\pig1-150\\xyz150\\xyz150\\014_new\\body_length\\centern.pcd", *full);
//    // Extract extreme points on the x-axis for left and right ear
//    pcl::PointCloud<pcl::PointXYZ>::Ptr left_ear_min_points = extractExtremePoints(left_ear, true);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr right_ear_max_points = extractExtremePoints(right_ear, true);
//
//    // Calculate centroids of these extreme points
//    pcl::PointXYZ left_ear_min_center = computeCentroid(left_ear_min_points);
//    pcl::PointXYZ right_ear_max_center = computeCentroid(right_ear_max_points);
//
//    // Compute ear root as the midpoint between the two centroids
//    pcl::PointXYZ ear_root_;
//    //pcl::PointXYZ ear_root;
//    ear_root_.x = (left_ear_min_center.x + right_ear_max_center.x) / 2.0f;
//    ear_root_.y = (left_ear_min_center.y + right_ear_max_center.y) / 2.0f;
//    ear_root_.z = (left_ear_min_center.z + right_ear_max_center.z) / 2.0f;
//    // 设置搜索半径
//    float radius = 0.02f; // 根据需要调整这个半径值 
//    // Step 4: 根据前中后中心找到对应领域的点集质心
//    pcl::PointXYZ ear_root = findEarRegionCentroid(head, ear_root_.x, ear_root_.z, ear_root_.y, radius);
//    pcl::PointXYZ front_center = findZ(front_part);
//    pcl::PointXYZ middle_center = findZ(middle_part);
//    pcl::PointXYZ back_center = findZ(back_part);
//    // 设置搜索半径
//    // Step 4: 根据前中后中心找到对应领域的点集质心
//    pcl::PointXYZ p1 = findRegionCentroid(front_part, front_center.x, front_center.z, front_center.y,radius);
//    pcl::PointXYZ p3 = findRegionCentroid(middle_part, middle_center.x, middle_center.z, middle_center.y, radius);
//    pcl::PointXYZ p5 = findRegionCentroid(back_part, back_center.x, back_center.z, back_center.y, radius);
//
//    // 找到尾根点
//    pcl::PointXYZ tail_root = findTailRootPoint(tail_part);
//
//    // Step 1: Calculate front part X-axis max centroid
//    pcl::PointXYZ front_max_x_centroid = findXAxisExtremaCentroid(front_part, false);
//
//    // Step 2: Calculate middle part X-axis min centroid
//    pcl::PointXYZ middle_min_x_centroid = findXAxisExtremaCentroid(middle_part, true);
//
//    // Step 3: Calculate P2 as the midpoint of the above two centroids
//    pcl::PointXYZ P2;
//    P2.x = (front_max_x_centroid.x + middle_min_x_centroid.x) / 2.0f;
//    P2.y = (front_max_x_centroid.y + middle_min_x_centroid.y) / 2.0f;
//    P2.z = (front_max_x_centroid.z + middle_min_x_centroid.z) / 2.0f;
//    pcl::PointXYZ p2 = findRegionCentroid(front_part, P2.x, P2.z, P2.y,radius);
//    // Step 4: Calculate middle part X-axis max centroid
//    pcl::PointXYZ middle_max_x_centroid = findXAxisExtremaCentroid(middle_part, false);
//
//    // Step 5: Calculate back part X-axis min centroid
//    pcl::PointXYZ back_min_x_centroid = findXAxisExtremaCentroid(back_part, true);
//
//    // Step 6: Calculate P4 as the midpoint of the above two centroids
//    pcl::PointXYZ P4;
//    P4.x = (middle_max_x_centroid.x + back_min_x_centroid.x) / 2.0f;
//    P4.y = (middle_max_x_centroid.y + back_min_x_centroid.y) / 2.0f;
//    P4.z = (middle_max_x_centroid.z + back_min_x_centroid.z) / 2.0f;
//    pcl::PointXYZ p4 = findRegionCentroid(front_part, P4.x, P4.z, P4.y,radius);
//    // 创建一个点集，用于拟合曲线
//    std::vector<Eigen::Vector3d> points;
//    // 将有效点加入点集中
//    if (isPointValid(ear_root)) points.push_back(Eigen::Vector3d(ear_root.x, ear_root.y, ear_root.z));
//    if (isPointValid(p1)) points.push_back(Eigen::Vector3d(p1.x, p1.y, p1.z));
//    if (isPointValid(p2)) points.push_back(Eigen::Vector3d(p2.x, p2.y, p2.z));
//    if (isPointValid(p3)) points.push_back(Eigen::Vector3d(p3.x, p3.y, p3.z));
//    if (isPointValid(p4)) points.push_back(Eigen::Vector3d(p4.x, p4.y, p4.z));
//    if (isPointValid(p5)) points.push_back(Eigen::Vector3d(p5.x, p5.y, p5.z));
//    if (isPointValid(tail_root)) points.push_back(Eigen::Vector3d(tail_root.x, tail_root.y, tail_root.z));
//    // 确保有足够的点来拟合曲线
//    if (points.size() < 3) {
//        std::cerr << "Not enough valid points to fit a curve." << std::endl;
//        return -1;
//    }
//    // 拟合三维曲线
//    Eigen::VectorXd x_coeff, y_coeff, z_coeff;
//    std::tie(x_coeff, y_coeff, z_coeff) = fit3DCurve(points);
//    // 生成样本点
//    std::vector<Eigen::Vector3d> curve_points = sample3DCurve(x_coeff, y_coeff, z_coeff);
//    // 计算曲线的长度
//    double curve_length = calculateCurveLength(curve_points);
//    std::cout << "Curve Length: " << curve_length << std::endl;
//    // 将曲线长度写入到txt文件
//    std::ofstream output_file("E:\\pig1-150\\xyz150\\xyz150\\014_new\\body_length\\curve_length.txt");  // 打开文件（会自动创建或覆盖）
//
//    if (output_file.is_open()) {
//        // 将曲线长度写入文件
//        output_file << "Curve Length: " << curve_length << std::endl;
//        output_file.close();  // 关闭文件
//        std::cout << "Curve Length saved to curve_length.txt" << std::endl;
//    }
//    else {
//        std::cerr << "Unable to open file for writing." << std::endl;
//    }
//    // 输出样本点到文件
//    std::string filename = "E:\\pig1-150\\xyz150\\xyz150\\014_new\\body_length\\curve_points.txt";
//    outputCurveToFile(curve_points, filename);
//    //// 创建点云对象
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    //for (const auto& point : points) {
//    //    pcl::PointXYZ pclPoint(point.x(), point.y(), point.z());
//    //    pointCloud->points.push_back(pclPoint);
//    //}
//    //// 设置点云属性
//    //pointCloud->width = pointCloud->points.size();
//    //pointCloud->height = 1;
//    //pointCloud->is_dense = true;
//
//    // 保存点云
//    //pcl::io::savePCDFileASCII("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\3\\cut\\seven_points_3.pcd", *pointCloud);
//
//    //// 可视化点云
//    //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//    //viewer->setBackgroundColor(0, 0, 0);
//
//    //// 添加点云并设置颜色为红色
//    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pointCloud, 255, 0, 0);
//    //viewer->addPointCloud<pcl::PointXYZ>(pointCloud, single_color, "seven points cloud");
//    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "seven points cloud");
//
//    //// 显示坐标系
//    //viewer->addCoordinateSystem(1.0);
//    //viewer->initCameraParameters();
//
//    //// 主循环保持窗口开启
//    //while (!viewer->wasStopped()) {
//    //    viewer->spinOnce(150);
//    //}
//    return 0;
//}





