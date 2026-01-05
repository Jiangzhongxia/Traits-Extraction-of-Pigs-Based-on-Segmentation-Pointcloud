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
//// 拟合平行于zoy面的平面方程 (x = constant)
//float fitPlaneFromMinY(const pcl::PointXYZRGB& min_y_point) {
//    // 返回平面的 x 值（constant）
//    return min_y_point.y;
//}
//
//// 计算点到平面 (x = constant) 的垂直距离
//float pointToPlaneDistance(const pcl::PointXYZRGB& point, float plane_y) {
//    return std::fabs(point.y - plane_y);
//}
//
//void
//VisualizeCurve(ON_NurbsCurve& curve, double r, double g, double b, bool show_cps)
//{
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, cloud, 8);
//    // 遍历点云，找到y轴最大值和最小值
//    pcl::PointXYZRGB min_y_point = cloud->points[0];
//    pcl::PointXYZRGB max_y_point = cloud->points[0];
//    for (const auto& point : cloud->points) {
//        if (point.y < min_y_point.y) {
//            min_y_point = point;
//        }
//        if (point.y > max_y_point.y) {
//            max_y_point = point;
//        }
//    }
//    // 拟合平行于zoy面的平面 (x = constant)
//    float plane_x = fitPlaneFromMinY(min_y_point);
//
//    // 计算最大y值点到平面的垂直距离
//    float distance = pointToPlaneDistance(max_y_point, plane_x);
//    std::cout << "Distance from max y point to the plane: " << distance << std::endl;
//}
//
//void PointCloud2Vector2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec2d& data)
//{
//    for (unsigned i = 0; i < cloud->size(); i++)
//    {
//        pcl::PointXYZ& p = cloud->at(i);
//        if (!std::isnan(p.z) && !std::isnan(p.y))
//            data.push_back(Eigen::Vector2d(p.z, p.y));  // Projecting to zoy plane
//    }
//}
//pcl::PointCloud<pcl::PointXYZ>::Ptr findAndSlicePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& back_part) {
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
//    pcl::PointXYZ min_lower_x = lower_part->points[0];
//    pcl::PointXYZ min_upper_x = upper_part->points[0];
//
//    for (const auto& point : lower_part->points) {
//        if (point.x < min_lower_x.x) {
//            min_lower_x = point;
//        }
//    }
//
//    for (const auto& point : upper_part->points) {
//        if (point.x < min_upper_x.x) {
//            min_upper_x = point;
//        }
//    }
//
//
//    // 使用通过滤波器切出x轴方向2cm范围的切片
//    pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setFilterFieldName("x");
//    pass.setFilterLimits(min_lower_x.x, min_lower_x.x + SLICE_THICKNESS);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr lower_slice(new pcl::PointCloud<pcl::PointXYZ>());
//    pass.setInputCloud(lower_part);
//    pass.filter(*lower_slice);
//
//    pass.setFilterLimits(min_upper_x.x, min_upper_x.x + SLICE_THICKNESS);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr upper_slice(new pcl::PointCloud<pcl::PointXYZ>());
//    pass.setInputCloud(upper_part);
//    pass.filter(*upper_slice);
//
//    // 合并两个切片
//    *lower_slice += *upper_slice;
//
//    // 保存合并后的切片为PCD文件
//    //pcl::io::savePCDFileASCII("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\6\\cut\\chest_grith\\points_cut.pcd", *lower_slice);
//
//    return lower_slice;
//}
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
//
//int main(int argc, char** argv) {
//
//    // 左耳、右耳和前中后三个部分的点云文件路径
//    std::string full_path = "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\80.xyz";
//    std::string head_path = "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\59.xyz";
//    std::string left_ear_path = "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\57.xyz";
//    std::string right_ear_path = "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\58.xyz";
//    std::string front_part_path = "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\56.xyz";
//    std::string middle_part_path = "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\55.xyz";
//    std::string back_part_path = "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\54.xyz";
//    std::string Afront_path = "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\50.xyz";
//    std::string Bfront_path = "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\51.xyz";
//    std::string output_path = "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\chest_grith\\points.pcd";
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
//    pcl::PointXYZ centroid = computeCentroid(middle_part);
//
//    // 将其他点云的坐标系原点移动到full的质心
//    translateToCentroid(middle_part, centroid);
//    //pcl::io::savePCDFileASCII("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\6\\cut\\chest_grith\\55_cen.pcd", *middle_part);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr thoracic_slice(new pcl::PointCloud<pcl::PointXYZ>);
//    thoracic_slice = findAndSlicePointCloud(middle_part);
//    //saveSplineAsPCD(spline, "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\3\\cut\\chest_grith\\thoracic_spline_curve.pcd", 100);
//    pcl::on_nurbs::NurbsDataCurve2d data;
//    PointCloud2Vector2d(thoracic_slice, data.interior);
//
//    // 初始化曲线
//    unsigned order(3);
//    unsigned n_control_points(10);
//    ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2dTDM::initNurbsCurve2D(order, data.interior, n_control_points);
//
//    // 曲线拟合
//    pcl::on_nurbs::FittingCurve2dTDM::Parameter curve_params;
//    curve_params.smoothness = 0.000001;
//    curve_params.rScale = 1.0;
//
//    pcl::on_nurbs::FittingCurve2dSDM fit(&data, curve);
//    fit.assemble(curve_params);
//    fit.solve();
//    VisualizeCurve(fit.m_nurbs, 1.0, 0.0, 0.0, true);
//    return 0;
//}