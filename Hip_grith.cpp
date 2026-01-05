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
//const float SLICE_THICKNESS = 0.04;
//
//void
//VisualizeCurve(ON_NurbsCurve& curve, double r, double g, double b, bool show_cps)
//{
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, cloud, 8);
//    // Save the point cloud to a PCD file
//    pcl::io::savePCDFileASCII("E:\\pig1-150\\xyz150\\xyz150\\150\\Hip_grith\\curve_points.pcd", *cloud);
//    std::cout << "Saved " << cloud->points.size() << " data points to curve_points.pcd." << std::endl;
//    // Calculate the total length of the curve
//    double total_length = 0.0;
//    for (std::size_t i = 0; i < cloud->size() - 1; i++)
//    {
//        pcl::PointXYZRGB& p1 = cloud->at(i);
//        pcl::PointXYZRGB& p2 = cloud->at(i + 1);
//
//        // Calculate Euclidean distance between p1 and p2
//        double distance = std::sqrt(
//            std::pow(p2.x - p1.x, 2) +
//            std::pow(p2.y - p1.y, 2) +
//            std::pow(p2.z - p1.z, 2)
//        );
//
//        total_length += distance;
//    }
//    std::cout << "Total length of the curve: " << total_length << " units." << std::endl;
//    // 将曲线长度写入到txt文件
//    std::ofstream output_file("E:\\pig1-150\\xyz150\\xyz150\\150\\Hip_grith\\curve_length.txt");  // 打开文件（会自动创建或覆盖）
//
//    if (output_file.is_open()) {
//        // 将曲线长度写入文件
//        output_file << "Curve Length: " << total_length << std::endl;
//        output_file.close();  // 关闭文件
//        std::cout << "Curve Length saved to curve_length.txt" << std::endl;
//    }
//    else {
//        std::cerr << "Unable to open file for writing." << std::endl;
//    }
//
//    std::cout << "Total length of the curve: " << total_length << " units." << std::endl;
//
//    //for (std::size_t i = 0; i < cloud->size() - 1; i++)
//    //{
//    //    pcl::PointXYZRGB& p1 = cloud->at(i);
//    //    pcl::PointXYZRGB& p2 = cloud->at(i + 1);
//    //    std::ostringstream os;
//    //    os << "line_" << r << "_" << g << "_" << b << "_" << i;
//    //    viewer.addLine<pcl::PointXYZRGB>(p1, p2, r, g, b, os.str());
//
//    //}
//
//
//    //if (show_cps)
//    //{
//    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cps(new pcl::PointCloud<pcl::PointXYZ>);
//    //    for (int i = 0; i < curve.CVCount(); i++)
//    //    {
//    //        ON_3dPoint cp;
//    //        curve.GetCV(i, cp);
//
//    //        pcl::PointXYZ p;
//    //        p.x = float(cp.x);
//    //        p.y = float(cp.y);
//    //        p.z = float(cp.z);
//    //        cps->push_back(p);
//    //    }
//    //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cps, 255 * r, 255 * g, 255 * b);
//    //    viewer.addPointCloud<pcl::PointXYZ>(cps, handler, "cloud_cps");
//
//    //}
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
//pcl::PointCloud<pcl::PointXYZ>::Ptr findAndSlicePointCloud_back(const pcl::PointCloud<pcl::PointXYZ>::Ptr& back_part) {
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
//    //pcl::io::savePCDFileASCII("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\Hip_grith\\points_cut.pcd", *lower_slice);
//
//    return lower_slice;
//}
//
//pcl::PointCloud<pcl::PointXYZ>::Ptr findAndSlicePointCloud_middle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& back_part) {
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
//
//    // 使用通过滤波器切出x轴方向2cm范围的切片
//    pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setFilterFieldName("x");
//    pass.setFilterLimits( max_lower_x.x - SLICE_THICKNESS,max_lower_x.x);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr lower_slice(new pcl::PointCloud<pcl::PointXYZ>());
//    pass.setInputCloud(lower_part);
//    pass.filter(*lower_slice);
//
//    pass.setFilterLimits( max_upper_x.x - SLICE_THICKNESS,max_upper_x.x);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr upper_slice(new pcl::PointCloud<pcl::PointXYZ>());
//    pass.setInputCloud(upper_part);
//    pass.filter(*upper_slice);
//
//    // 合并两个切片
//    *lower_slice += *upper_slice;
//
//    // 保存合并后的切片为PCD文件
//    //pcl::io::savePCDFileASCII("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\Hip_grith\\points_cut.pcd", *lower_slice);
//
//    return lower_slice;
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
//// 用于将点云数据转换为2D NURBS曲线拟合所需的格式
//void PointCloud2Vector2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::on_nurbs::NurbsDataCurve2d& data) {
//    // 假设转换为 (z, y) 的数据点
//    for (const auto& point : cloud->points) {
//        data.interior.push_back(Eigen::Vector2d(point.z, point.y));
//    }
//}
//
//void mirrorAndCombinePointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr& left_part,
//    pcl::PointCloud<pcl::PointXYZ>::Ptr& right_part) {
//    // 计算左右两部分点云的中心
//    Eigen::Vector4f left_center, right_center;
//    pcl::compute3DCentroid(*left_part, left_center);
//    pcl::compute3DCentroid(*right_part, right_center);
//
//    // 计算中心轴（假设是沿 y 轴对称）
//    float center_z = (left_center[2] + right_center[2]) / 2.0f;
//
//    // 1. 将左部分点云镜像复制到右边
//    pcl::PointCloud<pcl::PointXYZ>::Ptr mirrored_left_part(new pcl::PointCloud<pcl::PointXYZ>());
//    for (const auto& point : left_part->points) {
//        pcl::PointXYZ mirrored_point = point;
//        mirrored_point.z = 2 * center_z - mirrored_point.z;  // 沿y轴对称
//        mirrored_left_part->points.push_back(mirrored_point);
//    }
//
//    // 2. 将右部分点云镜像复制到左边
//    pcl::PointCloud<pcl::PointXYZ>::Ptr mirrored_right_part(new pcl::PointCloud<pcl::PointXYZ>());
//    for (const auto& point : right_part->points) {
//        pcl::PointXYZ mirrored_point = point;
//        mirrored_point.z = 2 * center_z - mirrored_point.z;  // 沿y轴对称
//        mirrored_right_part->points.push_back(mirrored_point);
//    }
//
//    // 3. 将左右两部分点云及它们的镜像合并
//    *left_part += *mirrored_left_part;  // 左部分加上右部分的镜像
//    *right_part += *mirrored_right_part;  // 右部分加上左部分的镜像
//}
//
//// 分割点云并进行对称性约束处理
//void fitAndVisualizeNurbsCurveWithSymmetry(pcl::PointCloud<pcl::PointXYZ>::Ptr& thoracic_slice_back) {
//    // 检查点云是否为空
//    if (thoracic_slice_back->empty()) {
//        std::cerr << "Point cloud is empty!" << std::endl;
//        return;
//    }
//
//    // 将点云分为左右两部分
//    pcl::PointCloud<pcl::PointXYZ>::Ptr left_part(new pcl::PointCloud<pcl::PointXYZ>());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr right_part(new pcl::PointCloud<pcl::PointXYZ>());
//
//    // 找到z轴的中值
//    pcl::PointXYZ min_z_point = thoracic_slice_back->points[0];
//    pcl::PointXYZ max_z_point = thoracic_slice_back->points[0];
//
//    for (const auto& point : thoracic_slice_back->points) {
//        if (point.z < min_z_point.z) {
//            min_z_point = point;
//        }
//        if (point.z > max_z_point.z) {
//            max_z_point = point;
//        }
//    }
//
//    float z_mid = (min_z_point.z + max_z_point.z) / 2.0f;
//
//    // 分割点云到左右部分
//    for (const auto& point : thoracic_slice_back->points) {
//        if (point.z <= z_mid) {
//            left_part->points.push_back(point);
//        }
//        else {
//            right_part->points.push_back(point);
//        }
//    }
//
//    //mirrorAndCombinePointClouds(left_part, right_part);
//    // 合并左右两部分
//    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_part(new pcl::PointCloud<pcl::PointXYZ>());
//    *combined_part =   *left_part+ *right_part;
//    //pcl::io::savePCDFileASCII("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\Hip_grith\\combined_part_cen.pcd", *combined_part);
//    // 转换点云数据为NURBS曲线拟合所需的格式
//    pcl::on_nurbs::NurbsDataCurve2d data;
//    PointCloud2Vector2d(combined_part, data);
//
//    // 设置NURBS曲线的阶数和控制点数量
//    unsigned order = 3;  // 3阶曲线
//    unsigned n_control_points = 15;  // 设置控制点的数量，具体值可以调整
//
//    // 初始化NURBS曲线
//    ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2dTDM::initNurbsCurve2D(order, data.interior, n_control_points);
//
//    // 设置曲线拟合的参数
//    pcl::on_nurbs::FittingCurve2dTDM::Parameter curve_params;
//    curve_params.smoothness = 0.000150;  // 平滑度，可以根据需要调整
//    curve_params.rScale = 1.0;  // 比例因子
//
//    // 使用FittingCurve2dSDM进行拟合
//    pcl::on_nurbs::FittingCurve2dSDM fit(&data, curve);
//    fit.assemble(curve_params);
//    fit.solve();
//
//    // 可视化拟合曲线的结果
//    VisualizeCurve(fit.m_nurbs, 1.0, 0.0, 0.0, true);  // 设定曲线颜色为红色
//
//    //// 可视化点云和拟合曲线
//    //pcl::visualization::PCLVisualizer viewer("NURBS Curve Fitting Viewer");
//    //viewer.setSize(800, 600);
//    //viewer.setBackgroundColor(255, 255, 255);  // 设置背景为白色
//    //viewer.addPointCloud<pcl::PointXYZ>(combined_part, "cloud");
//    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud");  // 设置点云为黑色
//    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "cloud");  // 设置点云的大小
//
//
//    //// 循环显示直到关闭窗口
//    //while (!viewer.wasStopped()) {
//    //    viewer.spinOnce(150);  // 每150ms更新一次视图
//    //}
//}
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
//    std::string output_path = "E:\\pig1-150\\xyz150\\xyz150\\150\\Hip_grith\\points.pcd";
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
//
//    // 将其他点云的坐标系原点移动到full的质心
//    translateToCentroid(full, centroid);
//    translateToCentroid(head, centroid);
//    translateToCentroid(left_ear, centroid);
//    translateToCentroid(right_ear, centroid);
//    translateToCentroid(front_part, centroid);
//    translateToCentroid(middle_part, centroid);
//    translateToCentroid(back_part, centroid);
//    translateToCentroid(Afront_part, centroid);
//    translateToCentroid(Bfront_part, centroid);
//    //pcl::io::savePCDFileASCII("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\Hip_grith\\55_cen.pcd", *middle_part);
//    //pcl::io::savePCDFileASCII("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\Hip_grith\\54_cen.pcd", *back_part);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr thoracic_slice_middle(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr thoracic_slice_back(new pcl::PointCloud<pcl::PointXYZ>);
//    thoracic_slice_middle = findAndSlicePointCloud_middle(middle_part);
//    thoracic_slice_back = findAndSlicePointCloud_back(back_part);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr final_part(new pcl::PointCloud<pcl::PointXYZ>());
//    *final_part = *thoracic_slice_middle + *thoracic_slice_back;
//    //pcl::io::savePCDFileASCII("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\Hip_grith\\thoracic_slice_middle.pcd", *thoracic_slice_middle);
//    //pcl::io::savePCDFileASCII("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\Hip_grith\\thoracic_slice_back.pcd", *thoracic_slice_back);
//    ////saveSplineAsPCD(spline, "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\3\\cut\\chest_grith\\thoracic_spline_curve.pcd", 150);
//    //pcl::on_nurbs::NurbsDataCurve2d data;
//    //PointCloud2Vector2d(thoracic_slice_back, data.interior);
//
//    //// 初始化曲线
//    //unsigned order(3);
//    //unsigned n_control_points(10);
//    //ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2dTDM::initNurbsCurve2D(order, data.interior, n_control_points);
//
//    //// 曲线拟合
//    //pcl::on_nurbs::FittingCurve2dTDM::Parameter curve_params;
//    //curve_params.smoothness = 0.000150;
//    //curve_params.rScale = 1.0;
//
//    //pcl::on_nurbs::FittingCurve2dSDM fit(&data, curve);
//    //fit.assemble(curve_params);
//    //fit.solve();
//    //VisualizeCurve(fit.m_nurbs, 1.0, 0.0, 0.0, true);
//    ////------------可视化曲线拟合的结果-------------------------
//    //viewer.setSize(800, 600);
//    //viewer.setBackgroundColor(255, 255, 255);
//    //viewer.addPointCloud<pcl::PointXYZ>(thoracic_slice_back, "cloud");
//    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud");//腿部切割点云
//    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "cloud");
//    //while (!viewer.wasStopped())
//    //{
//    //    viewer.spinOnce(150);
//    //}
//    // 对点云进行曲线拟合并可视化
//    fitAndVisualizeNurbsCurveWithSymmetry(final_part);
//
//    return 0;
//}