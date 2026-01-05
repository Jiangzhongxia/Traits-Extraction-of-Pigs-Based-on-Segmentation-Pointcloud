//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <string>
//#include <vector>
//#include <limits>
//#include <algorithm>
//#include <random>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d_pdm.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d_tdm.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d_sdm.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d_apdm.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
//#include <pcl/io/pcd_io.h>
//#include <boost/thread/thread.hpp>
//#include <pcl/surface/on_nurbs/triangulation.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/surface/convex_hull.h>
//#include <pcl/filters/crop_hull.h>
//#include <boost/thread/thread.hpp>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/filters/uniform_sampling.h>
//
//pcl::visualization::PCLVisualizer viewer("Curve Fitting 2D");
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
//
//void VisualizeCurve(ON_NurbsCurve& curve, double r, double g, double b, bool show_cps)
//{
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, cloud, 8);
//
//    for (std::size_t i = 0; i < cloud->size() - 1; i++)
//    {
//        pcl::PointXYZRGB& p1 = cloud->at(i);
//        pcl::PointXYZRGB& p2 = cloud->at(i + 1);
//        std::ostringstream os;
//        os << "line_" << r << "_" << g << "_" << b << "_" << i;
//        viewer.addLine<pcl::PointXYZRGB>(p1, p2, r, g, b, os.str());
//    }
//
//    if (show_cps)
//    {
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cps(new pcl::PointCloud<pcl::PointXYZ>);
//        for (int i = 0; i < curve.CVCount(); i++)
//        {
//            ON_3dPoint cp;
//            curve.GetCV(i, cp);
//
//            pcl::PointXYZ p;
//            p.x = float(cp.x);
//            p.y = float(cp.y);
//            p.z = float(cp.z);
//            cps->push_back(p);
//        }
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cps, 255 * r, 255 * g, 255 * b);
//        viewer.addPointCloud<pcl::PointXYZ>(cps, handler, "cloud_cps");
//    }
//}
//
//// Function to save XYZ format point cloud data
//bool saveXYZFile(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//    std::ofstream outfile(filename);
//    if (!outfile.is_open()) {
//        std::cerr << "Failed to open file: " << filename << std::endl;
//        return false;
//    }
//
//    for (const auto& point : cloud->points) {
//        outfile << point.x << " " << point.y << " " << point.z << "\n";
//    }
//
//    outfile.close();
//    return true;
//}
//
//
//int main()
//{
//    // 输入文件路径
//    std::string input_file = "E:\\Pig_Cattle\\Pig\\label\\pointm2ae_test\\2874——12\\upsample\\55_cloud.xyz"; // 替换为你的XYZ文件路径
//
//    // 读取点云数据
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    std::ifstream file(input_file);
//    if (!file.is_open())
//    {
//        std::cerr << "Failed to open file: " << input_file << std::endl;
//        return -1;
//    }
//
//    std::string line;
//    while (getline(file, line))
//    {
//        pcl::PointXYZ point;
//        std::istringstream iss(line);
//        iss >> point.x >> point.y >> point.z;
//        cloud->push_back(point);
//    }
//    file.close();
////
//    //// 变换为zoy平面上的2D原始点云（PCL中定义的2D曲线拟合方法默认在xoy平面方向曲线拟合）
//    //pcl::PointCloud<pcl::PointXYZ>::iterator it_1;
//    //for (it_1 = cloud->begin(); it_1 != cloud->end(); ++it_1)
//    //{
//    //    float x = it_1->x;
//    //    float y = it_1->y;
//    //    float z = it_1->z;
//
//    //    it_1->x = z;
//    //    it_1->y = y;
//    //    it_1->z = x;
//    //}
//
//    // 转换点云为2D格式
//    pcl::on_nurbs::NurbsDataCurve2d data;
//    PointCloud2Vector2d(cloud, data.interior);
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
//    //VisualizeCurve(fit.m_nurbs, 1.0, 0.0, 0.0, false);
//
//
//
//    // 获取原始点云的坐标范围
//    float min_x = std::numeric_limits<float>::max();
//    float max_x = -std::numeric_limits<float>::max();
//    float min_y = std::numeric_limits<float>::max();
//    float max_y = -std::numeric_limits<float>::max();
//    float min_z = std::numeric_limits<float>::max();
//    float max_z = -std::numeric_limits<float>::max();
//
//    for (const auto& point : cloud->points)
//    {
//        if (point.x < min_x) min_x = point.x;
//        if (point.x > max_x) max_x = point.x;
//        if (point.y < min_y) min_y = point.y;
//        if (point.y > max_y) max_y = point.y;
//        if (point.z < min_z) min_z = point.z;
//        if (point.z > max_z) max_z = point.z;
//    }
//
//
//    // 上采样
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upsampled(new pcl::PointCloud<pcl::PointXYZ>);
//    int num_upsample_points = cloud->points.size() * 3; // 上采样因子
//    for (int j = 0; j < num_upsample_points; ++j)
//    {
//
//        double t_start = 
// ;  // 起始节点值
//        double t_end = fit.m_nurbs.Knot(fit.m_nurbs.KnotCount() - 1);  // 终止节点值
//
//        // 输出节点值以进行调试
//        std::cout << "t_start: " << t_start << std::endl;
//        std::cout << "t_end: " << t_end << std::endl;
//
//
//
//        double t = fit.m_nurbs.Knot(0) + (fit.m_nurbs.Knot(fit.m_nurbs.KnotCount() - 1) - fit.m_nurbs.Knot(0)) * j / (num_upsample_points - 1);
//        double point2d[2];
//        fit.m_nurbs.Evaluate(t, 0, 3, point2d);
//
//        pcl::PointXYZ upsampled_point;
//        upsampled_point.x = min_x + static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (max_x - min_x);
//        upsampled_point.y = point2d[1]; // y to y
//        upsampled_point.z = point2d[0]; // x to z
//
//        // 检查上采样点是否在原始点云的坐标范围内
//        if (upsampled_point.x >= min_x && upsampled_point.x <= max_x )
//            //upsampled_point.y >= min_y && upsampled_point.y <= max_y &&
//            //upsampled_point.z >= min_z && upsampled_point.z <= max_z)
//        {
//            cloud_upsampled->push_back(upsampled_point);
//        }
//    }
//
//
//    //// 创建统计滤波器对象
//    //pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    //sor.setInputCloud(cloud_upsampled);
//    //sor.setMeanK(10);  // 设置在计算平均距离时考虑的邻近点数
//    //sor.setStddevMulThresh(1.0);  // 设置标准差倍数阈值
//
//    //// 应用统计滤波器
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    //sor.filter(*cloud_filtered);
////
//
//     //计算第一个点云数据的凸包
//    pcl::ConvexHull<pcl::PointXYZ> hull;
//    hull.setInputCloud(cloud);
//    hull.setDimension(3);
//    std::vector<pcl::Vertices> polygons;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
//    hull.setComputeAreaVolume(true);
//    hull.reconstruct(*surface_hull, polygons);
//
//    // 使用凸包对第二个点云数据进行滤波
//    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::CropHull<pcl::PointXYZ> cropHull;
//    cropHull.setInputCloud(cloud_upsampled);
//    cropHull.setHullIndices(polygons);
//    cropHull.setHullCloud(surface_hull);
//    cropHull.setDim(3);
//    cropHull.filter(*filtered_cloud);
//
//    // 输出滤波后的点云个数
//    std::cout << "Filtered cloud size: " << filtered_cloud->points.size() << std::endl;
//
//    // 合并第一个点云和滤波后的点云
//    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    *merged_cloud = *cloud + *filtered_cloud;
//
//    // 对合并后的点云进行上采样
//    pcl::PointCloud<pcl::PointXYZ>::Ptr upsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
//    uniform_sampling.setInputCloud(merged_cloud);
//    uniform_sampling.setRadiusSearch(0.005); // 设置采样半径，值越小点越密
//    uniform_sampling.filter(*upsampled_cloud);
//
//    //// 输出合并后的点云
//    //if (!saveXYZFile("E:\\Pig_Cattle\\Pig\\label\\pointm2ae_test\\2874——12\\upsample\\filtered_cloud_55_up.xyz", upsampled_cloud)) {
//    //    return -1;
//    //}
//
//    //// 变换为zoy平面上的2D原始点云（PCL中定义的2D曲线拟合方法默认在xoy平面方向曲线拟合）
//    //pcl::PointCloud<pcl::PointXYZ>::iterator it_1;
//    //for (it_1 = cloud->begin(); it_1 != cloud->end(); ++it_1)
//    //{
//    //    float x = it_1->x;
//    //    float y = it_1->y;
//    //    float z = it_1->z;
//
//    //    it_1->x = z;
//    //    it_1->y = y;
//    //    it_1->z = x;
//    //}
//   ////可视化
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Convex Hull Filtering and Upsampling"));
//    viewer->setWindowName("Reconstructed Convex Hull and Filtered Point Cloud");
//    viewer->setBackgroundColor(255, 255, 255);
//    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
//    viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "cloud2");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
//
//    // 凸包
//    viewer->addPolygonMesh<pcl::PointXYZ>(surface_hull, polygons, "hull");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "hull");
//    viewer->setRepresentationToWireframeForAllActors();
////
//    // 滤波后的点云
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filteredColor(filtered_cloud, 0, 255, 0);
//    viewer->addPointCloud<pcl::PointXYZ>(filtered_cloud, filteredColor, "filtered_cloud");
//
//    //// 合并后的点云
//    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> mergedColor(merged_cloud, 0, 0, 255);
//    //viewer->addPointCloud<pcl::PointXYZ>(merged_cloud, mergedColor, "merged_cloud");
//
//    //// 上采样后的点云
//    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> upsampledColor(upsampled_cloud, 255, 255, 0);
//    //viewer->addPointCloud<pcl::PointXYZ>(upsampled_cloud, upsampledColor, "upsampled_cloud");
//
//    while (!viewer->wasStopped()) {
//        viewer->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
//    }
//
//    return 0;
//}



//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <string>
//#include <vector>
//#include <limits>
//#include <algorithm>
//#include <random>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d_pdm.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d_tdm.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d_sdm.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d_apdm.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
//#include <pcl/io/pcd_io.h>
//#include <boost/thread/thread.hpp>
//#include <pcl/surface/on_nurbs/triangulation.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/surface/convex_hull.h>
//#include <pcl/filters/crop_hull.h>
//#include <pcl/filters/uniform_sampling.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>
//#include <pcl/common/common.h>
//using namespace std;
//
//pcl::visualization::PCLVisualizer viewer("Curve Fitting 2D");
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
//
//void VisualizeCurve(ON_NurbsCurve& curve, double r, double g, double b, bool show_cps)
//{
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, cloud, 8);
//
//    for (std::size_t i = 0; i < cloud->size() - 1; i++)
//    {
//        pcl::PointXYZRGB& p1 = cloud->at(i);
//        pcl::PointXYZRGB& p2 = cloud->at(i + 1);
//        std::ostringstream os;
//        os << "line_" << r << "_" << g << "_" << b << "_" << i;
//        viewer.addLine<pcl::PointXYZRGB>(p1, p2, r, g, b, os.str());
//    }
//
//    if (show_cps)
//    {
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cps(new pcl::PointCloud<pcl::PointXYZ>);
//        for (int i = 0; i < curve.CVCount(); i++)
//        {
//            ON_3dPoint cp;
//            curve.GetCV(i, cp);
//
//            pcl::PointXYZ p;
//            p.x = float(cp.x);
//            p.y = float(cp.y);
//            p.z = float(cp.z);
//            cps->push_back(p);
//        }
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cps, 255 * r, 255 * g, 255 * b);
//        viewer.addPointCloud<pcl::PointXYZ>(cps, handler, "cloud_cps");
//    }
//}
//
//// Function to save XYZ format point cloud data
//bool saveXYZFile(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//    std::ofstream outfile(filename);
//    if (!outfile.is_open()) {
//        std::cerr << "Failed to open file: " << filename << std::endl;
//        return false;
//    }
//
//    for (const auto& point : cloud->points) {
//        outfile << point.x << " " << point.y << " " << point.z << "\n";
//    }
//
//    outfile.close();
//    return true;
//}
//
//
//// Function to perform statistical upsampling using KNN
//pcl::PointCloud<pcl::PointXYZ>::Ptr statisticalUpsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int num_samples, int k_neighbors) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upsampled(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//    kdtree.setInputCloud(cloud);
//
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_int_distribution<> dis(0, cloud->points.size() - 1);
//
//    for (int i = 0; i < num_samples; ++i) {
//        int random_index = dis(gen);
//        pcl::PointXYZ query_point = cloud->points[random_index];
//
//        std::vector<int> point_idx_knn_search(k_neighbors);
//        std::vector<float> point_knn_squared_distance(k_neighbors);
//
//        if (kdtree.nearestKSearch(query_point, k_neighbors, point_idx_knn_search, point_knn_squared_distance) > 0) {
//            pcl::PointXYZ new_point;
//            float total_weight = 0.0f;
//            for (int j = 0; j < k_neighbors; ++j) {
//                float distance = sqrt(point_knn_squared_distance[j]);
//                float weight = 1.0f / (distance + std::numeric_limits<float>::epsilon());
//                new_point.x += cloud->points[point_idx_knn_search[j]].x * weight;
//                new_point.y += cloud->points[point_idx_knn_search[j]].y * weight;
//                new_point.z += cloud->points[point_idx_knn_search[j]].z * weight;
//                total_weight += weight;
//            }
//            new_point.x /= total_weight;
//            new_point.y /= total_weight;
//            new_point.z /= total_weight;
//            cloud_upsampled->push_back(new_point);
//        }
//    }
//
//    return cloud_upsampled;
//}
//
//// Function for MLS Upsampling
//pcl::PointCloud<pcl::PointXYZ>::Ptr mlsUpsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float search_radius) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upsampled(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
//    mls.setInputCloud(cloud);
//    mls.setSearchRadius(search_radius);
//
//    //mls.setPolynomialFit(true);
//    mls.setPolynomialOrder(2);
//    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
//    mls.setUpsamplingRadius(0.01);
//    mls.setUpsamplingStepSize(0.005);
//
//    mls.process(*cloud_upsampled);
//
//    return cloud_upsampled;
//}
//
//pcl::PointCloud<pcl::PointXYZ>::Ptr processSlice(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_slice, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
//    // 转换点云为2D格式
//    pcl::on_nurbs::NurbsDataCurve2d data;
//    PointCloud2Vector2d(cloud_slice, data.interior);
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
//
//    // 获取原始点云的坐标范围
//    float min_x = std::numeric_limits<float>::max();
//    float max_x = -std::numeric_limits<float>::max();
//    float min_y = std::numeric_limits<float>::max();
//    float max_y = -std::numeric_limits<float>::max();
//    float min_z = std::numeric_limits<float>::max();
//    float max_z = -std::numeric_limits<float>::max();
//
//    for (const auto& point : cloud_slice->points)
//    {
//        if (point.x < min_x) min_x = point.x;
//        if (point.x > max_x) max_x = point.x;
//        if (point.y < min_y) min_y = point.y;
//        if (point.y > max_y) max_y = point.y;
//        if (point.z < min_z) min_z = point.z;
//        if (point.z > max_z) max_z = point.z;
//    }
//
//    // 上采样
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upsampled(new pcl::PointCloud<pcl::PointXYZ>);
//    int num_upsample_points = cloud_slice->points.size() * 20; // 上采样因子
//    for (int j = 0; j < num_upsample_points; ++j)
//    {
//        double t_start = fit.m_nurbs.Knot(0);  // 起始节点值
//        double t_end = fit.m_nurbs.Knot(fit.m_nurbs.KnotCount() - 1);  // 终止节点值
//
//        double t = t_start + (t_end - t_start) * j / (num_upsample_points - 1);
//        double point2d[2];
//        fit.m_nurbs.Evaluate(t, 0, 2, point2d);
//
//        pcl::PointXYZ upsampled_point;
//        upsampled_point.x = min_x + static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (max_x - min_x);
//        upsampled_point.y = point2d[1]; // y to y
//        upsampled_point.z = point2d[0]; // x to z
//
//        // 检查上采样点是否在原始点云的坐标范围内
//        if (upsampled_point.x >= min_x && upsampled_point.x <= max_x)
//        {
//            cloud_upsampled->push_back(upsampled_point);
//        }
//    }
//
//    //// 创建统计滤波器对象
//    //pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    //sor.setInputCloud(cloud_upsampled);
//    //sor.setMeanK(10);  // 设置在计算平均距离时考虑的邻近点数
//    //sor.setStddevMulThresh(1.0);  // 设置标准差倍数阈值
//
//    //// 应用统计滤波器
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    //sor.filter(*cloud_filtered);
//
//    // 计算第一个点云数据的凸包
//    pcl::ConvexHull<pcl::PointXYZ> hull;
//    hull.setInputCloud(cloud);
//    hull.setDimension(3);
//    std::vector<pcl::Vertices> polygons;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
//    hull.setComputeAreaVolume(true);
//    hull.reconstruct(*surface_hull, polygons);
//
//    // 使用凸包对第二个点云数据进行滤波
//    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::CropHull<pcl::PointXYZ> cropHull;
//    cropHull.setInputCloud(cloud_upsampled);
//    cropHull.setHullIndices(polygons);
//    cropHull.setHullCloud(surface_hull);
//    cropHull.setDim(3);
//    cropHull.filter(*filtered_cloud);
//
//    // 合并第一个点云和滤波后的点云
//    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    *merged_cloud = *cloud_slice + *filtered_cloud;
//    std::cout << "merged_cloud size: " << merged_cloud->points.size() << std::endl;
//    // 对合并后的点云进行上采样
//    pcl::PointCloud<pcl::PointXYZ>::Ptr upsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
//    uniform_sampling.setInputCloud(merged_cloud);
//    uniform_sampling.setRadiusSearch(0.0005); // 设置采样半径，值越小点越密
//    uniform_sampling.filter(*upsampled_cloud);
//    //int num_samples = 4000;
//    //int k_neighbors = 10;
//    //upsampled_cloud = statisticalUpsample(merged_cloud, num_samples, k_neighbors);
//    std::cout << "upsampled_cloud size: " << upsampled_cloud->points.size() << std::endl;
//    return upsampled_cloud;
//}
//
//pcl::PointCloud<pcl::PointXYZ>::Ptr processSlice_2(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
//    // 转换点云为2D格式
//    pcl::on_nurbs::NurbsDataCurve2d data;
//    PointCloud2Vector2d(cloud, data.interior);
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
//
//    // 获取原始点云的坐标范围
//    float min_x = std::numeric_limits<float>::max();
//    float max_x = -std::numeric_limits<float>::max();
//    float min_y = std::numeric_limits<float>::max();
//    float max_y = -std::numeric_limits<float>::max();
//    float min_z = std::numeric_limits<float>::max();
//    float max_z = -std::numeric_limits<float>::max();
//
//    for (const auto& point : cloud->points)
//    {
//        if (point.x < min_x) min_x = point.x;
//        if (point.x > max_x) max_x = point.x;
//        if (point.y < min_y) min_y = point.y;
//        if (point.y > max_y) max_y = point.y;
//        if (point.z < min_z) min_z = point.z;
//        if (point.z > max_z) max_z = point.z;
//    }
//
//    // 上采样
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upsampled(new pcl::PointCloud<pcl::PointXYZ>);
//    int num_upsample_points = cloud->points.size() * 20; // 上采样因子
//    for (int j = 0; j < num_upsample_points; ++j)
//    {
//        double t_start = fit.m_nurbs.Knot(0);  // 起始节点值
//        double t_end = fit.m_nurbs.Knot(fit.m_nurbs.KnotCount() - 1);  // 终止节点值
//
//        double t = t_start + (t_end - t_start) * j / (num_upsample_points - 1);
//        double point2d[2];
//        fit.m_nurbs.Evaluate(t, 0, 2, point2d);
//
//        pcl::PointXYZ upsampled_point;
//        upsampled_point.x = min_x + static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (max_x - min_x);
//        upsampled_point.y = point2d[1]; // y to y
//        upsampled_point.z = point2d[0]; // x to z
//
//        // 检查上采样点是否在原始点云的坐标范围内
//        if (upsampled_point.x >= min_x && upsampled_point.x <= max_x)
//        {
//            cloud_upsampled->push_back(upsampled_point);
//        }
//    }
//
//    // 创建统计滤波器对象
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud(cloud_upsampled);
//    sor.setMeanK(10);  // 设置在计算平均距离时考虑的邻近点数
//    sor.setStddevMulThresh(1.0);  // 设置标准差倍数阈值
//
//    // 应用统计滤波器
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    sor.filter(*cloud_filtered);
//
//
//    return cloud_filtered;
//}
//
//int main()
//{
//    // 输入文件路径
//    std::string input_file = "E:\\Pig_Cattle\\Pig\\label\\pointm2ae_test\\2874——12\\cloud_label_55.xyz";
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // 读取点云数据
//    std::ifstream file(input_file);
//    if (!file.is_open())
//    {
//        std::cerr << "Failed to open file: " << input_file << std::endl;
//        return -1;
//    }
//
//    std::string line;
//    while (getline(file, line))
//    {
//        pcl::PointXYZ point;
//        std::istringstream iss(line);
//        iss >> point.x >> point.y >> point.z;
//        cloud->push_back(point);
//    }
//    file.close();
//
//    // 获取x轴的最小值和最大值
//    float min_x = std::numeric_limits<float>::max();
//    float max_x = -std::numeric_limits<float>::max();
//    for (const auto& point : cloud->points)
//    {
//        if (point.x < min_x) min_x = point.x;
//        if (point.x > max_x) max_x = point.x;
//    }
//
//    // 计算切片的边界
//    float range1 = min_x + (max_x - min_x) * 0.35;
//    float range2 = min_x + (max_x - min_x) * 0.65;
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr slice1(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr slice2(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr slice3(new pcl::PointCloud<pcl::PointXYZ>);
//
//    for (const auto& point : cloud->points)
//    {
//        if (point.x < range1) {
//            slice1->push_back(point);
//        }
//        else if (point.x < range2) {
//            slice2->push_back(point);
//        }
//        else {
//            slice3->push_back(point);
//        }
//    }
//    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_slice2 = processSlice_2(slice2);
//    *cloud += *processed_slice2;
//
//    //// 对每个切片进行上采样处理
//    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_slice1 = processSlice(slice1,cloud);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_slice3 = processSlice(slice3, cloud);
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr processed_slice2 = processSlice_2(slice2);
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr processed_slice3 = processSlice(slice3);
//
//    // 合并上采样后的点云
//    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    *combined_cloud += *processed_slice1;
//    *combined_cloud += *cloud;
//    *combined_cloud += *processed_slice3;
//
//    // ---------------创建上采样对象-----------------
//    pcl::PointCloud<pcl::PointXYZ>::Ptr up_combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> up;
//    up.setInputCloud(combined_cloud);
//    //----------------建立搜索对象-----------------
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
//    up.setSearchMethod(tree);
//    //--------设置搜索邻域的半径-------------------
//    up.setSearchRadius(0.1);
//
//    up.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
//    // ----------设置采样的半径----------------
//    up.setUpsamplingRadius(0.04);
//    // -------采样步长的大小-------------
//    up.setUpsamplingStepSize(0.02);
//    up.process(*up_combined_cloud);
//
//    // 创建统计滤波器对象
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud(up_combined_cloud);
//    sor.setMeanK(15);  // 设置在计算平均距离时考虑的邻近点数
//    sor.setStddevMulThresh(1.0);  // 设置标准差倍数阈值
//
//    // 应用统计滤波器
//    pcl::PointCloud<pcl::PointXYZ>::Ptr up_combined_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    sor.filter(*up_combined_cloud_filtered);
//
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr upsampled_combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    //int num_samples = 4000;
//    //int k_neighbors = 10;
//    //upsampled_combined_cloud = statisticalUpsample(combined_cloud, num_samples, k_neighbors);
//    //float search_radius = 0.05;
//    //upsampled_combined_cloud = mlsUpsample(combined_cloud, search_radius);
//    //std::cout << "upsampled_cloud size: " << upsampled_combined_cloud->points.size() << std::endl;
//     //输出合并后的点云
//    //if (!saveXYZFile("E:\\Pig_Cattle\\Pig\\label\\pointm2ae_test\\2874——12\\upsample\\processedslice1_55.xyz", processed_slice1))
//    //{
//    //    std::cerr << "Failed to save combined point cloud." << std::endl;
//    //    return -1;
//    //}
//    //if (!saveXYZFile("E:\\Pig_Cattle\\Pig\\label\\pointm2ae_test\\2874——12\\upsample\\processedslice2_55.xyz", processed_slice2))
//    //{
//    //    std::cerr << "Failed to save combined point cloud." << std::endl;
//    //    return -1;
//    //}
//    //if (!saveXYZFile("E:\\Pig_Cattle\\Pig\\label\\pointm2ae_test\\2874——12\\upsample\\processedslice3_55.xyz", processed_slice3))
//    //{
//    //    std::cerr << "Failed to save combined point cloud." << std::endl;
//    //    return -1;
//    //}
//    if (!saveXYZFile("E:\\Pig_Cattle\\Pig\\label\\pointm2ae_test\\2874——12\\upsample\\55_up_combined_cloud_filtered.xyz", up_combined_cloud_filtered))
//    {
//        std::cerr << "Failed to save combined point cloud." << std::endl;
//        return -1;
//    }
//    std::cout << "Point cloud processing and saving completed." << std::endl;
//
//    //// 可视化
//    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Convex Hull Filtering and Upsampling"));
//    //viewer->setWindowName("Reconstructed Convex Hull and Filtered Point Cloud");
//    //viewer->setBackgroundColor(255, 255, 255);
//    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
//    //viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "cloud2");
//    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
//
//    //// 凸包
//    //viewer->addPolygonMesh<pcl::PointXYZ>(surface_hull, polygons, "hull");
//    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "hull");
//    //viewer->setRepresentationToWireframeForAllActors();
//
//    return 0;
//}
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/surface/mls.h>
//#include <pcl/search/kdtree.h> //单独使用kd树的头文件（方法之一）
//#include <boost/thread/thread.hpp>
//#include <pcl/visualization/pcl_visualizer.h>
//
//
//using namespace std;
//
//int main()
//{
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_up(new pcl::PointCloud<pcl::PointXYZ>);
//
//    if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:\\all_file_for_better_future\\paper\\images\\deeplearning——module\\014_new\\14_up.pcd", *cloud) != 0)
//    {
//        return -1;
//    }
//    cout << "原始点云个数：" << cloud->points.size() << endl;
//    // ---------------创建上采样对象-----------------
//    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> up;
//    up.setInputCloud(cloud);
//    //----------------建立搜索对象-----------------
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
//    up.setSearchMethod(tree);
//    //--------设置搜索邻域的半径-------------------
//    up.setSearchRadius(0.05);
//
//    up.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
//    // ----------设置采样的半径----------------
//    up.setUpsamplingRadius(0.01);
//    // -------采样步长的大小-------------
//    up.setUpsamplingStepSize(0.01);
//
//    up.process(*cloud_up);
//    pcl::io::savePCDFileASCII("E:\\all_file_for_better_future\\paper\\images\\deeplearning——module\\014_new\\14.pcd", *cloud_up);
//
//    cout << "上采样后点云的个数：" << cloud_up->points.size() << endl;
//    //---------显示点云-----------------------
//    boost::shared_ptr<pcl::visualization::PCLVisualizer>
//        viewer(new pcl::visualization::PCLVisualizer("显示点云"));
//
//    int v1(0), v2(0);
//    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//    viewer->setBackgroundColor(0, 0, 0, v1);
//    viewer->addText("point clouds", 10, 10, "v1_text", v1);
//    viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
//    viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
//    viewer->addText("Upsampled point clouds", 10, 10, "v2_text", v2);
//
//    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
//    viewer->addPointCloud<pcl::PointXYZ>(cloud_up, "cloud_up", v2);
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_up", v2);
//    //viewer->addCoordinateSystem(1.0);
//    //viewer->initCameraParameters();
//    while (!viewer->wasStopped())
//    {
//        viewer->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }
//
//    return 0;
//}
//
