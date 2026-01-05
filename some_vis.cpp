







////法向量可视化++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <pcl/point_types.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <thread>  // 用于 std::this_thread::sleep_for
//#include <chrono>  // 用于 std::chrono::milliseconds
//using namespace std;
//// 读取XYZ文件并包含法向量信息
//bool loadXYZWithNormals(const std::string& filename, pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
//{
//    std::ifstream file(filename);
//    if (!file.is_open())
//    {
//        std::cerr << "Failed to open file: " << filename << std::endl;
//        return false;
//    }
//
//    std::string line;
//    while (std::getline(file, line))
//    {
//        pcl::PointNormal point;
//        std::istringstream iss(line);
//        iss >> point.x >> point.y >> point.z >> point.normal_x >> point.normal_y >> point.normal_z;
//        cloud->push_back(point);
//    }
//    file.close();
//    return true;
//}
//
//int main(int argc, char** argv)
//{
//    //if (argc < 3)
//    //{
//    //    std::cerr << "Usage: " << argv[0] << " <input_file1.xyz> <input_file2.xyz>" << std::endl;
//    //    return -1;
//    //}
//
//    std::string input_file1 = "E:\\Pig_Cattle\\Pig\\label\\pointm2ae_test\\2875——3\\upsample\\cloud_label_55_up.xyz";
//    std::string input_file2 = "E:\\Pig_Cattle\\Pig\\label\\pointm2ae_test\\2875——3\\ply\\cloud_label_55_oriented.xyz";
//
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud1(new pcl::PointCloud<pcl::PointNormal>);
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointNormal>);
//
//    if (!loadXYZWithNormals(input_file1, cloud1) || !loadXYZWithNormals(input_file2, cloud2))
//    {
//        return -1;
//    }
//
//    // 创建两个可视化窗口
//    pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("3D Viewer 1"));
//    pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("3D Viewer 2"));
//
//    // 设置第一个可视化窗口
//    viewer1->setBackgroundColor(0, 0, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> color1(cloud1, 255, 0, 0);
//    viewer1->addPointCloud<pcl::PointNormal>(cloud1, color1, "cloud1");
//    viewer1->addPointCloudNormals<pcl::PointNormal>(cloud1, 10, 0.05, "normals1");
//    viewer1->addCoordinateSystem(1.0);
//    viewer1->initCameraParameters();
//
//    // 设置第二个可视化窗口
//    viewer2->setBackgroundColor(0, 0, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> color2(cloud2, 0, 255, 0);
//    viewer2->addPointCloud<pcl::PointNormal>(cloud2, color2, "cloud2");
//    viewer2->addPointCloudNormals<pcl::PointNormal>(cloud2, 10, 0.05, "normals2");
//    viewer2->addCoordinateSystem(1.0);
//    viewer2->initCameraParameters();
//
//    while (!viewer1->wasStopped() || !viewer2->wasStopped())
//    {
//        viewer1->spinOnce(100);
//        viewer2->spinOnce(100);
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    }
//
//    return 0;
//}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//最小二乘上采样及法向量计算++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <vector>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/surface/mls.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/uniform_sampling.h>
//using namespace std;
//
//bool loadXYZFile(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//    std::ifstream file(filename);
//    if (!file.is_open()) {
//        std::cerr << "Failed to open file: " << filename << std::endl;
//        return false;
//    }
//
//    std::string line;
//    while (std::getline(file, line)) {
//        pcl::PointXYZ point;
//        std::istringstream iss(line);
//        iss >> point.x >> point.y >> point.z;
//        cloud->push_back(point);
//    }
//    file.close();
//    return true;
//}
//
//bool saveXYZFile(const std::string& filename, pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
//    std::ofstream file(filename);
//    if (!file.is_open()) {
//        std::cerr << "Failed to open file: " << filename << std::endl;
//        return false;
//    }
//
//    for (const auto& point : cloud->points) {
//        file << point.x << " " << point.y << " " << point.z << " "
//            << point.normal_x << " " << point.normal_y << " " << point.normal_z << std::endl;
//    }
//    file.close();
//    return true;
//}
//
//void getMinMax3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ& min_pt, pcl::PointXYZ& max_pt) {
//    min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
//    max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<float>::max();
//
//    for (const auto& point : cloud->points) {
//        if (point.x < min_pt.x) min_pt.x = point.x;
//        if (point.y < min_pt.y) min_pt.y = point.y;
//        if (point.z < min_pt.z) min_pt.z = point.z;
//        if (point.x > max_pt.x) max_pt.x = point.x;
//        if (point.y > max_pt.y) max_pt.y = point.y;
//        if (point.z > max_pt.z) max_pt.z = point.z;
//    }
//}
//
//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> slicePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ& min_pt, const pcl::PointXYZ& max_pt, char axis) {
//    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> slices(15);
//    for (auto& slice : slices) {
//        slice = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
//    }
//
//    float step = 0.0f;
//    if (axis == 'x') step = (max_pt.x - min_pt.x) / 15;
//    else if (axis == 'y') step = (max_pt.y - min_pt.y) / 15;
//    else if (axis == 'z') step = (max_pt.z - min_pt.z) / 15;
//
//    for (const auto& point : cloud->points) {
//        int slice_index = 0;
//        if (axis == 'x') slice_index = std::min(static_cast<int>((point.x - min_pt.x) / step), 14);
//        else if (axis == 'y') slice_index = std::min(static_cast<int>((point.y - min_pt.y) / step), 14);
//        else if (axis == 'z') slice_index = std::min(static_cast<int>((point.z - min_pt.z) / step), 14);
//
//        slices[slice_index]->points.push_back(point);
//    }
//
//    return slices;
//}
//
//std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> slicePointCloudNormal(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud, const pcl::PointXYZ& min_pt, const pcl::PointXYZ& max_pt, char axis) {
//    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> slices(15);
//    for (auto& slice : slices) {
//        slice = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
//    }
//
//    float step = 0.0f;
//    if (axis == 'x') step = (max_pt.x - min_pt.x) / 15;
//    else if (axis == 'y') step = (max_pt.y - min_pt.y) / 15;
//    else if (axis == 'z') step = (max_pt.z - min_pt.z) / 15;
//
//    for (const auto& point : cloud->points) {
//        int slice_index = 0;
//        if (axis == 'x') slice_index = std::min(static_cast<int>((point.x - min_pt.x) / step), 14);
//        else if (axis == 'y') slice_index = std::min(static_cast<int>((point.y - min_pt.y) / step), 14);
//        else if (axis == 'z') slice_index = std::min(static_cast<int>((point.z - min_pt.z) / step), 14);
//
//        if (slice_index >= 0 && slice_index < slices.size()) {
//            slices[slice_index]->points.push_back(point);
//        }
//    }
//
//    return slices;
//}
//
//pcl::PointCloud<pcl::PointNormal>::Ptr filterSlices(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& ref_slices, const std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& target_slices) {
//    pcl::PointCloud<pcl::PointNormal>::Ptr filtered(new pcl::PointCloud<pcl::PointNormal>);
//
//    for (size_t i = 0; i < ref_slices.size(); ++i) {
//        pcl::PointXYZ ref_min_pt, ref_max_pt;
//        getMinMax3D(ref_slices[i], ref_min_pt, ref_max_pt);
//
//        for (const auto& point : target_slices[i]->points) {
//            if (point.x >= ref_min_pt.x && point.x <= ref_max_pt.x &&
//                point.y >= ref_min_pt.y && point.y <= ref_max_pt.y &&
//                point.z >= ref_min_pt.z && point.z <= ref_max_pt.z) {
//                filtered->points.push_back(point);
//            }
//        }
//    }
//
//    return filtered;
//}
//
//int main(int argc, char** argv) {
//    std::string input_file = "E:\\Pig_Cattle\\Pig\\label\\pointm2ae_test\\3\\55.xyz";
//    std::string output_file = "E:\\Pig_Cattle\\Pig\\label\\pointm2ae_test\\3\\up\\55_filtered.xyz";
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    if (!loadXYZFile(input_file, cloud)) {
//        return -1;
//    }
//    cout << "原始点云个数：" << cloud->points.size() << endl;
//
//    pcl::PointXYZ min_pt, max_pt,min_pt_2, max_pt_2;
//    getMinMax3D(cloud, min_pt, max_pt);
//    getMinMax3D(cloud, min_pt_2, max_pt_2);
//
//    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> up;
//    up.setInputCloud(cloud);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//    up.setSearchMethod(tree);
//    up.setSearchRadius(0.1);
//    up.setComputeNormals(true);
//    up.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
//    up.setUpsamplingRadius(0.08);
//    up.setUpsamplingStepSize(0.002);
//    up.setPolynomialOrder(2);
//    up.setProjectionMethod(pcl::MLSResult::ProjectionMethod::SIMPLE);
//    up.setNumberOfThreads(4);
//    up.setPointDensity(500);
//
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointNormal>);
//    cloud_out->clear();
//    up.process(*cloud_out);
//    cout << "上采样后点云的个数：" << cloud_out->points.size() << endl;
//
//    //auto ref_slices_x = slicePointCloud(cloud, min_pt, max_pt, 'x');
//    //auto target_slices_x = slicePointCloudNormal(cloud_out, min_pt, max_pt, 'x');
//    //cout << "开始x滤波：" << endl;
//
//    //auto filtered_x = filterSlices(ref_slices_x, target_slices_x);
//    //pcl::PointCloud<pcl::PointNormal>::Ptr final_filtered(new pcl::PointCloud<pcl::PointNormal>);
//    //*final_filtered += *filtered_x;
//
//    //auto ref_slices_y = slicePointCloud(cloud, min_pt, max_pt, 'y');
//    //auto target_slices_y = slicePointCloudNormal(final_filtered, min_pt, max_pt, 'y');
//    //cout << "开始y滤波：" << endl;
//    //auto filtered_y = filterSlices(ref_slices_y, target_slices_y);
//    //*final_filtered += *filtered_y;
//
//    //auto ref_slices_z = slicePointCloud(cloud, min_pt, max_pt, 'z');
//    //auto target_slices_z = slicePointCloudNormal(final_filtered, min_pt, max_pt, 'z');
//    //cout << "开始z滤波：" << endl;
//    //auto filtered_z = filterSlices(ref_slices_z, target_slices_z);
//    //*final_filtered += *filtered_z;
//
//    auto ref_slices_x = slicePointCloud(cloud, min_pt, max_pt, 'x');
//    auto ref_slices_y = slicePointCloud(cloud, min_pt, max_pt, 'y');
//    auto ref_slices_z = slicePointCloud(cloud, min_pt, max_pt, 'z');
//
//    auto target_slices_x = slicePointCloudNormal(cloud_out, min_pt, max_pt, 'x');
//    auto target_slices_y = slicePointCloudNormal(cloud_out, min_pt, max_pt, 'y');
//    auto target_slices_z = slicePointCloudNormal(cloud_out, min_pt, max_pt, 'z');
//
//    auto filtered_x = filterSlices(ref_slices_x, target_slices_x);
//    auto filtered_y = filterSlices(ref_slices_y, target_slices_y);
//    auto filtered_z = filterSlices(ref_slices_z, target_slices_z);
//
//    pcl::PointCloud<pcl::PointNormal>::Ptr final_filtered(new pcl::PointCloud<pcl::PointNormal>);
//    *final_filtered += *filtered_x;
//    *final_filtered += *filtered_y;
//    *final_filtered += *filtered_z;
//
//
//    // 创建滤波对象
//    pcl::VoxelGrid<pcl::PointNormal> sor;
//    sor.setInputCloud(final_filtered);
//    sor.setLeafSize(0.008f, 0.008f, 0.008f); // 设置体素网格的大小，可以根据需要调整
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
//    sor.filter(*cloud_filtered);
//    // 创建滤波对象
////// 创建滤波器对象
////    pcl::StatisticalOutlierRemoval<pcl::PointNormal> sa;
////    sa.setInputCloud(cloud_filtered);
////    sa.setMeanK(15); // 设置用于统计分析的临近点数量
////    sa.setStddevMulThresh(0.8); // 设置标准差乘数的阈值
////
//    //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered_sa(new pcl::PointCloud<pcl::PointNormal>);
//    //sor.filter(*cloud_filtered_sa);
////
//    std::cout << "滤波后的点数: " << cloud_filtered->points.size() << std::endl;
//
//    //std::cout << "均匀采样后点云有 " << cloud_filtered->points.size() << " 个点." << std::endl;
//
//    cout << "滤波后点云的个数：" << cloud_filtered->points.size() << endl;
//    if (!saveXYZFile(output_file, cloud_filtered)) {
//        return -1;
//    }
//
//    return 0;
//}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//////边界可视化++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/ModelCoefficients.h>  // RANSAC相关头文件
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/filters/project_inliers.h>  // 投影滤波 
//#include <pcl/surface/concave_hull.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/geometry/planar_polygon.h>  // 定义多边形
//#include <iostream>
//#include <fstream>
//
//bool loadXYZFile(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//    std::ifstream infile(filename);
//    if (!infile.is_open()) {
//        std::cerr << "无法打开文件: " << filename << std::endl;
//        return false;
//    }
//
//    std::string line;
//    while (std::getline(infile, line)) {
//        std::istringstream iss(line);
//        pcl::PointXYZ point;
//        if (!(iss >> point.x >> point.y >> point.z)) {
//            break;  // 出现错误
//        }
//        cloud->points.push_back(point);
//    }
//    cloud->width = cloud->points.size();
//    cloud->height = 1;
//    cloud->is_dense = true;
//
//    infile.close();
//    return true;
//}
//
//bool saveXYZFile(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//    std::ofstream outfile(filename);
//    if (!outfile.is_open()) {
//        std::cerr << "无法打开文件: " << filename << std::endl;
//        return false;
//    }
//
//    for (const auto& point : cloud->points) {
//        outfile << point.x << " " << point.y << " " << point.z << std::endl;
//    }
//
//    outfile.close();
//    return true;
//}
//
//double calculatePolygonArea(const pcl::PointCloud<pcl::PointXYZ>& points) {
//    double area = 0.0;
//    size_t n = points.size();
//    for (size_t i = 0; i < n; ++i) {
//        const pcl::PointXYZ& p1 = points[i];
//        const pcl::PointXYZ& p2 = points[(i + 1) % n];
//        area += (p1.x * p2.y - p2.x * p1.y);
//    }
//    return std::abs(area) / 2.0;
//}
//
//int main(int argc, char** argv) {
//    //--------------------------加载点云数据----------------------------
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    if (!loadXYZFile("E:\\Pig_Cattle\\Pig\\label\\pointm2ae_test\\12\\up\\55.xyz", cloud)) {
//        return -1;
//    }
//    std::cerr << "原始点云点的个数: " << cloud->points.size() << std::endl;
//
//    //-------------------------RANSAC拟合平面---------------------------
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    seg.setOptimizeCoefficients(true);
//    seg.setModelType(pcl::SACMODEL_PLANE);
//    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setDistanceThreshold(0.01);
//    seg.setInputCloud(cloud);
//    seg.segment(*inliers, *coefficients);
//
//    //-----------------------点云投影到平面----------------------------
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::ProjectInliers<pcl::PointXYZ> proj;
//    proj.setModelType(pcl::SACMODEL_PLANE);
//    proj.setInputCloud(cloud);
//    proj.setModelCoefficients(coefficients);
//    proj.filter(*cloud_projected);
//    std::cerr << "投影后点的个数: " << cloud_projected->points.size() << std::endl;
//
//    //if (!saveXYZFile("投影点云.xyz", cloud_projected)) {
//    //    return -1;
//    //}
//
//    //---------------提取投影平面点云的凹多边形边界-------------------
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::ConcaveHull<pcl::PointXYZ> chull;
//    chull.setInputCloud(cloud_projected);
//    chull.setAlpha(0.1);
//    chull.reconstruct(*cloud_hull);
//    std::cerr << "凹多边形的点数: " << cloud_hull->points.size() << std::endl;
//
//    //if (!saveXYZFile("凹多边形.xyz", cloud_hull)) {
//    //    return -1;
//    //}
//
//    // 增加多边形
//    pcl::PlanarPolygon<pcl::PointXYZ> polygon;
//    pcl::PointCloud<pcl::PointXYZ> contour;
//    contour.width = cloud_hull->width;
//    contour.height = 1;
//    contour.is_dense = false;
//    contour.resize(contour.height * contour.width);
//
//    for (size_t i = 0; i < cloud_hull->points.size(); ++i) {
//        contour.points[i] = cloud_hull->points[i];
//    }
//
//    polygon.setContour(contour);
//    double result = calculatePolygonArea(contour);
//    std::cout << "多边形面积为：" << result << std::endl;
//
//    pcl::visualization::PCLVisualizer viewer("Viewer");
//    viewer.addPolygon(polygon, 255, 0, 0, "polygon", 0);
//    viewer.addPointCloud<pcl::PointXYZ>(cloud_hull, "sample cloud");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
//
//    while (!viewer.wasStopped()) {
//        viewer.spinOnce(1);
//    }
//
//    return 0;
//}
///边界点索引+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/surface/concave_hull.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/search/kdtree.h>
//#include <iostream>
//#include <fstream>
//#include <vector>
//
//using namespace pcl;
//using namespace std;
//
//bool loadXYZFile(const std::string& filename, PointCloud<PointXYZ>::Ptr cloud) {
//    std::ifstream infile(filename);
//    if (!infile.is_open()) {
//        std::cerr << "Failed to open file " << filename << std::endl;
//        return false;
//    }
//
//    std::string line;
//    while (std::getline(infile, line)) {
//        std::istringstream iss(line);
//        PointXYZ point;
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
//std::vector<int> detectHoleBoundary(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ>::Ptr cloud_hull, float alpha) {
//    std::vector<int> boundary_indices;
//
//    // Step 1: Create the concave hull (Alpha Shape)
//    ConcaveHull<PointXYZ> concave_hull;
//    concave_hull.setInputCloud(cloud);
//    concave_hull.setAlpha(alpha);
//    concave_hull.reconstruct(*cloud_hull);
//
//    // Print the concave hull points
//    std::cout << "Concave hull points:" << std::endl;
//    for (const auto& point : cloud_hull->points) {
//        std::cout << point.x << " " << point.y << " " << point.z << std::endl;
//    }
//
//    // Step 2: Extract boundary points from the concave hull
//    std::set<int> boundary_set;
//    for (const auto& point : cloud_hull->points) {
//        for (size_t i = 0; i < cloud->points.size(); ++i) {
//            if (cloud->points[i].x == point.x &&
//                cloud->points[i].y == point.y &&
//                cloud->points[i].z == point.z) {
//                boundary_set.insert(i);
//            }
//        }
//    }
//    boundary_indices.assign(boundary_set.begin(), boundary_set.end());
//
//    return boundary_indices;
//}
//
//void visualizePointCloudWithBoundary(PointCloud<PointXYZ>::Ptr cloud, const PointCloud<PointXYZ>::Ptr cloud_hull, const std::vector<int>& boundary_indices) {
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//    viewer->setBackgroundColor(0, 0, 0);
//    viewer->addPointCloud<PointXYZ>(cloud, "sample cloud");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//
//    // Highlight boundary points in red
//    PointCloud<PointXYZ>::Ptr boundary_cloud(new PointCloud<PointXYZ>);
//    for (size_t i = 0; i < boundary_indices.size(); ++i) {
//        boundary_cloud->points.push_back(cloud->points[boundary_indices[i]]);
//    }
//    pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> boundary_color(boundary_cloud, 255, 0, 0);
//    viewer->addPointCloud<PointXYZ>(boundary_cloud, boundary_color, "boundary points");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "boundary points");
//
//    // Visualize the concave hull points in blue
//    pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> hull_color(cloud_hull, 0, 0, 255);
//    viewer->addPointCloud<PointXYZ>(cloud_hull, hull_color, "concave hull");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "concave hull");
//
//    // Print boundary indices to console
//    std::cout << "Boundary indices:" << std::endl;
//    for (size_t i = 0; i < boundary_indices.size(); ++i) {
//        std::cout << boundary_indices[i] << " ";
//    }
//    std::cout << std::endl;
//
//    // Main loop
//    while (!viewer->wasStopped()) {
//        viewer->spinOnce(100);
//        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    }
//}
//
//int main(int argc, char** argv) {
//    //if (argc < 2) {
//    //    std::cerr << "Usage: " << argv[0] << " <input.xyz>" << std::endl;
//    //    return (-1);
//    //}
//
//    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
//    string input_file = "E:\\Pig_Cattle\\Pig\\label\\pointm2ae_test\\8\\up\\55.xyz";
//    // Load point cloud data from XYZ file
//    if (!loadXYZFile(input_file, cloud)) {
//        return (-1);
//    }
//
//    // Set the alpha value for the concave hull
//    float alpha = 0.8f; // Adjust this value as needed
//
//    // Create a point cloud to store the concave hull
//    PointCloud<PointXYZ>::Ptr cloud_hull(new PointCloud<PointXYZ>);
//
//    // Detect hole boundary points
//    std::vector<int> boundary_indices = detectHoleBoundary(cloud, cloud_hull, alpha);
//
//    // Visualize point cloud with boundary points and concave hull highlighted
//    visualizePointCloudWithBoundary(cloud, cloud_hull, boundary_indices);
//
//    return 0;
//}

////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

////点云切块操作
//#include <iostream>
//#include <fstream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/crop_box.h>
//#include <pcl/common/common.h>
//
//typedef pcl::PointXYZ PointT;
//
//pcl::PointCloud<PointT>::Ptr readXYZ(const std::string& filename) {
//    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
//    std::ifstream infile(filename);
//    float x, y, z;
//    while (infile >> x >> y >> z) {
//        cloud->points.emplace_back(x, y, z);
//    }
//    cloud->width = cloud->points.size();
//    cloud->height = 1;
//    cloud->is_dense = true;
//    return cloud;
//}
//
//void saveXYZ(const pcl::PointCloud<PointT>::Ptr& cloud, const std::string& filename) {
//    std::ofstream outfile(filename);
//    for (const auto& point : cloud->points) {
//        outfile << point.x << " " << point.y << " " << point.z << std::endl;
//    }
//}
//
//int main(int argc, char** argv) {
//    //if (argc != 4) {
//    //    std::cerr << "Usage: " << argv[0] << " <input_xyz_a> <input_pcd_b> <output_xyz>" << std::endl;
//    //    return -1;
//    //}
//
//    std::string input_xyz_a = "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\10\\59.xyz";
//    std::string input_pcd_b = "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\10\\cut\\10.pcd";
//    std::string output_xyz = "E:\\Pig_Cattle\\Pig\\label\\after_label_two\\10\\cut\\59.xyz";
//
//    // 读取点云a (xyz格式)
//    pcl::PointCloud<PointT>::Ptr cloud_a = readXYZ(input_xyz_a);
//
//    // 读取点云b (pcd格式)
//    pcl::PointCloud<PointT>::Ptr cloud_b(new pcl::PointCloud<PointT>);
//    if (pcl::io::loadPCDFile(input_pcd_b, *cloud_b) == -1) {
//        PCL_ERROR("Couldn't read file %s \n", input_pcd_b.c_str());
//        return (-1);
//    }
//
//    // 计算点云a的最小外接长方体
//    PointT min_pt, max_pt;
//    pcl::getMinMax3D(*cloud_a, min_pt, max_pt);
//
//    // 创建CropBox对象
//    pcl::CropBox<PointT> crop_box;
//    crop_box.setMin(Eigen::Vector4f(min_pt.x, min_pt.y, min_pt.z, 1.0));
//    crop_box.setMax(Eigen::Vector4f(max_pt.x, max_pt.y, max_pt.z, 1.0));
//    crop_box.setInputCloud(cloud_b);
//
//    pcl::PointCloud<PointT>::Ptr cloud_cropped(new pcl::PointCloud<PointT>);
//    crop_box.filter(*cloud_cropped);
//
//    // 保存裁剪后的点云为新的xyz文件
//    saveXYZ(cloud_cropped, output_xyz);
//    std::cout << "finish" <<std::endl ;
//    return 0;
//}

