//// C++ 标准库
//#include <iostream>
//#include <string>
//#include <vector>
//#include <fstream>
//#include <cstdint> // 用于固定大小的类型，如 uint16_t
//using namespace std;
//
//// PCL 库
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
//
//// stb_image 库
//#define STB_IMAGE_IMPLEMENTATION
//#include "stb_image.h"
//
//// 定义点云类型
//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointCloud<PointT> PointCloud;
//
//// 相机内参
//const double camera_factor = 1000;
//const double camera_cx = 327.1;
//const double camera_cy = 323.0;
//const double camera_fx = 504.4;
//const double camera_fy = 504.6;
//
//// 读取深度图像 (16位单通道)
//vector<uint16_t> readDepthImage(const std::string& depth_file, int& width, int& height) {
//    ifstream file(depth_file, ios::binary);
//    if (!file.is_open()) {
//        cerr << "Cannot open depth file: " << depth_file << endl;
//        return {};
//    }
//
//    // 假设深度图是PNG文件读取后的16位深度数据
//    file.seekg(0, ios::end);
//    size_t file_size = file.tellg();
//    file.seekg(0, ios::beg);
//
//    vector<uint16_t> depth_data(file_size / sizeof(uint16_t));
//    file.read(reinterpret_cast<char*>(depth_data.data()), file_size);
//
//    // 这里假设深度图像的大小你已知, 设置为固定大小
//    width = 1280;
//    height = 800;
//    file.close();
//
//    return depth_data;
//}
//
//int main(int argc, char** argv) {
//    // 图像矩阵: RGB 和 深度图像
//    int rgb_width, rgb_height, depth_width, depth_height;
//    int channels;
//
//    // 使用 stb_image 读取 RGB 图像
//    unsigned char* rgb_data = stbi_load("H:/点云公共数据集/罗涛数据/data/5.28_afternoon_15/sn_CP3L44P0002D/Color/00333870-1.png", &rgb_width, &rgb_height, &channels, 3);
//    if (!rgb_data) {
//        cerr << "Failed to load RGB image!" << endl;
//        return -1;
//    }
//
//    // 使用自定义函数读取 16 位深度图像
//    vector<uint16_t> depth_data = readDepthImage("H:/点云公共数据集/罗涛数据/data/5.28_afternoon_15/sn_CP3L44P0002D/Depth/00333870-1.png", depth_width, depth_height);
//    if (depth_data.empty()) {
//        cerr << "Failed to load depth image!" << endl;
//        stbi_image_free(rgb_data);
//        return -1;
//    }
//
//    // 确保 RGB 图像和深度图像尺寸一致
//    if (rgb_width != depth_width || rgb_height != depth_height) {
//        cerr << "RGB and depth image sizes do not match!" << endl;
//        stbi_image_free(rgb_data);
//        return -1;
//    }
//
//    // 点云变量
//    PointCloud::Ptr cloud(new PointCloud);
//
//    // 遍历深度图
//    for (int m = 0; m < depth_height; m++) {
//        for (int n = 0; n < depth_width; n++) {
//            // 获取深度图中(m,n)处的值
//            uint16_t d = depth_data[m * depth_width + n];
//            if (d == 0) continue;  // 若深度为0，跳过此点
//
//            // 创建一个点
//            PointT p;
//            p.z = double(d) / camera_factor;
//            p.x = (n - camera_cx) * p.z / camera_fx;
//            p.y = (m - camera_cy) * p.z / camera_fy;
//
//            // 获取RGB颜色
//            p.b = rgb_data[(m * rgb_width + n) * 3];
//            p.g = rgb_data[(m * rgb_width + n) * 3 + 1];
//            p.r = rgb_data[(m * rgb_width + n) * 3 + 2];
//
//            // 将点加入点云
//            cloud->points.push_back(p);
//        }
//    }
//
//    // 设置并保存点云
//    cloud->height = 1;
//    cloud->width = cloud->points.size();
//    cloud->is_dense = false;
//    cout << "Point cloud size = " << cloud->points.size() << endl;
//    pcl::io::savePCDFile("H:/点云公共数据集/罗涛数据/data/5.28_afternoon_15/sn_CP3L44P0002D/pointclouds/00333870-1.pcd", *cloud);
//
//    // 显示点云
//    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//    viewer.showCloud(cloud);
//    while (!viewer.wasStopped()) {}
//
//    // 清理资源
//    stbi_image_free(rgb_data);
//    cloud->points.clear();
//    cout << "Point cloud saved." << endl;
//    return 0;
//}
