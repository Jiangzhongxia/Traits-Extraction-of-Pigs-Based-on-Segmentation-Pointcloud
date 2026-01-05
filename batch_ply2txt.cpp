//#include <pcl/io/obj_io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/surface/poisson.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/filters/uniform_sampling.h>
//#include <pcl/filters/farthest_point_sampling.h>
//#include <pcl/common/distances.h>
//#include <boost/filesystem.hpp>
//#include <iostream>
//
//
//// Function to sample and save point cloud
//void sampleAndSavePointCloud(const std::string& input_file, const std::string& output_file, int num_samples) {
//    // Load OBJ file
//    pcl::PolygonMesh mesh;
//    if (pcl::io::loadOBJFile(input_file, mesh) == -1) {
//        PCL_ERROR("Couldn't read the OBJ file\n");
//        return;
//    }
//
//    // Convert the polygon mesh to a point cloud
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
//
//    // -------------------------------最远点采样---------------------------------
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);
//    std::random_device rd;
//    int random_seed = rd();   // 随机选取一个点作为种子点
//    int sampleNum = 2500;    // 采样点个数
//    pcl::FarthestPointSampling<pcl::PointXYZ> fps;
//    fps.setInputCloud(cloud); // 读取点云
//    fps.setSeed(random_seed); // 设置第一个种子点
//    fps.setSample(sampleNum); // 设置采样点个数
//    fps.filter(*cloud_sampled);    // 进行采样
//
//    // Save to PCD file
//    pcl::io::savePCDFileASCII(output_file, *cloud_sampled);
//}
//
//// Function to process directory of OBJ files
//void processDirectory(const std::string& input_dir, const std::string& output_dir, float leaf_size) {
//    boost::filesystem::path dir(input_dir);
//    if (!boost::filesystem::exists(dir) || !boost::filesystem::is_directory(dir)) {
//        std::cerr << "Invalid input directory: " << input_dir << std::endl;
//        return;
//    }
//
//    for (boost::filesystem::directory_iterator itr(dir); itr != boost::filesystem::directory_iterator(); ++itr) {
//        if (boost::filesystem::is_regular_file(itr->path()) && itr->path().extension() == ".obj") {
//            std::string input_file = itr->path().string();
//            std::string output_file = (boost::filesystem::path(output_dir) / itr->path().stem()).string() + ".pcd";
//            sampleAndSavePointCloud(input_file, output_file, leaf_size);
//        }
//    }
//}
//
//int main(int argc, char** argv) {
//    //if (argc != 4) {
//    //    std::cerr << "Usage: " << argv[0] << " <input_directory> <output_directory> <leaf_size>" << std::endl;
//    //    return -1;
//    //}
//
//    std::string input_dir = "H:\\PIG\\output";
//    std::string output_dir = "H:\\PIG\\pcd";
//    float leaf_size = 2500;
//
//    processDirectory(input_dir, output_dir, leaf_size);
//
//    return 0;
//}

//#include <iostream>
//#define NOMINMAX
//#include <Windows.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <fstream>
//
//// 将 PCD 文件转换为 TXT 文件的函数
//void convertPCDToTXT(const std::string& input_file, const std::string& output_file) {
//    pcl::PointCloud<pcl::PointXYZ> cloud;
//
//    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, cloud) == -1) {
//        PCL_ERROR("Couldn't read file %s \n", input_file.c_str());
//        return;
//    }
//
//    std::ofstream ofs(output_file);
//    if (!ofs.is_open()) {
//        std::cerr << "Error opening file: " << output_file << std::endl;
//        return;
//    }
//
//    for (const auto& point : cloud.points) {
//        ofs << point.x << " " << point.y << " " << point.z << "\n";
//    }
//
//    ofs.close();
//    std::cout << "Converted " << input_file << " to " << output_file << std::endl;
//}
//
//// 遍历文件夹并处理所有 PCD 文件
//void processFolder(const std::string& source_folder, const std::string& destination_folder) {
//    WIN32_FIND_DATA findFileData;
//    HANDLE hFind = FindFirstFile((source_folder + "\\*.pcd").c_str(), &findFileData);
//
//    if (hFind == INVALID_HANDLE_VALUE) {
//        std::cerr << "No PCD files found in the folder: " << source_folder << std::endl;
//        return;
//    }
//
//    int file_counter = 1;
//    do {
//        std::string pcd_file = source_folder + "\\" + findFileData.cFileName;
//        std::string txt_file = destination_folder + "\\cloud_" + std::to_string(file_counter++) + ".txt";
//
//        convertPCDToTXT(pcd_file, txt_file);
//
//    } while (FindNextFile(hFind, &findFileData) != 0);
//
//    FindClose(hFind);
//}
//
//int main() {
//    std::string source_folder = "H:\\PIG\\pcd\\";  // 输入你的 PCD 文件夹路径
//    std::string destination_folder = "H:\\PIG\\txt\\";  // 输出 TXT 文件夹路径
//
//    // 遍历文件夹并将 .pcd 文件转换为 .txt
//    processFolder(source_folder, destination_folder);
//
//    return 0;
//}
