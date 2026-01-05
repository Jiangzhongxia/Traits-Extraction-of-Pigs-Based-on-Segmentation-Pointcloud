//#include <iostream>
//#include <vector>
//#include <fstream>
//#include <sstream>  // 添加此行
//#include <string>
//#include <windows.h>
//
//using namespace std;
//
//// 获取文件夹中所有 .xyz 文件名
//vector<string> getAllXyzFiles(const string& folderPath) {
//    vector<string> xyzFiles;
//    WIN32_FIND_DATAA findFileData;
//    HANDLE hFind = FindFirstFileA((folderPath + "\\*.xyz").c_str(), &findFileData);
//
//    if (hFind == INVALID_HANDLE_VALUE) {
//        cerr << "无法找到路径: " << folderPath << endl;
//        return xyzFiles;
//    }
//
//    do {
//        if (!(findFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
//            xyzFiles.emplace_back(findFileData.cFileName);
//        }
//    } while (FindNextFileA(hFind, &findFileData) != 0);
//
//    FindClose(hFind);
//    return xyzFiles;
//}
//
//// 处理点云文件，仅提取 xyz 坐标
//void processPointCloudFile(const string& inputFilePath, const string& outputFilePath) {
//    ifstream inputFile(inputFilePath);
//    ofstream outputFile(outputFilePath);
//
//    if (!inputFile.is_open()) {
//        cerr << "无法打开文件: " << inputFilePath << endl;
//        return;
//    }
//    if (!outputFile.is_open()) {
//        cerr << "无法创建输出文件: " << outputFilePath << endl;
//        return;
//    }
//
//    string line;
//    while (getline(inputFile, line)) {
//        // 使用 stringstream 解析每一行
//        stringstream ss(line);
//        float x, y, z;
//        ss >> x >> y >> z;
//        if (ss.fail()) {
//            // 如果解析失败，跳过该行
//            continue;
//        }
//        // 写入提取的 xyz 坐标
//        outputFile << x << " " << y << " " << z << endl;
//    }
//
//    inputFile.close();
//    outputFile.close();
//    cout << "处理完成: " << outputFilePath << endl;
//}
//
//// 遍历文件夹内的所有 .xyz 文件并处理
//void processAllFiles(const string& folderPath) {
//    vector<string> xyzFiles = getAllXyzFiles(folderPath);
//
//    if (xyzFiles.empty()) {
//        cerr << "未找到任何 .xyz 文件。" << endl;
//        return;
//    }
//
//    for (const auto& fileName : xyzFiles) {
//        string inputFilePath = folderPath + "\\" + fileName;
//        string outputFilePath = folderPath + "\\processed_" + fileName;
//
//        cout << "正在处理文件: " << inputFilePath << endl;
//        processPointCloudFile(inputFilePath, outputFilePath);
//    }
//}
//
//int main() {
//    string folderPath = "E:\\pig1-150\\1\\cut\\"; // 请修改为你的文件夹路径
//    processAllFiles(folderPath);
//    return 0;
//}
//// C++头文件
//#include <cmath>
//#include <random>
// PCL头文件
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/filters/farthest_point_sampling.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
//#include <iostream>
//#include <string>
//#include <windows.h>
//
//int main(int argc, char** argv)
//{
//    // 输入文件夹路径和输出文件夹路径，这里你可以根据实际情况修改
//    std::string input_folder_path = "E:/pig1-150/xyz150/train_sample";
//    std::string output_folder_path = "E:/pig1-150/xyz150/train_sample_output";
//
//    // 确保输出文件夹存在，如果不存在则创建
//    {
//        DWORD fileAttributes = GetFileAttributes(output_folder_path.c_str());
//        if (fileAttributes == INVALID_FILE_ATTRIBUTES || !(fileAttributes & FILE_ATTRIBUTE_DIRECTORY))
//        {
//            // 使用系统命令创建文件夹，这里使用了Windows下的mkdir命令创建，同样不是很优雅但简单有效，也可以用更复杂的创建逻辑替代
//            std::string create_cmd = "mkdir " + output_folder_path;
//            system(create_cmd.c_str());
//        }
//    }
//
//    // 查找输入文件夹下的所有.pcd文件
//    WIN32_FIND_DATA findFileData;
//    HANDLE hFind = FindFirstFile((input_folder_path + "/*.pcd").c_str(), &findFileData);
//    if (hFind == INVALID_HANDLE_VALUE)
//    {
//        std::cerr << "无法打开输入文件夹或找不到.pcd文件: " << input_folder_path << std::endl;
//        return -1;
//    }
//
//    do
//    {
//        std::string file_name = findFileData.cFileName;
//        // 构建完整的文件路径
//        std::string file_path = input_folder_path + "/" + file_name;
//
//        // ------------------------------读取点云数据---------------------------------
//        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
//        if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(file_path, *cloud) < 0)
//        {
//            PCL_ERROR("Could not read file: %s\n", file_path.c_str());
//            continue; // 如果读取失败，跳过当前文件，继续处理下一个文件
//        }
//        pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZINormal>);
//
//        // -------------------------------最远点采样---------------------------------
//        std::random_device rd;
//        int random_seed = rd();   // 随机选取一个点作为种子点
//        int sampleNum = 2048;    // 采样点个数
//        pcl::FarthestPointSampling<pcl::PointXYZINormal> fps;
//        fps.setInputCloud(cloud); // 读取点云
//        fps.setSeed(random_seed); // 设置第一个种子点
//        fps.setSample(sampleNum); // 设置采样点个数
//        fps.filter(*filtered);    // 进行采样
//
//        // 提取文件名（不包含路径和扩展名），用于构建输出文件名
//        std::string filename = file_name.substr(0, file_name.length() - 4);
//        // 构建输出文件完整路径
//        std::string output_file_path = output_folder_path + "/" + filename + "_sampled.pcd";
//
//        // -------------------------------结果保存（ASCII格式）-----------------------------------
//        pcl::io::savePCDFileASCII(output_file_path, *filtered);
//        std::cout << "Saved sampled point cloud to: " << output_file_path << std::endl;
//    } while (FindNextFile(hFind, &findFileData));
//
//    FindClose(hFind);
//    return 0;
//}















