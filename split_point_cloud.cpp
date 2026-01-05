//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <vector>
//#include <unordered_map>
//
//
//#include <map>
//#include <string>
//#include <sys/stat.h>
//
//#ifdef _WIN32
//#include <direct.h>
//#define mkdir _mkdir
//#else
//#include <unistd.h>
//#endif
//
//
//// 检查并创建目录的函数
//void createDirectory(const std::string& path) {
//    // 检查目录是否存在，不存在则创建
//    struct stat info;
//    if (stat(path.c_str(), &info) != 0) {
//        std::cerr << "Creating directory: " << path << std::endl;
//        if (mkdir(path.c_str()) != 0) {
//            std::cerr << "Error creating directory: " << path << std::endl;
//        }
//    }
//    else if (info.st_mode & S_IFDIR) {
//        std::cerr << "Directory already exists: " << path << std::endl;
//    }
//}
//
//// 定义PointXYZL类型，其中包含x, y, z和label
//struct PointXYZL {
//    float x, y, z;
//    int label;
//};
//
//int main(int argc, char** argv) {
//    std::string filename = "E:\\pig1-150\\xyz150\\train——data\\150.txt";  // 替换为你的文件路径
//    std::string base_dir = "E:\\pig1-150\\xyz150\\xyz150\\150\\";
//    std::ifstream infile(filename);
//
//    if (!infile.is_open()) {
//        std::cerr << "Could not open file: " << filename << std::endl;
//        return -1;
//    }
//
//    std::unordered_map<int, std::vector<PointXYZL>> cloud_map;
//    std::string line;
//
//    // 读取每一行数据
//    while (std::getline(infile, line)) {
//        std::istringstream iss(line);
//        PointXYZL point;
//        if (!(iss >> point.x >> point.y >> point.z >> point.label)) {
//            std::cerr << "Error parsing line: " << line << std::endl;
//            continue; // 处理错误行
//        }
//
//        // 将点添加到对应的类别中
//        cloud_map[point.label].push_back(point);
//    }
//
//    infile.close();
//
//    // 保存每个类别的点云到不同的XYZ文件中
//    for (const auto& pair : cloud_map) {
//        // 构建输出文件名
//        std::string output_dir = base_dir;
//        std::string output_filename = output_dir + std::to_string(pair.first) + ".xyz";
//        // 检查并创建目录（如果不存在）
//        createDirectory(base_dir);
//
//        std::ofstream outfile(output_filename);
//        if (!outfile.is_open()) {
//            std::cerr << "Could not open file for writing: " << output_filename << std::endl;
//            continue;
//        }
//
//        for (const auto& point : pair.second) {
//            outfile << point.x << " " << point.y << " " << point.z << "\n"; // 只保存xyz坐标
//        }
//
//        outfile.close();
//        std::cout << "Saved " << pair.second.size() << " points to " << output_filename << std::endl;
//    }
//
//    return 0;
//}
