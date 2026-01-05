//#include <iostream>
//#include <vector>
//#include <fstream>
//#include <string>
//#define NOMINMAX
//#include <windows.h>
//#include <algorithm>   
//#include <pcl/io/ply_io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/common/time.h>
//#include <pcl/filters/radius_outlier_removal.h>//半径滤波器
//#include <pcl/filters/farthest_point_sampling.h> 
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
//using namespace std;
//
//// 函数用于读取文件夹中的所有PLY文件名，并将文件名连同文件根目录保存在容器中
//std::vector<std::wstring> getFilesInDirectory(const std::wstring& directory) {
//    std::vector<std::wstring> files;
//    WIN32_FIND_DATAW findFileData;
//    std::wstring searchPath = directory + L"\\*.ply";
//    HANDLE hFind = FindFirstFileW(searchPath.c_str(), &findFileData);
//    if (hFind != INVALID_HANDLE_VALUE) {
//        do {
//            if (!(findFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
//                // 如果不是目录，则将PLY文件名连同文件根目录保存到容器中
//                files.push_back(directory + L"\\" + findFileData.cFileName);
//            }
//        } while (FindNextFileW(hFind, &findFileData) != 0);
//        FindClose(hFind);
//    }
//    return files;
//}
//
//// 函数用于读取文件夹中的所有Pcd文件名，并将文件名连同文件根目录保存在容器中
//std::vector<std::wstring> getFilesInDirectory_pcd(const std::wstring& directory) {
//    std::vector<std::wstring> files;
//    WIN32_FIND_DATAW findFileData;
//    std::wstring searchPath = directory + L"\\*.pcd";
//    HANDLE hFind = FindFirstFileW(searchPath.c_str(), &findFileData);
//    if (hFind != INVALID_HANDLE_VALUE) {
//        do {
//            if (!(findFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
//                // 如果不是目录，则将PLY文件名连同文件根目录保存到容器中
//                files.push_back(directory + L"\\" + findFileData.cFileName);
//            }
//        } while (FindNextFileW(hFind, &findFileData) != 0);
//        FindClose(hFind);
//    }
//    return files;
//}
//
//// 函数用于将PLY文件格式转换为PCD格式
//void convertPlyToPcd(const std::wstring& plyFilename, const std::wstring& outputPath) {
//    // 将wstring类型转换为string类型
//    std::string plyFilenameStr(plyFilename.begin(), plyFilename.end());
//    // 加载PLY文件
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    if (pcl::io::loadPLYFile<pcl::PointXYZ>(plyFilenameStr, *cloud) == -1) {
//        PCL_ERROR("Couldn't read file %ls\n", plyFilename.c_str());
//        return;
//    }
//    // 提取文件名（不包括根目录）和路径
//    size_t lastSlashIndex = plyFilename.find_last_of(L"\\");
//    std::wstring filename = plyFilename.substr(lastSlashIndex + 1);
//    size_t lastDotIndex = filename.find_last_of(L".");
//    std::wstring pcdFilename = outputPath + L"\\" + filename.substr(0, lastDotIndex) + L".pcd";
//
//    std::string pcdFilenameStr(pcdFilename.begin(), pcdFilename.end());
//    // 保存为PCD文件
//    pcl::io::savePCDFileASCII(pcdFilenameStr, *cloud);
//    std::wcout << L"Saved " << pcdFilename << std::endl;
//}
//
//// 函数用于遍历容器中的每个PLY文件并调用convertPlyToPcd函数进行格式转换
//void convertPlyFilesToPcd(const std::vector<std::wstring>& plyFiles, const std::wstring& outputPath) {
//    for (const auto& plyFile : plyFiles) {
//        convertPlyToPcd(plyFile, outputPath);
//    }
//}
//// 函数用于遍历容器中的每个pcd文件并使用直通滤波进行处理
//void allpcd_Passthough_filter(const std::vector<std::wstring>& pcdFiles, const std::wstring& outputPath) {
//
//    for (const auto& pcdFile : pcdFiles) {
//        // 读取点云数据
//        // 将wstring类型转换为string类型
//        std::string pcdFilenameStr(pcdFile.begin(), pcdFile.end());
//        // 加载点云
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//        cout << "->正在读入点云..." << endl;
//        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFilenameStr, *cloud) == -1) {
//            PCL_ERROR("Couldn't read file %ls\n", pcdFile.c_str());
//        }
//        // 提取文件名（不包括根目录）和路径
//        size_t lastSlashIndex = pcdFile.find_last_of(L"\\");
//        std::wstring filename = pcdFile.substr(lastSlashIndex + 1);
//        size_t lastDotIndex = filename.find_last_of(L".");
//        std::wstring pcdFilename_save = outputPath + L"\\" + filename.substr(0, lastDotIndex) + L".pcd";
//        std::string pcdFilenameStr_save(pcdFilename_save.begin(), pcdFilename_save.end());
//        // pcl::io::loadPCDFile<pcl::PointXYZ>("F:\\Pig_Cattle\\cattle\\1_cattle.pcd", *cloud);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2(new pcl::PointCloud<pcl::PointXYZ>);//滤波后点云
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_3(new pcl::PointCloud<pcl::PointXYZ>);
//        ///直通滤波
//        // 创建滤波器对象	
//        // 创建滤波器对象，第一次对Z卡阈值
//        pcl::PassThrough<pcl::PointXYZ> pass;
//        pass.setInputCloud(cloud);
//        pass.setFilterFieldName("z"); //通过z滤波
//        //2
//        pass.setFilterLimits(1.5, 3);
//        //1
//        //pass.setFilterLimits(1.5, 3.14);
//        //pass.setNegative(true);//设置为true，则输出范围外的点
//        pass.filter(*cloud_filtered);
//        cout << "finish z" << endl;
//        // 第二次对X卡阈值滤波
//        pass.setInputCloud(cloud_filtered);
//        pass.setFilterFieldName("x");
//        //2
//        pass.setFilterLimits(-2.21, 2.21);
//        //1
//        //pass.setFilterLimits(-2, 2.2);
//        //pass.setNegative(true);
//        pass.filter(*cloud_filtered_2);
//        cout << "finish x" << endl;
//        // 第三次对Y卡阈值滤波
//        pass.setInputCloud(cloud_filtered_2);
//        pass.setFilterFieldName("y");
//        //2
//        pass.setFilterLimits(-1.57, 1.78);
//        //1
//        //pass.setFilterLimits(-1.6, 1.7);
//        pass.setNegative(false);
//        pass.filter(*cloud_filtered_3);
//        cout << "finish y" << endl;
//
//        ///保存下采样点云
//        pcl::PCDWriter writer;
//        pcl::io::savePCDFileASCII<pcl::PointXYZ>(pcdFilenameStr_save, *cloud_filtered_3);
//        std::wcout << L"Saved " << pcdFilename_save << std::endl;
//    }
//}
//
//// 函数用于遍历容器中的每个pcd文件并使用统计滤波进行处理
//void allpcd_Statistical_filter(const std::vector<std::wstring>& pcdFiles, const std::wstring& outputPath) {
//    for (const auto& pcdFile : pcdFiles) {
//        // 读取点云数据
//        // 将wstring类型转换为string类型
//        std::string pcdFilenameStr(pcdFile.begin(), pcdFile.end());
//        // 加载点云
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//        cout << "->正在读入点云..." << endl;
//        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFilenameStr, *cloud) == -1) {
//            PCL_ERROR("Couldn't read file %ls\n", pcdFile.c_str());
//        }
//        cout << "Cloud before filtering:\n " << *cloud << endl;
//        // 提取文件名（不包括根目录）和路径
//        size_t lastSlashIndex = pcdFile.find_last_of(L"\\");
//        std::wstring filename = pcdFile.substr(lastSlashIndex + 1);
//        size_t lastDotIndex = filename.find_last_of(L".");
//        std::wstring pcdFilename_save = outputPath + L"\\" + filename.substr(0, lastDotIndex) + L".pcd";
//        std::string pcdFilenameStr_save(pcdFilename_save.begin(), pcdFilename_save.end());
//         // -----------------统计滤波-------------------
//         // 创建滤波器，对每个点分析的临近点的个数设置为230 ，并将标准差的倍数设置为0.8 这意味着如果一
//         // 个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//        sor.setInputCloud(cloud);   //设置待滤波的点云
//        sor.setMeanK(50);           //设置在进行统计时考虑查询点邻近点数
//        sor.setStddevMulThresh(1.2);  //设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
//            //即：当判断点的k近邻平均距离(mean distance)大于全局的1倍标准差+平均距离(global distances mean and standard)，则为离群点。
//         sor.filter(*cloud_filtered); //存储内点
//         cout << "Cloud after filtering: \n" << *cloud_filtered << endl;
//        ///保存下采样点云
//        pcl::PCDWriter writer;
//        pcl::io::savePCDFileASCII<pcl::PointXYZ>(pcdFilenameStr_save, *cloud_filtered);
//        std::wcout << L"Saved " << pcdFilename_save << std::endl;
//    }
//}
//
//// 函数用于遍历容器中的每个pcd文件并使用快速采样滤波进行处理
//void allpcd_uniform_filter(const std::vector<std::wstring>& pcdFiles, const std::wstring& outputPath) {
//    for (const auto& pcdFile : pcdFiles) {
//        // 读取点云数据
//        // 将wstring类型转换为string类型
//        std::string pcdFilenameStr(pcdFile.begin(), pcdFile.end());
//        // 加载点云
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//        cout << "->正在读入点云..." << endl;
//        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFilenameStr, *cloud) == -1) {
//            PCL_ERROR("Couldn't read file %ls\n", pcdFile.c_str());
//        }
//        cout << "Cloud before filtering:\n " << *cloud << endl;
//        // 提取文件名（不包括根目录）和路径
//        size_t lastSlashIndex = pcdFile.find_last_of(L"\\");
//        std::wstring filename = pcdFile.substr(lastSlashIndex + 1);
//        size_t lastDotIndex = filename.find_last_of(L".");
//        std::wstring pcdFilename_save = outputPath + L"\\" + filename.substr(0, lastDotIndex) + L".pcd";
//        std::string pcdFilenameStr_save(pcdFilename_save.begin(), pcdFilename_save.end());
//        cout << "The points data:  " << cloud->points.size() << endl;
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//        // 实现从点云中每every_k_points个点采样一次
//        int every_k_points = 5; // 采样间隔
//        pcl::Indices indices;
//        for (size_t i = 0; i < cloud->size(); i += every_k_points)
//        {
//            indices.push_back(i);
//        }
//        cout << "采样后点的个数为：" << indices.size() << endl;
//        pcl::copyPointCloud(*cloud, indices, *cloud_filtered);
//        cout << "Cloud after filtering: \n" << *cloud_filtered << endl;
//        ///保存下采样点云
//        pcl::io::savePCDFileASCII<pcl::PointXYZ>(pcdFilenameStr_save, *cloud_filtered);
//        std::wcout << L"Saved " << pcdFilename_save << std::endl;
//    }
//}
//
//// 函数用于遍历容器中的每个pcd文件并RANSAC去除地面
//void allpcd_RANASC(const std::vector<std::wstring>& pcdFiles, const std::wstring& outputPath) {
//    for (const auto& pcdFile : pcdFiles) {
//        // 读取点云数据
//        // 将wstring类型转换为string类型
//        std::string pcdFilenameStr(pcdFile.begin(), pcdFile.end());
//        // 加载点云
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//        cout << "->正在读入点云..." << endl;
//        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFilenameStr, *cloud) == -1) {
//            PCL_ERROR("Couldn't read file %ls\n", pcdFile.c_str());
//        }
//        cout << "Point cloud data: " << cloud->points.size() << " points" << endl;
//        // 提取文件名（不包括根目录）和路径
//        size_t lastSlashIndex = pcdFile.find_last_of(L"\\");
//        std::wstring filename = pcdFile.substr(lastSlashIndex + 1);
//        size_t lastDotIndex = filename.find_last_of(L".");
//        std::wstring pcdFilename_save = outputPath + L"\\" + filename.substr(0, lastDotIndex) + L".pcd";
//        std::string pcdFilenameStr_save(pcdFilename_save.begin(), pcdFilename_save.end());
//        //创建分割时所需要的模型系数对象coefficients及存储内点的点索引集合对象inliers。
//        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//        // 创建分割对象
//        pcl::SACSegmentation<pcl::PointXYZ> seg;
//        // 可选择配置，设置模型系数需要优化
//        seg.setOptimizeCoefficients(true);
//        // 必须配置，设置分割的模型类型、所用随机参数估计方法
//        seg.setModelType(pcl::SACMODEL_PLANE);
//        seg.setMethodType(pcl::SAC_RANSAC);
//        seg.setDistanceThreshold(0.15);// 距离阈值 单位m。距离阈值决定了点被认为是局内点时必须满足的条件
//        //距离阈值表示点到估计模型的距离最大值。
//        seg.setInputCloud(cloud);//输入点云
//        seg.segment(*inliers, *coefficients);//实现分割，并存储分割结果到点集合inliers及存储平面模型系数coefficients
//        if (inliers->indices.size() == 0)
//        {
//            PCL_ERROR("Could not estimate a planar model for the given dataset.");
//        }
//        //***********************************************************************
//        //-----------平面模型的系数 A,B,C,D-----------
//        double A = coefficients->values[0];
//        double B = coefficients->values[1];
//        double C = coefficients->values[2];
//        double D = coefficients->values[3];
//        //***********************************************************************
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
//        vector<int>pointIdxVec;
//
//        for (int i = 0; i < cloud->points.size(); ++i)
//        {
//            float x0 = cloud->points[i].x;
//            float y0 = cloud->points[i].y;
//            float z0 = cloud->points[i].z;
//
//            float absDistance = fabs(A * x0 + B * y0 + C * z0 + D) / sqrt(A * A + B * B + C * C);//计算点到平面的距离
//
//            if (absDistance > 0.04)//距离阈值
//            {
//                pointIdxVec.push_back(i);
//            }
//        }
//        pcl::copyPointCloud(*cloud, pointIdxVec, *cloud0);
//        ///保存下采样点云
//        pcl::PCDWriter writer;
//        pcl::io::savePCDFileASCII<pcl::PointXYZ>(pcdFilenameStr_save, *cloud0);
//        std::wcout << L"Saved " << pcdFilename_save << std::endl;
//    }
//}
//
//// 函数用于遍历容器中的每个pcd文件并使用半径滤波
//void allpcd_radius_filter(const std::vector<std::wstring>& pcdFiles, const std::wstring& outputPath) {
//    for (const auto& pcdFile : pcdFiles) {
//        // 读取点云数据
//        // 将wstring类型转换为string类型
//        std::string pcdFilenameStr(pcdFile.begin(), pcdFile.end());
//        // 加载点云
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//        cout << "->正在读入点云..." << endl;
//        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFilenameStr, *cloud) == -1) {
//            PCL_ERROR("Couldn't read file %ls\n", pcdFile.c_str());
//        }
//        cout << "Point cloud data: " << cloud->points.size() << " points" << endl;
//        // 提取文件名（不包括根目录）和路径
//        size_t lastSlashIndex = pcdFile.find_last_of(L"\\");
//        std::wstring filename = pcdFile.substr(lastSlashIndex + 1);
//        size_t lastDotIndex = filename.find_last_of(L".");
//        std::wstring pcdFilename_save = outputPath + L"\\" + filename.substr(0, lastDotIndex) + L".pcd";
//        std::string pcdFilenameStr_save(pcdFilename_save.begin(), pcdFilename_save.end());
//        // -------------------------------半径滤波----------------------------------------
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::StopWatch time;
//        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
//        ror.setInputCloud(cloud);     // 输入点云
//        ror.setRadiusSearch(0.05);        // 设置半径为0.1m范围内找临近点
//        ror.setMinNeighborsInRadius(80); // 设置查询点的邻域点集数小于50删除
//        ror.filter(*cloud_radius);       // 执行滤波
//        //pcl::io::savePCDFileASCII("cloud_radius.pcd", *cloud_radius);
//        cout << "滤波前有: " << cloud->size() << " 个点 " << endl;
//        cout << "滤波后有: " << cloud_radius->size() << " 个点 " << endl;
//        cout << "运行时间:" << time.getTime() << "毫秒" << endl;
//        ///保存下采样点云
//        pcl::PCDWriter writer;
//        pcl::io::savePCDFileASCII<pcl::PointXYZ>(pcdFilenameStr_save, *cloud_radius);
//        std::wcout << L"Saved " << pcdFilename_save << std::endl;
//    }
//}
//
//
//// 函数用于遍历容器中的每个pcd文件并使用最远点采样
//void allpcd_fps(const std::vector<std::wstring>& pcdFiles, const std::wstring& outputPath) {
//    for (const auto& pcdFile : pcdFiles) {
//        // 读取点云数据
//        // 将wstring类型转换为string类型
//        std::string pcdFilenameStr(pcdFile.begin(), pcdFile.end());
//        // 加载点云
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//        cout << "->正在读入点云..." << endl;
//        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFilenameStr, *cloud) == -1) {
//            PCL_ERROR("Couldn't read file %ls\n", pcdFile.c_str());
//        }
//        cout << "Point cloud data: " << cloud->points.size() << " points" << endl;
//        // 提取文件名（不包括根目录）和路径
//        size_t lastSlashIndex = pcdFile.find_last_of(L"\\");
//        std::wstring filename = pcdFile.substr(lastSlashIndex + 1);
//        size_t lastDotIndex = filename.find_last_of(L".");
//        std::wstring pcdFilename_save = outputPath + L"\\" + filename.substr(0, lastDotIndex) + L".pcd";
//        std::string pcdFilenameStr_save(pcdFilename_save.begin(), pcdFilename_save.end());
//        // -------------------------------最远点采样---------------------------------
//        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
//        std::random_device rd;
//        int random_seed = rd();   // 随机选取一个点作为种子点
//        int sampleNum = 2500;    // 采样点个数
//        pcl::FarthestPointSampling<pcl::PointXYZ> fps;
//        fps.setInputCloud(cloud); // 读取点云
//        fps.setSeed(random_seed); // 设置第一个种子点
//        fps.setSample(sampleNum); // 设置采样点个数
//        fps.filter(*filtered);    // 进行采样
//        //pcl::io::savePCDFileASCII("cloud_radius.pcd", *cloud_radius);
//        cout << "滤波前有: " << cloud->size() << " 个点 " << endl;
//        cout << "滤波后有: " << filtered->size() << " 个点 " << endl;
//        ///保存下采样点云
//        pcl::PCDWriter writer;
//        pcl::io::savePCDFileASCII<pcl::PointXYZ>(pcdFilenameStr_save, *filtered);
//        std::wcout << L"Saved " << pcdFilename_save << std::endl;
//    }
//}
//
//int main() {
//    //string pcd_path = "F:\\Pig_Cattle\\cattle\\1_cattle.pcd";
//    //string pcd_save_path = "F:\\Pig_Cattle\\cattle\\2_cattle_passthough.pcd";
//    //// 进行滤波操作
//    //filterPointCloud(pcd_path, pcd_save_path);
//    //std::cout << "Filtered cloud saved!" << std::endl;
//
//    std::wstring directory = L"F:\\Pig_Cattle\\cattle\\ply_1"; // 更改为您的文件夹路径
//    std::wstring outputPath = L"F:\\Pig_Cattle\\cattle\\pcd_2";
//    std::wstring outputPath_filter = L"F:\\Pig_Cattle\\cattle\\pcd_1_filter";
//    std::wstring outputPath_pure = L"E:\\Pig_Cattle\\cattle\\2_fps";
//    std::wstring outputPath_pcd = L"E:\\Pig_Cattle\\cattle\\pure_2";
//    std::vector<std::wstring> plyFiles = getFilesInDirectory(directory);
//    std::vector<std::wstring> pcdFiles = getFilesInDirectory_pcd(outputPath_filter);
//    std::vector<std::wstring> pcdFiles_pure = getFilesInDirectory_pcd(outputPath_pcd);
//    //显示文件名
//    std::wcout << L"Files in directory:\n";
//    //for (const auto& file : pcdFiles) {
//    //    std::wcout << file << L"\n";
//    //}
//    for (const auto& file : pcdFiles_pure) {
//        std::wcout << file << L"\n";
//    }
//    // 指定保存文件的路径
//    // 将所有PLY文件转换为PCD文件
//    //convertPlyFilesToPcd(plyFiles, outputPath);
//    //allpcd_Passthough_filter(pcdFiles, outputPath_filter);
//    //allpcd_Statistical_filter(pcdFiles_pure, outputPath_pure);
//    //allpcd_uniform_filter(pcdFiles, outputPath_pure);
//    //allpcd_RANASC(pcdFiles, outputPath_pure);
//    //allpcd_radius_filter(pcdFiles_pure, outputPath_pure);
//    allpcd_fps(pcdFiles_pure, outputPath_pure);
//    return 0;
//}