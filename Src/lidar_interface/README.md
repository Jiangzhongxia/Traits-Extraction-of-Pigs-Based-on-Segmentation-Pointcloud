# 激光雷达接口模块

## 概述
该模块实现了与图达通灵雀w激光雷达的接口，支持同步收集猪的左右两边点云信息。

## 功能
- 初始化两个激光雷达设备（左侧和右侧）
- 实时采集点云数据
- 同步左右两侧点云数据
- 提供数据处理回调接口

## 文件结构
- `LivoxLidarInterface.h` - 接口头文件
- `LivoxLidarInterface.cpp` - 实现文件

## 使用方法
```cpp
// 创建接口实例
LivoxLidarInterface lidar_interface;

// 配置设备
std::vector<LivoxLidarInterface::DeviceConfig> configs;
LivoxLidarInterface::DeviceConfig left_config;
left_config.ip_address = "192.168.1.10";
left_config.port = 65000;
left_config.device_type = "left";
configs.push_back(left_config);

LivoxLidarInterface::DeviceConfig right_config;
right_config.ip_address = "192.168.1.11";
right_config.port = 65000;
right_config.device_type = "right";
configs.push_back(right_config);

// 初始化
if (!lidar_interface.initialize(configs)) {
    // 处理初始化错误
}

// 设置数据回调
lidar_interface.setDataCallback([](const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                 const std::string& side) {
    std::cout << "Received " << side << " cloud with " << cloud->size() << " points" << std::endl;
});

// 开始采集
lidar_interface.startCapture();

// 获取点云数据
auto left_cloud = lidar_interface.getLeftPointCloud();
auto right_cloud = lidar_interface.getRightPointCloud();

// 同步融合点云
pcl::PointCloud<pcl::PointXYZI>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZI>);
if (lidar_interface.syncPointClouds(combined_cloud)) {
    std::cout << "Combined cloud has " << combined_cloud->size() << " points" << std::endl;
}

// 停止采集
lidar_interface.stopCapture();
```

## 注意事项
- 实际部署时需要安装Livox SDK
- 需要根据实际设备IP地址和端口配置DeviceConfig
- 该模块为实时数据采集模块，需要持续运行