# 数据可视化模块

## 概述
该模块提供点云数据可视化功能，包括原始点云显示、分割结果可视化、测量结果展示等。

## 功能
- 显示原始点云数据
- 显示分割后的点云（不同颜色表示不同部位）
- 显示测量结果文本
- 显示点云边界框
- 保存截图功能
- 交互式可视化窗口

## 文件结构
- `VisualizationManager.h` - 接口头文件
- `VisualizationManager.cpp` - 实现文件

## 使用方法
```cpp
// 创建可视化管理器实例
VisualizationManager viz_manager;

// 设置可视化配置（可选）
VisualizationManager::VisualizationConfig config;
config.window_width = 1200;
config.window_height = 800;
config.show_segmented_cloud = true;
config.show_measurements = true;
viz_manager.setConfig(config);

// 初始化可视化窗口
if (!viz_manager.initialize()) {
    std::cerr << "Failed to initialize visualization" << std::endl;
    return -1;
}

// 显示原始点云
viz_manager.displayPointCloud(original_cloud);

// 显示分割后的点云
std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> segments;
// ... 填充分割结果
viz_manager.displaySegmentedCloud(segments);

// 显示测量结果
MeasurementResult measurements;
// ... 填入测量结果
viz_manager.displayMeasurements(measurements);

// 显示边界框
viz_manager.displayBoundingBox(original_cloud);

// 设置用户交互回调（可选）
viz_manager.setUserInteractionCallback([](const pcl::visualization::KeyboardEvent& event) {
    if (event.getKeySym() == "q" && event.keyDown()) {
        std::cout << "Quit key pressed" << std::endl;
    }
});

// 运行可视化循环
viz_manager.runVisualizationLoop();

// 保存截图
viz_manager.saveScreenshot("visualization_output.png");
```

## 可视化配置说明
- `show_original_cloud`: 是否显示原始点云
- `show_segmented_cloud`: 是否显示分割后的点云
- `show_measurements`: 是否显示测量结果
- `show_bounding_boxes`: 是否显示边界框
- `show_coordinate_system`: 是否显示坐标系
- `coordinate_system_scale`: 坐标系缩放比例
- `window_width`: 窗口宽度
- `window_height`: 窗口高度
- `window_title`: 窗口标题
- `point_size`: 点大小
- `show_axes`: 是否显示轴

## 部位颜色映射
不同部位使用不同颜色显示：
- background: 灰色 (128, 128, 128)
- head: 红色 (255, 0, 0)
- torso: 绿色 (0, 255, 0)
- front_left_leg: 蓝色 (0, 0, 255)
- front_right_leg: 青色 (255, 255, 0)
- hind_left_leg: 洋红色 (255, 0, 255)
- hind_right_leg: 黄色 (0, 255, 255)
- tail: 紫色 (128, 0, 128)
- ear: 橙色 (255, 165, 0)
- neck: 棕色 (139, 69, 19)

## 注意事项
- 需要安装PCL可视化模块
- 可视化窗口运行时会阻塞主线程
- 大规模点云可能影响渲染性能
- 确保在主线程中调用可视化函数（对于GUI应用）