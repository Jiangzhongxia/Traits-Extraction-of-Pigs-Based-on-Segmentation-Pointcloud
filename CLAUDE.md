# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# IsoConstraints 点云预处理及体尺计算代码库

## 项目概述

IsoConstraints 是一个用于点云预处理和动物体尺计算的专业软件系统。基于带等值面约束的泊松方程来优化点云法向量并重建表面，特别适用于畜牧业中的动物体尺测量应用。该项目通过改进的泊松重建算法，能够处理带未定向法线的点云数据，实现法向量定向和表面重建。

## 核心架构组件

### 1. 泊松重建核心模块 (`Src/`)
- **主要文件**: `PoissonRecon.cpp`
- **职责**: 实现IsoConstraints算法核心，包括粗点云定向、隐式场定向和泊松表面重建
- **关键数据结构**: `MultiGridOctreeData.h` (多网格八叉树数据结构), `MarchingCubes.cpp` (等值面提取算法), `Octree.h` (八叉树空间分割)

### 2. 点云处理模块 (`pcl/` & `Src/pointcloud_processing/`)
- **主要文件**: `pclknn.cpp`
- **职责**: 使用PCL库进行KNN搜索功能
- **功能**: 为每个点找到K个最近邻点，八叉树加速搜索，点云预处理和滤波

### 3. 体尺测量模块 (`体尺测量/` & `Src/measurement/`)
- **主要文件**:
  - `Body_length_measurement.cpp` - 体长测量
  - `Withers_heigh.cpp` - 肩高测量
  - `Chest_depth.cpp` - 胸深测量
  - `Chest_width.cpp` - 胸宽测量
  - `Hip_height.cpp` - 髋高测量
  - `Hip_width.cpp` - 髋宽测量
  - `abdominal_grith.cpp` - 腹围测量
  - `abdominal_width.cpp` - 腹宽测量
  - `abdominal_height.cpp` - 腹高测量

### 4. 激光雷达接口模块 (`Src/lidar_interface/`)
- **主要文件**: `LivoxLidarInterface.cpp`
- **职责**: 与图达通灵雀w激光雷达通信
- **功能**: 双目激光雷达数据同步，实时点云数据采集，数据格式转换

### 5. 点云分割模块 (`Src/segmentation/`)
- **主要文件**: `ONNXSegmenter.cpp`
- **职责**: 使用ONNX模型进行点云分割
- **功能**: 将完整动物点云分割为不同部位，头部、四肢、躯干等部位识别，模型推理和置信度计算

### 6. 主应用程序控制器 (`Src/application/`)
- **主要文件**: `PigMeasurementApp.cpp`
- **职责**: 协调各模块工作
- **功能**: 管理应用程序生命周期，处理用户输入和配置，错误处理和状态监控

## 数据流架构

```
双目激光雷达 → 数据同步 → 点云预处理 → 点云分割 → 体尺计算 → 结果输出/可视化
```

## 开发环境与依赖

- **操作系统**: Windows 10/11 或 Linux Ubuntu 20.04+
- **编程语言**: C++17, Python 3.x
- **核心库**: PCL 1.10+ (Point Cloud Library), Eigen3
- **并行计算**: OpenMP
- **机器学习**: ONNX Runtime
- **图像处理**: OpenCV
- **数学库**: CGAL (用于法向量估计)
- **GUI框架**: Qt/ImGui
- **激光雷达接口**: Livox SDK

## 构建与运行

### 编译环境
- Windows平台
- Visual Studio (项目文件: `PoissonRecon.sln` - 需要手动创建)
- PCL库 (版本 1.10.1)
- 需要将PCL的bin目录添加到系统环境变量

### CMake 构建
```bash
mkdir build
cd build
cmake ..
make
```

### 主程序运行
```bash
./Bin/x64/Release/PoissonRecon.exe coarse_filein fine_filein fileout use_implicit_orient is_noisy_input oriented_optimized
```

### 示例运行命令
```bash
./Bin/x64/Release/PoissonRecon.exe ./data/xyzrgb_statuette_coarse.xyz ./data/xyzrgb_statuette_fine.xyz ./results/xyzrgb_statuette.ply true false true
```

### 点云法线估计
```bash
./jet/normals_estimation.exe input_xyz output_xyz neighbour_size
```

### 点云随机采样
```bash
cd ./subsample
python random_sample.py
```

## 代码结构目录

```
IsoConstraints-main/
├── Src/                    # 核心算法模块
│   ├── application/       # 应用程序控制
│   ├── lidar_interface/   # 激光雷达接口
│   ├── pointcloud_processing/ # 点云处理
│   ├── segmentation/      # 点云分割
│   ├── measurement/       # 体尺计算
│   ├── optimization/      # 性能优化
│   └── visualization/     # 可视化管理
├── pcl/                  # PCL点云处理工具
├── subsample/            # Python预处理脚本
├── 体尺测量/              # 体尺测量模块
├── docs/                 # 文档
├── jet/                  # 法向量估计工具
├── results/              # 结果输出目录
└── data/                 # 数据文件目录
```

## 编码规范

- C++代码遵循标准C++17规范
- 使用OpenMP进行并行计算
- 点云数据格式为`.xyz`（包含位置和法线信息）或`.ply`格式
- 体尺测量代码使用PCL库进行点云处理
- 注释使用中文，与现有代码库保持一致

## 常用开发任务

### 运行单个测试
```bash
# 编译并运行特定的体尺测量程序
g++ -o body_length_measurement Body_length_measurement.cpp -lpcl_common -lpcl_io
./body_length_measurement
```

### 编译PCL模块
```bash
g++ -o pclknn pclknn.cpp -lpcl_common -lpcl_io -lpcl_kdtree
```

### 编译Python预处理脚本
```bash
cd subsample/
python -c "import numpy, scipy.spatial; print('Dependencies OK')"
```

## 关键技术特性

1. **高性能计算**
   - 多线程并行处理
   - 八叉树空间加速
   - GPU加速支持

2. **工业级稳定性**
   - 实时数据处理
   - 异常处理机制
   - 数据完整性保障

3. **模块化设计**
   - 清晰的模块职责分离
   - 可扩展的插件架构
   - 配置驱动的设计

4. **专业算法**
   - 带等值面约束的泊松重建
   - 动物专用体尺测量算法
   - 3D曲线拟合和几何计算