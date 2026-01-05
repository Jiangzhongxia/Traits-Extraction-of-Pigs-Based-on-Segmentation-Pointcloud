# 泊松重建核心模块文档 (Src)

## 模块职责

该模块是整个项目的核心，实现了基于等值面约束的泊松重建算法 (IsoConstraints)。主要功能包括：
- 点云法向量定向
- 带等值面约束的泊松方程求解
- 粗到细隐式场定向
- 表面重建

## 入口与启动

主入口文件：`PoissonRecon.cpp`

关键函数：
- `Orient_coarse()` - 定向粗点云
- `ImplicitOrient()` - 隐式场定向功能
- `PoissonRecon()` - 泊松重建函数
- `main()` - 程序主入口

## 对外接口

### 命令行接口
```
./PoissonRecon.exe coarse_filein fine_filein fileout use_implicit_orient is_noisy_input oriented_optimized
```

参数说明：
1. `coarse_filein`: 包含未定向法线的粗点云文件名
2. `fine_filein`: 包含未定向法线的细点云文件名
3. `fileout`: 输出网格的文件名，定向点云将命名为fileout[:-4]+"_oriented.xyz"
4. `use_implicit_orient`: 是否使用粗到细隐式场定向方法
5. `is_noisy_input`: 输入是否为带噪声点云
6. `oriented_optimized`: "true"使用定向法线，"false"使用优化法线(仅3D草图)

## 关键依赖与配置

- **PCL库**：用于KNN搜索
- **OpenMP**：并行计算支持
- **CGAL**：法线估计 (通过./jet/normals_estimation.exe)

### 核心参数配置
- `alpha=1e4, beta=1e-2/1e-4` (噪声/非噪声输入)
- `pointweight=1.0/10.0` (噪声/非噪声输入)
- `coarse_orient_depth=7` (粗点云定向深度)
- `recon_depth=10` (重建深度)

## 数据模型

### 核心数据结构
- `Octree<Real>` - 八叉树数据结构
- `Octree2<Real>` - 用于粗点云定向的改进八叉树
- `PointInfo` - 点信息结构
- `NormalInfo` - 法线信息结构
- `SmoothCoeff` - 平滑系数结构

### 文件格式支持
- 输入: `.xyz` 文件 (包含位置和法线信息，格式: x y z nx ny nz)
- 输出: `.ply` 文件 (重建的网格) 和 `.xyz` 文件 (定向点云)

## 测试与质量

该模块通过以下方式测试：
- 多个示例点云文件在 `./data/` 目录
- 对比重建结果在 `./results/` 目录
- 参数配置支持噪声和非噪声输入测试

## 常见问题 (FAQ)

1. **内存需求高**: 大规模点云定向过程内存需求高，建议使用粗到细隐式场定向方法
2. **推荐配置**: 建议至少16GB内存，对粗点集采样50K-60K点
3. **文件格式**: 输入点云必须包含未定向法线，可通过 `./jet/normals_estimation.exe` 估计

## 相关文件清单

- `PoissonRecon.cpp` - 主算法实现
- `MultiGridOctreeData.h/.inl` - 八叉树数据结构与算法
- `Octree.h/.inl` - 八叉树实现
- `Geometry.h/.cpp` - 几何计算
- `SparseMatrix.h/.inl` - 稀疏矩阵运算
- `MarchingCubes.h/.cpp` - 等值面提取

## 变更记录 (Changelog)

- **2025-12-15**: 模块文档创建
  - 记录了IsoConstraints算法的核心功能
  - 识别了主要接口和配置参数