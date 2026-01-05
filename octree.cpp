#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include <iostream>
#include <vector>
#include <ctime>

int
main()
{
    srand((unsigned int)time(NULL));
    // 1、定义并实例化一个共享的PointCloud结构，并使用随机点填充它。
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        (*cloud)[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud)[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud)[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }

    /*-----------------------------------------------------------------------------
     * 2、创建一个八叉树实例，使用八叉树分辨率进行初始化。这个八叉树在它的叶节点中*
     *保留了一个点索引向量。分辨率参数描述最低八叉树级别上最小体素的长度。因此，  *
     *八叉树的深度是分辨率的函数，也是点云的空间维数的函数。如果知道点云的边界框，*
     *则应该使用finebeliingBox方法将其分配给八叉树。                              *
     *然后，为PointCloud分配一个指针，并将所有的点添加到八叉树中。                *
     -----------------------------------------------------------------------------*/
    float resolution = 128.0f;                                            // 八叉树分辨率
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);// 使用分辨率初始化八叉树

    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    pcl::PointXYZ searchPoint;

    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    /*------------------------------------------------------------------------------
     * 3、一旦PointCloud与八叉树相关联，就可以执行搜索操作。这里使用的第一个搜索法 *
     *是“Voxel搜索中的邻居”。它将搜索点分配给相应的叶节点体素，并返回点索引的向量*
     *这些指数与属于同一体素范围内的点有关。                                       *
     *因此，搜索点与搜索结果之间的距离取决于八叉树的分辨率参数。                   *
     * ----------------------------------------------------------------------------*/

    std::vector<int> pointIdxVec;

    if (octree.voxelSearch(searchPoint, pointIdxVec))
    {
        std::cout << "Neighbors within voxel search at (" << searchPoint.x
            << " " << searchPoint.y
            << " " << searchPoint.z << ")"
            << std::endl;

        for (std::size_t i = 0; i < pointIdxVec.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxVec[i]].x
            << " " << (*cloud)[pointIdxVec[i]].y
            << " " << (*cloud)[pointIdxVec[i]].z << std::endl;
    }

    /*-------------------------------------------------------------------------------
     *其次，证明了K最近邻搜索。在这个例子中，K被设置为10。                           *
     *“K最近邻搜索”方法将搜索结果写入两个独立的向量中。                            *
     *第一个是pointIdxNKNSearch，它将包含搜索结果(引用相关PointCloud数据集的索引)。  *
     *第二个向量在搜索点和最近邻之间保持相应的平方距离。                             *
     *-------------------------------------------------------------------------------*/

    int K = 10;

    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    std::cout << "K nearest neighbor search at (" << searchPoint.x
        << " " << searchPoint.y
        << " " << searchPoint.z
        << ") with K=" << K << std::endl;
    // K近邻搜索
    if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxNKNSearch[i]].x
            << " " << (*cloud)[pointIdxNKNSearch[i]].y
            << " " << (*cloud)[pointIdxNKNSearch[i]].z
            << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

    /*-----------------------------------------------------------------------
    *“半径搜索中的邻居”的工作原理非常类似于“K最近邻搜索”。              *
    *它的搜索结果被写入两个分别描述点索引和平方搜索点距离的向量中。         *
    *-----------------------------------------------------------------------*/

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << searchPoint.x
        << " " << searchPoint.y
        << " " << searchPoint.z
        << ") with radius=" << radius << std::endl;

    // 半径搜索
    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxRadiusSearch[i]].x
            << " " << (*cloud)[pointIdxRadiusSearch[i]].y
            << " " << (*cloud)[pointIdxRadiusSearch[i]].z
            << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }
    system("pause");
    return 0;
}
