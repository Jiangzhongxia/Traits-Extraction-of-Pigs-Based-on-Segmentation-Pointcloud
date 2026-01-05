//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/common/centroid.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/geometry/planar_polygon.h>//定义多边形
//
//// 逆时针排序
//void CounterClockWiseSortPoints(pcl::PointCloud<pcl::PointXYZ>& vPoints)
//{
//	int cnt = static_cast<int>(vPoints.size());
//	if (cnt < 3)
//		return;
//	//计算中心
//	Eigen::Vector4f centroid;					// 质心
//	pcl::compute3DCentroid(vPoints, centroid);	// 齐次坐标，（c0,c1,c2,1）
//	pcl::PointXYZ center{ centroid[0],centroid[1] ,centroid[2] };
//	//若点a小于点b,即点a在点b逆时针方向,返回true,否则返回false
//	auto PointCmp = [](const pcl::PointXYZ& a, const pcl::PointXYZ& b, const pcl::PointXYZ& center)
//		{
//			if (a.x <= 0 && b.x > 0)
//				return true;
//			if (a.x == 0 && b.x == 0)
//				return a.y < b.y;
//			//向量OA和向量OB的叉积，向量OA和OB的叉积大于0，则向量OA在向量OB的逆时针方向，即点A小于点B。
//			float det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
//			if (det < 0)
//				return false;
//			if (det > 0)
//				return true;
//			//向量OA和向量OB共线，以距离判断大小
//			float d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
//			float d2 = (b.x - center.x) * (b.x - center.y) + (b.y - center.y) * (b.y - center.y);
//			return d1 > d2;
//		};
//
//	//冒泡排序
//	for (int i = 0; i < cnt - 1; i++)
//	{
//		for (int j = 0; j < cnt - i - 1; j++)
//		{
//			if (PointCmp(vPoints[j], vPoints[j + 1], center))
//				std::swap(vPoints[j], vPoints[j + 1]);
//		}
//	}
//}
//
//// 顺时针排序
//void ClockWiseSortPoints(pcl::PointCloud<pcl::PointXYZ>& vPoints)
//{
//	int cnt = static_cast<int>(vPoints.size());
//	if (cnt < 3)
//		return;
//	//计算中心
//	Eigen::Vector4f centroid;					// 质心
//	pcl::compute3DCentroid(vPoints, centroid);	// 齐次坐标，（c0,c1,c2,1）
//	pcl::PointXYZ center{ centroid[0],centroid[1] ,centroid[2] };
//	//若点a大于点b,即点a在点b顺时针方向,返回true,否则返回false
//	auto PointCmp = [](const pcl::PointXYZ& a, const pcl::PointXYZ& b, const pcl::PointXYZ& center)
//		{
//			if (a.x >= 0 && b.x < 0)
//				return true;
//			if (a.x == 0 && b.x == 0)
//				return a.y > b.y;
//			//向量OA和向量OB的叉积，向量OA和OB的叉积大于0，则向量OA在向量OB的逆时针方向，即点A小于点B。
//			float det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
//			if (det < 0)
//				return false;
//			if (det > 0)
//				return true;
//			//向量OA和向量OB共线，以距离判断大小
//			float d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
//			float d2 = (b.x - center.x) * (b.x - center.y) + (b.y - center.y) * (b.y - center.y);
//			return d1 > d2;
//		};
//
//	//冒泡排序
//	for (int i = 0; i < cnt - 1; i++)
//	{
//		for (int j = 0; j < cnt - i - 1; j++)
//		{
//			if (PointCmp(vPoints[j], vPoints[j + 1], center))
//				std::swap(vPoints[j], vPoints[j + 1]);
//		}
//	}
//}
//
//int
//main(int argc, char** argv)
//{
//	//--------------------------加载点云数据----------------------------
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//	pcl::PCDReader reader;
//	reader.read("E:\\Pig_Cattle\\Pig\\label\\after_label_two\\80forseg\\cut\\Hip_grith\\points_cut.pcd", *cloud);
//	std::cerr << "原始点云点的个数: " << cloud->points.size() << std::endl;
//	ClockWiseSortPoints(*cloud);
//	//CounterClockWiseSortPoints(*cloud); // 逆时针排序
//	//增加多边形
//	pcl::PlanarPolygon<pcl::PointXYZ> polygon;
//	pcl::PointCloud<pcl::PointXYZ> contour;
//	contour.width = cloud->width;
//	contour.height = 1;
//	contour.is_dense = false;
//	contour.resize(contour.height * contour.width);
//
//	for (size_t i = 0; i < cloud->points.size(); ++i)
//	{
//		cout << "第" << i << "个点的坐标为" << cloud->points[i] << endl;
//		contour.points[i] = cloud->points[i];
//	}
//
//	polygon.setContour(contour);
//	pcl::io::savePCDFileASCII("顺时针排列.pcd", *cloud);
//	pcl::visualization::PCLVisualizer viewer("Viewer");
//	viewer.setWindowName("平面点云边界点排序");
//	viewer.addPolygon(polygon, 255, 0, 0, "ploygon", 0);
//	viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud"); // 设置点云大
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce(1);
//	}
//	return 0;
//}


