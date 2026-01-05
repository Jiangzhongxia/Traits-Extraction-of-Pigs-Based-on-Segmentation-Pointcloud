/*
 * Name:        CalibIO.h
 * Description: Interface to calibration files.
 * Author(s):   stonlimart [stonlimart@hotmail.com]
 */

#ifndef HOLEFILLING_HOLE_FILLING_H
#define HOLEFILLING_HOLE_FILLING_H

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

#define PI 3.141592653

typedef pcl::PointNormal Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::KdTreeFLANN<Point> KDTree;

class BoundaryNode {
public:
    BoundaryNode();
    BoundaryNode(Point& p, Point& former, Point& latter);
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    Eigen::Vector3f orient;

    Eigen::Vector3f Ob;
    Eigen::Vector3f nextOb;

    std::vector<Point> Nbors;
    std::vector<float> g1s;
    std::vector<float> g2s;
    std::vector<Eigen::Vector3f> Oqs;
    std::vector<float> elastics;
    BoundaryNode* nearestNode;
    BoundaryNode* formerNode;
    bool isValid;
};

class HoleFilling {
public:
    HoleFilling();
    HoleFilling(pcl::PointCloud<pcl::PointXYZ>::Ptr& cld, std::vector<std::vector<int>>& bds);
    void compute();
    void visualization();

private:
    void initBoundaryNode(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int>& boundary, std::vector<std::vector<BoundaryNode>>& oneHoleB);
    void computeNextObAndNewCurve(std::vector<BoundaryNode>& bns, std::vector<BoundaryNode>& newBns);
    void computeNeighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<BoundaryNode>& bns, std::vector<std::vector<BoundaryNode>>& oneHoleB);
    void addToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<BoundaryNode>& bns);
    void computeOqsandElatics(std::vector<BoundaryNode>& bns);
    void computeOrient(std::vector<BoundaryNode>& bns);
    void delInvalid(std::vector<BoundaryNode>& bns);
    void computeNorm(std::vector<BoundaryNode>& bns);
    void PositionOptim(std::vector<BoundaryNode>& bns);

    double sigmaD;
    double sigmaN;
    double sigmaR;
    double radiusNbor;
    int kNbors;
    bool knnOrRadius;
    int maxIter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::vector<std::vector<int>> boundaries;
    std::vector<std::vector<std::vector<BoundaryNode>>> allHoleNodes;
    KDTree kdtree;
};

#endif

