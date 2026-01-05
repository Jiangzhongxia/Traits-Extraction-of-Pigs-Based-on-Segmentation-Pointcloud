#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "hole_filling.h"

BoundaryNode::BoundaryNode(Point& p, Point& former, Point& latter)
    : position(p.x, p.y, p.z), normal(p.normal_x, p.normal_y, p.normal_z),
    Ob(p.x, p.y, p.z), nextOb(0, 0, 0), isValid(true) {
    Eigen::Vector3f tanVec(latter.x - former.x, latter.y - former.y, latter.z - former.z);
    orient = normal.cross(tanVec).normalized();
    nearestNode = nullptr;
    formerNode = nullptr;
}

BoundaryNode::BoundaryNode()
    : position(0, 0, 0), normal(0, 0, 0), orient(0, 0, 0),
    Ob(0, 0, 0), nextOb(0, 0, 0), isValid(true) {
    nearestNode = nullptr;
    formerNode = nullptr;
}

HoleFilling::HoleFilling()
    : sigmaD(1), sigmaN(0.9), sigmaR(0.15), radiusNbor(0.05), kNbors(15), knnOrRadius(true), maxIter(50) {
    cloud = nullptr;
}

HoleFilling::HoleFilling(pcl::PointCloud<pcl::PointXYZ>::Ptr& cld, std::vector<std::vector<int>>& bds)
    : sigmaD(1), sigmaN(1), sigmaR(0.12), radiusNbor(0.05), kNbors(20), knnOrRadius(true), maxIter(50) {
    cloud = cld;
    boundaries = bds;
    allHoleNodes.resize(boundaries.size());
}

void HoleFilling::initBoundaryNode(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int>& boundary, std::vector<std::vector<BoundaryNode>>& oneHoleB) {
    std::vector<BoundaryNode> bNodes, bNodesF;
    Eigen::Vector3f holeCenter(0, 0, 0);

    pcl::PointXYZ point_xyz, former_xyz, latter_xyz;
    Point point, former, latter;
    int nBoundary = boundary.size();

    for (int i = 0; i < nBoundary; ++i) {
        point_xyz = cloud->points[boundary[i]];
        holeCenter += Eigen::Vector3f(point_xyz.x, point_xyz.y, point_xyz.z);

        if (i == 0)
            former_xyz = cloud->points[boundary[nBoundary - 1]];
        else
            former_xyz = cloud->points[boundary[i - 1]];

        if (i == (nBoundary - 1))
            latter_xyz = cloud->points[boundary[0]];
        else
            latter_xyz = cloud->points[boundary[i + 1]];

        // ½« pcl::PointXYZ ×ª»»Îª Point
        point.x = point_xyz.x;
        point.y = point_xyz.y;
        point.z = point_xyz.z;

        former.x = former_xyz.x;
        former.y = former_xyz.y;
        former.z = former_xyz.z;

        latter.x = latter_xyz.x;
        latter.y = latter_xyz.y;
        latter.z = latter_xyz.z;

        BoundaryNode bn(point, former, latter);
        bNodes.push_back(bn);
    }

    holeCenter /= nBoundary;
    int count = 0;
    for (int i = 0; i < bNodes.size(); ++i) {
        Eigen::Vector3f tempV(holeCenter - bNodes[i].position);
        tempV = tempV - tempV.dot(bNodes[i].normal) * bNodes[i].normal;

        if (tempV.dot(bNodes[i].orient) < 0)
            ++count;
    }
    if (count > 0.5 * nBoundary) {
        for (int i = 0; i < bNodes.size(); ++i)
            bNodes[i].orient *= -1;
        bNodesF.resize(bNodes.size());
        bNodesF[0] = bNodes[0];
        for (int j = 1, i = bNodes.size() - j; j < bNodesF.size(); ++j, --i) {
            bNodesF[j] = bNodes[i];
        }
        oneHoleB.push_back(bNodesF);
    }
    else {
        oneHoleB.push_back(bNodes);
    }
    {
        {
            holeCenter /= nBoundary;
            int count = 0;
            for (int i = 0; i < bNodes.size(); ++i) {
                Eigen::Vector3f tempV(holeCenter - bNodes[i].position);
                tempV = tempV - tempV.dot(bNodes[i].normal) * bNodes[i].normal;

                if (tempV.dot(bNodes[i].orient) < 0)
                    ++count;
            }
            if (count > 0.5 * nBoundary) {
                for (int i = 0; i < bNodes.size(); ++i)
                    bNodes[i].orient *= -1;
                bNodesF.resize(bNodes.size());
                bNodesF[0] = bNodes[0];
                for (int j = 1, i = bNodes.size() - j; j < bNodesF.size(); ++j, --i) {
                    bNodesF[j] = bNodes[i];
                }
                oneHoleB.push_back(bNodesF);
            }
            else {
                oneHoleB.push_back(bNodes);
            }
        }
    }
}

void HoleFilling::compute() {
    for (int i = 0; i < boundaries.size(); ++i) {
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud, *cloud, *cloud2);
        allHoleNodes[i].resize(1);

        initBoundaryNode(cloud, boundaries[i], allHoleNodes[i]);
        std::vector<BoundaryNode> newBns;

        int it = 0;
        while (true) {
            computeNeighbors(cloud, allHoleNodes[i][it], allHoleNodes[i]);
            computeNextObAndNewCurve(allHoleNodes[i][it], newBns);

            if (newBns.size() < 5) {
                std::cout << "hole " << i + 1 << " is filled" << std::endl;
                break;
            }

            ++it;
            allHoleNodes[i].push_back(newBns);
        }

        for (int j = 0; j < allHoleNodes[i].size(); ++j) {
            addToCloud(cloud, allHoleNodes[i][j]);
        }
    }
}

void HoleFilling::computeNeighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<BoundaryNode>& bns, std::vector<std::vector<BoundaryNode>>& oneHoleB) {
    PointCloud::Ptr searchcloud(new PointCloud);
    Point p;
    int nBns = bns.size();

    for (int i = 0; i < nBns; ++i) {
        p.x = bns[i].position.x();
        p.y = bns[i].position.y();
        p.z = bns[i].position.z();
        p.normal_x = bns[i].normal.x();
        p.normal_y = bns[i].normal.y();
        p.normal_z = bns[i].normal.z();
        searchcloud->push_back(p);
    }

    kdtree.setInputCloud(searchcloud);
    float minAngle, tempAngle;
    int index;
    float radius = 2 * radiusNbor;
    int K = kNbors + 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    for (int i = 0; i < nBns; ++i) {
        if (kdtree.radiusSearch(searchcloud->points[i], radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            for (int j = 0; j < pointIdxNKNSearch.size(); ++j) {
                if (i == pointIdxNKNSearch[j])
                    continue;
                bns[i].Nbors.push_back(searchcloud->points[pointIdxNKNSearch[j]]);
            }
        }
    }

    for (int i = 0; i < bns.size(); ++i) {
        minAngle = 1000;
        for (int j = 0; j < bns[i].Nbors.size(); ++j) {
            tempAngle = acos(bns[i].orient.dot(Eigen::Vector3f(bns[i].Nbors[j].x, bns[i].Nbors[j].y, bns[i].Nbors[j].z) - bns[i].position) /
                (bns[i].orient.norm() * (Eigen::Vector3f(bns[i].Nbors[j].x, bns[i].Nbors[j].y, bns[i].Nbors[j].z) - bns[i].position).norm()));
            if (tempAngle < minAngle) {
                minAngle = tempAngle;
                index = j;
            }
        }
        for (int k = 0; k < oneHoleB.size(); ++k) {
            for (int j = 0; j < oneHoleB[k].size(); ++j) {
                if (Eigen::Vector3f(bns[i].Nbors[index].x, bns[i].Nbors[index].y, bns[i].Nbors[index].z) == oneHoleB[k][j].position) {
                    bns[i].nearestNode = &oneHoleB[k][j];
                }
            }
        }
    }
}

void HoleFilling::computeNextObAndNewCurve(std::vector<BoundaryNode>& bns, std::vector<BoundaryNode>& newBns) {
    for (int i = 0; i < bns.size(); ++i) {
        bns[i].nextOb = bns[i].Ob;
        for (int j = 0; j < bns[i].Nbors.size(); ++j) {
            Eigen::Vector3f nq(bns[i].Nbors[j].normal_x, bns[i].Nbors[j].normal_y, bns[i].Nbors[j].normal_z);
            Eigen::Vector3f sq(bns[i].Nbors[j].x, bns[i].Nbors[j].y, bns[i].Nbors[j].z);
            float dq = (bns[i].position - sq).norm();
            float g1 = exp(-dq * dq / 2 * sigmaD * sigmaD);
            float g2 = (bns[i].normal - nq).norm();
            float elastic = 0.5 * g1 * g2 * dq;
            bns[i].nextOb += elastic * (bns[i].position - sq) / dq;
        }
    }

    for (int i = 0; i < bns.size(); ++i) {
        if ((bns[i].position - bns[i].nextOb).norm() > radiusNbor / 2) {
            BoundaryNode newBn;
            newBn.position = bns[i].nextOb;
            newBn.normal = bns[i].normal;
            newBn.orient = bns[i].orient;
            newBns.push_back(newBn);
        }
    }
}

void HoleFilling::addToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<BoundaryNode>& bns) {
    for (int i = 0; i < bns.size(); ++i) {
        pcl::PointXYZ point;
        point.x = bns[i].position.x();
        point.y = bns[i].position.y();
        point.z = bns[i].position.z();
        cloud->push_back(point);
    }
}

void HoleFilling::visualization() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> originalColor(cloud, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, originalColor, "original cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original cloud");

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

//// Load point cloud from XYZ file
//bool loadXYZFile(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//    std::ifstream file(filename);
//    if (!file.is_open()) {
//        std::cerr << "Failed to open file: " << filename << std::endl;
//        return false;
//    }
//
//    std::string line;
//    while (std::getline(file, line)) {
//        pcl::PointXYZ point;
//        std::istringstream iss(line);
//        iss >> point.x >> point.y >> point.z;
//        cloud->push_back(point);
//    }
//    file.close();
//    return true;
//}

//int main(int argc, char** argv) {
//    if (argc < 3) {
//        std::cout << "Usage: <program> <input_point_cloud> <boundary_indices>" << std::endl;
//        return -1;
//    }
//
//    std::string input_file = argv[1];
//    std::string boundary_indices_file = argv[2];
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    if (!loadXYZFile(input_file, cloud)) {
//        return -1;
//    }
//
//    std::ifstream fileBNS(boundary_indices_file);
//    std::vector<std::vector<int>> boundaries;
//    if (fileBNS.is_open()) {
//        std::string line;
//        while (std::getline(fileBNS, line)) {
//            std::istringstream iss(line);
//            std::vector<int> boundary;
//            int index;
//            while (iss >> index) {
//                boundary.push_back(index);
//            }
//            boundaries.push_back(boundary);
//        }
//    }
//    else {
//        std::cerr << "Failed to open boundary indices file: " << boundary_indices_file << std::endl;
//        return -1;
//    }
//
//    HoleFilling hf(cloud, boundaries);
//    hf.compute();
//    hf.visualization();
//    return 0;
//}