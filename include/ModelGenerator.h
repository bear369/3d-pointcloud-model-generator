#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <memory>
#include <string>

typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudN;

class ModelGenerator
{
public:
    ModelGenerator();
    ~ModelGenerator();

    // Reconstruct surface using Poisson algorithm
    pcl::PolygonMesh::Ptr poissonReconstruction(PointCloudN::Ptr cloud, int depth = 10);

    // Reconstruct surface using Ball Pivoting algorithm
    pcl::PolygonMesh::Ptr ballPivoting(PointCloudN::Ptr cloud, float radius = 0.05f);

    // Save mesh to file
    bool saveMesh(const pcl::PolygonMesh::Ptr mesh, const std::string& filepath);

    // Print mesh statistics
    void printMeshStats(const pcl::PolygonMesh::Ptr mesh);

private:
    // Validate and repair mesh
    void validateMesh(pcl::PolygonMesh::Ptr mesh);
};