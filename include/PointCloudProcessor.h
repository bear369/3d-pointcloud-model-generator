#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <string>
#include <memory>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudN;

class PointCloudProcessor
{
public:
    PointCloudProcessor();
    ~PointCloudProcessor();

    // Load point cloud from file
    PointCloud::Ptr loadPointCloud(const std::string& filepath);

    // Save point cloud to file
    bool savePointCloud(const PointCloud::Ptr cloud, const std::string& filepath);

    // Remove outliers using statistical method
    PointCloud::Ptr removeOutliers(PointCloud::Ptr cloud, int neighbors = 50, double std_ratio = 1.0);

    // Downsample point cloud using voxel grid
    PointCloud::Ptr downsampling(PointCloud::Ptr cloud, float leaf_size);

    // Fill missing regions in point cloud
    PointCloud::Ptr fillMissingRegions(PointCloud::Ptr cloud);

    // Estimate surface normals
    PointCloudN::Ptr estimateNormals(PointCloud::Ptr cloud, float radius = 0.05f);

    // Segment planar surfaces
    PointCloud::Ptr segmentPlane(PointCloud::Ptr cloud, float distance_threshold = 0.01f);

private:
    // Fill missing data using Moving Least Squares
    PointCloud::Ptr mlsFilling(PointCloud::Ptr cloud);
};