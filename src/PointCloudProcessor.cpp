#include "PointCloudProcessor.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

PointCloudProcessor::PointCloudProcessor()
{
}

PointCloudProcessor::~PointCloudProcessor()
{
}

PointCloud::Ptr PointCloudProcessor::loadPointCloud(const std::string& filepath)
{
    auto cloud = std::make_shared<PointCloud>();
    
    if (filepath.find(".pcd") != std::string::npos) {
        if (pcl::io::loadPCDFile<PointT>(filepath, *cloud) == -1) {
            PCL_ERROR("Couldn't read file %s\n", filepath.c_str());
            return nullptr;
        }
    }
    else if (filepath.find(".ply") != std::string::npos) {
        if (pcl::io::loadPLYFile<PointT>(filepath, *cloud) == -1) {
            PCL_ERROR("Couldn't read file %s\n", filepath.c_str());
            return nullptr;
        }
    }
    else {
        std::cerr << "Unsupported file format. Use .pcd or .ply" << std::endl;
        return nullptr;
    }

    return cloud;
}

bool PointCloudProcessor::savePointCloud(const PointCloud::Ptr cloud, const std::string& filepath)
{
    if (filepath.find(".pcd") != std::string::npos) {
        return pcl::io::savePCDFile(filepath, *cloud) == 0;
    }
    else if (filepath.find(".ply") != std::string::npos) {
        return pcl::io::savePLYFile(filepath, *cloud) == 0;
    }
    return false;
}

PointCloud::Ptr PointCloudProcessor::removeOutliers(PointCloud::Ptr cloud, int neighbors, double std_ratio)
{
    auto cloud_filtered = std::make_shared<PointCloud>();

    // Create statistical outlier removal filter
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(neighbors);
    sor.setStddevMulThresh(std_ratio);
    sor.filter(*cloud_filtered);

    std::cout << "Outlier removal: " << cloud->size() << " -> " << cloud_filtered->size() << std::endl;
    return cloud_filtered;
}

PointCloud::Ptr PointCloudProcessor::downsampling(PointCloud::Ptr cloud, float leaf_size)
{
    auto cloud_filtered = std::make_shared<PointCloud>();

    // Voxel grid filter
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*cloud_filtered);

    std::cout << "Downsampling: " << cloud->size() << " -> " << cloud_filtered->size() << std::endl;
    return cloud_filtered;
}

PointCloud::Ptr PointCloudProcessor::fillMissingRegions(PointCloud::Ptr cloud)
{
    // Use Moving Least Squares to fill missing data
    return mlsFilling(cloud);
}

PointCloud::Ptr PointCloudProcessor::mlsFilling(PointCloud::Ptr cloud)
{
    // Create KD-tree search structure
    auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
    tree->setInputCloud(cloud);

    // Use MLS for upsampling and filling
    pcl::MovingLeastSquares<PointT, PointT> mls;
    mls.setComputeNormals(false);
    mls.setInputCloud(cloud);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.05);
    
    // Upsampling parameters
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(0.03);
    mls.setUpsamplingStepSize(0.01);

    auto cloud_filled = std::make_shared<PointCloud>();
    mls.process(*cloud_filled);

    std::cout << "MLS Filling: " << cloud->size() << " -> " << cloud_filled->size() << " points" << std::endl;
    return cloud_filled;
}

PointCloudN::Ptr PointCloudProcessor::estimateNormals(PointCloud::Ptr cloud, float radius)
{
    auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
    auto cloud_with_normals = std::make_shared<PointCloudN>();

    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    // Merge points and normals
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    std::cout << "Normals estimated for " << cloud_with_normals->size() << " points" << std::endl;
    return cloud_with_normals;
}

PointCloud::Ptr PointCloudProcessor::segmentPlane(PointCloud::Ptr cloud, float distance_threshold)
{
    // Not implemented - can be extended
    return cloud;
}