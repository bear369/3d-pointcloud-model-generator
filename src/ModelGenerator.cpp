#include "ModelGenerator.h"
#include <pcl/surface/poisson.h>
#include <pcl/surface/ball_pivoting.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <iostream>

ModelGenerator::ModelGenerator()
{
}

ModelGenerator::~ModelGenerator()
{
}

pcl::PolygonMesh::Ptr ModelGenerator::poissonReconstruction(PointCloudN::Ptr cloud, int depth)
{
    if (!cloud || cloud->empty()) {
        std::cerr << "Error: Empty point cloud" << std::endl;
        return nullptr;
    }

    std::cout << "\n[Surface Reconstruction] Starting Poisson reconstruction with depth=" << depth << std::endl;

    auto mesh = std::make_shared<pcl::PolygonMesh>();

    // Poisson surface reconstruction
    pcl::Poisson<PointNT> poisson;
    poisson.setInputCloud(cloud);
    poisson.setDepth(depth);
    poisson.setDegree(9);
    poisson.setManifold(false);
    poisson.setOutputPolygons(false);
    poisson.setIsoDivide(8);
    poisson.setConfidence(false);
    
    try {
        poisson.reconstruct(*mesh);
        std::cout << "Poisson reconstruction completed successfully" << std::endl;
        printMeshStats(mesh);
    }
    catch (const std::exception& e) {
        std::cerr << "Poisson reconstruction error: " << e.what() << std::endl;
        return nullptr;
    }

    return mesh;
}

pcl::PolygonMesh::Ptr ModelGenerator::ballPivoting(PointCloudN::Ptr cloud, float radius)
{
    if (!cloud || cloud->empty()) {
        std::cerr << "Error: Empty point cloud" << std::endl;
        return nullptr;
    }

    std::cout << "\n[Surface Reconstruction] Starting Ball Pivoting reconstruction with radius=" << radius << std::endl;

    auto mesh = std::make_shared<pcl::PolygonMesh>();

    try {
        // Ball Pivoting algorithm
        pcl::BallPivoting<PointNT> bpa;
        bpa.setInputCloud(cloud);
        bpa.setPivotingRadius(radius);
        bpa.reconstruct(*mesh);

        std::cout << "Ball Pivoting reconstruction completed successfully" << std::endl;
        printMeshStats(mesh);
    }
    catch (const std::exception& e) {
        std::cerr << "Ball Pivoting reconstruction error: " << e.what() << std::endl;
        return nullptr;
    }

    return mesh;
}

bool ModelGenerator::saveMesh(const pcl::PolygonMesh::Ptr mesh, const std::string& filepath)
{
    if (!mesh) {
        std::cerr << "Error: Null mesh pointer" << std::endl;
        return false;
    }

    try {
        if (filepath.find(".stl") != std::string::npos) {
            pcl::io::saveSTLFile(filepath, *mesh);
            std::cout << "\n[Output] Mesh saved (STL format): " << filepath << std::endl;
        }
        else if (filepath.find(".ply") != std::string::npos) {
            pcl::io::savePLYFile(filepath, *mesh);
            std::cout << "\n[Output] Mesh saved (PLY format): " << filepath << std::endl;
        }
        else if (filepath.find(".obj") != std::string::npos) {
            pcl::io::saveOBJFile(filepath, *mesh);
            std::cout << "\n[Output] Mesh saved (OBJ format): " << filepath << std::endl;
        }
        else if (filepath.find(".vtk") != std::string::npos) {
            pcl::io::saveVTKFile(filepath, *mesh);
            std::cout << "\n[Output] Mesh saved (VTK format): " << filepath << std::endl;
        }
        else {
            std::cerr << "Unsupported file format. Use .stl, .ply, .obj, or .vtk" << std::endl;
            return false;
        }
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error saving mesh: " << e.what() << std::endl;
        return false;
    }
}

void ModelGenerator::printMeshStats(const pcl::PolygonMesh::Ptr mesh)
{
    if (!mesh) return;
    
    std::cout << "\n--- Mesh Statistics ---" << std::endl;
    std::cout << "Vertices: " << mesh->cloud.width * mesh->cloud.height << std::endl;
    std::cout << "Polygons: " << mesh->polygons.size() << std::endl;
}

void ModelGenerator::validateMesh(pcl::PolygonMesh::Ptr mesh)
{
    // Can be extended: mesh validation and repair
}