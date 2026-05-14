#include <iostream>
#include <fstream>
#include <chrono>
#include "PointCloudProcessor.h"
#include "ModelGenerator.h"

void printBanner()
{
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════╗\n";
    std::cout << "║   3D Point Cloud to Model Generator (PCL + OpenCV) ║\n";
    std::cout << "║          Copyright 2026 - All Rights Reserved      ║\n";
    std::cout << "╚════════════════════════════════════════════════════╝\n";
    std::cout << std::endl;
}

void printUsage()
{
    std::cout << "\nUsage Instructions:\n";
    std::cout << "1. Place point cloud file (*.pcd or *.ply) in ./input/ folder\n";
    std::cout << "2. Run this application\n";
    std::cout << "3. Generated 3D model will be saved in ./output/ folder\n\n";
}

int main(int argc, char** argv)
{
    printBanner();

    // Input/Output paths
    std::string inputPath = "input/pointcloud.pcd";
    std::string outputPath = "output/model.stl";

    // Allow command-line parameters
    if (argc >= 2) {
        inputPath = argv[1];
    }
    if (argc >= 3) {
        outputPath = argv[2];
    }

    std::cout << "[Config] Input:  " << inputPath << std::endl;
    std::cout << "[Config] Output: " << outputPath << std::endl;

    auto startTime = std::chrono::high_resolution_clock::now();

    try {
        // ========== 1. Load point cloud ==========
        std::cout << "\n[Step 1/6] Loading point cloud..." << std::endl;
        PointCloudProcessor processor;
        auto cloud = processor.loadPointCloud(inputPath);
        
        if (!cloud || cloud->empty()) {
            std::cerr << "ERROR: Failed to load point cloud or file is empty" << std::endl;
            printUsage();
            return -1;
        }
        std::cout << "SUCCESS: Loaded " << cloud->size() << " points" << std::endl;

        // ========== 2. Remove outliers ==========
        std::cout << "\n[Step 2/6] Removing outliers..." << std::endl;
        cloud = processor.removeOutliers(cloud, 50, 1.0);
        std::cout << "SUCCESS: Outlier removal completed" << std::endl;

        // ========== 3. Downsampling ==========
        std::cout << "\n[Step 3/6] Downsampling point cloud..." << std::endl;
        cloud = processor.downsampling(cloud, 0.01f);
        std::cout << "SUCCESS: Downsampling completed" << std::endl;

        // ========== 4. Fill missing data ==========
        std::cout << "\n[Step 4/6] Filling missing regions (handling occlusions)..." << std::endl;
        cloud = processor.fillMissingRegions(cloud);
        std::cout << "SUCCESS: Missing region filling completed" << std::endl;

        // ========== 5. Estimate normals ==========
        std::cout << "\n[Step 5/6] Estimating surface normals..." << std::endl;
        auto cloud_with_normals = processor.estimateNormals(cloud, 0.05f);
        std::cout << "SUCCESS: Normal estimation completed" << std::endl;

        // ========== 6. Surface reconstruction ==========
        std::cout << "\n[Step 6/6] Reconstructing 3D surface model..." << std::endl;
        ModelGenerator modelGen;
        auto mesh = modelGen.poissonReconstruction(cloud_with_normals, 10);

        if (!mesh) {
            std::cerr << "ERROR: Surface reconstruction failed" << std::endl;
            return -1;
        }

        // ========== Save model ==========
        if (!modelGen.saveMesh(mesh, outputPath)) {
            std::cerr << "ERROR: Failed to save mesh model" << std::endl;
            return -1;
        }

        // ========== Complete ==========
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime);

        std::cout << "\n╔════════════════════════════════════════════════════╗\n";
        std::cout << "║              PROCESS COMPLETED SUCCESSFULLY        ║\n";
        std::cout << "╚════════════════════════════════════════════════════╝\n";
        std::cout << "\nTotal processing time: " << duration.count() << " seconds\n";
        std::cout << std::endl;

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "\nFATAL ERROR: " << e.what() << std::endl;
        return -1;
    }
}