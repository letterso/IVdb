#include <iostream>
#include <chrono>
#include <random>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

#include "ivox3d/ivox3d.h"
#include "sparse_voxel_grid/sparse_voxel_grid.hpp"

// Use aligned allocator for PCL points to match iVox requirements
using AlignedPointVector = std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>;

// Function to generate random points
AlignedPointVector generateRandomPoints(int num_points, float range = 1000.0f) {
    AlignedPointVector points;
    points.reserve(num_points);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-range, range);
    
    for (int i = 0; i < num_points; ++i) {
        pcl::PointXYZ pt;
        pt.x = dis(gen);
        pt.y = dis(gen);
        pt.z = dis(gen);
        points.push_back(pt);
    }
    
    return points;
}

int main() {
    std::cout << "Performance comparison between iVox and Sparse Voxel Grid" << std::endl;
    
    // Define common parameters for both structures
    const double resolution = 0.5;
    const size_t num_test_points = 100000;
    const size_t num_query_points = 1000;
    const size_t capacity = 100000;

    // Generate test data
    std::cout << "Generating " << num_test_points << " test points..." << std::endl;
    AlignedPointVector test_points = generateRandomPoints(num_test_points);
    
    std::cout << "Generating " << num_query_points << " query points..." << std::endl;
    AlignedPointVector query_points = generateRandomPoints(num_query_points);

    // Test iVox
    std::cout << "\n--- Testing iVox ---" << std::endl;
    
    faster_lio::IVox<3> ivox({resolution, 1.0/resolution, faster_lio::IVox<3>::NearbyType::NEARBY6, capacity});
    
    // Measure insertion time for iVox
    auto start_time = std::chrono::high_resolution_clock::now();
    ivox.AddPoints(test_points);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto insertion_time_ivox = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "iVox insertion time: " << insertion_time_ivox.count() << " ms for " << num_test_points << " points" << std::endl;
    std::cout << "iVox number of valid grids: " << ivox.NumValidGrids() << std::endl;

    // Measure nearest neighbor search for iVox
    start_time = std::chrono::high_resolution_clock::now();
    size_t found_count_ivox = 0;
    for (const auto& query_point : query_points) {
        AlignedPointVector closest_pts;
        if (ivox.GetClosestPoint(query_point, closest_pts, 5)) {
            found_count_ivox++;
        }
    }
    end_time = std::chrono::high_resolution_clock::now();
    auto search_time_ivox = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "iVox nearest neighbor search time: " << search_time_ivox.count() << " ms for " << num_query_points << " queries" << std::endl;
    std::cout << "iVox found closest points for: " << found_count_ivox << " / " << num_query_points << " queries" << std::endl;

    // Test Sparse Voxel Grid
    std::cout << "\n--- Testing Sparse Voxel Grid ---" << std::endl;
    
    rko_lio::core::SparseVoxelGrid sparse_grid(resolution, 10000, capacity); // max 10000 points per voxel
    
    // Measure insertion time for Sparse Voxel Grid
    start_time = std::chrono::high_resolution_clock::now();
    sparse_grid.AddPoints(test_points);
    end_time = std::chrono::high_resolution_clock::now();
    auto insertion_time_sparse = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "Sparse Voxel Grid insertion time: " << insertion_time_sparse.count() << " ms for " << num_test_points << " points" << std::endl;

    // Measure nearest neighbor search for Sparse Voxel Grid
    start_time = std::chrono::high_resolution_clock::now();
    size_t found_count_sparse = 0;
    for (const auto& query_point : query_points) {
        AlignedPointVector closest_pts;
        if (sparse_grid.GetClosestPoint(query_point, closest_pts, 5)) {
            found_count_sparse++;
        }
    }
    end_time = std::chrono::high_resolution_clock::now();
    auto search_time_sparse = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "Sparse Voxel Grid nearest neighbor search time: " << search_time_sparse.count() << " ms for " << num_query_points << " queries" << std::endl;
    std::cout << "Sparse Voxel Grid found closest points for: " << found_count_sparse << " / " << num_query_points << " queries" << std::endl;

    // Summary
    std::cout << "\n--- Summary ---" << std::endl;
    std::cout << "Insertion Performance:" << std::endl;
    std::cout << "  iVox: " << insertion_time_ivox.count() << " ms" << std::endl;
    std::cout << "  Sparse Voxel Grid: " << insertion_time_sparse.count() << " ms" << std::endl;
    
    std::cout << "\nNearest Neighbor Search Performance:" << std::endl;
    std::cout << "  iVox: " << search_time_ivox.count() << " ms" << std::endl;
    std::cout << "  Sparse Voxel Grid: " << search_time_sparse.count() << " ms" << std::endl;

    // Compare efficiency
    std::cout << "\nEfficiency Comparison:" << std::endl;
    if (insertion_time_ivox.count() < insertion_time_sparse.count()) {
        std::cout << "  Insertion: iVox is " << (double)insertion_time_sparse.count() / insertion_time_ivox.count() << "x faster" << std::endl;
    } else {
        std::cout << "  Insertion: Sparse Voxel Grid is " << (double)insertion_time_ivox.count() / insertion_time_sparse.count() << "x faster" << std::endl;
    }
    
    if (search_time_ivox.count() < search_time_sparse.count()) {
        std::cout << "  NN Search: iVox is " << (double)search_time_sparse.count() / search_time_ivox.count() << "x faster" << std::endl;
    } else {
        std::cout << "  NN Search: Sparse Voxel Grid is " << (double)search_time_ivox.count() / search_time_sparse.count() << "x faster" << std::endl;
    }

    return 0;
}