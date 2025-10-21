#include <iostream>
#include <chrono>
#include <random>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

#include "ivox3d/ivox3d.h"
#include "ivdb/ivdb.hpp"

// Use aligned allocator for PCL points to match iVox requirements
using AlignedPointVector = std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>;

// Function to generate a point cloud with obstacles for mapping simulation
AlignedPointVector generateMappingPointcloud(float space_size_x, float space_size_y, float space_size_z, float resolution, float obstacle_coverage = 0.3f) {
    AlignedPointVector points;
    
    // Calculate number of points per dimension based on resolution
    int num_x = static_cast<int>(space_size_x / resolution);
    int num_y = static_cast<int>(space_size_y / resolution);
    int num_z = static_cast<int>(space_size_z / resolution);
    
    // Ensure we have at least 1 point in each dimension
    num_x = std::max(1, num_x);
    num_y = std::max(1, num_y);
    num_z = std::max(1, num_z);
    
    // Calculate actual resolution used
    float actual_res_x = space_size_x / num_x;
    float actual_res_y = space_size_y / num_y;
    float actual_res_z = space_size_z / num_z;
    
    // Generate points with obstacle patterns for mapping
    for (int x = 0; x < num_x; ++x) {
        for (int y = 0; y < num_y; ++y) {
            for (int z = 0; z < num_z; ++z) {
                float rand_val = static_cast<float>(rand()) / RAND_MAX;
                
                // Place points with obstacle-style distribution
                if (rand_val < obstacle_coverage) {
                    pcl::PointXYZ pt;
                    pt.x = -space_size_x / 2.0f + x * actual_res_x;
                    pt.y = -space_size_y / 2.0f + y * actual_res_y;
                    pt.z = -space_size_z / 2.0f + z * actual_res_z;
                    points.push_back(pt);
                }
            }
        }
    }
    
    std::cout << "Generated mapping point cloud with obstacles with dimensions: " 
              << num_x << " x " << num_y << " x " << num_z 
              << " (total: " << points.size() << " points)" << std::endl;
    std::cout << "Actual resolution: " << actual_res_x << " x " << actual_res_y << " x " << actual_res_z << std::endl;
    
    return points;
}

// Function to generate random points
AlignedPointVector generateRandomPoints(int num_points, float range = 50.0f) {
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
    const double ivox_resolution = 0.5;
    const size_t ivox_capacity = 1000000;
    const double ivdb_resolution = 0.5; // root 大小为2^(2+3)*ivdb_resolution, 2,3为默认层级大小
    const size_t ivdb_capacity = 100;

    // Define mapping parameters
    const float space_size_x = 50.0f;  // 20m x 20m x 10m space
    const float space_size_y = 50.0f;
    const float space_size_z = 50.0f;

    // Define test point size
    const size_t num_query_points = 1000;

    // Generate test data   
    AlignedPointVector test_points = generateMappingPointcloud(space_size_x, space_size_y, space_size_z, 0.5f, 0.3f);
    
    std::cout << "Generating " << num_query_points << " query points..." << std::endl;
    AlignedPointVector query_points = generateRandomPoints(num_query_points, 100);

    // Test iVox
    std::cout << "\n--- Testing iVox ---" << std::endl;
    
    faster_lio::IVox<3> ivox({ivox_resolution, 1.0/ivox_resolution, faster_lio::IVox<3>::NearbyType::NEARBY18, ivox_capacity});
    
    // Measure insertion time for iVox
    auto start_time = std::chrono::high_resolution_clock::now();
    ivox.AddPoints(test_points);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto insertion_time_ivox = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "iVox insertion time: " << insertion_time_ivox.count() << " ms for " << test_points.size() << " points" << std::endl;
    std::cout << "iVox number of valid grids: " << ivox.NumValidGrids() << std::endl;

    // Measure nearest neighbor search for iVox
    start_time = std::chrono::high_resolution_clock::now();
    size_t found_count_ivox = 0;
    for (const auto& query_point : query_points) {
        AlignedPointVector closest_pts;
        if (ivox.GetClosestPoint(query_point, closest_pts, 5)) {
            found_count_ivox++;
            // std::cout << "found_count_ivox: " << found_count_ivox
            //           << " query_point: "<<  "[ " << query_point.x << ", " << query_point.y << ", " << query_point.z << " ]" << std::endl;
            // for (const auto &closest_pt : closest_pts)
            // {
            //     std::cout << "[ " << closest_pt.x << ", " << closest_pt.y << ", " << closest_pt.z << " ]" << std::endl;
            // }
        }
    }
    end_time = std::chrono::high_resolution_clock::now();
    auto search_time_ivox = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "iVox nearest neighbor search time: " << search_time_ivox.count() << " ms for " << num_query_points << " queries" << std::endl;
    std::cout << "iVox found closest points for: " << found_count_ivox << " / " << num_query_points << " queries" << std::endl;

    // Test Sparse Voxel Grid
    std::cout << "\n--- Testing Sparse Voxel Grid ---" << std::endl;

    IVdb ivdb({ivdb_resolution, 10, ivdb_capacity, IVdb<>::NearbyType::NEARBY26});

    // Measure insertion time for Sparse Voxel Grid
    start_time = std::chrono::high_resolution_clock::now();
    ivdb.AddPoints(test_points);
    end_time = std::chrono::high_resolution_clock::now();
    auto insertion_time_sparse = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "iVdb insertion time: " << insertion_time_sparse.count() << " ms for " << test_points.size() << " points" << std::endl;
    std::cout << "Sparse number of Pointcloud: " << ivdb.Pointcloud().size() << std::endl;
    // Measure nearest neighbor search for Sparse Voxel Grid
    start_time = std::chrono::high_resolution_clock::now();
    size_t found_count_sparse = 0;
    for (const auto& query_point : query_points) {
        AlignedPointVector closest_pts;
        if (ivdb.GetClosestPoint(query_point, closest_pts, 5)) {
            found_count_sparse++;
            // std::cout << "found_count_sparse: " << found_count_sparse
            //           << " query_point: "<<  "[ " << query_point.x << ", " << query_point.y << ", " << query_point.z << " ]" << std::endl;
            // for (const auto &closest_pt : closest_pts)
            // {
            //     std::cout << "[ " << closest_pt.x << ", " << closest_pt.y << ", " << closest_pt.z << " ]" << std::endl;
            // }
        }
    }
    end_time = std::chrono::high_resolution_clock::now();
    auto search_time_sparse = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "iVdb nearest neighbor search time: " << search_time_sparse.count() << " ms for " << num_query_points << " queries" << std::endl;
    std::cout << "iVdb found closest points for: " << found_count_sparse << " / " << num_query_points << " queries" << std::endl;

    // Summary
    std::cout << "\n--- Summary ---" << std::endl;
    std::cout << "Insertion Performance:" << std::endl;
    std::cout << "  iVox: " << insertion_time_ivox.count() << " ms" << std::endl;
    std::cout << "  iVdb: " << insertion_time_sparse.count() << " ms" << std::endl;
    
    std::cout << "\nNearest Neighbor Search Performance:" << std::endl;
    std::cout << "  iVox: " << search_time_ivox.count() << " ms" << std::endl;
    std::cout << "  iVdb: " << search_time_sparse.count() << " ms" << std::endl;

    // Compare efficiency
    std::cout << "\nEfficiency Comparison:" << std::endl;
    if (insertion_time_ivox.count() < insertion_time_sparse.count()) {
        std::cout << "  Insertion: iVox is " << (double)insertion_time_sparse.count() / insertion_time_ivox.count() << "x faster" << std::endl;
    } else {
        std::cout << "  Insertion: iVdb is " << (double)insertion_time_ivox.count() / insertion_time_sparse.count() << "x faster" << std::endl;
    }
    
    if (search_time_ivox.count() < search_time_sparse.count()) {
        std::cout << "  NN Search: iVox is " << (double)search_time_sparse.count() / search_time_ivox.count() << "x faster" << std::endl;
    } else {
        std::cout << "  NN Search: iVdb is " << (double)search_time_ivox.count() / search_time_sparse.count() << "x faster" << std::endl;
    }

    return 0;
}