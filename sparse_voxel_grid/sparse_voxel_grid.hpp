// straight copy from kinematic_icp, thanks Tiziano
// MIT License

// Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
// Stachniss.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once
#include <Eigen/Core>
#include <vector>
#include <list>
#include <unordered_map>
#include <memory>
#include <future>
#include <pcl/common/centroid.h>

#include "bonxai/bonxai.hpp"

namespace rko_lio::core {

using VoxelBlock = std::vector<Eigen::Vector3d>;
using PointType = pcl::PointXYZ;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

struct DistPoint {
    double dist_sq;
    const Eigen::Vector3d* point;

    bool operator<(const DistPoint& other) const { return dist_sq < other.dist_sq; }
};

class SparseVoxelGrid {
  public:
    explicit SparseVoxelGrid(const double voxel_size, const unsigned int max_points_per_voxel, const size_t capacity);
    ~SparseVoxelGrid() = default;
    
    void Clear() { 
      map_.clear(Bonxai::ClearOption::CLEAR_MEMORY); 
      lru_cache_.clear();
      lru_map_.clear();
    }
    bool Empty() const { return map_.activeCellsCount() == 0; }
    void AddPoints(const PointVector& points);
    std::vector<Eigen::Vector3d> Pointcloud() const;
    bool GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num = 5, double max_range = 5.0);

   private:
    double voxel_size_;
    unsigned int max_points_per_voxel_;
    size_t capacity_;
    size_t update_count;
    Bonxai::VoxelGrid<VoxelBlock> map_;
    
    std::list<Bonxai::CoordT> lru_cache_;
    std::unordered_map<Bonxai::CoordT, std::list<Bonxai::CoordT>::iterator> lru_map_;
    std::future<void> future_;
};

} // namespace rko_lio::core
