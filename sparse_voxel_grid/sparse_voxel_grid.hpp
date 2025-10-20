#ifndef IVDB_H
#define IVDB_H

#include <Eigen/Core>
#include <vector>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <pcl/common/centroid.h>

#include "bonxai/bonxai.hpp"

using VoxelBlock = std::vector<Eigen::Vector3d>;
using PointType = pcl::PointXYZ;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

struct DistPoint {
    double dist_sq;
    const Eigen::Vector3d* point;

    bool operator<(const DistPoint& other) const { return dist_sq < other.dist_sq; }
};

class IVdb {
  public:
    explicit IVdb(const double voxel_size, const unsigned int max_points_per_voxel, const size_t capacity);
    ~IVdb() = default;
    
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
    Bonxai::VoxelGrid<VoxelBlock> map_;
    
    std::list<Bonxai::CoordT> lru_cache_;
    std::unordered_map<Bonxai::CoordT, std::list<Bonxai::CoordT>::iterator> lru_map_;

    void UpdateLRU(const std::unordered_set<Bonxai::CoordT> &voxel_coords);
};

#endif