#ifndef IVDB_H
#define IVDB_H

#include <Eigen/Core>
#include <vector>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <pcl/common/centroid.h>

#include "bonxai/bonxai.hpp"

template <typename PointType = pcl::PointXYZ>
class IVdb
{
public:
  using VoxelBlock = std::vector<Eigen::Vector3d>;
  using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

  enum class NearbyType
  {
    NEARBY18,
    NEARBY26,
    NEARBY124,
  };

  struct Options
  {
    double voxel_size_ = 0.5;                       // vdb_leaf_grid分辨率
    unsigned int max_points_per_voxel_ = 10;        // vdb_leaf_grid内最大点数
    std::size_t capacity_ = 1000;                   // vdb_root_grid最大容量    
    NearbyType nearby_type_ = NearbyType::NEARBY26; // nearby range
    uint8_t inner_grid_log2_dim_ = 2;               // vdb_inner_grid数量
    uint8_t leaf_grid_log2_dim_ = 3;                 // vdb_leaf_grid数量
  };

  struct DistPoint
  {
    double dist_sq;
    const Eigen::Vector3d *point;

    bool operator<(const DistPoint &other) const { return dist_sq < other.dist_sq; }
  };

  explicit IVdb(Options options);
  ~IVdb() = default;

  void Clear()
  {
    map_.clear(Bonxai::ClearOption::CLEAR_MEMORY);
    lru_cache_.clear();
    lru_map_.clear();
  }
  bool Empty() const { return map_.activeCellsCount() == 0; }
  void AddPoints(const PointVector &points);
  std::vector<Eigen::Vector3d> Pointcloud() const;
  bool GetClosestPoint(const PointType &point, PointVector &closest_points, size_t max_num = 5);

private:
  Options options_;
  Bonxai::VoxelGrid<VoxelBlock> map_;
  std::vector<Bonxai::CoordT> nearby_grids_, nearby_grids_98_;

  std::list<Bonxai::CoordT> lru_cache_;
  std::unordered_map<Bonxai::CoordT, std::list<Bonxai::CoordT>::iterator> lru_map_;
  
  void GenerateNearbyGrids();
  void UpdateLRU(const std::unordered_set<Bonxai::CoordT> &voxel_coords);
};

template class IVdb<pcl::PointXYZ>;
template class IVdb<pcl::PointXYZINormal>;

#endif