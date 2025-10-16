#include "sparse_voxel_grid.hpp"

#include <Eigen/Core>
#include <array>
#include <cstdint>

#include "bonxai/grid_coord.hpp"

namespace {
using Bonxai::CoordT;

// constexpr std::array<Bonxai::CoordT, 27> shifts{
//     CoordT{.x = -1, .y = -1, .z = -1}, CoordT{.x = -1, .y = -1, .z = 0}, CoordT{.x = -1, .y = -1, .z = 1},
//     CoordT{.x = -1, .y = 0, .z = -1},  CoordT{.x = -1, .y = 0, .z = 0},  CoordT{.x = -1, .y = 0, .z = 1},
//     CoordT{.x = -1, .y = 1, .z = -1},  CoordT{.x = -1, .y = 1, .z = 0},  CoordT{.x = -1, .y = 1, .z = 1},

//     CoordT{.x = 0, .y = -1, .z = -1},  CoordT{.x = 0, .y = -1, .z = 0},  CoordT{.x = 0, .y = -1, .z = 1},
//     CoordT{.x = 0, .y = 0, .z = -1},   CoordT{.x = 0, .y = 0, .z = 0},   CoordT{.x = 0, .y = 0, .z = 1},
//     CoordT{.x = 0, .y = 1, .z = -1},   CoordT{.x = 0, .y = 1, .z = 0},   CoordT{.x = 0, .y = 1, .z = 1},

//     CoordT{.x = 1, .y = -1, .z = -1},  CoordT{.x = 1, .y = -1, .z = 0},  CoordT{.x = 1, .y = -1, .z = 1},
//     CoordT{.x = 1, .y = 0, .z = -1},   CoordT{.x = 1, .y = 0, .z = 0},   CoordT{.x = 1, .y = 0, .z = 1},
//     CoordT{.x = 1, .y = 1, .z = -1},   CoordT{.x = 1, .y = 1, .z = 0},   CoordT{.x = 1, .y = 1, .z = 1}};

constexpr std::array<Bonxai::CoordT, 19> shifts{
    CoordT{.x = 0, .y = 0, .z = 0}, CoordT{.x = -1, .y = 0, .z = 0}, CoordT{.x = 1, .y = 0, .z = 0}, CoordT{.x = 0, .y = 1, .z = 0},  
    CoordT{.x = 0, .y = -1, .z = 0},  CoordT{.x = 0, .y = 0, .z = -1}, CoordT{.x = 0, .y = 0, .z = 1},  CoordT{.x = 1, .y = 1, .z = 0},  
    CoordT{.x = -1, .y = 1, .z = 0},  CoordT{.x = 1, .y = -1, .z = 0}, CoordT{.x = -1, .y = -1, .z = 0},  CoordT{.x = 1, .y = 0, .z = 1}, 
    CoordT{.x = -1, .y = 0, .z = 1},  CoordT{.x = 1, .y = 0, .z = -1}, CoordT{.x = -1, .y = 0, .z = -1},  CoordT{.x = 0, .y = 1, .z = 1},
    CoordT{.x = 0, .y = -1, .z = 1},  CoordT{.x = 0, .y = 1, .z = -1}, CoordT{.x = 0, .y = -1, .z = -1} };

constexpr uint8_t inner_grid_log2_dim = 2;
constexpr uint8_t leaf_grid_log2_dim = 3;
} // namespace


namespace rko_lio::core {

SparseVoxelGrid::SparseVoxelGrid(const double voxel_size,
                                 const unsigned int max_points_per_voxel,
                                 const size_t capacity)
    : voxel_size_(voxel_size),
      max_points_per_voxel_(max_points_per_voxel),
      capacity_(capacity),
      map_(voxel_size, inner_grid_log2_dim, leaf_grid_log2_dim),
      accessor_(map_.createAccessor()) {}

// std::tuple<Eigen::Vector3d, double> SparseVoxelGrid::GetClosestNeighbor(const Eigen::Vector3d& query) const {
//   Eigen::Vector3d closest_neighbor = Eigen::Vector3d::Zero();
//   double closest_distance = std::numeric_limits<double>::max();
//   const auto const_accessor = map_.createConstAccessor();
//   const Bonxai::CoordT voxel = map_.posToCoord(query);
//   std::for_each(shifts.cbegin(), shifts.cend(), [&](const Bonxai::CoordT& voxel_shift) {
//     const Bonxai::CoordT query_voxel = voxel + voxel_shift;
//     const VoxelBlock* voxel_points = const_accessor.value(query_voxel);
//     if (voxel_points != nullptr) {
//       const Eigen::Vector3d& neighbor =
//           *std::min_element(voxel_points->cbegin(), voxel_points->cend(), [&](const auto& lhs, const auto& rhs) {
//             return (lhs - query).norm() < (rhs - query).norm();
//           });
//       double distance = (neighbor - query).norm();
//       if (distance < closest_distance) {
//         closest_neighbor = neighbor;
//         closest_distance = distance;
//       }
//     }
//   });
//   return std::make_tuple(closest_neighbor, closest_distance);
// }

// void SparseVoxelGrid::AddPoints(const std::vector<Eigen::Vector3d>& points) {
//   const double map_resolution = std::sqrt(voxel_size_ * voxel_size_ / max_points_per_voxel_);
//   std::for_each(points.cbegin(), points.cend(), [&](const Eigen::Vector3d& p) {
//     const auto voxel_coordinates = map_.posToCoord(p);
//     VoxelBlock* voxel_points = accessor_.value(voxel_coordinates, /*create_if_missing=*/true);
//     if (voxel_points->size() == max_points_per_voxel_ ||
//         std::any_of(voxel_points->cbegin(), voxel_points->cend(),
//                     [&](const auto& voxel_point) { return (voxel_point - p).norm() < map_resolution; })) {
//       return;
//     }
//     voxel_points->reserve(max_points_per_voxel_);
//     voxel_points->emplace_back(p);
//   });
// }

bool SparseVoxelGrid::GetClosestPoint(const PointType& point,
                                      PointVector& closest_point, int max_num,
                                      double max_range){
    std::vector<DistPoint> candidates;

    const auto const_accessor = map_.createConstAccessor();
    Eigen::Vector3d pt(point.x, point.y, point.z);
    const Bonxai::CoordT voxel = map_.posToCoord(pt);
    const double max_range_sq = max_range * max_range;

    std::for_each(shifts.cbegin(), shifts.cend(), [&](const Bonxai::CoordT& voxel_shift) {
        const Bonxai::CoordT query_voxel = voxel + voxel_shift;
        const VoxelBlock* voxel_points = const_accessor.value(query_voxel);
        if (voxel_points != nullptr) {
            for (const auto& point_in_voxel : *voxel_points) {
                double dist_sq = (point_in_voxel - pt).squaredNorm();
                if (dist_sq < max_range_sq) {
                    candidates.push_back({dist_sq, &point_in_voxel});
                }
            }
        }
    });

    if (candidates.empty()) {
        return false;
    }

    if (candidates.size() > static_cast<size_t>(max_num)) {
        std::nth_element(candidates.begin(), candidates.begin() + max_num - 1, candidates.end());
        candidates.resize(max_num);
    }
    std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

    closest_point.clear();
    closest_point.reserve(candidates.size());
    for (const auto& candidate : candidates) {
        PointType point;
        point.x = candidate.point->x();
        point.y = candidate.point->y();
        point.z = candidate.point->z();
        closest_point.push_back(point);
    }

    return true;
}

void SparseVoxelGrid::AddPoints(const PointVector& points) {
  std::cout<<"SparseVoxelGrid"<<std::endl;
  const double map_resolution = std::sqrt(voxel_size_ * voxel_size_ / max_points_per_voxel_);
  // const auto const_accessor = map_.createConstAccessor();
  std::for_each(points.cbegin(), points.cend(), [&](const PointType& point) {
    Eigen::Vector3d p(point.x, point.y, point.z);
    const auto voxel_coordinates = map_.posToCoord(p);

    // Update LRU cache
    // const VoxelBlock* voxel_points = const_accessor.value(query_voxel);
    // if (voxel_points != nullptr && lru_cache_.size() > capacity_) {
    //     lru_cache_.push_front(voxel_coordinates);
    //     // Get key to evict
    //     const Bonxai::CoordT key_to_evict = lru_cache_.back();
    //     lru_cache_.pop_back();
    //     // Remove from bonxai map
    //     auto& inner_grid = voxel_points->second;
    //     auto& leaf_grid = inner_grid.cell(inner_key);
    //     map_.rootMap().erase(key_to_evict);
    // }

    auto it = lru_map_.find(voxel_coordinates);
    if (it != lru_map_.end()) {
      // Move to front
      lru_cache_.splice(lru_cache_.begin(), lru_cache_, it->second);
    } else {
      // Add to front
      lru_cache_.push_front(voxel_coordinates);
      lru_map_[voxel_coordinates] = lru_cache_.begin();

      // Check for eviction
      if (lru_map_.size() > capacity_) {
        // Get key to evict
        const Bonxai::CoordT key_to_evict = lru_cache_.back();
        lru_cache_.pop_back();
        lru_map_.erase(key_to_evict);

        // Remove from bonxai map
        // auto& root_map = map_.rootMap();
        // auto& inner_grid = root_map.at(key_to_evict);
        // for (auto inner_it = inner_grid.mask().beginOn(); inner_it; ++inner_it) {
        //   const int32_t inner_index = *inner_it;
        //   auto& leaf_grid = inner_grid.cell(inner_index);
        //   inner_grid.mask().setOff(inner_index);
        //   leaf_grid.reset();
        // }
        map_.rootMap().erase(key_to_evict);
      }
    }

    VoxelBlock* voxel_points = accessor_.value(voxel_coordinates, /*create_if_missing=*/true);
    if (voxel_points->size() == max_points_per_voxel_ ||
        std::any_of(voxel_points->cbegin(), voxel_points->cend(),
                    [&](const auto& voxel_point) { return (voxel_point - p).norm() < map_resolution; })) {
      return;
    }
    voxel_points->reserve(max_points_per_voxel_);
    voxel_points->emplace_back(p);
  });
}

// void SparseVoxelGrid::RemovePointsFarFromLocation(const Eigen::Vector3d& origin) {
//   auto is_too_far_away = [&](const VoxelBlock& block) { return (block.front() - origin).norm() > clipping_distance_; };

//   std::vector<Bonxai::CoordT> keys_to_delete;
//   auto& root_map = map_.rootMap();
//   for (auto& [key, inner_grid] : root_map) {
//     for (auto inner_it = inner_grid.mask().beginOn(); inner_it; ++inner_it) {
//       const int32_t inner_index = *inner_it;
//       auto& leaf_grid = inner_grid.cell(inner_index);
//       const auto& voxel_block = leaf_grid->cell(leaf_grid->mask().findFirstOn());
//       if (is_too_far_away(voxel_block)) {
//         inner_grid.mask().setOff(inner_index);
//         leaf_grid.reset();
//       }
//     }
//     if (inner_grid.mask().isOff()) {
//       keys_to_delete.push_back(key);
//     }
//   }
//   for (const auto& key : keys_to_delete) {
//     root_map.erase(key);
//   }
// }

// void SparseVoxelGrid::Update(const std::vector<Eigen::Vector3d>& points, const Sophus::SE3d& pose) {
//   std::vector<Eigen::Vector3d> points_transformed(points.size());
//   std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
//                  [&](const auto& point) { return pose * point; });
//   const Eigen::Vector3d& origin = pose.translation();
//   AddPoints(points_transformed);
//   RemovePointsFarFromLocation(origin);
// }

std::vector<Eigen::Vector3d> SparseVoxelGrid::Pointcloud() const {
  std::vector<Eigen::Vector3d> point_cloud;
  point_cloud.reserve(map_.activeCellsCount() * max_points_per_voxel_);
  map_.forEachCell([&point_cloud, this](const VoxelBlock& block, const auto&) {
    point_cloud.insert(point_cloud.end(), block.cbegin(), block.cend());
  });
  return point_cloud;
}

} // namespace rko_lio::core
