#include "sparse_voxel_grid.hpp"

#include <Eigen/Core>
#include <queue>
#include <array>
#include <cstdint>

#include "bonxai/grid_coord.hpp"

namespace {
using Bonxai::CoordT;

constexpr std::array<CoordT, 125> shifts{
    // x = -2 plane
    // y = -2
    CoordT{.x = -2, .y = -2, .z = -2}, CoordT{.x = -2, .y = -2, .z = -1}, CoordT{.x = -2, .y = -2, .z = 0}, CoordT{.x = -2, .y = -2, .z = 1}, CoordT{.x = -2, .y = -2, .z = 2},
    // y = -1
    CoordT{.x = -2, .y = -1, .z = -2}, CoordT{.x = -2, .y = -1, .z = -1}, CoordT{.x = -2, .y = -1, .z = 0}, CoordT{.x = -2, .y = -1, .z = 1}, CoordT{.x = -2, .y = -1, .z = 2},
    // y = 0
    CoordT{.x = -2, .y = 0, .z = -2}, CoordT{.x = -2, .y = 0, .z = -1}, CoordT{.x = -2, .y = 0, .z = 0}, CoordT{.x = -2, .y = 0, .z = 1}, CoordT{.x = -2, .y = 0, .z = 2},
    // y = 1
    CoordT{.x = -2, .y = 1, .z = -2}, CoordT{.x = -2, .y = 1, .z = -1}, CoordT{.x = -2, .y = 1, .z = 0}, CoordT{.x = -2, .y = 1, .z = 1}, CoordT{.x = -2, .y = 1, .z = 2},
    // y = 2
    CoordT{.x = -2, .y = 2, .z = -2}, CoordT{.x = -2, .y = 2, .z = -1}, CoordT{.x = -2, .y = 2, .z = 0}, CoordT{.x = -2, .y = 2, .z = 1}, CoordT{.x = -2, .y = 2, .z = 2},

    // x = -1 plane
    // y = -2
    CoordT{.x = -1, .y = -2, .z = -2}, CoordT{.x = -1, .y = -2, .z = -1}, CoordT{.x = -1, .y = -2, .z = 0}, CoordT{.x = -1, .y = -2, .z = 1}, CoordT{.x = -1, .y = -2, .z = 2},
    // y = -1
    CoordT{.x = -1, .y = -1, .z = -2}, CoordT{.x = -1, .y = -1, .z = -1}, CoordT{.x = -1, .y = -1, .z = 0}, CoordT{.x = -1, .y = -1, .z = 1}, CoordT{.x = -1, .y = -1, .z = 2},
    // y = 0
    CoordT{.x = -1, .y = 0, .z = -2}, CoordT{.x = -1, .y = 0, .z = -1}, CoordT{.x = -1, .y = 0, .z = 0}, CoordT{.x = -1, .y = 0, .z = 1}, CoordT{.x = -1, .y = 0, .z = 2},
    // y = 1
    CoordT{.x = -1, .y = 1, .z = -2}, CoordT{.x = -1, .y = 1, .z = -1}, CoordT{.x = -1, .y = 1, .z = 0}, CoordT{.x = -1, .y = 1, .z = 1}, CoordT{.x = -1, .y = 1, .z = 2},
    // y = 2
    CoordT{.x = -1, .y = 2, .z = -2}, CoordT{.x = -1, .y = 2, .z = -1}, CoordT{.x = -1, .y = 2, .z = 0}, CoordT{.x = -1, .y = 2, .z = 1}, CoordT{.x = -1, .y = 2, .z = 2},

    // x = 0 plane (containing the center point (0, 0, 0))
    // y = -2
    CoordT{.x = 0, .y = -2, .z = -2}, CoordT{.x = 0, .y = -2, .z = -1}, CoordT{.x = 0, .y = -2, .z = 0}, CoordT{.x = 0, .y = -2, .z = 1}, CoordT{.x = 0, .y = -2, .z = 2},
    // y = -1
    CoordT{.x = 0, .y = -1, .z = -2}, CoordT{.x = 0, .y = -1, .z = -1}, CoordT{.x = 0, .y = -1, .z = 0}, CoordT{.x = 0, .y = -1, .z = 1}, CoordT{.x = 0, .y = -1, .z = 2},
    // y = 0
    CoordT{.x = 0, .y = 0, .z = -2}, CoordT{.x = 0, .y = 0, .z = -1}, CoordT{.x = 0, .y = 0, .z = 0}, CoordT{.x = 0, .y = 0, .z = 1}, CoordT{.x = 0, .y = 0, .z = 2},
    // y = 1
    CoordT{.x = 0, .y = 1, .z = -2}, CoordT{.x = 0, .y = 1, .z = -1}, CoordT{.x = 0, .y = 1, .z = 0}, CoordT{.x = 0, .y = 1, .z = 1}, CoordT{.x = 0, .y = 1, .z = 2},
    // y = 2
    CoordT{.x = 0, .y = 2, .z = -2}, CoordT{.x = 0, .y = 2, .z = -1}, CoordT{.x = 0, .y = 2, .z = 0}, CoordT{.x = 0, .y = 2, .z = 1}, CoordT{.x = 0, .y = 2, .z = 2},

    // x = 1 plane
    // y = -2
    CoordT{.x = 1, .y = -2, .z = -2}, CoordT{.x = 1, .y = -2, .z = -1}, CoordT{.x = 1, .y = -2, .z = 0}, CoordT{.x = 1, .y = -2, .z = 1}, CoordT{.x = 1, .y = -2, .z = 2},
    // y = -1
    CoordT{.x = 1, .y = -1, .z = -2}, CoordT{.x = 1, .y = -1, .z = -1}, CoordT{.x = 1, .y = -1, .z = 0}, CoordT{.x = 1, .y = -1, .z = 1}, CoordT{.x = 1, .y = -1, .z = 2},
    // y = 0
    CoordT{.x = 1, .y = 0, .z = -2}, CoordT{.x = 1, .y = 0, .z = -1}, CoordT{.x = 1, .y = 0, .z = 0}, CoordT{.x = 1, .y = 0, .z = 1}, CoordT{.x = 1, .y = 0, .z = 2},
    // y = 1
    CoordT{.x = 1, .y = 1, .z = -2}, CoordT{.x = 1, .y = 1, .z = -1}, CoordT{.x = 1, .y = 1, .z = 0}, CoordT{.x = 1, .y = 1, .z = 1}, CoordT{.x = 1, .y = 1, .z = 2},
    // y = 2
    CoordT{.x = 1, .y = 2, .z = -2}, CoordT{.x = 1, .y = 2, .z = -1}, CoordT{.x = 1, .y = 2, .z = 0}, CoordT{.x = 1, .y = 2, .z = 1}, CoordT{.x = 1, .y = 2, .z = 2},

    // x = 2 plane
    // y = -2
    CoordT{.x = 2, .y = -2, .z = -2}, CoordT{.x = 2, .y = -2, .z = -1}, CoordT{.x = 2, .y = -2, .z = 0}, CoordT{.x = 2, .y = -2, .z = 1}, CoordT{.x = 2, .y = -2, .z = 2},
    // y = -1
    CoordT{.x = 2, .y = -1, .z = -2}, CoordT{.x = 2, .y = -1, .z = -1}, CoordT{.x = 2, .y = -1, .z = 0}, CoordT{.x = 2, .y = -1, .z = 1}, CoordT{.x = 2, .y = -1, .z = 2},
    // y = 0
    CoordT{.x = 2, .y = 0, .z = -2}, CoordT{.x = 2, .y = 0, .z = -1}, CoordT{.x = 2, .y = 0, .z = 0}, CoordT{.x = 2, .y = 0, .z = 1}, CoordT{.x = 2, .y = 0, .z = 2},
    // y = 1
    CoordT{.x = 2, .y = 1, .z = -2}, CoordT{.x = 2, .y = 1, .z = -1}, CoordT{.x = 2, .y = 1, .z = 0}, CoordT{.x = 2, .y = 1, .z = 1}, CoordT{.x = 2, .y = 1, .z = 2},
    // y = 2
    CoordT{.x = 2, .y = 2, .z = -2}, CoordT{.x = 2, .y = 2, .z = -1}, CoordT{.x = 2, .y = 2, .z = 0}, CoordT{.x = 2, .y = 2, .z = 1}, CoordT{.x = 2, .y = 2, .z = 2}
};

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

// constexpr std::array<Bonxai::CoordT, 19> shifts{
//     CoordT{.x = 0, .y = 0, .z = 0}, CoordT{.x = -1, .y = 0, .z = 0}, CoordT{.x = 1, .y = 0, .z = 0}, CoordT{.x = 0, .y = 1, .z = 0},  
//     CoordT{.x = 0, .y = -1, .z = 0},  CoordT{.x = 0, .y = 0, .z = -1}, CoordT{.x = 0, .y = 0, .z = 1},  CoordT{.x = 1, .y = 1, .z = 0},  
//     CoordT{.x = -1, .y = 1, .z = 0},  CoordT{.x = 1, .y = -1, .z = 0}, CoordT{.x = -1, .y = -1, .z = 0},  CoordT{.x = 1, .y = 0, .z = 1}, 
//     CoordT{.x = -1, .y = 0, .z = 1},  CoordT{.x = 1, .y = 0, .z = -1}, CoordT{.x = -1, .y = 0, .z = -1},  CoordT{.x = 0, .y = 1, .z = 1},
//     CoordT{.x = 0, .y = -1, .z = 1},  CoordT{.x = 0, .y = 1, .z = -1}, CoordT{.x = 0, .y = -1, .z = -1} };

constexpr uint8_t inner_grid_log2_dim = 2;
constexpr uint8_t leaf_grid_log2_dim = 3;
} // namespace

IVdb::IVdb(const double voxel_size,
                                 const unsigned int max_points_per_voxel,
                                 const size_t capacity)
    : voxel_size_(voxel_size),
      max_points_per_voxel_(max_points_per_voxel),
      capacity_(capacity),
      map_(voxel_size, inner_grid_log2_dim, leaf_grid_log2_dim) {}

bool IVdb::GetClosestPoint(const PointType& point,
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
    std::sort(candidates.begin(), candidates.begin());

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

void IVdb::AddPoints(const PointVector &points)
{
  const double map_resolution = std::sqrt(voxel_size_ * voxel_size_ / max_points_per_voxel_);
  std::unordered_set<Bonxai::CoordT> voxel_coords;
  auto accessor = map_.createAccessor();
  std::for_each(points.cbegin(), points.cend(), [&](const PointType &point)
                {
    Eigen::Vector3d p(point.x, point.y, point.z);
    const auto voxel_coordinate = map_.posToCoord(p);
    VoxelBlock* voxel_points = accessor.value(voxel_coordinate, /*create_if_missing=*/true);
    if (voxel_points->size() == max_points_per_voxel_ ||
        std::any_of(voxel_points->cbegin(), voxel_points->cend(),
                    [&](const auto& voxel_point) { return (voxel_point - p).norm() < map_resolution; })) {
      return;
    }
    voxel_points->reserve(max_points_per_voxel_);
    voxel_points->emplace_back(p);
    voxel_coords.insert(map_.getRootKey(voxel_coordinate)); });

  UpdateLRU(voxel_coords);
}

// bool IVdb::GetClosestPoint(const PointType &point,
//                                       PointVector &closest_point, int max_num,
//                                       double max_range)
// {
//   Eigen::Vector3d pt(point.x, point.y, point.z);
//   const Bonxai::CoordT voxel = map_.posToCoord(pt);

//   auto custom_comp = [](const std::pair<double, VoxelBlock> &lhs, const std::pair<double, VoxelBlock> &rhs)
//   {
//     return lhs.first > rhs.first;
//   };

//   std::priority_queue<std::pair<double, VoxelBlock>, std::vector<std::pair<double, VoxelBlock>>, decltype(custom_comp)> candidates(custom_comp);
//   const double max_range_sq = max_range * max_range;

//   // Define the 3x3x3 root grid search space around the center
//   const int32_t root_step = 1 << 2^5;
//   Bonxai::CoordT center_root_key = map_.getRootKey(voxel);
//   const int32_t min_root_z = center_root_key.z - root_step;
//   const int32_t max_root_z = center_root_key.z + root_step;
//   const int32_t min_root_y = center_root_key.y - root_step;
//   const int32_t max_root_y = center_root_key.y + root_step;
//   const int32_t min_root_x = center_root_key.x - root_step;
//   const int32_t max_root_x = center_root_key.x + root_step;

//   for (int32_t z = min_root_z; z <= max_root_z; z += root_step)
//   {
//     for (int32_t y = min_root_y; y <= max_root_y; y += root_step)
//     {
//       for (int32_t x = min_root_x; x <= max_root_x; x += root_step)
//       {
//         CoordT root_key = {x, y, z};
//         auto root_it = map_.rootMap().find(root_key);
//         auto &inner_grid = root_it->second;
//         for (auto inner_it = inner_grid.mask().beginOn(); inner_it; ++inner_it)
//         {
//           const int32_t inner_index = *inner_it;
//           auto &leaf_grid = inner_grid.cell(inner_index);
//           if (!leaf_grid)
//             continue;
//           for (auto leaf_it = leaf_grid->mask().beginOn(); leaf_it; ++leaf_it)
//           {
//             const uint32_t leaf_index = *leaf_it;
//             const VoxelBlock &voxel_data = leaf_grid->cell(leaf_index);
//             double dist_sq = (voxel_data[0] - point.x) * (voxel_data[0] - point.x) +
//                              (voxel_data[1] - point.y) * (voxel_data[1] - point.y) +
//                              (voxel_data[2] - point.z) * (voxel_data[2] - point.z);
//             if (dist_sq <= max_range_sq)
//             {
//               if (candidates.size() < max_num)
//               {
//                 candidates.push({dist_sq, voxel_data});
//               }
//               else if (dist_sq < candidates.top().first)
//               {
//                 candidates.pop();
//                 candidates.push({dist_sq, voxel_data});
//               }
//             }
//           }
//         }
//       }
//     }
//   }

//   closest_point.clear();
//   closest_point.reserve(candidates.size());

//   while (!candidates.empty())
//   {
//     const auto &candidate = candidates.top();
//     PointType point;
//     point.x = candidate.second[0];
//     point.y = candidate.second[1];
//     point.z = candidate.second[2];
//     closest_point.push_back(point);
//     candidates.pop();
//   }

//   return true;
// }

// void IVdb::AddPoints(const PointVector &points)
// {
//   std::unordered_set<Bonxai::CoordT> voxel_coords;
//   auto accessor = map_.createAccessor();
//   std::for_each(points.cbegin(), points.cend(), [&](const PointType &point)
//                 {
//     Eigen::Vector3d p(point.x, point.y, point.z);
//     const auto voxel_coordinate = map_.posToCoord(p);
//     if(accessor.value(voxel_coordinate, false) == nullptr){
//       accessor.setValue(voxel_coordinate, p);
//     }
//     voxel_coords.insert(map_.getRootKey(voxel_coordinate)); });
//   // UpdateLRU(voxel_coords);
// }

void IVdb::UpdateLRU(const std::unordered_set<Bonxai::CoordT>& voxel_coords) {
    std::vector<Bonxai::CoordT> delete_voxel_coords;
    delete_voxel_coords.reserve(voxel_coords.size());
    std::for_each(voxel_coords.cbegin(), voxel_coords.cend(), [&](const Bonxai::CoordT& voxel_coord) {
        auto it = lru_map_.find(voxel_coord);
        if (it != lru_map_.end()) {
            // Move to front
            lru_cache_.splice(lru_cache_.begin(), lru_cache_, it->second);
        } else {
            // Add to front
            lru_cache_.push_front(voxel_coord);
            lru_map_[voxel_coord] = lru_cache_.begin();

            // Check for eviction
            if (lru_map_.size() > capacity_) {
                // Get key to evict
                const Bonxai::CoordT key_to_evict = lru_cache_.back();
                lru_cache_.pop_back();
                lru_map_.erase(key_to_evict);
                delete_voxel_coords.push_back(key_to_evict);
            }
        }
    });

    if (!delete_voxel_coords.empty()) {
        auto start_time = std::chrono::high_resolution_clock::now();
        std::for_each(delete_voxel_coords.cbegin(), delete_voxel_coords.cend(), [&](const Bonxai::CoordT& voxel_coord) {
            auto root_it = map_.rootMap().find(voxel_coord);
            if (root_it != map_.rootMap().end()) {
                auto& inner_grid = root_it->second;
                for (auto inner_it = inner_grid.mask().beginOn(); inner_it; ++inner_it) {
                    const int32_t inner_index = *inner_it;
                    auto& leaf_grid = inner_grid.cell(inner_index);
                    if (leaf_grid->mask().isOff()) {
                        inner_grid.mask().setOff(inner_index);
                        leaf_grid.reset();
                    }
                }
                map_.rootMap().erase(voxel_coord);
            }
        });
        auto end_time = std::chrono::high_resolution_clock::now();
        auto insertion_time_ivox2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::cout << "  UpdateLRU " << delete_voxel_coords.size() << " : "<< insertion_time_ivox2.count() << " ms" << std::endl;
    }
}

std::vector<Eigen::Vector3d> IVdb::Pointcloud() const
{
  std::vector<Eigen::Vector3d> point_cloud;
  point_cloud.reserve(map_.activeCellsCount() * max_points_per_voxel_);
  map_.forEachCell([&point_cloud, this](const VoxelBlock &block, const auto &)
                   { point_cloud.insert(point_cloud.end(), block.cbegin(), block.cend()); });
  std::cout << map_.memUsage() << std::endl;
  return point_cloud;
}
