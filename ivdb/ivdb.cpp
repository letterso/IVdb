#include "ivdb.hpp"

#include <Eigen/Core>
#include <queue>
#include <array>
#include <cstdint>

#include "bonxai/grid_coord.hpp"

template <typename PointType>
IVdb<PointType>::IVdb(Options options)
    : options_(options),
      map_(options_.voxel_size_, options_.inner_grid_log2_dim_, options_.leaf_grid_log2_dim_)
{
    GenerateNearbyGrids();
}

template <typename PointType>
void IVdb<PointType>::GenerateNearbyGrids()
{
    if (options_.nearby_type_ == NearbyType::NEARBY18)
    {
        nearby_grids_ = std::vector<Bonxai::CoordT>{
            Bonxai::CoordT{.x = 0, .y = 0, .z = 0}, Bonxai::CoordT{.x = -1, .y = 0, .z = 0}, Bonxai::CoordT{.x = 1, .y = 0, .z = 0}, Bonxai::CoordT{.x = 0, .y = 1, .z = 0},
            Bonxai::CoordT{.x = 0, .y = -1, .z = 0}, Bonxai::CoordT{.x = 0, .y = 0, .z = -1}, Bonxai::CoordT{.x = 0, .y = 0, .z = 1}, Bonxai::CoordT{.x = 1, .y = 1, .z = 0},
            Bonxai::CoordT{.x = -1, .y = 1, .z = 0}, Bonxai::CoordT{.x = 1, .y = -1, .z = 0}, Bonxai::CoordT{.x = -1, .y = -1, .z = 0}, Bonxai::CoordT{.x = 1, .y = 0, .z = 1},
            Bonxai::CoordT{.x = -1, .y = 0, .z = 1}, Bonxai::CoordT{.x = 1, .y = 0, .z = -1}, Bonxai::CoordT{.x = -1, .y = 0, .z = -1}, Bonxai::CoordT{.x = 0, .y = 1, .z = 1},
            Bonxai::CoordT{.x = 0, .y = -1, .z = 1}, Bonxai::CoordT{.x = 0, .y = 1, .z = -1}, Bonxai::CoordT{.x = 0, .y = -1, .z = -1}};
    }
    else if (options_.nearby_type_ == NearbyType::NEARBY26)
    {
        nearby_grids_ = std::vector<Bonxai::CoordT>{
            Bonxai::CoordT{.x = -1, .y = -1, .z = -1}, Bonxai::CoordT{.x = -1, .y = -1, .z = 0}, Bonxai::CoordT{.x = -1, .y = -1, .z = 1},
            Bonxai::CoordT{.x = -1, .y = 0, .z = -1}, Bonxai::CoordT{.x = -1, .y = 0, .z = 0}, Bonxai::CoordT{.x = -1, .y = 0, .z = 1},
            Bonxai::CoordT{.x = -1, .y = 1, .z = -1}, Bonxai::CoordT{.x = -1, .y = 1, .z = 0}, Bonxai::CoordT{.x = -1, .y = 1, .z = 1},

            Bonxai::CoordT{.x = 0, .y = -1, .z = -1}, Bonxai::CoordT{.x = 0, .y = -1, .z = 0}, Bonxai::CoordT{.x = 0, .y = -1, .z = 1},
            Bonxai::CoordT{.x = 0, .y = 0, .z = -1}, Bonxai::CoordT{.x = 0, .y = 0, .z = 0}, Bonxai::CoordT{.x = 0, .y = 0, .z = 1},
            Bonxai::CoordT{.x = 0, .y = 1, .z = -1}, Bonxai::CoordT{.x = 0, .y = 1, .z = 0}, Bonxai::CoordT{.x = 0, .y = 1, .z = 1},

            Bonxai::CoordT{.x = 1, .y = -1, .z = -1}, Bonxai::CoordT{.x = 1, .y = -1, .z = 0}, Bonxai::CoordT{.x = 1, .y = -1, .z = 1},
            Bonxai::CoordT{.x = 1, .y = 0, .z = -1}, Bonxai::CoordT{.x = 1, .y = 0, .z = 0}, Bonxai::CoordT{.x = 1, .y = 0, .z = 1},
            Bonxai::CoordT{.x = 1, .y = 1, .z = -1}, Bonxai::CoordT{.x = 1, .y = 1, .z = 0}, Bonxai::CoordT{.x = 1, .y = 1, .z = 1}};
    }
    else if (options_.nearby_type_ == NearbyType::NEARBY124)
    {
        nearby_grids_ = std::vector<Bonxai::CoordT>{
            Bonxai::CoordT{.x = -1, .y = -1, .z = -1}, Bonxai::CoordT{.x = -1, .y = -1, .z = 0}, Bonxai::CoordT{.x = -1, .y = -1, .z = 1},
            Bonxai::CoordT{.x = -1, .y = 0, .z = -1}, Bonxai::CoordT{.x = -1, .y = 0, .z = 0}, Bonxai::CoordT{.x = -1, .y = 0, .z = 1},
            Bonxai::CoordT{.x = -1, .y = 1, .z = -1}, Bonxai::CoordT{.x = -1, .y = 1, .z = 0}, Bonxai::CoordT{.x = -1, .y = 1, .z = 1},

            Bonxai::CoordT{.x = 0, .y = -1, .z = -1}, Bonxai::CoordT{.x = 0, .y = -1, .z = 0}, Bonxai::CoordT{.x = 0, .y = -1, .z = 1},
            Bonxai::CoordT{.x = 0, .y = 0, .z = -1}, Bonxai::CoordT{.x = 0, .y = 0, .z = 0}, Bonxai::CoordT{.x = 0, .y = 0, .z = 1},
            Bonxai::CoordT{.x = 0, .y = 1, .z = -1}, Bonxai::CoordT{.x = 0, .y = 1, .z = 0}, Bonxai::CoordT{.x = 0, .y = 1, .z = 1},

            Bonxai::CoordT{.x = 1, .y = -1, .z = -1}, Bonxai::CoordT{.x = 1, .y = -1, .z = 0}, Bonxai::CoordT{.x = 1, .y = -1, .z = 1},
            Bonxai::CoordT{.x = 1, .y = 0, .z = -1}, Bonxai::CoordT{.x = 1, .y = 0, .z = 0}, Bonxai::CoordT{.x = 1, .y = 0, .z = 1},
            Bonxai::CoordT{.x = 1, .y = 1, .z = -1}, Bonxai::CoordT{.x = 1, .y = 1, .z = 0}, Bonxai::CoordT{.x = 1, .y = 1, .z = 1}};

        nearby_grids_98_ = std::vector<Bonxai::CoordT>{
            Bonxai::CoordT{.x = -2, .y = -2, .z = -2}, Bonxai::CoordT{.x = -2, .y = -2, .z = -1}, Bonxai::CoordT{.x = -2, .y = -2, .z = 0}, Bonxai::CoordT{.x = -2, .y = -2, .z = 1}, Bonxai::CoordT{.x = -2, .y = -2, .z = 2},
            Bonxai::CoordT{.x = -2, .y = -1, .z = -2}, Bonxai::CoordT{.x = -2, .y = -1, .z = -1}, Bonxai::CoordT{.x = -2, .y = -1, .z = 0}, Bonxai::CoordT{.x = -2, .y = -1, .z = 1}, Bonxai::CoordT{.x = -2, .y = -1, .z = 2},
            Bonxai::CoordT{.x = -2, .y = 0, .z = -2}, Bonxai::CoordT{.x = -2, .y = 0, .z = -1}, Bonxai::CoordT{.x = -2, .y = 0, .z = 0}, Bonxai::CoordT{.x = -2, .y = 0, .z = 1}, Bonxai::CoordT{.x = -2, .y = 0, .z = 2},
            Bonxai::CoordT{.x = -2, .y = 1, .z = -2}, Bonxai::CoordT{.x = -2, .y = 1, .z = -1}, Bonxai::CoordT{.x = -2, .y = 1, .z = 0}, Bonxai::CoordT{.x = -2, .y = 1, .z = 1}, Bonxai::CoordT{.x = -2, .y = 1, .z = 2},
            Bonxai::CoordT{.x = -2, .y = 2, .z = -2}, Bonxai::CoordT{.x = -2, .y = 2, .z = -1}, Bonxai::CoordT{.x = -2, .y = 2, .z = 0}, Bonxai::CoordT{.x = -2, .y = 2, .z = 1}, Bonxai::CoordT{.x = -2, .y = 2, .z = 2},

            Bonxai::CoordT{.x = -1, .y = -2, .z = -2}, Bonxai::CoordT{.x = -1, .y = -2, .z = -1}, Bonxai::CoordT{.x = -1, .y = -2, .z = 0}, Bonxai::CoordT{.x = -1, .y = -2, .z = 1}, Bonxai::CoordT{.x = -1, .y = -2, .z = 2},
            Bonxai::CoordT{.x = -1, .y = -1, .z = -2}, Bonxai::CoordT{.x = -1, .y = -1, .z = 2},
            Bonxai::CoordT{.x = -1, .y = 0, .z = -2}, Bonxai::CoordT{.x = -1, .y = 0, .z = 2},
            Bonxai::CoordT{.x = -1, .y = 1, .z = -2}, Bonxai::CoordT{.x = -1, .y = 1, .z = 2},
            Bonxai::CoordT{.x = -1, .y = 2, .z = -2}, Bonxai::CoordT{.x = -1, .y = 2, .z = -1}, Bonxai::CoordT{.x = -1, .y = 2, .z = 0}, Bonxai::CoordT{.x = -1, .y = 2, .z = 1}, Bonxai::CoordT{.x = -1, .y = 2, .z = 2},

            Bonxai::CoordT{.x = 0, .y = -2, .z = -2}, Bonxai::CoordT{.x = 0, .y = -2, .z = -1}, Bonxai::CoordT{.x = 0, .y = -2, .z = 0}, Bonxai::CoordT{.x = 0, .y = -2, .z = 1}, Bonxai::CoordT{.x = 0, .y = -2, .z = 2},
            Bonxai::CoordT{.x = 0, .y = -1, .z = -2}, Bonxai::CoordT{.x = 0, .y = -1, .z = 2},
            Bonxai::CoordT{.x = 0, .y = 0, .z = -2}, Bonxai::CoordT{.x = 0, .y = 0, .z = 2},
            Bonxai::CoordT{.x = 0, .y = 1, .z = -2}, Bonxai::CoordT{.x = 0, .y = 1, .z = 2},
            Bonxai::CoordT{.x = 0, .y = 2, .z = -2}, Bonxai::CoordT{.x = 0, .y = 2, .z = -1}, Bonxai::CoordT{.x = 0, .y = 2, .z = 0}, Bonxai::CoordT{.x = 0, .y = 2, .z = 1}, Bonxai::CoordT{.x = 0, .y = 2, .z = 2},

            Bonxai::CoordT{.x = 1, .y = -2, .z = -2}, Bonxai::CoordT{.x = 1, .y = -2, .z = -1}, Bonxai::CoordT{.x = 1, .y = -2, .z = 0}, Bonxai::CoordT{.x = 1, .y = -2, .z = 1}, Bonxai::CoordT{.x = 1, .y = -2, .z = 2},
            Bonxai::CoordT{.x = 1, .y = -1, .z = -2}, Bonxai::CoordT{.x = 1, .y = -1, .z = 2},
            Bonxai::CoordT{.x = 1, .y = 0, .z = -2}, Bonxai::CoordT{.x = 1, .y = 0, .z = 2},
            Bonxai::CoordT{.x = 1, .y = 1, .z = -2}, Bonxai::CoordT{.x = 1, .y = 1, .z = 2},
            Bonxai::CoordT{.x = 1, .y = 2, .z = -2}, Bonxai::CoordT{.x = 1, .y = 2, .z = -1}, Bonxai::CoordT{.x = 1, .y = 2, .z = 0}, Bonxai::CoordT{.x = 1, .y = 2, .z = 1}, Bonxai::CoordT{.x = 1, .y = 2, .z = 2},

            Bonxai::CoordT{.x = 2, .y = -2, .z = -2}, Bonxai::CoordT{.x = 2, .y = -2, .z = -1}, Bonxai::CoordT{.x = 2, .y = -2, .z = 0}, Bonxai::CoordT{.x = 2, .y = -2, .z = 1}, Bonxai::CoordT{.x = 2, .y = -2, .z = 2},
            Bonxai::CoordT{.x = 2, .y = -1, .z = -2}, Bonxai::CoordT{.x = 2, .y = -1, .z = -1}, Bonxai::CoordT{.x = 2, .y = -1, .z = 0}, Bonxai::CoordT{.x = 2, .y = -1, .z = 1}, Bonxai::CoordT{.x = 2, .y = -1, .z = 2},
            Bonxai::CoordT{.x = 2, .y = 0, .z = -2}, Bonxai::CoordT{.x = 2, .y = 0, .z = -1}, Bonxai::CoordT{.x = 2, .y = 0, .z = 0}, Bonxai::CoordT{.x = 2, .y = 0, .z = 1}, Bonxai::CoordT{.x = 2, .y = 0, .z = 2},
            Bonxai::CoordT{.x = 2, .y = 1, .z = -2}, Bonxai::CoordT{.x = 2, .y = 1, .z = -1}, Bonxai::CoordT{.x = 2, .y = 1, .z = 0}, Bonxai::CoordT{.x = 2, .y = 1, .z = 1}, Bonxai::CoordT{.x = 2, .y = 1, .z = 2},
            Bonxai::CoordT{.x = 2, .y = 2, .z = -2}, Bonxai::CoordT{.x = 2, .y = 2, .z = -1}, Bonxai::CoordT{.x = 2, .y = 2, .z = 0}, Bonxai::CoordT{.x = 2, .y = 2, .z = 1}, Bonxai::CoordT{.x = 2, .y = 2, .z = 2}};
    }
    else
    {
        std::cout << "[ERROR] Unknown nearby_type!" << std::endl;
    }
}

template <typename PointType>
bool IVdb<PointType>::GetClosestPoint(const PointType &point,
                                      PointVector &closest_points, size_t max_num)
{
    std::vector<DistPoint> candidates;

    const auto const_accessor = map_.createConstAccessor();
    Eigen::Vector3d pt(point.x, point.y, point.z);
    const Bonxai::CoordT voxel = map_.posToCoord(pt);

    std::for_each(nearby_grids_.cbegin(), nearby_grids_.cend(), [&](const Bonxai::CoordT &voxel_shift)
                  {
        const Bonxai::CoordT query_voxel = voxel + voxel_shift;
        const VoxelBlock* voxel_points = const_accessor.value(query_voxel);
        if (voxel_points != nullptr) {
            for (const auto& point_in_voxel : *voxel_points) {
                double dist_sq = (point_in_voxel - pt).squaredNorm();
                candidates.push_back({dist_sq, &point_in_voxel});
            }
        } });

    if (options_.nearby_type_ == NearbyType::NEARBY124 && candidates.size() < max_num)
    {
        std::for_each(nearby_grids_98_.cbegin(), nearby_grids_98_.cend(), [&](const Bonxai::CoordT &voxel_shift)
                      {
        const Bonxai::CoordT query_voxel = voxel + voxel_shift;
        const VoxelBlock* voxel_points = const_accessor.value(query_voxel);
        if (voxel_points != nullptr) {
            for (const auto& point_in_voxel : *voxel_points) {
                double dist_sq = (point_in_voxel - pt).squaredNorm();
                candidates.push_back({dist_sq, &point_in_voxel});
            }
        } });
    }

    if (candidates.empty())
    {
        return false;
    }

    if (candidates.size() > max_num)
    {
        std::nth_element(candidates.begin(), candidates.begin() + max_num - 1, candidates.end());
        candidates.resize(max_num);
    }
    std::sort(candidates.begin(), candidates.end());

    closest_points.clear();
    closest_points.reserve(candidates.size());
    for (const auto &candidate : candidates)
    {
        PointType closest_point;
        closest_point.x = candidate.point->x();
        closest_point.y = candidate.point->y();
        closest_point.z = candidate.point->z();
        closest_points.push_back(closest_point);
    }

    return true;
}

template <typename PointType>
void IVdb<PointType>::AddPoints(const PointVector &points)
{
    const double map_resolution = std::sqrt(options_.voxel_size_ * options_.voxel_size_ / options_.max_points_per_voxel_);
    const double map_resolution_sq = map_resolution * map_resolution;
    std::unordered_set<Bonxai::CoordT> voxel_coords;
    auto accessor = map_.createAccessor();
    std::for_each(points.cbegin(), points.cend(), [&](const PointType &point)
                  {
    Eigen::Vector3d p(point.x, point.y, point.z);
    const auto voxel_coordinate = map_.posToCoord(p);
    VoxelBlock* voxel_points = accessor.value(voxel_coordinate, /*create_if_missing=*/true);
    if (voxel_points->size() == options_.max_points_per_voxel_ ||
        std::any_of(voxel_points->cbegin(), voxel_points->cend(),
                    [&](const auto& voxel_point) { return (voxel_point - p).squaredNorm() < map_resolution_sq; })) {
      return;
    }
    voxel_points->reserve(options_.max_points_per_voxel_);
    voxel_points->emplace_back(p);
    voxel_coords.insert(map_.getRootKey(voxel_coordinate)); });

    UpdateLRU(voxel_coords);
}

template <typename PointType>
void IVdb<PointType>::UpdateLRU(const std::unordered_set<Bonxai::CoordT> &voxel_coords)
{
    std::vector<Bonxai::CoordT> delete_voxel_coords;
    delete_voxel_coords.reserve(voxel_coords.size());
    std::for_each(voxel_coords.cbegin(), voxel_coords.cend(), [&](const Bonxai::CoordT &voxel_coord)
                  {
        auto it = lru_map_.find(voxel_coord);
        if (it != lru_map_.end()) {
            // Move to front
            lru_cache_.splice(lru_cache_.begin(), lru_cache_, it->second);
        } else {
            // Add to front
            lru_cache_.push_front(voxel_coord);
            lru_map_[voxel_coord] = lru_cache_.begin();

            // Check for eviction
            if (lru_map_.size() > options_.capacity_) {
                // Get key to evict
                const Bonxai::CoordT key_to_evict = lru_cache_.back();
                lru_cache_.pop_back();
                lru_map_.erase(key_to_evict);
                delete_voxel_coords.push_back(key_to_evict);
            }
        } });

    if (!delete_voxel_coords.empty())
    {
        std::for_each(delete_voxel_coords.cbegin(), delete_voxel_coords.cend(), [&](const Bonxai::CoordT &voxel_coord)
                      {
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
            } });
    }
}

template <typename PointType>
std::vector<Eigen::Vector3d> IVdb<PointType>::Pointcloud() const
{
    std::vector<Eigen::Vector3d> point_cloud;
    point_cloud.reserve(map_.activeCellsCount() * options_.max_points_per_voxel_);
    map_.forEachCell([&point_cloud, this](const VoxelBlock &block, const auto &)
                     { point_cloud.insert(point_cloud.end(), block.cbegin(), block.cend()); });
    return point_cloud;
}
