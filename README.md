# IVdb

IVdb参考[rko_lio](https://github.com/PRBonn/rko_lio)的sparse_voxel_grid，基于[Bonxai](https://github.com/facontidavide/Bonxai)g构建，接口和ivox一致，可以直接替换在faster-lio/fast-lio2中使用。

IVdb使用 `bonxai` 库（vdb）作为数据结构底层，实现了动态增加、删减点云和最近邻点的获取。

## 依赖项

- C++17
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Point Cloud Library (PCL)](https://pointclouds.org/)

## 使用

```bash
mkdir build
cd build
cmake ..
make
```

## 性能测试

主程序 `vox_comparison` ，主要对比了两种不同的体素（Voxel）地图数据结构在处理三维点云数据时的性能。对比的重点在于点云的**插入时间**和**最近邻搜索效率**。

在线性模式和NEARBY26下，测试结果如下：

> 因为点云为随机生成，每次测试结果都有点不一样

```bash
Performance comparison between iVox and Sparse Voxel Grid
Generated mapping point cloud with obstacles with dimensions: 100 x 100 x 100 (total: 299648 points)
Actual resolution: 0.5 x 0.5 x 0.5
Generating 1000 query points...

--- Testing iVox ---
iVox insertion time: 778 ms for 299648 points
iVox number of valid grids: 299648
iVox nearest neighbor search time: 23 ms for 1000 queries
iVox found closest points for: 14 / 1000 queries

--- Testing Sparse Voxel Grid ---
iVdb insertion time: 202 ms for 299648 points
Sparse number of Pointcloud: 299648
iVdb nearest neighbor search time: 1 ms for 1000 queries
iVdb found closest points for: 14 / 1000 queries

--- Summary ---
Insertion Performance:
  iVox: 778 ms
  iVdb: 202 ms

Nearest Neighbor Search Performance:
  iVox: 23 ms
  iVdb: 1 ms

Efficiency Comparison:
  Insertion: iVdb is 3.85149x faster
  NN Search: iVdb is 23x faster
```