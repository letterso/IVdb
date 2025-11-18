# IVdb

IVdb参考[rko_lio](https://github.com/PRBonn/rko_lio)的sparse_voxel_grid，基于[Bonxai](https://github.com/facontidavide/Bonxai)构建，接口和ivox一致，可以直接替换在faster-lio/fast-lio2中使用，可参考[faster-lio-vdb](https://github.com/letterso/faster-lio-vdb)。

IVdb使用 `bonxai` 库（vdb）作为数据结构底层，实现了动态增加、删减点云和最近邻点的获取，详细说明可参考[ivdb](./doc/VDB.md)。

## 环境

- ubuntu 22.04
- C++17
- TBB
- Eigen
- Point Cloud Library (PCL)

## 使用

```bash
mkdir build
cd build
cmake ..
make
```