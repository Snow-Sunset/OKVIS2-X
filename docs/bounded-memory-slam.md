# 激光 SLAM 的 RAM 工作集与无历史落盘模式

当前在线运行不再创建视觉 `okvis-history-*` 会话目录，也不写 `.frame` 或
`.two_pose` 文件。`memory_bounded: true` 保留传感器队列和局部地图预算控制，
但视觉历史改为仅驻留内存：释放冷帧图像，保留长期回环需要的特征与图观测。
全局可视化体素图默认保留全历史，固定分辨率融合，不受局部点云预算/窗口裁剪。
**这不是整进程 1 GiB 硬上限；尚未通过实际长数据集的整进程内存验收。**

## 数据生命周期

| 数据 | RAM 中保留 | 默认磁盘行为 |
|---|---|---|
| 当前相机帧 | 跟踪、优化、CNN、可视化需要的图像 | 不归档 |
| 冷历史帧 | 尺寸、标定、ID、关键点、描述子、反投影、路标关联与分类结果 | 不归档；释放灰度和深度图 |
| 冻结视觉图边 | 线性化因子及恢复视觉约束需要的原始观测和路标 | 不归档 |
| IMU | 传播、预积分和异步去畸变需要的短滚动窗口 | 不归档 |
| 激光输入 | 有界 FIFO，满载时背压 | 默认不创建 raw spool |
| 活动平面地图 | 滑动窗口内的有界体素；稳定后释放拟合样本 | 默认不落盘 |
| 已完成平面地图 | 受预算约束的 RAM 缓存，保护当前参考地图 | 默认驱逐后释放；持久化 chunks 开启时才保存平面文件 |
| 可视化点云 | 全局历史源几何及固定世界体素融合结果；内存随场景增长 | 默认不写 chunks 或 PCD |
| 轨迹、回环索引、稀疏图 | 长期位姿关系和查询索引 | 显式轨迹导出仍可用 |

释放图像只在前端安全点进行。活动 IMU/关键帧/回环窗口内的帧、外部
`shared_ptr` 持有的帧及尚在分类的帧受到保护。冷帧进入活动窗口后仍可用
驻留特征完成匹配和视觉约束恢复，但已经释放的图像不能重新加载。
重叠率和跟踪质量计算使用保留的原始图像尺寸，不依赖历史像素。

历史描述子、关键点、图观测、DBoW 索引和稀疏图仍随序列增长。取消落盘
减少磁盘 I/O 与历史图像占用，但不能据此认为剩余历史数据具有固定内存上限。

## 配置

VIO 配置（例如 `config/hilti22/okvis2-lidar.yaml`）：

```yaml
estimator_parameters:
  memory_bounded: true
  do_loop_closures: true
```

估计器已移除 `memory_archive_directory` 和 `frame_cache_mb` 参数，旧 YAML 中
这两个键不会重新开启归档。`memory_bounded: false` 保留原有全内存图像生命周期，
同样不会自动启用历史归档，但不执行冷图像释放和相应队列限制。

SE 激光配置（例如 `config/hilti22/se2-lidar.yaml`）：

```yaml
general:
  memory_bounded: true
  memory_archive_directory: "./okvis-history" # 仅显式开启激光持久化时使用
  pointcloud_memory_mb: 96
  plane_cache_memory_mb: 32
  plane_active_max_cells: 8192
  plane_window_half_extent: 50.0
  lidar_queue_max_points: 200000
  lidar_spool_enabled: false
  pointcloud_persistent_chunks: false
  pointcloud_window_half_extent: 50.0
  mapping_queue_max_frames: 4
  state_queue_max_updates: 32
  use_point_to_plane_factors: true
  lidar_plane_only: true
  build_lidar_pointcloud_map: true
  pointcloud_map_global: true
```

RViz 默认发布完整全局体素地图。`pointcloud_map_global: true` 保留全历史，
按固定 `pointcloud_map_voxel_size` 融合重叠体素，不自动粗化或截断。局部平面配准
继续滑动。改成 `false` 才恢复局部点云窗口与预算驱逐。

两个 `memory_bounded` 分别控制估计器与激光建图。激光的
`memory_archive_directory` 仅在显式启用 spool/chunks 时生效；默认不创建
`okvis-lidar-*` 目录。`pointcloud_memory_mb` 仅控制可选局部点云模式或显式
持久化缓存。默认全局 RAM 图不受此预算限制，不能将它计入固定内存承诺。`plane_cache_memory_mb` 控制完成地图的缓存预算，
活动/当前参考及因子持有的地图另计，不等同于分配器 RSS。

输入和映射包队列采用背压；状态和对齐更新超预算会明确报错，不能静默
丢弃历史位姿修正或约束。ROS 轨迹显示默认最多 10000 点，视觉路标显示
默认最多 100000 点，可通过 `path_max_points` 和 `landmark_max_points` 调整。

## 可选激光导出与退出

`lidar_spool_enabled: true` 才保留输入测量序列。当前 ROS 输入接口不传强度，
不能把 spool 当成完整原始传感器包备份。`pointcloud_persistent_chunks: true`
才写入历史点云块和平面档案，归档目录应位于实际 SSD/eMMC，不能使用内存盘。

有界模式默认不导出 PCD。显式启用 `save_lidar_pointcloud_map` 后，仅导出
默认全局 RAM 图的完整历史；可选局部模式仅包含当前保留区域。PCD 采用
有界外部排序、归并及全局体素去重，临时文件位于目标 PCD 文件系统。
回环更新子地图锚点，无需重写历史局部 XYZ 块。

正常退出先排空优化和位姿发布，再完成剩余可传播测量及建图。无历史落盘
模式不会额外写视觉归档。有界模式跳过全历史 final BA，保留在线回环、
稀疏图求解和轨迹输出。已有磁盘历史文件不会自动删除，也没有进程重启续跑入口。

## 验证与限制

RAM 模式组件回归覆盖：冷灰度/深度图释放、外部共享引用固定、活动帧保护、
历史描述子和反投影保留、图像释放前后重叠率一致，以及无文件/目录创建。
旧显式归档库接口仍保留供兼容与独立测试使用，在线 `ThreadedSlam` 不调用它们。

此前点云导出压力测试和无输入启动内存测量属于旧分页版本，不能证明本轮
RAM-only 历史模式的运行峰值。最终验收需要长数据集持续测量 RSS/PSS、
回环及退出峰值，并比较 ATE/RPE、回环召回、有效激光因子和处理时延。
神经网络分支需要在启用 NN 的环境单独验证。
