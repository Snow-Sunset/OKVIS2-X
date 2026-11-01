# LiDAR 点云地图与子地图保留

平面配准之外维护独立点云地图。点保存在子地图坐标系中，发布/查询时使用
最新锚点位姿变换到世界坐标。默认不写历史文件，不创建 raw spool 或 cloud chunks。

## 配置

在使用的 `se2-lidar*.yaml` 的 `general` 下配置：

```yaml
general:
  lidar_plane_only: true
  use_point_to_plane_factors: true
  write_mesh_output: false
  build_lidar_pointcloud_map: true
  pointcloud_map_voxel_size: 0.1
  pointcloud_map_publish_interval: 1.0
  max_submaps: 32
  lidar_spool_enabled: false
  pointcloud_persistent_chunks: false
  pointcloud_window_half_extent: 50.0
```

- `max_submaps`：点云和平面缓存共用的保留目标。点云最多保留 N 张有驻留点
  的子地图，包含活动图。
  超限时回收最近最少写入的旧图，整张释放；优化器位姿更新不刷新写入顺序。
  活动图不会因点数或字节数超过阈值而被截断。
  平面缓存最多保留 N 张已完成图，按最近访问回收。
  活动图不计入，当前参考图与查询中保护的图继续保留，可能临时超过目标。
- `max_submaps` 必须大于零，默认 32；数量回收始终生效。
  原有两个独立数量键不再读取。
  SE 配置中的 `memory_bounded` 已移除，旧键被忽略。
  旧 `pointcloud_memory_mb`、`plane_cache_memory_mb` 已移除且不会转换为图数。
- `pointcloud_map_voxel_size`：子地图内的固定体素宽度，单位米；每个体素
  保留距离中心最近的实测点，不平均跨表面的点。
- `pointcloud_window_half_extent`：RAM 模式下始终按世界坐标滑动的空间窗口。
  该空间过滤继续生效，数量限制并不保证窗口外的旧图一定保留满 N 张。
- `pointcloud_map_publish_interval`：最小墙钟发布间隔；`0` 表示每次变化均可发布。
- `build_lidar_pointcloud_map: false`：关闭点云存储、发布、导出，不关闭平面配准。

因此，子地图数确定了保留的历史范围，单张图的大小决定实际内存占用。
外部快照、因子、世界坐标输出、DDS 和 RViz 仍有各自的内存开销；此配置不保证
固定 MiB 或整进程 RSS 上限。活动平面图的窗口和 cell 数量限制仍然独立生效。

## 回环、显示与查询

源点保留子地图锚点关系，回环后可以分别修正仍在内存或磁盘档案中的片段。
优化器位姿优先于较旧的传感器包；单独修正位姿不会把已驱逐的 RAM 几何恢复。

RViz 话题 `/okvis/okvis_lidar_map` 为 `sensor_msgs/msg/PointCloud2`，坐标系 `world`。
每条消息完整发布当前保留的源点，变换到世界坐标，不做第二次显示抽样或点数截断。
重叠子地图在该显示路径可能包含重复点；`lidarPointCloudMap()` 查询和显式 PCD
导出则按固定世界体素融合。发布器保留最新消息，RViz 无需累计旧消息。
子地图回收会更新地图版本，后续快照不再包含被释放的旧图。

## 显式持久化与保存

`pointcloud_persistent_chunks: true` 时写入历史
点云和平面档案。点云超出驻留图数时先成功写盘再释放，历史仍可用于查询和导出；
平面缓存可重新加载旧图。点云磁盘写入按固定点数分块，与驻留图数独立。
不开启持久化时，驱逐的几何直接释放，无法恢复。

显式启用 `save_lidar_pointcloud_map` 或设置 `lidar_pointcloud_map_filename`
才导出 PCD。导出使用最新锚点和固定世界体素去重；RAM 模式导出当前保留几何，
持久化模式也包含历史 chunks。导出使用有界外部排序和原子重命名，临时文件
位于目标文件系统。进程退出后 RAM 数据释放，没有重启续跑入口。

## 验证

组件回归覆盖不同大小子地图的数量回收、活动图保护、不可变旧快照、回环位姿修正、
RAM 不落盘、持久化回收无损、磁盘失败不丢唯一副本及冷平面图重新加载。

```bash
ctest --test-dir <build-dir> -R '^okvis_(lidar_pointcloud_map|bounded_plane_map|plane_only_mapping|bounded_submap_history|lidar_map_publisher)$' --output-on-failure
```
