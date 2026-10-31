# 全局可视化体素地图

LiDAR 平面配准之外维护独立的全局可视化地图。默认保留整个运行历史，统一到
世界坐标，在固定分辨率体素中融合重叠点，再完整发布到 `/okvis/okvis_lidar_map`。
不使用粗全局预览、自动增大体素、20 万点上限或显示抽点，不自动写历史文件。

## 配置

在使用的 `se2-lidar*.yaml` 的 `general` 下配置：

```yaml
general:
  lidar_plane_only: true
  use_point_to_plane_factors: true
  write_mesh_output: false
  build_lidar_pointcloud_map: true
  pointcloud_map_global: true
  pointcloud_map_voxel_size: 0.1
  pointcloud_map_publish_interval: 1.0
  memory_bounded: true
  lidar_spool_enabled: false
  pointcloud_persistent_chunks: false
  pointcloud_memory_mb: 96
  pointcloud_window_half_extent: 50.0
```

- `pointcloud_map_global` 默认 `true`：保留全历史，融合成一张世界坐标体素图。
  不因距离当前位置远或超过 `pointcloud_memory_mb` 而驱逐历史。
- `pointcloud_map_voxel_size`：固定体素宽度，单位米；默认 0.1 m。每个占用体素
  保留距离中心最近的实测点，不平均跨表面的点。跨子地图重叠体素也合并。
- `pointcloud_map_publish_interval`：两次发布的最小墙钟间隔，单位秒。`0` 表示
  每次变化都可发布；无变化不重复组装，正常退出强制发布最后快照。
- `pointcloud_memory_mb`：只约束可选局部显示模式或显式持久化 chunks 的驻留缓存。
  默认全局 RAM 历史不受该参数裁剪，内存随场景增长。
- `pointcloud_window_half_extent`：仅在 `pointcloud_map_global: false`、有界且无
  持久化 chunks 时使用，按世界坐标局部窗口释放历史。
- `build_lidar_pointcloud_map: false`：关闭全局可视化地图的存储、发布和导出，
  不关闭配准所用的局部平面地图。

`memory_bounded` 继续控制活动平面地图、平面热缓存和传感器队列；它不代表
全局可视化历史的固定内存上限。所带 Hilti/VBR LiDAR 示例显式开启全局模式。
旧 `pointcloud_map_max_publish_points` 参数不再生效。

原有近远距离过滤、去畸变和配准前采样仍生效。可视化地图接收建图线程处理的
有效端点，不受 `n_factors_per_state` 配准因子数限制；它不是原始雷达包的无损备份。

## 回环与地图融合

源几何在内部保留关键帧锚点关系，以便回环后分别修正历史片段。对外发布的是
一张融合后的全局图，不按子地图分开发送，也不在重访区域重复发布同体素点。

每次需要发布时，获取不可变源快照及最新锚定位姿，在地图锁外变换到世界坐标，
按固定体素重新融合。这样回环后的旧位置不会残留在一个只增不减的世界 hash 中。
优化器位姿优先于延迟传感器包；首个扫描之前到达的优化锚点也会保留。

局部平面配准地图仍可滑动和驱逐，与全局可视化历史的生命周期独立。
全局图的“固定体素融合”定义了地图本身的分辨率，融合后发布全部占用体素，
没有第二层显示抽样或自适应粗化。

## RViz 与内存

默认话题 `/okvis/okvis_lidar_map` 为 `sensor_msgs/msg/PointCloud2`，坐标系 `world`。
RViz 添加 PointCloud2，订阅该话题并将 Fixed Frame 设为 `world`。每条消息是
完整全局快照，RViz 不需要累计旧消息。发布器保留最后一条消息，迟加入订阅者
也能收到当前全图。

ROS 消息直接写入紧凑 XYZ float，12 字节/点；不再先构建完整 PCL 副本。
全局源几何、不可变快照、世界融合 hash、输出点数组、DDS 缓存、RViz 主存和
显存仍有各自开销。固定分辨率、全历史、不落盘模式无法维持固定 96 MiB 上限。
内部按锚点保留的重叠源几何也会占用内存，即使发布时已经融合成一个体素。

后台线程合并显示请求，不丢弃位姿更新。完整世界融合的工作量随保留源点增长，
不能沿用旧 20 万显示预算的性能数字。可增大发布间隔降低频率，单次仍发布全图。
目前没有真实长数据集的整进程峰值或实时性保证。

## 显式保存

默认不写 raw spool、cloud chunks 或 PCD。显式启用 `save_lidar_pointcloud_map`
或设置 `lidar_pointcloud_map_filename` 才导出 PCD；默认全局模式导出完整历史，
采用最新锚点并按相同的世界体素规则去重。导出使用有界外部排序和原子重命名，
临时文件放在目标 PCD 文件系统。正常运行不调用该导出路径。

全局 RAM 地图进程结束后释放，已有历史目录不会自动删除，也没有重启续跑入口。
`lidar_spool_enabled` 和 `pointcloud_persistent_chunks` 仍为显式离线选项，默认关闭。

## 验证

组件回归覆盖跨 50 m 窗口历史保留、超过旧点云预算、重访全局体素融合、回环
迁移后无旧位置残影、延迟位姿拒绝、首扫描前锚点修正、发布/查询/PCD 一致、
无历史文件创建及局部模式对照。ROS 测试覆盖 200001 点完整发布、紧凑 XYZ
布局、时间/坐标系及迟加入订阅者。

```bash
ctest --test-dir <build-dir> -R '^okvis_(global_voxel_mapping|lidar_pointcloud_map|plane_only_mapping|bounded_submap_history|lidar_map_publisher)$' --output-on-failure
```

后台发布线程及历史基准的边界见 [实时性能说明](lidar-map-realtime-performance.md)。
