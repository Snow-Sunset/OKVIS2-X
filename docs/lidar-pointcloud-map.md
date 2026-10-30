# 激光累计点云地图

在纯平面模式下新增独立的 XYZ 点云地图。它接收优化状态对应的去畸变激光端点，保留未匹配到平面的有效点；不会启用占据地图、占据场残差或 mesh。

参考 [FAST-LIVO2 的 `handleLIO()`](https://github.com/hku-mars/FAST-LIVO2/blob/main/src/LIVMapper.cpp) 中降采样、状态更新后变换入图、世界点云输出及 PCD 保存的流程。这里继续使用 OKVIS 的状态估计和子地图锚点，未移植 FAST-LIVO2 的滤波器。

## 运行配置

在使用的 `se2-lidar*.yaml` 的 `general` 下配置：

```yaml
general:
  lidar_plane_only: true
  use_point_to_plane_factors: true
  write_mesh_output: false
  build_lidar_pointcloud_map: true
  pointcloud_map_voxel_size: 0.1
  pointcloud_map_publish_interval: 1.0
  pointcloud_map_max_publish_points: 200000
```

- `pointcloud_map_voxel_size`：累计地图的采样体素宽度，单位米，与平面拟合的 `plane_voxel_size` 独立。每个体素保留距离体素中心最近的实测点，不平均跨表面的点。
- `pointcloud_map_publish_interval`：两次累计地图发布的最小墙钟时间间隔，单位秒。`0` 表示每次地图变化都可发布；地图未变化时不重复组装。结束时强制发布最终快照。
- `pointcloud_map_max_publish_points`：后台实时显示的采样点数预算，默认 20 万；`0` 表示不限制，完整 PCD 保存不受此预算影响。
- `build_lidar_pointcloud_map: false`：关闭累计点云地图的存储、发布和保存，不关闭已有点面约束。

原有近远距离筛选、去畸变和批次降采样仍生效；累计地图接收映射线程处理的全部有效端点，不受 `n_factors_per_state` 配准点数限制。映射队列积压时仍沿用现有的批次跳过策略，所以不是逐个原始点的无损录制。

## ROS2 查看

默认节点命名空间下，新增话题为 `/okvis/okvis_lidar_map`，消息类型 `sensor_msgs/msg/PointCloud2`，坐标系为 `world`。实际前缀随启动的 namespace/remap 改变。

在 RViz 中设置：

1. Fixed Frame：`world`。
2. 添加 PointCloud2，Topic 选择 `/okvis/okvis_lidar_map`。
3. Reliability：Reliable；Durability：Transient Local；Decay Time：`0`。

每条消息是覆盖累计地图范围的显示快照，超过点数预算时进行均匀抽样，RViz 无需累计历史消息。发布器保留最后一条消息，因此节点还在运行时，后启动的 RViz 也能收到地图。

`/okvis/okvis_points_alignment` 仍是实时/子地图配准点的调试输出，与累计地图独立。

## 保存 PCD

标准 ROS2 节点和网络节点新增参数：

```yaml
save_lidar_pointcloud_map: true
lidar_pointcloud_map_filename: ""
```

正常退出时默认保存到 `<csv_path>/lidar_map.pcd`；未设置 `csv_path` 时为 `/tmp/lidar_map.pcd`。可通过 `lidar_pointcloud_map_filename` 指定完整路径，父目录会自动创建。设 `save_lidar_pointcloud_map: false` 可关闭文件保存。

使用节点已有的 shutdown 服务正常退出，才会执行最终导出；强制结束进程不能保证保存。空地图不写文件。输出为二进制 PCD，仅包含 XYZ；目前的逐点输入接口没有提供强度和 RGB 信息。

离线应用可调用 `SubmappingInterface::saveLidarPointCloudMap(filename)` 导出同一份地图。

## 位姿修正与边界

地图按子地图局部坐标保存采样端点，组装快照时使用最新锚定位姿，并在世界坐标系再做一次体素去重。回环/最终优化的位姿回调先合并待应用的锚定位姿，由建图/快照消费者应用，避免阻塞状态发布线程，即使没有下一帧激光，最终保存也会反映修正。优化器提供的位姿优先于队列中较旧的传感器包，避免把修正后的地图移回旧位置。

状态回调只交接小量位姿，不获取点云存储锁。已完成子地图复用不可变点缓存，只有变化的子地图重建缓存；获取缓存引用和位姿后，全局变换、采样、去重与 ROS 消息转换在专用显示线程进行。显示请求可以合并，不丢弃位姿更新；退出时强制等待最终请求。完整 PCD 在锁外组装和写入。地图覆盖所有保留的子地图，当前未实现局部滑动窗口或全局内存上限。较长序列仍需关注存储规模；默认显示点数预算控制实时显示组装和传输的成本，完整 PCD 组装成本仍随全图增长。子地图内部没有逐点的非刚性重新建图。

## 验证

```bash
cmake -S . -B <build-dir> -DBUILD_POINT_TO_PLANE_TESTS=ON
cmake --build <build-dir> -j2
ctest --test-dir <build-dir> -R '^okvis_(point_to_plane|plane_only_mapping|lidar_pointcloud_map|lidar_map_publisher)$' --output-on-failure
```

新增检查覆盖：体素去重、负坐标、无效点、子地图重叠、锚点旋转平移、无新扫描时的位姿修正、过期扫描包、累计地图发布/节流/禁用、最终快照、PCD 写入及回读，以及 ROS2 的 XYZ/时间戳/坐标系和晚加入订阅者。合成测试不替代真实数据的地图质量与性能回放。

卡顿排查、合成性能对比及验证边界见 [实时性能修复说明](lidar-map-realtime-performance.md)。
