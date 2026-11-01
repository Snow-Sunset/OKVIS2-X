# 独立全局激光彩色 PCD 地图

## 功能与接口

本模块采用“关键帧扫描选择 → 逐点去畸变 → 世界坐标合并 → 体素采样 → PCD 保存”
流程。彩色按本次确认采用**世界坐标高度伪彩色**，不是相机真彩色；固定高度范围从
蓝、青、绿、黄渐变到红。PCD 和 RViz 共用同一调色函数，闭环修正改变世界高度后
会重新计算颜色。

`okvis::GlobalLidarPcdMapper` 是可选旁路模块。C++ 接口默认关闭，但提供的
`okvis2x_node_subscriber.launch.xml` 在 `lidar:=true` 分支默认传入开启参数；
其他启动分支保持原样。可用 `global_lidar_pcd:=false` 显式关闭：

- 单独订阅 `/okvis/lidar`，沿用当前节点的 topic remap 与 Ouster 时间参考参数。
  订阅和地图读者匹配事件使用不自动加入主节点 executor 的独立 callback group，
  由专用接收线程处理；不会在 SLAM 默认回调队列中取出/反序列化第二份大点云。
  `input_reliability: auto` 根据 remap 后话题的发布端 QoS 选择 Reliable 或 Best Effort，
  旁路建图线程每秒重新检查，适配晚启动的 bag/驱动；DDS depth 始终为 1。
  也可在启动前显式指定 `reliable` / `best_effort`。这只改变新模块的订阅，
  不修改现有估计器的输入或关键帧判断；切换 QoS 不清空已经接收的扫描和地图。
  订阅回调保留有界 FIFO，最多 8 帧、总载荷 32 MiB；已经取走的活动扫描不会被
  后来的扫描替换。短时优化延迟不再把关键帧需要的等待扫描覆盖成最新一帧。
- 现有优化图回调额外传入不可变状态更新的共享引用。前后端和 SubmappingInterface
  不调用新模块的几何处理、点云合并或磁盘操作，新模块也不访问其可修改内部图。
- 独立建图线程拥有点云解码、短时 IMU 支持、轨迹查询、去畸变、归档和导出。
  另一个显示线程从不可变快照组装全局消息和调用 DDS 发布；显示慢不阻塞地图合并。
  两线程之间只有一个最新等待快照，不排队历史全局点云。Linux 上降低这两个旁路
  线程的 nice 优先级，保留现有 SLAM 线程调度设置。
- 原始扫描 TF 使用第三个轻量线程，独立于归档读取、地图着色和 DDS 地图发布。
  前端只排入最多 64 个小位姿，保留传感器时间顺序；不保存额外原始扫描载荷。
- 使用已有逐点时间戳解码与 `Trajectory::getStatePosePrefix()`，按每批最多 4096 点
  查询运动位姿，应用完整 `T_SL` 旋转和平移。同一发射时刻的多线点共用一次位姿查询和
  LiDAR 到关键帧的组合变换，不为每条线复制一个 4×4 位姿；跨批次的相同时间点也保留。
- **只合并关键帧扫描，每个关键帧最多一帧激光扫描。** 在扫描逐点时间范围及两端
  默认 0.05 秒的容差内，寻找尚未合并的真实 SLAM 关键帧；多个候选时选择距扫描
  中点最近的一个，等距时选较早帧。扫描中点落在普通帧上不会否决扫描覆盖的关键帧。
  关键帧依据现有 `State::isKeyframe`，不另外按激光请求或定时器创建。
  只有 header 时间的扫描以 header 时间及相同容差匹配。扫描先于优化状态到达时，
  等到整个匹配时间窗口的状态到达再决定；无邻近关键帧或关键帧已合并时跳过。
  没有匹配扫描的关键帧不会生成空点云。
  在完整解码前先检查时间字段和坐标有限性，等待相同的状态时间窗口；没有未合并关键帧
  时直接释放原始扫描，不分配测量数组、不排序、不去畸变。支持 Ouster 的三种时间参考、
  Hesai、BLK 和仅 header 时间，处理大小端与行 padding。无效 XYZ 的异常时间不会阻塞窗口。
- 点保存在关键帧局部坐标系；优化回调修正锚点后，历史地图随之更新。正常关闭时在
  最终 BA 和优化发布完成后再排空新模块，导出的世界坐标使用最终可用锚点位姿。
  闭环和最终优化只修正已有点，不会重复合并同一个关键帧的扫描。

未到达的位姿/IMU 支持会等待；已经过期或退出时仍不足的数据会计数并明确记录。
不会把旧时刻位姿套到当前扫描。只有 header 时间、没有逐点时间字段的输入只能使用
该 header 时间，无法恢复传感器未提供的扫描内运动。

### 给 SLAM 留出处理资源

新增 `global_lidar_pcd.worker_cpu_budget: 0.35`，启动时读取。建图、显示两个线程分别
以单核 35% CPU 时间占用为软目标，在 4096 点批次和其他工作边界主动短暂让出执行；
Linux 使用线程 CPU 时钟，不把磁盘/DDS 等待时间算成计算。原始扫描 TF 不受节流。
`1.0` 关闭该节奏控制，合法范围 `(0, 1]`。它不改变体素分辨率、显示采样或点数，
但较小值可能延长地图更新延迟；吞吐不足时仍可能触发原有的有界扫描队列丢弃。
这是合作式预算，不能限制 DDS 内部线程、RViz/GPU、内存带宽或磁盘 I/O，也不是实时调度保证。

建图线程最多每 100 ms 生成一份变化后的显示快照，不再为随后被覆盖的中间状态反复复制
历史索引。增量发布缓存候选计划，地图版本、采样或读者未改变时不重新遍历和排序历史；
补齐静态历史的后续批次也复用该计划。大扫描释放、失败退出时状态引用的释放移出共享锁，
避免 allocator 工作延长优化状态入队等待。旁路失败还会关闭自己的 DDS 扫描读者。

这些调整没有改动估计器的输入 QoS、关键帧策略、优化器或局部建图参数。
需要同版本回放对比来确认实际负载和失跟是否改善，不能仅凭代码推断漂移已经解决。

### 二倍速下的实时激光时间覆盖

旁路降载后仍观察到二倍速后端反馈滞后及实时扫描时间缺口，后续另行修改了
主估计器实时 LiDAR 缓存和跟踪反馈时序。该缓存独立于本模块，也不改变局部建图缓存。
`config/vbr/okvis2-lidar-driving.yaml` 的 `estimator_parameters` 新增可选参数：

```yaml
realtime_lidar_buffer_scans: 16
realtime_lidar_buffer_bytes: 67108864
```

等待扫描数量上限为 16，活动扫描与等待扫描的连续数组载荷合计上限为 64 MiB。
超限时保留靠近后端时间的旧等待扫描，替换最远未来等待扫描；消费者跳过过旧完整扫描，
只取最近完整扫描及后继已就绪前缀，不逐幅追赶积压。该预算不包含 ROS 原始消息、
临时复制、消费者工作集或整个进程的其他内存。旧配置缺省时使用相同默认值。
这两个字段不是 `global_lidar_pcd` ROS 参数，不控制 PCD 地图或显示点数。

本轮保持地图密度、显示采样、真实关键帧规则及优化器精度相关配置；新增发布等待计时
和激光时间范围诊断以支持后续回放验证。详见 `docs/campus-train0-replay-drift-analysis.md`。
源码及测试已修改，按要求未编译、运行 C++ 测试或启动回放，尚无二倍速性能改善结论。

后续二倍速复测发现日志仍运行旧的单等待扫描实现，当前工作区可执行文件也没有新缓存
标记。已新增启动日志 `Live LiDAR history v2`（打印有效预算）及
`Backend map matching v2`（打印匹配候选规模），重新构建并启动后应先核对这些标记。
同时为后端地图匹配减少观测位姿计算与候选深拷贝，新增细分计时；这不改变全局 PCD
线程、关键帧规则或显示采样。版本核验、失跟后恢复开销和手动更新命令详见上述分析文档。

## 启动与保存

首次接入需要重新编译并 source 安装目录。原有 `lidar:=true` launch 命令现在会默认
启用全局 PCD 地图，也可以显式指定开启及输出文件：

```bash
global_lidar_pcd:=true \
global_lidar_pcd_file:=/tmp/okvis-global-rgb.pcd
```

例如原来的命令形式为：

```bash
ros2 launch okvis okvis2x_node_subscriber.launch.xml \
  lidar:=true \
  config_filename:=/absolute/path/to/okvis2-lidar.yaml \
  se_config_filename:=/absolute/path/to/se2-lidar.yaml \
  global_lidar_pcd:=true \
  global_lidar_pcd_file:=/tmp/okvis-global-rgb.pcd
```

`global_lidar_pcd_file` 留空时，默认写入：

```text
<csv_path>/global_lidar_pcd/lidar-cloud-<unique-session>/global_lidar_map_rgb.pcd
```

启动日志打印实际路径。每次会话的归档目录独立；用户指定同名 PCD 输出时，成功导出
通过原子 rename 替换该文件，失败不会用半个 PCD 覆盖之前的完整文件。

可以通过现有正常关闭服务结束采集并等待最终 BA、地图排空与保存：

```bash
ros2 service call /okvis/shutdown std_srvs/srv/SetBool '{data: true}'
```

也支持现有 SIGINT 正常退出流程。`kill -9` 无法执行最后的导出和尾部排空；已经写入的
分块档案仍保留，但未落盘尾部和最后的位姿更新不能保证保留。

通过 `ros2 run` 启动时，可使用新增的 `config/global_lidar_pcd.yaml` 作为 ROS 参数文件，
再传入现有的 `config_filename` 和 `se_config_filename`。这些是 ROS 参数，不是原有
OKVIS/se2 配置文件中的字段。标准节点和网络处理节点均接入了状态更新接口；当前
launch 的专用开关作用于 `lidar:=true` 分支。其他入口可用 ROS 参数启用。

## 内存与 RViz

默认配置呈现黑底、高度伪彩色历史地图、白色当前原始扫描和绿色轨迹。
历史地图只随真实 SLAM 关键帧增长，不裁剪后半段轨迹，也不设置总显示点数上限。

千万点整图每次发布都要重新读归档、变换、序列化并上传 RViz，即使移到后台线程，
仍会造成显示停顿。因此默认改用独立的 `okvis/GlobalLidarMap` RViz 插件：

- `/okvis/global_lidar_map_markers` 只传新增/修改的关键帧局部点块；每批最多两个
  65536 点归档块，批次最快每 0.1 秒发送一次。**这是单批工作预算，不是总显示点数限制。**
- 插件每个绘制周期最多处理两个块；未修改历史几何常驻 RViz，普通位置/姿态修正
  只更新关键帧 Ogre 节点，不重新上传该帧所有点，也不在每个绘制周期重新生成点数组。
  若第一个块已用完本周期 4 ms 软上传预算，第二块推迟到下一周期；不删除任何历史点。
  未新增几何时历史锚点 TF 查询降到约 10 Hz，新增/替换几何立即检查，避免空闲时按 GUI 帧率扫描整条轨迹。
- 静态关键帧锚点通过带会话前缀的 `/tf_static` frame 传输。纯 XY/yaw 修正仅更新 TF；
  世界高度变化估计达到地图体素分辨率（默认 0.1 m）时，分块重算高度颜色。
  最终 PCD 始终使用最终可用位姿和精确世界高度颜色。
- 新连接的 RViz 分批补齐历史，同时优先发送新关键帧；新读者连接不清空已有读者地图。
  显示端最多等待 8 个块，DDS Keep Last/depth 4；过载时重连请求补齐，使用内容版本
  跳过已经应用的块，避免静默丢失历史。队列内不额外复制点数组。
- RViz 只保留显示几何与少量块/位姿元数据，不额外缓存全部历史 Marker 消息。
  无订阅者时不生成密集显示载荷。SLAM 建图线程与显示传输线程仍独立。
- 兼容整图话题仅在地图/采样变化或新读者加入时发布；静止地图不再按间隔反复序列化整图。
  后加入读者由匹配事件触发补发缓存。静止时 `ros2 topic hz` 没有持续频率属于正常行为。

| 数据 | 默认策略 |
| --- | --- |
| 完整采样地图 | 每个 SLAM 关键帧最多一帧扫描，0.1 m 局部体素；历史分块写磁盘 |
| 热点云 | 最多 1 个有点的驻留子图，达到 65536 体素即归档 |
| 增量显示 | 固定采样步长 1，全量保留的采样地图；无总显示点数上限 |
| 兼容整图接口 | `/okvis/global_lidar_map`，Reliable/Volatile/depth 1；默认 RViz 关闭 |
| 点云输入 | DDS depth 1；一个活动扫描加最多 8 个等待扫描，总等待载荷 32 MiB |
| 单扫描输入 | 最多 262144 点、16 MiB 原始载荷，超限拒绝并计数 |
| 优化更新 | 最多 32 份等待更新，含活动更新的估算动态内存预算 16 MiB |
| 最终 PCD 导出 | 外部排序归并，全局体素去重，默认 8 MiB 排序工作区 |

`preview_sampling_step=1` 显示全部已保留地图点；它们仍经过 `voxel_size=0.1 m`
的地图体素采样，不等于全部原始激光回波。步长 2 每两个点显示一个，步长 3 每三个点
显示一个；采样相位跨归档块和子图连续。运行时改变步长会逐块替换显示，最终完整
PCD 不受显示步长影响。不会随地图增长自动粗化或删除早期点云。

**保留全部显示点意味着 RViz 的 CPU/GPU 点云内存仍随所选点数增长，不能固定总 RAM。**
有界队列避免积压多幅整图；如果硬件容量不足，只调整采样步长。磁盘、锚点元数据
以及完整采集历史也随轨迹长度增长。

### 显示项

提供的 `config/rviz2/rviz2_okvis2x_config.rviz` 中：

| 显示项 | 设置 |
| --- | --- |
| `Global LiDAR PCD (Incremental)` | 开启；Class `okvis/GlobalLidarMap`；话题 `/okvis/global_lidar_map_markers`；2 像素点 |
| `Current LiDAR Scan (White)` | 开启；Class `okvis/LiveLidarScan`；`/ouster/points`；白色；2 像素；QoS Auto/depth 1；仅显示一帧 |
| `Path` | 开启；`/okvis/okvis_path`；绿色 |
| `Global LiDAR Full Snapshot (Slow)` | 关闭；保留 `/okvis/global_lidar_map` PointCloud2/RGB8 兼容入口 |
| `Landmarks` / `Frame-to-map points` | 默认关闭，避免重复绘制大量点 |

Fixed Frame 使用 `world`。不要同时开启全局整图、增量地图和旧的完整历史点云。
原生 MarkerArray 可用于协议调试，但默认插件才能保证位姿更新只移动场景节点。

白色原始扫描使用实际消息头 frame（当前 Ouster 是 `os_sensor`）。模块默认广播
`world -> os_sensor` 的动态 TF，使用标定 `T_WS * T_SL`，时间戳来自前端状态/bag，
而非墙钟。前端回调排入最多 64 个按时间排列的小位姿，专用 TF 线程立即唤醒并批量
发送；不会被全局地图的读取/发布间隔阻塞，也不会把一批位姿覆盖到只剩最后一帧。
这为 RViz 按扫描 header 时间插值保留两侧的位姿。缺少前端回调的入口在优化回调
接收时立即转发当前位姿，不等待建图队列处理。若已有外部节点发布相同 TF 链，设置
`global_lidar_pcd.publish_lidar_tf:=false`，避免重复发布。原始扫描不做累积。

白色扫描由独立的 `okvis/LiveLidarScan` RViz 插件显示，直接订阅原始 `/ouster/points`：

- DDS depth 为 1，最多保留 4 幅等待 TF 的扫描。每个绘制周期从新到旧选择
  **扫描原始时间戳已有 TF** 的最新一帧，只在显示成功后删除它及更早的等待扫描。
  新扫描尚无 TF 时，保留最后一帧正确显示的几何，下一绘制周期继续检查。
  不能只保留最新扫描，否则在 TF 始终落后一帧时会持续等不到变换。
- `Reliability: Auto` 每秒检查发布端 QoS：发布者全部使用 Reliable 时选择 Reliable；
  有 Best Effort 发布者时选择 Best Effort。也可以在显示属性中手动选择。
  这适配当前 Reliable rosbag 回放和使用 Best Effort 的实时驱动。
- 单帧最多 262144 点、16 MiB；等待队列原始 data 载荷至多 64 MiB，另有有界的
  回调/绘制临时引用和单帧渲染缓冲。不会积累历史白色扫描，也不限制全局地图的总点数。
  XYZ 支持 FLOAT32/FLOAT64、大小端和行 padding，仅去除非有限或不能表示为 float 的点。
- `Input` 区分停流与正常接收；`Transform` 连续一秒无显示进展时报告警告及 TF 原因。
  首次定位前没有有效位姿、时间不匹配或真实 TF 缺失仍会明确提示。

本机诊断曾在同一 6 秒窗口收到 Reliable 118 帧、Best Effort 3 帧，且接收扫描时
最新 `world -> os_sensor` TF 通常落后约 50 ms。当前安装的 Jazzy 原生 PointCloud2
使用 `PointCloud2TransportDisplay` 直接处理消息，没有等待该扫描 TF；增大旧配置中的
`Filter size` 无法修复这个接收路径。新显示项在 RViz 内等待准确时间的 TF，
不使用零变换、墙钟时间戳或未来偏移，也不把传感器坐标直接标成 world。

### 重编译与加载

新插件需要安装注册，必须重新编译、source，并**重启 RViz、加载更新后的配置**；
仅重启 SLAM 节点不够。源码修改不会自动替换已运行 RViz 的原生 PointCloud2 显示项。
本次按要求只修改源码，不执行以下编译/安装操作；命令留待用户自行应用：
在工作区根目录执行（沿用原项目所需的其他 CMake 选项）：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select okvis --symlink-install --cmake-args -DBUILD_ROS2=ON
source install/setup.bash
rviz2 -d src/OKVIS2-X/config/rviz2/rviz2_okvis2x_config.rviz
```

运行原有 launch 时会使用安装后的同一配置。手动添加显示项请选择
`okvis/GlobalLidarMap`，话题为 `/okvis/global_lidar_map_markers`；白色扫描请选择
`okvis/LiveLidarScan`，话题为 `/ouster/points`、`Reliability: Auto`。
若白色项仍出现 `Color Transformer`、`Decay Time` 等原生属性，说明仍加载旧显示项，
需重新加载配置或删除旧白色项后添加新插件。全局地图和轨迹显示项无需删除。
如果提示类不存在，首先检查是否在启动 RViz 的终端 source 了本次安装目录。

### Conda 环境下的链接报错

若编译在链接阶段报 `curl_*@CURL_OPENSSL_4` 未定义，同时提示
`miniconda3/lib` 的运行时搜索路径冲突，检查是否误选 Conda 的 `yaml-cpp`。
系统 ROS/RViz 与 Conda 的 YAML/curl 库混用，会使系统 GDAL/HDF5 找不到所需符号。
项目现在先加载 ROS 的 `yaml_cpp_vendor`；Linux 上其内置配置目录缺失时，显式选用
系统 YAML 配置并修复旧的 Conda 缓存路径，再导入 RViz/rosbag 依赖。
配置日志应显示 `OKVIS ROS yaml-cpp: /usr/lib/<架构>/cmake/yaml-cpp` 或有效的 ROS vendor 目录。

不需要删除整个 build/install，也不需要增加忽略未定义符号的链接参数。
在工作区根目录强制重新配置一次即可：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select okvis --cmake-force-configure
source install/setup.bash
```

若另有手动设置的 Conda `LD_LIBRARY_PATH`/`LIBRARY_PATH`，还需清除这些显式覆盖；
本修复针对 CMake 误选 YAML 引入的路径，不修改用户 shell 或 Conda 安装。

旧的整图接口仍支持 Reliable/Best Effort 两种读者；有订阅者才构建一个
16 字节/点的 XYZRGB 消息（100 万点约 16 MB），默认每 0.5 秒发布。
该接口的 `publish_interval` 不控制增量批次速度。大地图长轨迹优先使用增量显示。

## 参数

ROS 参数前缀均为 `global_lidar_pcd.`：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `enabled` | C++ 默认 `false`；LiDAR launch 默认 `true` | 启用独立模块 |
| `output_file` | 空字符串 | 最终彩色 PCD 文件；为空使用独立会话目录 |
| `archive_directory` | `<csv_path>/global_lidar_pcd` | 分块档案父目录 |
| `save_on_shutdown` | `true` | 排空并自动导出 PCD；false 仍保留归档 |
| `voxel_size` | `0.1` | 完整地图和最终全局去重分辨率，米 |
| `preview_sampling_step` | `1` | 显示采样步长，正整数；1 全量、2 一半、3 三分之一；支持运行时修改 |
| `publish_interval` | `0.5` | 兼容整图在变化/新读者加入时的最小墙钟发布间隔，秒，最小 0.1；支持运行时修改 |
| `publish_lidar_tf` | `true` | 发布原始 LiDAR frame 的标定动态 TF；已有外部 TF 时关闭，启动时读取 |
| `input_reliability` | `auto` | 建图旁路输入 QoS：auto / reliable / best_effort，按 remap 后话题发现发布者；depth 1，启动时读取 |
| `worker_cpu_budget` | `0.35` | 建图/显示线程各自占单核 CPU 的软目标，范围 `(0, 1]`；1 关闭节奏控制，启动时读取 |
| `keyframe_match_tolerance` | `0.05` | 扫描时间范围两端的关键帧匹配容差，秒，范围 0–1 |
| `height_min` / `height_max` | `-2.0` / `5.0` | 固定世界高度颜色范围，米 |
| `minimum_range` / `maximum_range` | `0.5` / `150.0` | 激光有效距离，米 |
| `max_scan_points` | `262144` | 扫描解码点数上限 |
| `max_scan_bytes` | `16777216` | 原始 PointCloud2 载荷上限 |
| `max_pending_scan_bytes` | `33554432` | 等待扫描总载荷上限，同时最多 8 帧 |
| `max_pending_update_bytes` | `16777216` | 排队及活动位姿更新的估算内存预算 |

launch 额外支持 `global_lidar_sampling_step`、`global_lidar_publish_interval`、
`global_lidar_height_min`、`global_lidar_height_max` 四个参数。

在原 launch 命令后添加 `global_lidar_sampling_step:=1` 可全量显示采样地图。
运行中也可以调整显示步长，无需重启：

```bash
ros2 param set /okvis/okvis global_lidar_pcd.preview_sampling_step 2
ros2 param set /okvis/okvis global_lidar_pcd.publish_interval 0.2
```

旧参数 `preview_max_points`、`preview_voxel_size` 和 launch 参数
`global_lidar_preview_points` 已移除，使用旧参数文件时需相应替换。

后台吞吐不足时只替换新模块自己的等待扫描，已有实时估计器和建图缓存不受修改。
每 100 个完整消费扫描和关闭时记录统计。位姿更新不能随意丢弃；如果更新队列超过
预算，停止这个旁路模块并尝试写 `.partial.pcd`，现有 SLAM 继续。磁盘/参数异常也
由模块内部报告。功能开启后仍会增加解码、磁盘 I/O 和 CPU 资源开销，不能保证任意
倍率回放都没有扫描丢弃或资源争用。

### 后半程地图稀疏的排查

关键帧建图只保证“每个合并扫描对应真实关键帧”，不保证每个关键帧都收到邻近扫描。
视觉关键帧间隔、行驶速度、输入丢帧、扫描匹配窗口，以及距离/无效点过滤都可能影响覆盖。
`preview_sampling_step: 1` 已显示全部保留点；`publish_interval` 只控制兼容整图接口的
刷新间隔，不会补齐未收到的扫描。当前建图路径不调用旧的自适应预览粗化接口。

本机最近一次完整会话 `lidar-cloud-2018683-110612697729230-0` 的日志显示合并
190 幅关键帧扫描、跳过 719 幅非关键帧扫描和 9 幅重复关键帧扫描。
归档含 7,627,728 点，最终按 0.1 m 世界体素去重的 PCD 为 6,200,288 点。
按关键帧 ID 顺序分三段，前段/中段/后段每帧保留点数中位数分别为
41,930 / 40,679 / 39,537，而相邻建图锚点直线间距中位数为
6.19 / 10.53 / 10.03 m，最大间距 91.76 m。这一会话的主要现象是
建图锚点空间覆盖不均，不能简单归因于 RViz 删除后半程点云。
这些统计不证明所有未覆盖关键帧均由 DDS 丢包造成，也不保证截图来自同一会话。

旧日志的 `replaced/rejected scans = 0` 只统计进入回调后模块自己的丢弃，
不包含 DDS 在回调前的丢失。此前本机相同输入的 Reliable / Best Effort 对比
已发现明显接收率差异，所以建图旁路也改为自动选择可靠性。
每 5 秒及结束时新增 `Global PCD coverage` 统计：实际收到扫描数、已知真实关键帧数、
已有地图点的关键帧数。已知关键帧可能仍在等待扫描/位姿，实时差值不能直接等同最终缺失。

需要提高每幅扫描的保留密度时，可在启动前将 `voxel_size` 从 `0.1` 改为 `0.05`，
保持 `preview_sampling_step: 1`；这会增加归档和 RViz 内存，不会恢复遗漏扫描。
本次没有擅自调整该值或估计器关键帧参数，也没有放宽匹配时间去套用远时刻扫描。

## 验证

`okvis_colored_lidar_map` 覆盖调色范围、XYZRGB 二进制格式、闭环锚点修正、跨块全局
去重、跨归档块/子图固定步长采样、200003 点全量遍历、零步长拒绝及导出失败时
保留旧文件；同时保留原有有界预览 API 的兼容测试。原 `okvis_lidar_pointcloud_map`
继续验证原有 XYZ 行为。

`okvis_global_lidar_pcd_mapper` 覆盖默认关闭、独立 ROS 输入、晚连接 RViz、步长 1
显示全部 5003 点、动态切换步长 3 后显示 1668 点、零步长拒绝、逐点时间的运动补偿、
跨 4096 点批次的重复时间戳、非零激光外参平移、退出时采用最终位姿修正，以及完整
5003 点 PCD 不受显示采样影响；同时覆盖已有读者在线时第二个读者接收静态地图、
超大扫描拒绝和位姿更新队列超限隔离。

关键帧选择回归测试覆盖扫描先于状态到达、普通帧扫描跳过、同关键帧只合并一次、
下一关键帧继续增长地图，以及重复优化回调只修正锚点；检查 RViz 和最终 PCD 均仅
保留两个关键帧选中的 7 个点。

另外覆盖三帧扫描先于关键帧到达、扫描中点恰好匹配普通帧而扫描范围覆盖真实关键帧、
FIFO 不覆盖中间扫描、Reliable 订阅读者、显示参数错误隔离及恢复。三段关键帧点云
共 12 个点均实时发布并保留在最终 PCD 中。

增量协议回归覆盖分批补齐全部历史、稳定块 ID、全局连续采样相位、静态地图不重发几何、
水平闭环只更新 TF、受影响高度块重着色、采样后空块删除，以及第二个读者连接时
不清空已有地图。另验证白色原始扫描 TF 使用前端位姿、传感器时间和非零激光外参。

此前版本曾在 `/tmp` 基于现有构建依赖完成相关编译/链接和地图测试；这不代表
当前源码修订已经通过构建。本次原始扫描显示修复遵守“不编译”要求，
只做源码/API 静态核对、YAML/XML 解析和 diff 检查，不执行编译、安装或 C++ 测试。

新补充的 `testLiveScanBuffer()` 回归源码覆盖：4 帧等待上限、保留较新的未就绪扫描、
空 frame/超大输入拒绝、TF 落后一帧时不饥饿、按原始时间插值、延迟 TF 到达后继续显示，
以及 XYZ 大小端/FLOAT32/FLOAT64/行 padding/NaN/溢出/损坏字段处理。该新增测试尚未运行；
新插件的实际加载、帧率及 GUI 行为也须在后续构建后验证。

`testMapInputReliability()` 新增回归源码覆盖 remap 话题发现、发布者晚启动、
Reliable 回放与 Best Effort 驱动切换、depth 1 / Volatile，以及切换后关键帧地图继续增长。
该测试同样尚未编译或运行；QoS 修复不改变此前保存的地图，需要后续构建后重新采集验证。

实际数据集的闭环精度、长期 RSS 和实时性能仍需要同一段 bag 回放测量。

## 话题没有显示时

首先检查实际节点参数，而不是仅检查 RViz 的显示项：

```bash
ros2 param get /okvis/okvis global_lidar_pcd.enabled
ros2 topic info /okvis/global_lidar_map --verbose
```

标准 launch 中节点全名为 `/okvis/okvis`。如果更改了节点名或 namespace，需要相应
调整命令。参数为 `false` 时不会创建发布者和建图线程；启用参数在启动时读取，
需要重启 launch，不能只在运行中 `ros2 param set`。

启动日志应出现 `Global PCD worker:`。默认显示项使用 `okvis/GlobalLidarMap`，
并检查 `/okvis/global_lidar_map_markers` 发布者。Fixed Frame 为 `world`。兼容整图
显示项才使用 RGB8、Best Effort、Volatile。有发布者但点数为零时，还需检查原始激光输入以及
覆盖扫描时间的优化位姿/IMU 数据；没有订阅者时模块有意跳过点云组装。
