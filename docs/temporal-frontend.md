# LK 轨迹前端与 BRISK 地图恢复

前端默认按以下流程处理图像，普通同步路径和独立跟踪路径使用相同的轨迹身份。

1. `detectAndDescribe()`：先用金字塔 LK 跟踪上一张已处理图像的特征，检查前后向误差和
   边界；优先保留长轨迹，在圆形 mask 外补充 BRISK 点，最后为保留点和新点计算 BRISK
   描述子。每个观测保存独立于地图点的 `FeatureTrack{id, length}`，新轨迹长度为 1。
2. `matchToMap()`：成熟轨迹通过历史观测中的轨迹 ID 直接连接已经初始化的地图点，
   检查重投影，并进入原有鲁棒位姿优化。长度在 2 到 `track_min_length - 1` 之间的
   连续轨迹是 reservation，不参加地图描述子匹配。新检测点和未成功直连的成熟轨迹
   保留原有 BRISK 地图恢复。回环地图融合仍使用原有匹配方式。
3. `matchMotionStereo()`：保留上一后端帧及按重叠率排序的关键帧作为参考；按相同轨迹 ID
   构造 temporal pairs，在相机反投影射线上运行相对位姿/纯旋转 RANSAC。验证通过后才
   使用原有 `triangulateFast()` 三角化，经非平行、正深度、两帧重投影检查，再创建地图点
   和观测。已有未初始化地图点更新前还要验证其全部已有观测。小于 8 对的集合等待更多
   观测，不绕过几何检查。原有同一时刻多相机 BRISK stereo 匹配保留，并跳过 reservation。

成熟指轨迹长度达到门限；是否已有可靠地图点是另一状态。成熟轨迹可以尚未建图；轨迹
连续性本身不代表三维几何有效。单目初始化期间，至少 15 个连续跟踪点允许系统继续等待
轨迹成熟和有效视差，避免因 reservation 阶段没有地图观测而反复重置。

## 配置

在已有相机配置的 `frontend_parameters` 中增加以下可选字段，省略时也使用这些默认值：

```yaml
frontend_parameters:
  use_temporal_tracking: true
  track_min_length: 3
  track_min_distance: 20.0
  track_max_fb_error: 1.0
  track_max_gap: 0.5
```

`track_min_length` 至少为 2。`track_min_distance` 是圆形 mask 的像素半径；
`track_max_fb_error` 是 LK 前后向检查的最大像素误差；`track_max_gap` 单位为秒。
LK 使用 21×21 窗口和最多 3 层金字塔。跟踪输入为 `CV_8UC1` 灰度图。
`max_num_keypoints` 同时限制保留轨迹和补点后的总点数。

图像尺寸改变、时间不递增或间隔超过 `track_max_gap` 时，跟踪器终止旧轨迹。
轨迹 ID 单调分配，系统重置也不复用旧 ID。轨迹长度统计该相机成功处理的连续图像，
不会因后端未接纳某帧而重置。LK 丢失后即使 BRISK 恢复到同一地图点，也分配新的轨迹 ID。

将 `use_temporal_tracking` 设为 `false` 可恢复原有 BRISK 检测、地图匹配和描述子时序
匹配，用于同数据段对照。`independent_tracking` 仅控制线程和接纳策略，与此开关独立。

## 前后端交接

`copyTrackingFrame()` 复制轨迹 ID 和长度，并让后端从自己的有效历史观测解析地图绑定，
不直接接纳前端 pose-only 求解的图观测。后端反馈快照包含局部地图和轨迹到地图点的绑定；
图合并后的新绑定覆盖前端缓存。前端只保留当前图像仍可见轨迹的绑定。后端重置通过新版本
失效反馈清除旧绑定。轨迹元数据随 `Frame` 复制、冷图像释放和同构建的 payload 归档保留；
payload 格式版本变为 `OKVFRM02`，不兼容旧构建的临时 payload 文件。保存地图中的历史帧
可以没有轨迹元数据，仍可用 BRISK 恢复和回环。

## 特征跟踪帧数可视化

```yaml
output_parameters:
  display_matches: true
```

图像仅叠加小绿点和跟踪帧数，例如 `18` 表示该特征已经连续跟踪 18 帧。出生帧计为 1。
不显示轨迹 ID、地图点 ID、关联来源、地图重投影或底部统计栏，图像尺寸保持不变。
旧配置中的 `display_track_ids` 不再影响绘图。

ROS 图像话题（节点命名空间内）：`cam0_tracks`/`cam1_tracks` 为独立前端每个已处理图像的
跟踪帧数，`cam0_matches`/`cam1_matches` 为后端接纳帧的同类视图。彩色相机使用 `rgb0_tracks`
和 `rgb0_matches`。两路各自使用容量 1 的队列和后台绘图线程，忙时替换旧图，不阻塞跟踪。
同步模式使用 `*_matches`。帧数统计连续处理的相机图像，不是后端接纳帧数。
绘图读取入队时复制的观测信息，后续后端图更新不会改变已排队的帧数。

## 后续替换 HFNet

当前实现不引入 HFNet 依赖。`TemporalFeatureTracker::propose()` 输出对应旧轨迹的候选
关键点；`commit()` 在描述子提取删除/重排关键点后提交最终轨迹。`class_id` 只临时保存
上一帧特征索引，不能作为长期轨迹 ID。后续用 HFNet 帧间匹配替换 propose 中的 LK 时，
保持这个对应关系和 commit 接口，地图关联、reservation 和时序三角化可继续使用。

## 验证

在启用 `BUILD_TESTS` 或 `BUILD_POINT_TO_PLANE_TESTS` 的构建中：

```bash
cmake --build <build-dir> --target okvis_temporal_frontend_test okvis_tracking_frontend_test okvis_frame_archive_test
ctest --test-dir <build-dir> -R 'okvis_(temporal_frontend|tracking_frontend|frame_archive)' --output-on-failure
```

覆盖图像平移下的 LK 精度、描述子删除/重排后的 ID、圆形 mask、间隔重置、年轻轨迹预留、
描述子不一致时的成熟轨迹关联、延迟图合并反馈、时序 RANSAC 外点过滤及单目建图。
真实序列的精度、跟踪存活率及实时耗时仍需使用相同输入频率与配置做回放对比。
