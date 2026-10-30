# LiDAR 点面因子

激光路径默认仅构建平面子地图并使用点面因子：不分配 Supereight 占据地图，不进行占据射线融合，不提取、保存或发布 mesh，不构造激光占据场残差。视觉与 IMU 约束继续工作。非激光的深度相机建图路径仍可使用原有占据地图。

累计 XYZ 点云地图的构建、ROS2 查看和 PCD 保存见 [激光点云地图说明](lidar-pointcloud-map.md)。

## 配置

在 **se2 配置文件**的 `general` 节点中设置，而不是在 `okvis2` 传感器配置中设置：

```yaml
general:
  use_map_to_live_factors: true
  use_map_to_map_factors: true
  use_point_to_plane_factors: true
  lidar_plane_only: true
  write_mesh_output: false
  plane_voxel_size: 1.0           # m，平面拟合体素，与 Supereight res 独立
  plane_max_variance: 0.01        # m²，PCA 最小特征值上限
  plane_eigenvalue_ratio: 0.1     # lambda_min / lambda_mid 上限
  plane_max_match_distance: 0.3   # m，点到平面的绝对距离上限
  plane_noise_floor: 0.03         # m，残差标准差下限
  plane_sigma_gate: 3.0          # 白化距离门限
  plane_min_points: 6
  plane_max_points: 100           # 每个体素保留的样本上限
```

`lidar_plane_only` 和 `use_point_to_plane_factors` 缺省均为 `true`。纯平面模式要求点面因子开启；矛盾配置会在启动时被拒绝，避免在没有占据地图时进入占据场残差路径。该模式下，即使误设 `write_mesh_output: true` 或 ROS 参数 `save_submap_meshes: true`，也不会提取、保存或发布 mesh。

如需显式恢复旧方案，必须同时设置 `lidar_plane_only: false` 与 `use_point_to_plane_factors: false`；mesh 另由 `write_mesh_output` 控制。也可设置 `lidar_plane_only: false`、`use_point_to_plane_factors: true`，保留占据建图但使用点面约束。

原有 `use_map_to_live_factors`、`use_map_to_map_factors` 仍控制相应约束是否启用，`enable_submapping` 也须启用。

## 数据与残差

建图线程将已有去畸变射线端点转换到子地图锚点坐标系，送入独立的 `PlaneMap`。每个体素对保留点进行 PCA，拒绝点数不足、线状及非平面分布。体素满后使用确定性的 reservoir sampling 更新样本，避免永久保留第一批扫描线。

完成的平面子地图通过 `shared_ptr<const PlaneMap>` 传给实时线程与后端延迟队列；发布后不再更新。地图仍相对于自身锚点保存，锚定位姿可参与回环优化。因子复制平面参数和权重，不在 `Evaluate()` 中查询或修改地图。

对于点 `p_B`，目标子地图的平面中心 `q_A` 与单位法向量 `n_A`：

```text
p_A = inverse(T_WA) * T_WB * p_B
e   = n_Aᵀ (p_A - q_A)
s²  = sensor_error² + plane_scatter + plane_noise_floor²
r   = e / s
```

在点所在体素和相邻 26 个体素中查询候选平面，检查平面支持范围、绝对距离及白化距离，再按高斯似然选择匹配。无有效平面时不添加因子，也不回退到占据场残差。首个子地图完成前无可用的历史平面地图，激光约束需等待地图初始化。

实时点筛选、map-to-map 重叠点筛选和子地图切换的激光重叠判断均使用平面匹配。候选子地图的 AABB 来自有限激光端点，并根据锚定位姿变换；给边界增加半个体素宽度，避免单平面地图的包围盒体积为零。首次子地图交接和连续低重叠触发切换保留原有帧数限制。

子地图位姿记录仍沿用现有容器，纯平面模式下其中的占据地图指针为空；map-to-live 的启用条件及激光关键帧触发条件使用平面地图引用。占据场切片和 mesh 子地图回调均不执行，对象 mesh 后处理也被跳过。旧占据地图顶视图在此模式下关闭，平面专用可视化尚未实现。

ROS2 节点仍可创建 mesh topic，但纯平面模式不会向这些 topic 发布 mesh 消息，也不调用发布端的 marching cubes。

## 当前实现边界

这是显式点面残差的第一版：采用单层体素 PCA 平面、平面散布及标量传感器噪声加权。尚未实现 FAST-LIVO2 的多层八叉树、法向量与中心的完整 6×6 参数协方差、逐点距离/角度噪声传播，以及 Ceres 求解外层的重新关联。平面散布不是平面参数协方差。

关联在添加因子时根据各图的当前位姿生成；实时图、完整图与回环延迟队列均支持新因子。单个因子的关联和权重在其生命周期内固定。子地图锚点调整会反映到因子中，但子地图内部的非刚性形变不会自动触发重新拟合。

边缘或多个相交表面落入一个体素时，该体素可能被拒绝；可通过减小 `plane_voxel_size` 改善，但需保证点数足够。实际精度和实时性需要数据集回放确认。

## 验证

可以仅启用本次新增的测试，避免打开整个工程的历史测试：

```bash
cmake -S . -B <build-dir> -DBUILD_POINT_TO_PLANE_TESTS=ON
cmake --build <build-dir> --target okvis_point_to_plane_test okvis_plane_only_mapping_test -j2
ctest --test-dir <build-dir> -R '^okvis_(point_to_plane|plane_only_mapping)$' --output-on-failure
```

测试覆盖平面拟合、相邻及负坐标体素查询、线状点拒绝、样本更新、距离门限、残差计算、仅残差求值、两个位姿块的有限差分雅可比、全局坐标变换不变性，以及三组正交平面下的 Ceres 位姿收敛。

`okvis_plane_only_mapping` 额外覆盖：占据地图与对象地图不分配、首个平面子地图交接、基于平面边界的候选选择、锚定位姿调整、连续低重叠切换、map-to-map 开关、已完成地图不可变性、mesh/占据场回调禁用、强制保存请求无效、以及无输入退出。测试使用合成点云，不替代真实数据回放。

开启原有激光调试选项时，日志输出 `Point-to-plane A -> B: matched/total matches`。新路径不使用旧占据场的梯度统计 CSV。
