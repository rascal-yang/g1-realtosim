# 成果、已知结论与历史索引

## 可复用结论

- 实际手机图像约束几何/布局，规则部分Blender建模，复杂玩偶复用图生3D形状/纹理；隐藏面和未经测量参数要保留推断标签。
- G1 + Revo2可以在Isaac中执行SONIC行走、受限IK慢抓、基于反馈的恢复。不同实验的GRAIL参与程度不同，不能把加载了GRAIL运行框架等同于应用了其残差；最终水瓶样例的GRAIL残差为零。
- 当前Agent来自交互会话，控制期间有确定性反馈监督器；没有独立无人值守GPT API服务。
- 真实执行记录与最终渲染分开保存，后续可以换渲染参数而不重复物理实验。
- 相机状态、本体和动作显式对齐；相同关节角数组不代表相同物理控制语义。

## 最终渲染对比

完整25.24秒，631帧，640×480、25 FPS，同一保存轨迹，顺序运行。

| 配置 | 正式渲染与保存 | 启动+预热 | 说明 |
|---|---:|---:|---|
| 优化PT128 | 664.83秒 | 10.40秒 | 累积128并检查稳定 |
| RT2 + 原生DLAA | 23.14秒 | 7.46秒 | 反弹4/镜面透射6，每状态4次更新 |

RT2快28.73倍；不是画质等价证明，也不代表跨场景固定加速。瓶身透射更接近PT128，细纹与高光仍不同；原生帧率感知验收尚未完成。旧RaytracedLighting与RT2为不同渲染模式，不混用名称。

## 本机历史索引

以下路径相对本文件可用；这些数据不随Git自动复制。

- [最新完整对比报告](../../experiments/20260921_rt2_vs_pt128_full/REPORT.md)
- [完整并排视频](../../experiments/20260921_rt2_vs_pt128_full/comparison_full.mp4)
- [RT2独立视频](../../experiments/20260921_rt2_vs_pt128_full/genie_rt2_native_quality/video.mp4)
- [10秒参数比较](../../experiments/20260921_genie_rt2_10s/REPORT.md)
- [训练视频导出路线调研](../../research/20260921_training_video_pipelines/REPORT.md)
- [照片场景重建](../../asset_tasks/20260920_photo_scene/REPORT.md)
- 机器人/手外观资产：`../../asset_tasks/20260921_robot_materials/assets/`
- 原始轨迹：`../../experiments/20260921_five_directions_128/runs/walk_03_far/episode_raw.h5`
- 对应USD：同目录`isaac_initial_scene.usdc`
- 原始控制/监督器工程：`../../experiments/20260921_five_directions_128/code/`与`scripts/`
- 本次独立验证输出：`../../experiments/20260921_repository_validation/`

历史记录保留各自原结论和失败证据；此次整理没有改变它们。skill中的恢复建议是有条件的经验，不是声称跨场景已经泛化。
