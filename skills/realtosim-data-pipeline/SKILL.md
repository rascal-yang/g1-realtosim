---
name: realtosim-data-pipeline
description: Assemble and run this repository's photo/video asset, saved G1 trajectory rendering, and aligned HDF5/LeRobot export workflow. Use for reproducible egocentric simulation dataset delivery or adapting its input contract.
---

# Real-to-Sim数据流水线

入口和命令见仓库根README，动作与时间契约见docs/DATA_CONTRACT.md。先区分用户输入是照片、生成模型、仅关节轨迹还是已有g1_capture_v1；不要声称只有照片或关节角就已有真实执行动作和接触标签。

## 路由

- 照片/视频先ingest，读取scene-asset-router并实际查看素材，填写尺度/布局/材质/纹理/物理来源。ingest成功只代表登记完成。
- 已有USD用package-assets收集依赖并异地打开。新资产几何或物理发生变化后，旧保存轨迹只能用于视觉重放检查，不能给新资产继承物理成功标签。
- g1_capture_v1 + 匹配USD用render。其他轨迹需显式适配或先物理执行采集。外部权重/引擎解释器使用local配置，不复制依赖进Git。
- export生成自包含对齐HDF5；LeRobot只使用官方writer。末尾不完整action块有mask；LeRobot丢弃该帧，不伪造动作补齐。

## 已知脆弱点

- 恢复全部G1、Revo2和动态场景物体；保留USD世界缩放。相机标定与输出尺寸必须匹配。
- RT2 + DLAA的4次更新不等于4采样。PT128每个新姿态需重新累积并检查稳定；源案例32采样/更新在第5次才完成有效128累积。
- 先输出短段确认USD/驱动/纹理，再执行全长，避免失败重复耗时。GPU渲染对比顺序运行，分别报告启动/预热/正式渲染保存时间。
- 旧Revo2数据虽有目标角，但没有逐步手驱动增益；查实际控制记录，保留未验证状态，不能默认所有手动作都用同一刚度。
- `observation.state`使用图像可部署的本体信息；特权物体状态留在原始遥测，若用于模型输入应明确命名。
- 原生帧率视觉检查、任务成功、控制数据完整性分别记录。代码/格式验证通过不自动使training_eligible=true。

每次交付包含可点击视频/资产/数据路径、精确来源和配置、验证结果及剩余限制。新实验放源码之外，原实验不覆盖。
