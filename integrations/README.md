# 在线控制与采集参考

这里保存三份关键历史实现，便于后续适配新的控制工程：

- `sonic_planner_goals.py`：实际SONIC ONNX planner + 世界位置目标 + 有界IK参考融合。
- `brainco_adapter.py`：双侧Revo2安装、主动/被动mimic关节、连续开合接口。
- `training_capture.py`：pre-action状态、实际身体执行指令/物理子步、相机及场景状态采集。

它们是源快照，仍依赖原工程的experiment/grasp_extensions等模块与资产目录，**不是独立可执行入口**。源路径和SHA256见`docs/source_inventory.json`。为避免误用，实际可独立运行的入口统一在`src/g1_realtosim`。

接入新的仿真任务时沿用以下时序：

1. Controller更新手目标；记录控制前机器人、手、所有物体及相机状态。
2. 在实际向PhysX写入点记录body/hand目标、刚度、阻尼和前馈，不用下一状态替代动作。
3. 每个物理子步记录应用指令和接触；动作后保存next_observation，包含最后一步。
4. 导出匹配初始USD及所有名称顺序；相机可低频预览，最终RGB从完整状态重渲染。

历史手采集只记录position_target，没有逐步手增益，后续集成应补齐实际写入时的增益与约束。复用历史默认值前先检查decision日志和驱动配置。实时抓取继续遵循仓库skills中的双手安装、桌面碰撞、连续动作与30秒恢复预算经验。
