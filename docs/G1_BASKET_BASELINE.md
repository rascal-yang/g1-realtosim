# G1 篮子抓取 baseline

现有办公室篮子资产的真实物理抓取入口，使用浮动基G1、双侧Revo2和SONIC站立平衡。当前操作角色为右手抓把手，左手避碰，尚不等于双手搬运或持物行走。

完整实验、配置和实测结果在 [`../../experiments/20260925_g1_basket`](../../experiments/20260925_g1_basket/README.md)。复用已验证的G1控制运行时、资产与本项目状态重渲染；没有用Panda结果代替G1验收。

可复用实现：

- `src/g1_realtosim/basket_supervisor.py`：物体坐标目标变换、实际状态门限、闭指、试提和持续保持。
- `src/g1_realtosim/basket_audit.py`：从完整物理next state检查离桌、接触、姿态和脚部稳定性，保留失败原因。
- `src/g1_realtosim/basket_export.py`：复用原G1 HDF5导出器，并保存实际记录的Revo2驱动增益、速度/力矩目标及独立物理标签。

示例（在仓库根目录，使用已配置Isaac/SONIC解释器）：

```bash
PYTHONPATH=src "$SIM_PYTHON" -m g1_realtosim.basket_supervisor \
  ../experiments/20260925_g1_basket/runs/NEW_RUN --review
PYTHONPATH=src "$SIM_PYTHON" -m g1_realtosim.basket_audit \
  ../experiments/20260925_g1_basket/runs/NEW_RUN
```

启动仿真与串行扩增命令见实验README。输入视频缺失，技能规格来自本会话按现有场景设计，视频到种子成功率不适用。5Hz在线RGB为预览，25FPS的RT2保存状态重渲染独立进行。物理成功、数据完整性与视觉质量分别验收；未完成原速时序视觉验收的数据保持`training_eligible=false`。

后续完整抓取—搬移—放置实验见 [G1_BASKET_PLACE.md](G1_BASKET_PLACE.md)，本页保留原提起保持阶段的验证范围。
