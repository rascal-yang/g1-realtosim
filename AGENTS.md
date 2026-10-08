# G1 Real-to-Sim维护说明

- 先读README和docs/DATA_CONTRACT.md。用户要新场景时使用skills/scene-asset-router；做G1/Revo2物理操作使用skills/g1-grasp-runtime-review；视频交付前使用skills/sim-visual-realism-review；运行数据流水线使用skills/realtosim-data-pipeline。
- 代码在src，具体照片配方在recipes；输出写仓库外../experiments或../asset_tasks。新任务新目录，保留失败证据，不覆盖历史数据。
- 本机依赖只写configs/local*.json，示例配置不绑定用户名。不要把模型、照片、USD大资产、MP4、HDF5或凭据加入Git。
- 建模必须保留尺寸、材料、纹理、摆放与物理参数的依据；估计不标为实测。引用资产不等于新模型生成。
- 重渲染只恢复保存状态，不冒充新物理试验。保留所有动态对象、双手、缩放和相机，禁止通过改几何掩盖视觉问题。
- 观察和动作的时间契约见docs；没有实际执行指令的运动轨迹不可伪造为监督动作。预览RGB、正式RGB、失败/恢复、质量验收独立标记。
- RT2默认是原生DLAA、反弹4/6、普通状态1次更新（支持2/4次对照；预热另计）；不是4采样也不是128采样。PT128必须验证每姿态累积，不能只看设置值。
- 原图/蒙太奇检查不证明无闪烁。未做原生帧率时间质量检查时保持accepted=false。
- 使用已有环境验证；不要无目的升级Isaac/torch/驱动。必要契约测试用python -m unittest discover -s tests -v。

- 用户授权的media/rendering_comparisons三份MP4可入Git，其他模型/场景/原始轨迹仍不入Git。渲染设置和已知边界见docs/RENDERING.md，保持未完成时序验收标签。
