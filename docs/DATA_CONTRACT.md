# 数据与动作契约

## 输入：g1_capture_v1

`episode.json` 相对路径以该JSON所在目录为基准。必须同时提供原始轨迹HDF5与对应的初始USD；不接受任意CSV直接冒充已采集物理轨迹。

- `observation/*`：控制前状态，50 Hz；`timestamp`、关节位置/速度、根位姿/角速度、各刚体世界位姿、操作物体位姿。
- `action/*`：控制器实际给出的目标位置、目标速度、前馈力矩、刚度、阻尼。**动作不是下一帧测得的关节角**。
- `physics/*`：每个5 ms物理子步的上述目标，以及实际施加effort等；每个控制步4条记录。
- `next_observation/*`：动作之后的状态；最后一个控制步也保留。
- `brainco/{left,right}`：双手各11关节的状态、position_target以及12个刚体位姿。现有采集顺序为 `extension.before_step → capture.begin → env.step`，因此手目标是在本次控制前更新的值。
- `scene/{name}`：所有额外动态对象的世界位姿。其USD prim必须在配置中声明。
- 属性：关节/刚体名称、dt、physics_dt、任务文本、相机标定、坐标与四元数约定、来源成功/质量标签。

全局单位：米、秒、弧度、牛顿/牛米。Z向上，右手系；四元数`wxyz`。相机矩阵`world_T_camera_cv`将OpenCV相机坐标（右/下/前）变换到世界。

原始预览只有5 FPS且低采样，不用它替换25 FPS正式RGB。需要支持新机器人、不同摄像头或任意轨迹格式时，新增显式适配器并验证，不能按数组长度猜关节顺序。

## 重渲染输出

- `frames.h5:/rgb`：25 FPS RGB，每帧有`control_index`、`timestamp`、`world_T_camera_cv`。
- `frames.h5`还包含指向原始轨迹的外链，方便审查；这些外链**不是可独立搬迁的完整归档**。
- `video.mp4`：H.264观看版本，未插帧。
- `protocol.json`：输入SHA256、相机与帧选择；`settings.json`：实际渲染设置。
- `timings.jsonl`、`complete.json`：逐帧和总计耗时。

所有保存的动态物体和双手都恢复世界位姿，保留原始世界缩放。重放不推进物理。元数据一致不等于独立验证RGB读回零延迟。

## 对齐HDF5输出

`export`生成自包含`episode.h5`，其中：

| 字段 | 语义 |
|---|---|
| `observation/rgb` | 正式渲染的图像，逐帧复制 |
| `observation/state` | G1位置/速度、左/右Revo2位置/速度、根四元数、本体坐标角速度 |
| `action/commands` | 每个RGB对应后续两个50 Hz控制步，每步保留全部4个物理子步的身体指令，再附双手目标 |
| `action/valid` | 末尾不完整动作块的有效位；无效项填零但绝不能执行 |
| `control_index`, `timestamp` | 对应原始控制前状态 |
| `camera/world_T_camera_cv` | 与RGB对应的相机矩阵 |

当前实例 state=137维。单个控制步 command=882维：`4 × 5 × 43 + 11 + 11`；每个RGB的action为`[2,882]`。5个身体字段依次是位置目标、速度目标、前馈力矩、刚度、阻尼；准确顺序以`state_names`和`action_names`为准。这是完整控制审计契约，不是说训练一定应预测1764个独立自由度。具体学习任务可基于该数据另建经过验证的低维投影，不能悄悄丢失重放所需指令。

43关节中包含历史兼容手指；实际Revo2作为两个独立articulation。Revo2每手6个主动驱动、5个被动mimic关节。导出保留11个目标槽便于核查，但被动槽不能作为独立电机执行。G1兼容手指与Revo2不应重复控制实体手指。

历史手驱动配置默认stiffness=4、damping=.12，但控制接口允许更改手刚度，旧采集文件未逐步保存手刚度。`--hand-contract`可附带验证材料，却不会自动把未验证合同标为通过。源成功与视觉标签保留在provenance中；当前导出默认training_eligible=false，需基于具体控制配置及视觉检查另行审查。新的在线采集应在实际手写入点记录手驱动增益和约束。

对本条1261控制步的轨迹：631个RGB中最后一帧只剩1个控制步，因此HDF5最后valid为`[true,false]`。LeRobot导出明确丢掉这一个不完整动作块，保留630帧，不能复制/假造末尾动作补齐。

## LeRobot

通过官方LeRobot0.4.3 writer写入其原生格式；不假称旧版v2.1。RGB由writer以其默认有损H.264参数编码，编码版本与无损HDF5不同，画质需另验；无损HDF5保留为主数据。`observation.state`及扁平化的`action`进入表格。动作展开顺序严格继承HDF5字段名；不导入特权物体真值到策略的普通state。

`next.done`表示导出episode的结束；对于截取片段这是截断边界，不代表物理环境terminated。任务成功单独保存在provenance。

`--allow-unreviewed`仅允许开发格式验证，输出保留未验收标签；不自动发布或上传数据。
