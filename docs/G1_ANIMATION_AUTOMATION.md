# G1动画流水线复用

当前提供 `g1_revo2_right_handle_basket_place_v1` 技能族：原办公室、同一空篮资产、G1/Revo2、固定站位、右手把手抓取和桌上放置。它不是通用抓取求解器。复用旧资产、运动学和接触执行器，重新计算新轨迹，不复制旧成功时序。

| 层 | 自动化内容 | 新任务需要提供 |
|---|---|---|
| 动作规格 | JSON验证、事件顺序和采样检查 | 意图、抓取先验、起点/落点、阶段时间、位姿来源 |
| IK与动画 | 双手FK、右臂IK、关节限制、Blender构建 | 目标掌位姿、抬升、退手；新机器人需要映射 |
| 轨迹导出 | 从实际blend连杆变换读回、关节命名、往返误差 | 同模型无需改代码；新模型需要适配 |
| 几何检查 | 每帧整手/前臂包围盒与桌面筛选 | 新场景障碍几何；当前入口拒绝改变桌面 |
| 物理执行 | 复用站立、趋近、闭合、抬升、搬移、支撑、释放、退手状态机 | 物体接触模型、抓法和任务成功判据 |
| 验收与数据 | 接触/平衡/完整任务检查、实际状态及命令、失败原因 | 新技能必须增加匹配的验收器 |

换同一篮子的起点、落点、闭合量、抬升高度和动作阶段时间，可先改JSON重算；每条仍需物理验收。手接触位置随物体位置变换，但当前位置可达性和夹持稳定性不继承。当前根偏航限pi/2，篮子动画初始偏航固定0，仍采用原桌面高度。不能靠改文件名把瓶子/玩偶当篮子。新物体应明确实现抓点、质量/摩擦/碰撞模型及成功判据；双臂、推门、持物行走需要增加协调/接触阶段控制器。

## 使用

配置模板见 `configs/basket_animation.example.json`。本机解释器、已导入Blender场景、已验证实验模板和SONIC路径在 `configs/local.animation.json`；跨机器使用 `configs/animation_runtime.example.json` 配置依赖。此版本复用本机实验运行时，不声称空白机器独立安装已验证。

```bash
cd g1-realtosim
conda activate sonic
# 先复制示例并修改需要的数值；为每次尝试选择新的输出目录。
PYTHONPATH=src python \
  -m g1_realtosim.animation_pipeline \
  --local configs/local.animation.json \
  --spec configs/basket_animation.example.json \
  --output ../experiments/my_new_basket_animation
```

加 `--render` 才生成完整动画视频；默认不渲染构建预览，节省反复检查数值时的GPU时间。每阶段用单调时钟记耗时，失败非零退出并保留目录；已有目录拒绝覆盖。逐次输出脚本快照、配置、资产源、IK/导出/筛选报告与物理参数。

`physical_commands.json` 列出这次目录对应的实际命令：

1. 启动 `scripts/launch.py trial_001`。
2. 在另一终端启动 `scripts/supervise.py <run目录> --review-initial`。
3. Agent/操作者检查真实 `observation_000.png`，记录 `reviewed_000.json`；否决则停止，不伪造审图。随后确定性接触状态机自动推进，无需每个控制步调用GPT。
4. 完成后运行 `scripts/postprocess.py trial_001`；只有完整任务成功才允许 `--render --skip-validation` 导出成功回放。

初始审图沿用抓取运行规范，不是每阶段都要用户确认。动画生成脚本不自动写审核标记。未实际跑物理的输出保持 `physical_success=null`。原始动作的34秒时间轴不是物理执行时间约束；控制器保留保守速度和接触门控，不能把修改动画时间当作成功修改真实执行速度。

本次提取过程中保留了贴图相对路径失败记录；修复为加载原始blend真实路径后打包。测试结果见工作区 `experiments/20260927_animation_reuse_*`。旧成功试验没有改写，新几何测试不能继承其成功标签。

## 提示词与效率

任务准备提示词见 [G1_ANIMATION_TASK_PROMPT.md](G1_ANIMATION_TASK_PROMPT.md)。固定提示词负责约束输入和记录证据；确定性计算交给脚本。不要让GPT每次重写IK、猜坐标或反复生成大段Blender代码。

脚本化主要减少每任务开发、排错和启动操作。物理推进与记录仍是耗时大项，不会因包装脚本从20分钟降到几秒。拆分在线图像采集和离线渲染、并行多环境属于后续性能工作，此次没有改变物理或采集频率。

实际验证：[最终运行报告](../../experiments/20260927_animation_reuse_final/REPORT.md)。原参数导出与历史导出一致；改位置/抬升/闭合/时间的变体也通过几何和读回检查，未运行新物理试验。默认生成约15秒，动画渲染另计。
