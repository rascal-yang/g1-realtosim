# 参考实现核查（2026-09-25）

核查包含官方论文页面、项目站点、实际源码/目录和许可证文本；下载证据与 HTTP 状态保存于本次实验 `research/`。下表不代表所有外部项目均已安装或执行。

| 项目 | 论文与开放情况 | 实际接口、许可与本次选择 |
|---|---|---|
| DexImit | [论文 2602.10105](https://arxiv.org/abs/2602.10105)，[公开代码](https://github.com/mujc2021/DexImit-Open)可读取 | `pipeline.sh` 接收 task_dir/output_dir；输入视频及手工标签，重建后 `pipeline/gen_traj.py` 调用 BODex/cuRobo/SAPIEN，默认双 UR5e 手平台。主代码 MIT，第三方模型/代码各自许可。不是即插即用 G1/Isaac 模块；本轮不安装第二套仿真链。 |
| DexMimicGen | [论文 2410.24185](https://arxiv.org/abs/2410.24185)，[公开仓库](https://github.com/NVlabs/dexmimicgen) | 实际树包含 robosuite 环境、随机动作演示、数据下载/回放和 BC 配置生成；未找到完整公开轨迹生成入口。代码为 [NVIDIA 非商业研究/评估许可](https://github.com/NVlabs/dexmimicgen/blob/main/LICENSE)，README 标数据集 CC-BY 4.0。仅借鉴对象坐标和双手同步，不虚构 `augment()` API。 |
| HumanoidMimicGen | [论文 2605.27724](https://arxiv.org/html/2605.27724v1)，[项目页](https://humanoidmimicgen.github.io/)可访问 | 项目页 Code 指向 `NVlabs/humanoidmimicgen`，本次网页与 GitHub API 均返回 404，因此未取得公开代码/代码许可证；网站 CC-BY-SA 4.0 不能替代代码许可。论文 §4.1 使用 Homie：上身目标关节状态与 `[vx,vy,yaw_rate,height]` 输入，输出腿部目标。仅借鉴流程，不把论文接口当成可调用模块。 |
| GRAIL | [公开仓库](https://github.com/NVlabs/GRAIL)、[论文 2606.05160](https://arxiv.org/abs/2606.05160) | 实际 package 入口 `python -m grail.pipelines.*`；已有本地 reconstruction / retarget / SONIC 工程。主代码 [NVIDIA 非商业许可](https://github.com/NVlabs/GRAIL/blob/main/LICENSE)，子模块各自许可。本轮直接复用本地 real-to-sim 资产与渲染器，不重新生成视频或训练策略。 |
| SONIC / GR00T WBC | [官方仓库](https://github.com/NVlabs/GR00T-WholeBodyControl)、[模型说明](https://github.com/NVlabs/GR00T-WholeBodyControl#model-card) | 源代码 Apache-2.0，权重 NVIDIA Open Model License。SONIC 是全身参考跟踪控制器，支持 G1/SMPL/teleop 参考；同仓库的 Decoupled WBC 是另一种控制路线，不能混称。官方上身/腕部支持不是本箱子带载步行验证。 |

## 本机实际 SONIC 接口

复用 `g1_closed_loop/sonic_planner_goals.py:SonicONNX.predict` 并实际运行 `check_sonic_planner.py`：输入 `context_mujoco_qpos[1,4,36]`、速度、mode、移动/朝向向量、随机种子等；输出有效 64 帧 G1 qpos，30 Hz重采样为50 Hz。正负方向指令均改变输出方向，四元数归一化检查通过。它是参考生成器，**不是直接腿部执行动作接口**。

本地 `SonicPlannerIK` 在测得根部状态上重新锚定移动参考，并对上身参考加入有限 IK；随后由 SONIC 低层跟踪。其实现与 Homie 的上身直控＋腿部策略接口不同。要加入 C，仍须验证上身命令权限、腕部误差、持物质量扰动、脚接触/平衡及抓握保持，并从实际到达状态接续 A；目前完整任务尝试数为0。

SONIC 本地相关代码仍依赖原控制工程，`g1-realtosim/integrations/` 自身不是独立控制器。本轮没有以绑定物体或绕过低层执行的方式假装带载成功。

## 本地资产许可

办公室是本用户照片驱动的既有资产；仓库尚未选择公开发行许可，见 [NOTICE.md](../NOTICE.md)。箱子扫描来自 [BEHAVE](https://virtualhumans.mpi-inf.mpg.de/behave/)，沿用先前登记的[研究用途许可证](https://virtualhumans.mpi-inf.mpg.de/behave/license.html)，不能据此宣称可无限制商业发布。Isaac、Panda 描述和网格按其安装包许可使用，不复制进源码 Git。运行输出留在本机实验目录。
