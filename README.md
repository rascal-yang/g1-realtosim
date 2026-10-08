# G1 Real-to-Sim

从真实场景照片/视频、已重建资产和 G1 仿真轨迹，生成 Isaac Sim 资产、第一视角视频与对齐的视觉模仿学习数据。

这是从已完成实验提炼的 **Agent 辅助工作流 + 可执行数据工具**。照片的几何、尺度和材质判断由 Agent 按 skills 路由到 Blender、图生 3D 或观测拟合；没有声称任意照片都能一键精确重建。重渲染和导出部分已参数化，默认采用 RT2 + 原生 DLAA。

```text
照片 / 视频 → 素材登记 → Agent 建模路由 → USD / PBR / 物理属性与证据
                                                    ↓
已有 G1 轨迹 + 匹配 Isaac USD → 保存状态重渲染 → RGB + 状态 + 实际控制指令
                                                    ↓
                                      对齐 HDF5 → LeRobot 0.4.3
```

## 快速开始：当前电脑

```bash
cd ~/code/zijian/g1-realtosim
conda activate sonic
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
python -m g1_realtosim --runtime configs/local.runtime.json doctor

# 已配置本机 walk_03_far 原始轨迹；新输出目录必须不存在
python -m g1_realtosim --runtime configs/local.runtime.json render \
  --episode configs/local.episode.json \
  --output ../experiments/my_replay/rt2 --profile rt2

python -m g1_realtosim export \
  --raw ../experiments/20260921_five_directions_128/runs/walk_03_far/episode_raw.h5 \
  --frames ../experiments/my_replay/rt2/frames.h5 \
  --output ../experiments/my_replay/aligned

python -m g1_realtosim --runtime configs/local.runtime.json export-lerobot \
  --aligned ../experiments/my_replay/aligned/episode.h5 \
  --output ../experiments/my_replay/lerobot --allow-unreviewed
```

最后一个命令是明确标记的开发导出；现有视频尚未完成原生帧率视觉验收，Revo 2 控制契约也保留审查状态，不能仅因格式可读取就标为训练质量通过。`episode.h5` 的数值与 RGB 自包含，不依赖源文件外链；原始高频遥测留在原实验归档中。

新电脑复制 `configs/*.example.json` 为本地配置，修改 Python、USD、轨迹路径。`local*.json` 不进入 Git。可使用 `python -m pip install -e . --no-deps` 安装 CLI；不要往正常工作的 Isaac 环境盲目安装另一版 torch/LeRobot。

## 接收新的场景

```bash
python -m g1_realtosim ingest /path/to/photos_or_video ../asset_tasks/my_room
```

输出原图/视频来源及 SHA256、归一化参考图、待填的重建计划。然后让 Agent 使用仓库内 `skills/scene-asset-router`，逐物体记录尺寸、摆放、材质、贴图、物理估计及证据。规则物体可用 `build-scene`；复杂有纹理物体用 Blender 或 Hunyuan 生成后导出 USD，再组装。

```bash
# 合成结构示例；不是照片自动重建
python -m g1_realtosim build-scene configs/scene.example.json ../asset_tasks/example_scene
# 把 USD 引用和贴图改写为相对路径，生成可搬迁目录
python -m g1_realtosim package-assets /path/to/scene.usda ../asset_tasks/portable_scene
```

原办公桌照片专用建模配方在 `recipes/office_photo_scene/`；复杂物体图生 3D 参数化入口在 `recipes/hunyuan/reconstruct.py`。详见 [建模流程](docs/RECONSTRUCTION.md)。

## 仓库结构

| 路径 | 内容 |
|---|---|
| `src/g1_realtosim/` | 素材登记、USD组装/打包、RT2/PT128渲染、同步校验、HDF5/LeRobot导出 |
| `configs/` | 可共享的示例配置；不跟踪本机配置 |
| `skills/` | 建模路由、抓取运行审查、视觉真实性、数据流水线经验 |
| `recipes/` | 已验证场景的建模配方、Hunyuan形状与纹理入口 |
| `integrations/` | SONIC planner、Revo2、原始采集器的关键参考实现；依赖见说明 |
| `docs/` | 环境、数据契约、输入限制、来源、基准和验证记录 |
| `tests/` | 防止时间错配、末尾补帧及动作替代错误的契约测试 |

输出统一放 `../experiments/<实验名>/` 或 `../asset_tasks/<任务名>/`，不混入源码。原有实验、资产、权重和照片没有移动或删除。

## 支持范围

- 当前渲染适配器支持已有 `g1_capture_v1`：G1逐刚体世界位姿、双手位姿、所有记录动态物体、相机标定及匹配USD。只给关节角/根轨迹时，需先经过仿真采集，不能凭空补出物体接触或实际控制指令。
- 当前相机：640×480，原有中心投影标定；50 Hz控制按步长2导出25 FPS。不任意裁剪或缩放后沿用旧内参。
- 任意新场景的动态物体必须全部映射到 `scene_prims`，不能只还原机器人和瓶子。
- 这里的 replay 不推进物理，不代表重新抓取成功。
- SONIC/GRAIL权重、Isaac Sim、Blender和图生3D模型是外部依赖；本仓库没有独立 GPT 服务或凭据。Agent来自交互会话。
- 成功轨迹与训练质量分开记录，历史实验没有被追认成通过视觉验收。

## 已完成验证与性能

完整25.24秒、631帧：RT2输出23.14秒，优化PT128输出664.83秒（均不含启动预热），约28.73倍；这是同轨迹单次测量。参见 [基准与历史资产索引](docs/RESULTS.md)。

本次整理实际执行了新仓库的渲染、办公桌USD依赖打包、完整轨迹HDF5导出和LeRobot读回验证。准确结果见 [整理验证记录](docs/VALIDATION.md)。

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
```

[数据契约](docs/DATA_CONTRACT.md) · [环境说明](docs/ENVIRONMENT.md) · [来源与许可边界](NOTICE.md)

## 人体交互视频到 G1

固定基座双臂搬箱与同资产位置/朝向扩增的实际入口见 [LOCOMANIP_BASELINE.md](docs/LOCOMANIP_BASELINE.md)。该 baseline 使用办公室资产、Panda 双臂、PhysX 接触验证和现有 RT2 渲染；尚未验证 G1 站立或持箱步行。

执行提示与验收要求见 [HUMAN_VIDEO_TO_SIM_PROMPT.md](docs/HUMAN_VIDEO_TO_SIM_PROMPT.md)，涵盖全身重建、独立物体跟踪、重定向、物理支撑基线与视觉质量检查。该文档不表示端到端抓取已通过。

G1 提篮的物理种子与扩增入口见 [G1 篮子 baseline](docs/G1_BASKET_BASELINE.md)。

G1 抓取—搬移—放置与独立目标扩增见 [篮子放置实验](docs/G1_BASKET_PLACE.md)。

GPT编写动作规格后，参数化生成Blender动画及G1参考轨迹的入口见 [动画自动化](docs/G1_ANIMATION_AUTOMATION.md)；可复用[任务提示词](docs/G1_ANIMATION_TASK_PROMPT.md)。当前技能族限定原办公室右手提篮，几何导出通过不等于物理成功。

## 渲染配置与对比

当前RT2原生DLAA默认普通状态1次更新，支持2/4次对照；首次采集预热单独处理。参数、历史A/B/C/D与最新D4/D2/D1计时、画质边界和运行命令见 [渲染说明](docs/RENDERING.md)，[四路历史视频](media/rendering_comparisons/abcd_20260921.mp4)已作为用户授权的样例纳入Git。保持视觉未验收标签；Git源码之外仍需对应场景/模型资源。
