# 仓库整理验证

日期：2026-09-21。本次输出保存在源码之外：`../../experiments/20260921_repository_validation/`。

## 实际执行

| 检查 | 结果 |
|---|---|
| CLI doctor | 成功；发现既有Isaac/pxr环境，LeRobot在独立venv |
| 照片/视频ingest | 原办公桌照片读取、EXIF规范化、SHA256和计划输出成功；视频抽取3帧参考成功 |
| 合成scene.json构建 | 米制Z-up USD创建成功；这不是新增真实场景重建 |
| 合成USD打包 | 成功 |
| 真实办公桌USD打包+换目录解析 | 成功；4个USD层、11个资源依赖来自新目录；OmniGlass.mdl由Isaac提供 |
| 新RT2 CLI | 实际渲染16帧成功，完整解码通过；查看末帧 |
| 新PT128 CLI | 实际渲染接触段3帧成功，完整解码通过；查看末帧 |
| 完整轨迹HDF5导出 | 631帧，state137维，action[2,882]，末帧mask保留不完整动作块 |
| LeRobot0.4.3导出 | 630帧；丢弃唯一末尾不完整动作块，作为未验收开发数据 |
| LeRobot官方loader读回 | 第0、300、629帧图像可读取，state/action与HDF5逐值一致 |
| 契约单元测试 | 4项通过：时间戳错配拒绝、物理子步错配拒绝、末尾mask及子步保真、venv解释器路径 |
| Python编译检查 | 核心代码、配方和参考集成通过 |
| Skill格式验证 | 4份skill通过官方quick_validate |

初次LeRobot子进程因Python软链接被resolve到了基础环境，未找到lerobot；已修复为保留venv解释器路径，并增加契约测试。原失败日志保留在验证目录。

## 未执行/未宣称

- 未重新调用Hunyuan生成模型；参数化配方仅编译检查，仍需原兼容模型环境。
- 未重新跑完整办公桌照片拟合与Blender建模；保留的是特定照片配方，不是泛化重建模型。
- 未迁移完整在线SONIC/GRAIL控制工程，未执行新的物理抓取或新策略训练。
- 连续视频原生帧率感知检查尚未完成，accepted=false。LeRobot可读取只验证格式和数值，不代表训练质量通过。
- 依赖包的跨机器GPU兼容性未验证；本机路径在忽略的local配置中。

机器可读证据：验证目录中的`asset_video_validation.json`、`lerobot_readback.json`、`aligned/validation.json`、各render的`complete.json`与`visual_realism_review.json`。
