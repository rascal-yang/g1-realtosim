# 当前机器的工具复用线索

2026-09-20 检查了以下路径及既有源码/报告。它们是发现工具的起点；每次运行重新检查文件、版本和依赖，不保证未来位置或运行状态不变。本次 skill 创建未重新执行模型推理。

## 已存在的工具与环境

| 用途 | 位置 |
|---|---|
| Blender，可不在 PATH 中 | `${HOME}/Projects/GRAIL/imports/blender/blender` |
| Hunyuan 推理 Python | `${HOME}/code/zijian/g1_closed_loop/.deps/plush-recon/bin/python` |
| Hunyuan3D-2 源码 | `${HOME}/code/zijian/g1_closed_loop/.deps/Hunyuan3D-2/` |
| 本地模型权重 | `${HOME}/code/zijian/g1_closed_loop/.deps/hunyuan-models/` |
| USD / Isaac Python | `${HOME}/miniconda3/envs/sonic/bin/python` |

既有记录是 Blender 4.3.2、Hunyuan3D-2 full shape + paint turbo、Isaac Sim 5.1。以当前实际探测结果为准，不把旧版本记录当成自动兼容保证。

## 可参考的既有实现

根目录：`${HOME}/code/zijian/g1_closed_loop/`。

- `plush_asset/README.md`：历史完整流程、环境隔离、材质修正与已验证范围。
- `plush_asset/reconstruct.py`：单物体形状生成、纹理生成、种子与结果记录。
- `plush_asset/build_blender.py`：导入、尺度规范化、PBR 和视觉绒毛。
- `plush_asset/project_photo.py`：原照片可见区域投影融合；含原玩偶专用标定，不能原样推广。
- `plush_asset/prepare_collisions.py`、`package_usd.py`、`capture_isaac.py`、`validate_delivery.py`：碰撞、打包、目标引擎检查。
- `office_asset/fit_structure_sfm.py`、`build_blender.py`：多视角约束和规则结构构建的参考。

**这些是特定任务脚本，不是通用 CLI。** 已检查的 `reconstruct.py` 和 `build_blender.py` 把输出写到 `data/wanou_v1`，并含特定输入、尺寸和模型路径。复用前将输入、输出、模型缓存、尺寸和种子显式参数化，在新任务目录运行；不要直接执行原脚本覆盖历史资产，也不要只复制后依赖已改变含义的 `__file__` 相对路径。

## 已知需要检查的地方

- 历史 CoACD 与 USD 库在同一进程使用曾崩溃。沿用隔离环境与独立子进程，使用文件传递网格/参数。
- 历史 Hunyuan 纹理遗漏眼睛、手套图案；原照片投影用于修正。后续对象同样需要查看生成结果，不能默认纹理正确。
- glTF/Blender/USD 的轴向和材质转换需要逐步确认。历史 Blender 导出曾丢失 sheen 效果；专用修复值不应无条件套用其他对象。
- 原刚性玩偶的高度、质量、摩擦和毛发参数是估计；不能作为新物体的已测量默认值。
- 原脚本有历史驱动库和权重转换处理。核实当前是否需要，避免将临时路径或模型特定补丁推广到所有环境。
- 图像生成能力从当前会话发现；不假定任何用户提及的 GPT/Image 模型名称就是可调用的具体接口。

需要更新接口或新增依赖时查阅官方文档，并记录实际版本：

- [Hunyuan3D-2](https://github.com/Tencent-Hunyuan/Hunyuan3D-2)
- [Blender Python API](https://docs.blender.org/api/)
- [Isaac Sim 5.1 文档](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/)
