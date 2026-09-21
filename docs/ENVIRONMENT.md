# 环境与外部依赖

已使用环境：Ubuntu22.04、Python3.11、Isaac Sim5.1、Isaac Lab2.3.2、SONIC conda环境；导出使用另一个venv中的LeRobot0.4.3。没有为整理仓库升级系统驱动、torch或Isaac。

## 安装方式

基础CPU处理需pyproject中的依赖，以及PATH中的ffmpeg/ffprobe。USD操作使用已经具有`pxr`的Python；Isaac重渲染必须使用安装了Isaac Sim的解释器，PT128启动还依赖Isaac Lab。先用`doctor`检查，再用`render --count 3`做小验证。

LeRobot建议单独venv，使用已经验证的兼容torch环境，再安装`lerobot==0.4.3 av==15.1.0`。不要将requirements直接安装进正常运行的Isaac环境。可沿用原先`.venv-export`；解释器路径必须保留venv软链接，不能resolve到基础Python。

本地路径集中于忽略的`configs/local.runtime.json`。例如：

```json
{"isaac_python":"${HOME}/miniconda3/envs/sonic/bin/python","export_python":"../.venv-export/bin/python","env":{}}
```

历史机器曾有内核NVIDIA580.173.02与用户库580.178.04不匹配，匹配库临时存于`/tmp/g1-driver-580.173.02/usr/lib/x86_64-linux-gnu`。本机配置显式使用该目录；换机或重启后重新检查，不能把临时目录当作永久依赖。CLI只给子进程设置已配置的环境，不更改系统包。

## 建模

Blender本机位置可在`skills/scene-asset-router/references/local-tools.md`查找。它未必在PATH中。

Hunyuan脚本：

```bash
/path/to/hunyuan/python recipes/hunyuan/reconstruct.py \
  --input /path/to/isolated_rgba.png --output /new/object_output \
  --source-root /path/to/Hunyuan3D-2 --models /path/to/hunyuan-models
```

形状和paint独立阶段分别使用`--stage shape`、`--stage paint`。paint需要已经生成的shape.glb。输出形状/纹理/推断视图以及生成记录。此入口由历史成功配方参数化，本次仓库整理未重新运行昂贵的模型推理。

历史Hunyuan paint loader使用模型本地.bin兼容缓存；脚本不会自动改写共享权重。已有模型转换记录的机器可以继续使用；新机器必须按对应上游版本建立兼容环境。图生3D没有自动推断真实尺寸、摩擦或弹性的保证。

## 在线控制

本仓库当前的主要输入是**已有采集轨迹**。无需SONIC/GRAIL推理即可重渲染和导出；生成新物理轨迹仍需原控制工程、机器人资产、权重与依赖。

`integrations/`保留关键接口参考，不能直接当成已经独立迁移的完整控制器。原控制入口、成功配置、资产与日志路径集中在RESULTS和source_inventory。后续迁移在线控制时逐项移除旧工程耦合并做新的物理测试，不在源码整理中暗改历史动作。
