# 从照片/视频到仿真资产

## Agent负责的环节

`ingest`只做素材登记、EXIF方向归一化与视频参考帧抽取。Agent读取图像后建立物体清单和带来源的尺寸/布局估计，按`skills/scene-asset-router`选择工具。未经标定的普通手机照片没有可靠绝对尺度，应明确使用桌高等假设或请求尺度锚点。

- 墙、地板、窗框、桌腿：Blender规则几何、尺度及布局拟合。
- 瓶子：旋转体建模、真实照片标签、透明材质分层。
- 玩偶等复杂轮廓：Hunyuan形状与纹理生成，再做尺度、UV和原照片对照；遮挡面是推断。
- 已有可用模型：复用并检查许可证、坐标轴、材质与碰撞依赖。

图生3D只解决部分形状/纹理任务。质量、重心、摩擦、弹性没有测量时保留估计标签；PBR粗糙度不等于摩擦系数。玩偶采用刚体代理时必须明确标注。

## 交付

每个物体至少记录：大小与尺度依据、世界摆放和支撑关系、材料及PBR参数、纹理/UV来源、视觉网格、碰撞体、物理参数和不确定性。对照真实照片使用匹配视角，检查窗户遮挡、桌高、目标大小与接近空间。

Agent可输出`scene.json`，参考`configs/scene.example.json`。USD对象引用需要先在Blender/USD中完成UV和材质；`build-scene`仅为原始几何提供标量材质，不会用默认颜色覆盖引用对象原有贴图，也不会伪装支持可形变体。引用对象的物理参数由引用USD定义，JSON中的physics用作声明，需检查实际属性；基础构造器未自动改写引用资产内部物理属性。

`package-assets`收集递归USD/贴图依赖，改写相对路径，重新打开检查。内置MDL由Isaac安装提供。UDIM、远程Nucleus资产或自定义MDL复杂依赖不属于当前已验证打包范围；不支持的依赖应报错，不能忽略。

## 已有办公桌案例配方

`recipes/office_photo_scene/*.py`保留原办公桌的像素标注、尺度假设、桌面贴图裁切与窗口拟合。它们**只适用于这组照片**，不能把这些像素坐标直接用于新场景。

工作目录需包含`inputs/photo_00.jpg`到`photo_06.jpg`、`assets/`和`reports/`。设定：

```bash
export REALTOSIM_CASE_DIR=/new/office_case
export REALTOSIM_SOURCE_MEDIA=/path/to/original_media
export REALTOSIM_PLUSH_ASSET=/path/to/existing/plush_asset
```

执行顺序：`fit_layout.py` → `fit_window.py` → `prepare_assets.py` → Blender执行`build_scene_blender.py` → USD Python执行`package_scene.py` → `build_bottle.py` → `write_manifest.py`。打包需要由Agent准备`assets/plush_placement.json`（translation_m、quaternion_wxyz、scale）；原配方中的材质和光照参数是该案例的估计值。Blender应运行 `blender -b --python recipes/office_photo_scene/build_scene_blender.py`。

重建完成后做Isaac实际查看、碰撞/落桌及机器人相机检查。`build-scene`和依赖打包成功本身不代表物理或外观验收通过。
