# qr_course：规则与已知测试场景

**中文** | [English](README_EN.md)

`config/rules.yaml` 保存规则名义尺寸、PDF 页码和待裁判确认问题。`scripts/course.py` 生成 M1 的 0/30/50 mm 平台 SDF 与描述文件；`known_surfaces.py` 由同一几何定义发布支撑面和 RViz 标记。

这是显式已知几何输入，不是感知结果；首期没有生成完整八项比赛场地。原始 `ROBOCON2027.xml` 的尺寸及缺失资源差异见开发文档，不能用它宣称比赛验收通过。

构建、架构与实际验收状态见 [QR 开发文档](../../docs/qr/README.md)。
