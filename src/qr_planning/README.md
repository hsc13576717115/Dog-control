# qr_planning：落足几何与轨迹原语

**中文** | [English](README_EN.md)

`include/qr_planning/Geometry.hpp` 提供凸区域裕量判断、三足几何中心和五次连续插值。空区域、非有限坐标、错误绕序不能作为可行支撑面。

完整单步服务当前位于 `custom_dog_control/src/precision/PrecisionRuntime.cpp` 的非实时侧，复用 Pinocchio 和 WBC 做采样预检；本包不是独立的通用落足搜索节点。测试入口：`ctest --test-dir build/qr_planning --output-on-failure`。

构建、架构与实际验收状态见 [QR 开发文档](../../docs/qr/README.md)。
