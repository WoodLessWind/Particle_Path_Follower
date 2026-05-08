# Particle Path Follower

一个用于 Unity 的粒子路径跟随扩展脚本。它会让附加了 ParticleSystem 的粒子沿多段三次贝塞尔曲线运动，并提供可视化编辑器来直接调整路径节点、横截面偏移、真空区域和段落过渡。

## 功能

- 粒子可按真实路径长度匀速推进，避免在曲率变化处堆积。
- 支持路径速度曲线，能够让粒子在不同进度区间拥有不同速度倍率。
- 支持 OneShot、Loop、PingPong 三种路径行进模式。
- 支持横截面偏移模式：None、Random、Repeat、PingPong。
- 支持圆形或椭圆形横截面映射。
- 支持内部真空区裁切，避免中心区域生成粒子。
- 提供 Scene 视图手柄，支持拖拽、吸附和对称联动编辑。

## 文件说明

- [ParticlePathFollower.cs](ParticlePathFollower.cs)：运行时逻辑
- [ParticlePathFollowerEditor.cs](ParticlePathFollowerEditor.cs)：Unity 编辑器扩展

## 安装

1. 将这两个脚本放入 Unity 项目的任意脚本目录中。
2. 在场景中新建或选中一个粒子对象，确保它挂载了 ParticleSystem。
3. 给该对象添加 ParticlePathFollower 组件。
4. 将 ParticlePathFollowerEditor.cs 放入 Unity 项目的 Editor 目录下，才能启用 Scene 视图手柄和 Inspector 扩展。

## 基本使用

1. 选中带有 ParticlePathFollower 的物体。
2. 在 Inspector 中编辑路径节点
3. 使用“添加路径节点”扩展路径，使用“自动计算平滑”快速生成平顺连接。
4. 在 Scene 视图中拖动锚点和控制柄，调整曲线形状。
5. 选中某一段的截面后，可直接编辑局部偏移范围和真空参数。
6. 启用预发射，在开始时就让粒子布满线段

## 参数说明

### 运动设置

- `speed`：粒子沿路径移动速度。
- `pathTravelMode`：路径行进模式。
- `speedOverPath`：沿路径进度的速度倍率曲线。
- `includeCurveInLifetime`：生命周期是否按速度曲线一起计算。
- `speedSampleCount`：速度曲线积分采样数。
- `autoSetLifetime`：是否自动根据路径长度和速度调整粒子生命周期。
- `alignToPath`：是否让粒子朝向路径前进方向。
- `overallRotationCompensation`：粒子偏移补偿。

### 偏移设置

- `offsetMode`：横截面偏移模式。
- `enableInnerVacuum`：是否启用内部真空区。
- `circularShape`：是否将偏移范围映射为圆形或椭圆形。
- `offsetFrequency`：Repeat 和 PingPong 模式的频率。

### 截面数据

- `startOffsetData`：起始点横截面数据。
- `applyInitialOffsetToWholePath`：是否让整条路径都使用起始横截面。
- `segmentOffsets`：各段路径的横截面数据数组。

## 编辑器操作

- 点击锚点或控制柄进行选中。
- 按住Shift点击线段可在线段上直接添加锚点
- 选中后可直接拖动位置手柄修改节点。
- 选中控制柄按住 Shift 可单独调整一边
- 选中控制柄后按住Shift选中另一边的控制点可恢复两边平滑
- 选中段落中心手柄可调整该段的局部截面。
- 按 Delete 或 Backspace 可删除当前选中的节点。

## 运行逻辑概览

脚本会先根据贝塞尔曲线预计算路径长度和切线缓存，再把粒子的年龄映射到路径距离，最后根据偏移模式在路径切线垂直方向上叠加横截面偏移。这样做的目标是让粒子在复杂曲线上仍保持更平滑、更稳定的运动表现。

## 注意事项

- 控制点至少需要 4 个，且通常以 3 个点为一组扩展新段。
- `autoSetLifetime` 开启时，脚本会在运行中自动修改 ParticleSystem 的 startLifetime。

## 示例工作流

1. 创建一个 ParticleSystem。
2. 挂载 ParticlePathFollower。
3. 编辑控制点，构建一条路径。
4. 调整 `offsetMode` 和截面范围，决定粒子在路径周围的分布。
5. 开启 `autoSetLifetime` 和 `alignToPath`，让粒子沿路径稳定运动。

## 许可证

本项目采用 [MIT License](LICENSE)。

