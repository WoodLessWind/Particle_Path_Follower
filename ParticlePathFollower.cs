using System;
using UnityEngine;

namespace Script.Particles
{
    /// <summary>
    /// 粒子路径跟随组件。
    /// 使附带的 ParticleSystem 粒子沿指定的贝塞尔曲线路径运动，并支持横截面的多种偏移模式与真空形状映射裁切。
    /// </summary>
    [RequireComponent(typeof(ParticleSystem))]
    [ExecuteAlways]
    public class ParticlePathFollower : MonoBehaviour
    {
        /// <summary>
        /// 粒子垂直于路径前向的偏移状态模式
        /// </summary>
        public enum OffsetMode
        {
            None,
            Random,
            Repeat,
            PingPong
        }

        [HideInInspector] public Vector3[] controlPoints =
        {
            new(0, 0, 0),
            new(0, 5, 0),
            new(5, 5, 0),
            new(5, 0, 0)
        };

        [Header("运动设置")] [Tooltip("粒子沿路径移动的速度")]
        public float speed = 1.0f;

        [Tooltip("是否自动根据路径长和速度计算并修改粒子的生命周期")] public bool autoSetLifetime = true;

        [Tooltip("是否改变粒子的朝向（使其对齐路径的前进方向）")] public bool alignToPath = true;

        [Header("偏移设置")] [Tooltip("垂直于路径的偏移模式")]
        public OffsetMode offsetMode = OffsetMode.None;

        [Tooltip("偏移范围（X/Y：水平/左右方向的最小与最大值，Z/W：垂直/上下方向的最小与最大值）")]
        public Vector4 offset = Vector4.zero;

        [Tooltip("是否启用内部真空区（在此范围内不会生成和偏移粒子）")] public bool enableInnerVacuum;
        [Tooltip("是否将偏移范围与真空区映射为圆/椭圆形")] public bool circularShape;
        [Tooltip("重复/来回模式的频率")] public float offsetFrequency = 1.0f;

        [Serializable]
        public class PathOffsetData
        {
            [Tooltip("偏移范围（X/Y：左右最小最大，Z/W：上下最小最大）")]
            public Vector4 offset = new Vector4(1f, -1f, 1f, -1f);

            [Tooltip("水平方向的真空区百分比")] [Range(0f, 1f)]
            public float innerVacuumX = 0f;

            [Tooltip("垂直方向的真空区百分比")] [Range(0f, 1f)]
            public float innerVacuumY = 0f;
        }

        [HideInInspector]
        public PathOffsetData[] segmentOffsets = { new PathOffsetData() };

        private ParticleSystem.Particle[] _particles;

        private ParticleSystem _particleSystem;
        private float[] _pathDistances;
        private Vector3[] _pathForwards;

        /// <summary>
        /// 缓存路径坐标数据表，用于实现平滑的匀速运动和距离查值。
        /// </summary>
        private Vector3[] _pathPoints;

        /// <summary>
        /// 贝塞尔曲线的实际物理累计总长。
        /// </summary>
        private float _totalPathLength;

        private void LateUpdate()
        {
            InitializeIfNeeded();
            UpdatePathCache();

            // 动态设置粒子的初始生命周期，使其刚好在到达路径终点时（或按设定速度走完全程）消亡
            if (autoSetLifetime && speed > 0.001f)
            {
                var main = _particleSystem.main;
                main.startLifetime = _totalPathLength / speed;
            }

            var count = _particleSystem.GetParticles(_particles);

            for (var i = 0; i < count; i++)
            {
                // 用粒子的真实存活时间乘以速度得到实际应该行走的物理距离
                var age = _particles[i].startLifetime - _particles[i].remainingLifetime;
                var distance = age * speed;

                Vector3 forwardDir;
                float segIdx;
                var newPos = GetPointAtDistance(distance, out forwardDir, out segIdx);
                var currentEffectiveSpeed = speed; // 记录实际速度

                if (offsetMode != OffsetMode.None)
                {
                    var currentOffsetX = 0f;
                    var currentOffsetY = 0f;

                    var segData = EvaluateOffsetData(segIdx);

                    // 计算该粒子的相对固定出生时间，以此作为重复与来回的依据，保证单颗粒子一生中的偏移保持固定，而不是在路径上扭波
                    var spawnTime = Time.time - age;

                    switch (offsetMode)
                    {
                        case OffsetMode.Random:
                            // 利用粒子自身的随机种子固定一个随身偏移量，以防止每帧闪烁
                            var randomT1 = _particles[i].randomSeed % 10000 / 10000f;
                            var randomT2 = _particles[i].randomSeed / 10000 % 10000 / 10000f; // 获取另一段随机值
                            var offRand = CalculateOffset2D(randomT1, randomT2, segData);
                            currentOffsetX = offRand.x;
                            currentOffsetY = offRand.y;
                            break;
                        case OffsetMode.Repeat:
                            var tRepeat = spawnTime * offsetFrequency % 1f;
                            if (tRepeat < 0f) tRepeat += 1f; // 确保正数区间
                            // 将沿对角线的直线移动改为“以中心进行圆周旋转”的方位参数
                            var angleR = tRepeat * Mathf.PI * 2f;
                            var t1R = Mathf.Cos(angleR) * 0.5f + 0.5f;
                            var t2R = Mathf.Sin(angleR) * 0.5f + 0.5f;
                            var offRepeat = CalculateOffset2D(t1R, t2R, segData);
                            currentOffsetX = offRepeat.x;
                            currentOffsetY = offRepeat.y;
                            break;
                        case OffsetMode.PingPong:
                            var tPingPong = Mathf.PingPong(spawnTime * offsetFrequency, 1f);
                            // 来回状态下，表现为围绕中心旋转到底后再反向旋转的来回扫描
                            var angleP = tPingPong * Mathf.PI * 2f;
                            var t1P = Mathf.Cos(angleP) * 0.5f + 0.5f;
                            var t2P = Mathf.Sin(angleP) * 0.5f + 0.5f;
                            var offPing = CalculateOffset2D(t1P, t2P, segData);
                            currentOffsetX = offPing.x;
                            currentOffsetY = offPing.y;
                            break;
                    }

                    if ((currentOffsetX != 0f || currentOffsetY != 0f) && forwardDir != Vector3.zero)
                    {
                        // 计算基于路径前向的左右和上下垂直向量
                        // 使用 Z 轴为正统基准，确保二维世界中 right 真正指向轨迹右侧
                        var right = Vector3.Cross(forwardDir, Vector3.forward).normalized;
                        if (right == Vector3.zero) right = Vector3.right;
                        var up = Vector3.Cross(right, forwardDir).normalized;

                        newPos += right * currentOffsetX + up * currentOffsetY;

                        // 差分采取靠近的一小步预测下一个点的位置，进而得出偏移后真实的运动方向和速度率
                        var delta = 0.05f;
                        Vector3 nextForwardDir;
                        float nextSegIdx;
                        var nextBasePos = GetPointAtDistance(distance + delta, out nextForwardDir, out nextSegIdx);

                        var nextRight = Vector3.Cross(nextForwardDir, Vector3.forward).normalized;
                        if (nextRight == Vector3.zero) nextRight = Vector3.right;
                        var nextUp = Vector3.Cross(nextRight, nextForwardDir).normalized;

                        var nextPos = nextBasePos + nextRight * currentOffsetX + nextUp * currentOffsetY;

                        // 偏移后的前进方向
                        var offsetForward = (nextPos - newPos).normalized;
                        if (offsetForward != Vector3.zero) forwardDir = offsetForward;

                        // 距离差除以 delta 表示基于中心路径速度的偏移速度比率（外侧弧长长会大于1，内侧弧长短会小于1）
                        var speedMultiplier = Vector3.Distance(nextPos, newPos) / delta;
                        currentEffectiveSpeed = speed * speedMultiplier;
                    }
                }

                // 将本地坐标系转换到粒子系统的模拟空间（或世界空间）
                if (_particleSystem.main.simulationSpace == ParticleSystemSimulationSpace.Local)
                    _particles[i].position = newPos;
                else
                    _particles[i].position = transform.TransformPoint(newPos);

                if (alignToPath && distance <= _totalPathLength)
                    if (forwardDir != Vector3.zero)
                    {
                        // 简单转换朝向（假设是2D旋转，如果是3D粒子需要设置3D旋转）
                        var angle = Mathf.Atan2(forwardDir.y, forwardDir.x) * Mathf.Rad2Deg;
                        _particles[i].rotation = angle;
                        _particles[i].velocity = forwardDir * currentEffectiveSpeed; // 更新速度向量，利用真实速度
                    }
            }

            _particleSystem.SetParticles(_particles, count);
        }

        private void InitializeIfNeeded()
        {
            if (_particleSystem == null) _particleSystem = GetComponent<ParticleSystem>();

            var maxParticles = _particleSystem.main.maxParticles;
            if (_particles == null || _particles.Length < maxParticles)
                _particles = new ParticleSystem.Particle[maxParticles];
        }

        /// <summary>
        /// 预计算并构建距离分布缓存查找表，将基于曲线 T 值的控制参数转为线性均匀的物理分布，防止粒子在曲率极变处堆积。
        /// </summary>
        /// <param name="stepsPerCurve">单段贝塞尔曲线的前向采样细分段数</param>
        private void UpdatePathCache(int stepsPerCurve = 25)
        {
            if (controlPoints == null || controlPoints.Length < 4) return;

            var curveCount = (controlPoints.Length - 1) / 3;
            var totalSteps = curveCount * stepsPerCurve;

            if (_pathPoints == null || _pathPoints.Length != totalSteps + 1 ||
                _pathForwards == null || _pathForwards.Length != totalSteps + 1 ||
                _pathDistances == null || _pathDistances.Length != totalSteps + 1)
            {
                _pathPoints = new Vector3[totalSteps + 1];
                _pathForwards = new Vector3[totalSteps + 1];
                _pathDistances = new float[totalSteps + 1];
            }

            _pathDistances[0] = 0f;
            var length = 0f;

            for (var i = 0; i <= totalSteps; i++)
            {
                var t = (float)i / totalSteps;
                _pathPoints[i] = EvaluateSplineRaw(t);

                // 向前微小采样以获取真实的平滑切向
                var forwardVec = EvaluateSplineRaw(t + 0.001f) - _pathPoints[i];
                if (forwardVec == Vector3.zero && i > 0) forwardVec = _pathPoints[i] - _pathPoints[i - 1];
                _pathForwards[i] = forwardVec.normalized;

                if (i > 0)
                {
                    length += Vector3.Distance(_pathPoints[i - 1], _pathPoints[i]);
                    _pathDistances[i] = length;
                }
            }

            _totalPathLength = length;
        }

        /// <summary>
        /// 基于全局二维极坐标系推演真实的横截面偏移量。
        /// 内置同心防形变量算法，解决了纯中心排除带来的十字形空缺伪影，并支撑了对于圆形缩放投影的挤压模拟。
        /// </summary>
        /// <param name="t1">第一轴向插值参数 (0~1)</param>
        /// <param name="t2">第二轴向插值参数 (0~1)</param>
        /// <returns>处理真空裁切及圆平滑折变后的最终 2D 实际偏移值</returns>
        private Vector2 CalculateOffset2D(float t1, float t2, PathOffsetData segData)
        {
            // 映射到 -1 到 +1 标准化包围盒
            var px = Mathf.Lerp(-1f, 1f, t1);
            var py = Mathf.Lerp(-1f, 1f, t2);

            // 1. 真空区处理 (2D 射线等比映射法) 避免分离处理产生的十字型空缺
            if (enableInnerVacuum)
            {
                var vx = Mathf.Clamp01(segData.innerVacuumX);
                var vy = Mathf.Clamp01(segData.innerVacuumY);

                if (vx > 0f || vy > 0f)
                {
                    var absPx = Mathf.Abs(px);
                    var absPy = Mathf.Abs(py);
                    var maxAbs = Mathf.Max(absPx, absPy);

                    if (maxAbs < 0.0001f)
                    {
                        px = vx; // 恰逢中心防止除零，偏移到边缘
                        py = 0f;
                    }
                    else
                    {
                        var tOuter = 1f / maxAbs;
                        var txInner = absPx > 0.0001f ? vx / absPx : float.MaxValue;
                        var tyInner = absPy > 0.0001f ? vy / absPy : float.MaxValue;
                        var tInner = Mathf.Min(txInner, tyInner);

                        var r = maxAbs;
                        var tNew = tInner + r * (tOuter - tInner);

                        px *= tNew;
                        py *= tNew;
                    }
                }
            }

            // 2. 圆/椭圆形状转换处理 (按同心比例将包裹射线的正方形盒子收缩并圆滑)
            if (circularShape)
            {
                var absPx = Mathf.Abs(px);
                var absPy = Mathf.Abs(py);
                var maxAbs = Mathf.Max(absPx, absPy);
                if (maxAbs > 0.0001f)
                {
                    var d = Mathf.Sqrt(px * px + py * py);
                    px = px * maxAbs / d;
                    py = py * maxAbs / d;
                }
            }

            // 3. 将最终得到的形状与镂空投射到真实世界偏移参数系
            var minX = Mathf.Min(segData.offset.x, segData.offset.y);
            var maxX = Mathf.Max(segData.offset.x, segData.offset.y);
            var minY = Mathf.Min(segData.offset.z, segData.offset.w);
            var maxY = Mathf.Max(segData.offset.z, segData.offset.w);

            var centerX = (minX + maxX) * 0.5f;
            var centerY = (minY + maxY) * 0.5f;
            var extentX = (maxX - minX) * 0.5f;
            var extentY = (maxY - minY) * 0.5f;

            var finalX = centerX + px * extentX;
            var finalY = centerY + py * extentY;

            // 补偿原始设定如果存在倒置的要求
            if (segData.offset.x > segData.offset.y) finalX = centerX - px * extentX;
            if (segData.offset.z > segData.offset.w) finalY = centerY - py * extentY;

            return new Vector2(finalX, finalY);
        }

        private PathOffsetData EvaluateOffsetData(float floatSegIdx)
        {
            if (segmentOffsets == null || segmentOffsets.Length == 0) return new PathOffsetData();
            if (segmentOffsets.Length == 1) return segmentOffsets[0];

            // 视每段偏移数据位于段落实体参数时间的中心(0.5, 1.5...)，对其进行连续线性插值过渡
            float shiftedIdx = floatSegIdx - 0.5f;
            if (shiftedIdx <= 0f) return segmentOffsets[0];
            if (shiftedIdx >= segmentOffsets.Length - 1) return segmentOffsets[^1];

            int idx = Mathf.FloorToInt(shiftedIdx);
            float t = shiftedIdx - idx;

            var d1 = segmentOffsets[idx];
            var d2 = segmentOffsets[idx + 1];

            return new PathOffsetData
            {
                offset = Vector4.Lerp(d1.offset, d2.offset, t),
                innerVacuumX = Mathf.Lerp(d1.innerVacuumX, d2.innerVacuumX, t),
                innerVacuumY = Mathf.Lerp(d1.innerVacuumY, d2.innerVacuumY, t)
            };
        }

        /// <summary>
        /// 根据粒子的真实移动物理距离查询其所处的贝塞尔曲线确切空间坐标与路径朝向。
        /// </summary>
        /// <param name="targetDistance">预计推进的物理距离(米)</param>
        /// <param name="forward">输出：此时该点指向前方平滑推移的切线向量</param>
        /// <param name="floatSegIdx">输出：此时所处曲线段落的连续浮点进度索引，用于平滑插值管径</param>
        /// <returns>推算所处在物理世界中的实际位置标点</returns>
        private Vector3 GetPointAtDistance(float targetDistance, out Vector3 forward, out float floatSegIdx)
        {
            if (_pathPoints == null || _pathPoints.Length < 2)
            {
                forward = Vector3.right;
                floatSegIdx = 0f;
                return Vector3.zero;
            }

            if (targetDistance <= 0f)
            {
                forward = _pathForwards[0];
                floatSegIdx = 0f;
                return _pathPoints[0];
            }

            var curveCount = (controlPoints.Length - 1) / 3;

            if (targetDistance >= _totalPathLength)
            {
                var last = _pathPoints.Length - 1;
                forward = _pathForwards[last];
                floatSegIdx = Mathf.Max(0, curveCount);
                return _pathPoints[last];
            }

            var totalSteps = _pathDistances.Length - 1;
            var stepPerCurve = totalSteps / (float)Mathf.Max(1, curveCount);

            for (var i = 1; i < _pathDistances.Length; i++)
                if (_pathDistances[i] >= targetDistance)
                {
                    var d0 = _pathDistances[i - 1];
                    var d1 = _pathDistances[i];
                    var t = (targetDistance - d0) / (d1 - d0);
                    var pos = Vector3.Lerp(_pathPoints[i - 1], _pathPoints[i], t);
                    // 核心修复点：将方向从“死板的折线段”改为插值平滑向量
                    forward = Vector3.Lerp(_pathForwards[i - 1], _pathForwards[i], t).normalized;

                    var exactStep = (i - 1) + t;
                    floatSegIdx = exactStep / stepPerCurve;
                    return pos;
                }

            var end = _pathPoints.Length - 1;
            forward = _pathForwards[end];
            floatSegIdx = Mathf.Max(0, curveCount);
            return _pathPoints[end];
        }

        /// <summary>
        /// 依据基础参数化演进率 t (0~1) 直接折算未经分布均化的第三次贝塞尔曲线估计点。
        /// </summary>
        /// <param name="t">整条串联曲线生命周期内的全长进度参数百分比</param>
        /// <returns>未进行修正的平滑折算顶点</returns>
        public Vector3 EvaluateSplineRaw(float t)
        {
            if (controlPoints == null || controlPoints.Length < 4) return Vector3.zero;

            var curveCount = (controlPoints.Length - 1) / 3;
            t = Mathf.Clamp01(t);
            if (MathF.Abs(t - 1f) < 0.0001f) return controlPoints[^1];

            // 根据 t 找出处于哪一段平滑曲线中
            var floatIndex = t * curveCount;
            var curveIndex = Mathf.FloorToInt(floatIndex);

            // 计算局部t
            var localT = floatIndex - curveIndex;
            var i = curveIndex * 3;

            return EvaluateCubicBezier(controlPoints[i], controlPoints[i + 1], controlPoints[i + 2],
                controlPoints[i + 3], localT);
        }

        /// <summary>
        /// 移除指定索引的路径节点（包含其控制曲柄），并自动修补连接两侧相邻的曲线。
        /// </summary>
        /// <param name="nodeIndex">节点索引（将自动换算为数组内步进）</param>
        public void DeleteNode(int nodeIndex)
        {
            if (controlPoints == null || controlPoints.Length <= 4) return; // 至少保留一段完整曲线（2个端点）

            var anchorIndex = nodeIndex * 3;
            if (anchorIndex < 0 || anchorIndex >= controlPoints.Length) return;

            int removeStart;
            int segmentToRemove;

            if (anchorIndex == 0)
            {
                // 删除起点：移除 [0], [1], [2]，以 [3] 成为新的起点
                removeStart = 0;
                segmentToRemove = 0;
            }
            else if (anchorIndex == controlPoints.Length - 1)
            {
                // 删除终点：移除尾部三个点
                removeStart = anchorIndex - 2;
                segmentToRemove = (controlPoints.Length - 1) / 3 - 1;
            }
            else
            {
                // 删除中间节点：移除 [i-1], [i], [i+1]，直接短接前后两条贝塞尔控制柄
                removeStart = anchorIndex - 1;
                segmentToRemove = nodeIndex;
            }

            var newPoints = new Vector3[controlPoints.Length - 3];
            Array.Copy(controlPoints, 0, newPoints, 0, removeStart);
            Array.Copy(controlPoints, removeStart + 3, newPoints, removeStart, controlPoints.Length - removeStart - 3);
            controlPoints = newPoints;

            if (segmentOffsets != null && segmentOffsets.Length > 1)
            {
                var newSegments = new PathOffsetData[segmentOffsets.Length - 1];
                if (segmentToRemove > 0)
                    Array.Copy(segmentOffsets, 0, newSegments, 0, segmentToRemove);
                if (segmentToRemove < segmentOffsets.Length - 1)
                    Array.Copy(segmentOffsets, segmentToRemove + 1, newSegments, segmentToRemove, segmentOffsets.Length - segmentToRemove - 1);
                segmentOffsets = newSegments;
            }
        }

        /// <summary>
        /// 沿当前连线的切线平滑发散延伸处，为该追踪组件追加开辟新一段附带控制曲柄手柄与连接端锚点的顺畅节点。
        /// </summary>
        public void AddSegment()
        {
            var lastPoint = controlPoints[^1];
            var secondLastPoint = controlPoints[^2];
            // 计算切向方向用来顺延新生成的点
            var dir = (lastPoint - secondLastPoint).normalized;
            var dist = Vector3.Distance(secondLastPoint, lastPoint);
            if (dist == 0) dist = 1f;

            Array.Resize(ref controlPoints, controlPoints.Length + 3);
            var n = controlPoints.Length;

            Array.Resize(ref segmentOffsets, segmentOffsets.Length + 1);
            segmentOffsets[^1] = new PathOffsetData();

            // 第一个新点是前一个锚点向外延伸的对向曲柄点
            controlPoints[n - 3] = lastPoint + dir * dist;

            // 假设默认将新锚点放置在顺延方向前方 2 倍 dist 的地方
            var newAnchorPos = lastPoint + dir * dist * 2f;

            // 假设新锚点的曲柄和前一个曲柄平行
            controlPoints[n - 1] = newAnchorPos;
            controlPoints[n - 2] = newAnchorPos - dir * dist;
        }

        /// <summary>
        /// 自动将所有的锚点与其连接手柄曲柄执行全局重构推衍。<br/>
        /// 它将消除多段贝塞尔之间的弯口折角，迫使整条曲线段形成柔和且张力等称的 C1 等阶并行贯通效果。
        /// </summary>
        public void AutoSmooth()
        {
            if (controlPoints == null || controlPoints.Length < 4) return;
            // 遍历所有中间的连接点（即索引为3，6，9...的点）
            for (var i = 3; i < controlPoints.Length - 1; i += 3)
            {
                var anchor = controlPoints[i];
                var prevAnchor = controlPoints[i - 3];
                var nextAnchor = controlPoints[i + 3];
                    
                // 计算出平滑方向：从上一个锚点指向下一个锚点
                var dir = (nextAnchor - prevAnchor).normalized;

                // 为了保持曲线张力不会太夸张，取该锚点与前后两锚点距离的大约 1/3 作为曲柄长度
                var distPrev = Vector3.Distance(anchor, prevAnchor) / 3f;
                var distNext = Vector3.Distance(anchor, nextAnchor) / 3f;

                // 取平均距离以确保两侧控制点距离完全相等，从而触发编辑器中的对称联动（关联）判定
                var avgDist = (distPrev + distNext) * 0.5f;

                controlPoints[i - 1] = anchor - dir * avgDist;
                controlPoints[i + 1] = anchor + dir * avgDist;
            }
        }

        /// <summary>
        /// 标准三次贝塞尔曲线 (Cubic Bezier Spline) 核心几何求解算法模型。
        /// </summary>
        /// <param name="p0">起始原心锚点</param>
        /// <param name="p1">起始点的衍生控制拖柄 (切线引导段)</param>
        /// <param name="p2">目标落点的背身前探控制拖柄 (切向入引段)</param>
        /// <param name="p3">目标汇聚锚点</param>
        /// <param name="t">0->1 闭合时段间的百分比值</param>
        /// <returns>拟合求值平滑输出标量</returns>
        public static Vector3 EvaluateCubicBezier(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {
            t = Mathf.Clamp01(t);
            var u = 1 - t;
            var tt = t * t;
            var uu = u * u;
            var uuu = uu * u;
            var ttt = tt * t;

            var p = uuu * p0; // (1-t)^3 * p0
            p += 3 * uu * t * p1; // 3(1-t)^2 * t * p1
            p += 3 * u * tt * p2; // 3(1-t) * t^2 * p2
            p += ttt * p3; // t^3 * p3

            return p;
        }
    }
}