using System;
using UnityEngine;

namespace Script.Particles
{
    [RequireComponent(typeof(ParticleSystem))]
    [ExecuteAlways]
    public class ParticlePathFollower : MonoBehaviour
    {
        public enum OffsetMode
        {
            None,
            Random,
            Repeat,
            PingPong
        }

        [Header("路径设置")] public Vector3[] controlPoints =
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

        [Tooltip("水平方向的真空区百分比")] [Range(0f, 1f)]
        public float innerVacuumX;

        [Tooltip("垂直方向的真空区百分比")] [Range(0f, 1f)]
        public float innerVacuumY;

        [Tooltip("是否将偏移范围与真空区映射为圆/椭圆形")] public bool circularShape;

        [Tooltip("重复/来回模式的频率")] public float offsetFrequency = 1.0f;

        private ParticleSystem.Particle[] _particles;

        private ParticleSystem _particleSystem;
        private float[] _pathDistances;
        private Vector3[] _pathForwards;

        // 缓存路径数据，用于实现匀速和按距离查值
        private Vector3[] _pathPoints;
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
                var newPos = GetPointAtDistance(distance, out forwardDir);
                var currentEffectiveSpeed = speed; // 记录实际速度

                if (offsetMode != OffsetMode.None)
                {
                    var currentOffsetX = 0f;
                    var currentOffsetY = 0f;

                    // 计算该粒子的相对固定出生时间，以此作为重复与来回的依据，保证单颗粒子一生中的偏移保持固定，而不是在路径上扭波
                    var spawnTime = Time.time - age;

                    switch (offsetMode)
                    {
                        case OffsetMode.Random:
                            // 利用粒子自身的随机种子固定一个随身偏移量，以防止每帧闪烁
                            var randomT1 = _particles[i].randomSeed % 10000 / 10000f;
                            var randomT2 = _particles[i].randomSeed / 10000 % 10000 / 10000f; // 获取另一段随机值
                            var offRand = CalculateOffset2D(randomT1, randomT2);
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
                            var offRepeat = CalculateOffset2D(t1R, t2R);
                            currentOffsetX = offRepeat.x;
                            currentOffsetY = offRepeat.y;
                            break;
                        case OffsetMode.PingPong:
                            var tPingPong = Mathf.PingPong(spawnTime * offsetFrequency, 1f);
                            // 来回状态下，表现为围绕中心旋转到底后再反向旋转的来回扫描
                            var angleP = tPingPong * Mathf.PI * 2f;
                            var t1P = Mathf.Cos(angleP) * 0.5f + 0.5f;
                            var t2P = Mathf.Sin(angleP) * 0.5f + 0.5f;
                            var offPing = CalculateOffset2D(t1P, t2P);
                            currentOffsetX = offPing.x;
                            currentOffsetY = offPing.y;
                            break;
                    }

                    if ((currentOffsetX != 0f || currentOffsetY != 0f) && forwardDir != Vector3.zero)
                    {
                        // 计算基于路径前向的左右和上下垂直向量
                        // 假设主要在XY平面，Z轴作为世界上的参照
                        var right = Vector3.Cross(Vector3.forward, forwardDir).normalized;
                        if (right == Vector3.zero) right = Vector3.right;
                        var up = Vector3.Cross(forwardDir, right).normalized;

                        newPos += right * currentOffsetX + up * currentOffsetY;

                        // 差分采取靠近的一小步预测下一个点的位置，进而得出偏移后真实的运动方向和速度率
                        var delta = 0.05f;
                        Vector3 nextForwardDir;
                        var nextBasePos = GetPointAtDistance(distance + delta, out nextForwardDir);

                        var nextRight = Vector3.Cross(Vector3.forward, nextForwardDir).normalized;
                        if (nextRight == Vector3.zero) nextRight = Vector3.right;
                        var nextUp = Vector3.Cross(nextForwardDir, nextRight).normalized;

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

        // 构建距离查值缓存表，确保粒子的分布疏密完全一致
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

        // 核心：基于全局 2D 坐标（避免分轴计算产生十字型真空），并推导真空切割和圆形挤压
        private Vector2 CalculateOffset2D(float t1, float t2)
        {
            // 映射到 -1 到 +1 标准化包围盒
            var px = Mathf.Lerp(-1f, 1f, t1);
            var py = Mathf.Lerp(-1f, 1f, t2);

            // 1. 真空区处理 (2D 射线等比映射法) 避免分离处理产生的十字型空缺
            if (enableInnerVacuum)
            {
                var vx = Mathf.Clamp01(innerVacuumX);
                var vy = Mathf.Clamp01(innerVacuumY);

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
            var minX = Mathf.Min(offset.x, offset.y);
            var maxX = Mathf.Max(offset.x, offset.y);
            var minY = Mathf.Min(offset.z, offset.w);
            var maxY = Mathf.Max(offset.z, offset.w);

            var centerX = (minX + maxX) * 0.5f;
            var centerY = (minY + maxY) * 0.5f;
            var extentX = (maxX - minX) * 0.5f;
            var extentY = (maxY - minY) * 0.5f;

            var finalX = centerX + px * extentX;
            var finalY = centerY + py * extentY;

            // 补偿原始设定如果存在倒置的要求
            if (offset.x > offset.y) finalX = centerX - px * extentX;
            if (offset.z > offset.w) finalY = centerY - py * extentY;

            return new Vector2(finalX, finalY);
        }

        // 根据实际物理距离获取贝塞尔曲线上的坐标
        private Vector3 GetPointAtDistance(float targetDistance, out Vector3 forward)
        {
            if (_pathPoints == null || _pathPoints.Length < 2)
            {
                forward = Vector3.right;
                return Vector3.zero;
            }

            if (targetDistance <= 0f)
            {
                forward = _pathForwards[0];
                return _pathPoints[0];
            }

            if (targetDistance >= _totalPathLength)
            {
                var last = _pathPoints.Length - 1;
                forward = _pathForwards[last];
                return _pathPoints[last];
            }

            for (var i = 1; i < _pathDistances.Length; i++)
                if (_pathDistances[i] >= targetDistance)
                {
                    var d0 = _pathDistances[i - 1];
                    var d1 = _pathDistances[i];
                    var t = (targetDistance - d0) / (d1 - d0);
                    var pos = Vector3.Lerp(_pathPoints[i - 1], _pathPoints[i], t);
                    // 核心修复点：将方向从“死板的折线段”改为插值平滑向量
                    forward = Vector3.Lerp(_pathForwards[i - 1], _pathForwards[i], t).normalized;
                    return pos;
                }

            var end = _pathPoints.Length - 1;
            forward = _pathForwards[end];
            return _pathPoints[end];
        }

        // 根据基础 t(0~1) 在样条曲线中进行粗略查值
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

        // 拓展路径段
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

            // 第一个新点是前一个锚点向外延伸的对向曲柄点
            controlPoints[n - 3] = lastPoint + dir * dist;

            // 假设默认将新锚点放置在顺延方向前方 3 倍 dist 的地方
            var newAnchorPos = lastPoint + dir * dist * 3f;

            // 假设新锚点的曲柄和前一个曲柄平行
            controlPoints[n - 1] = newAnchorPos;
            controlPoints[n - 2] = newAnchorPos - dir * dist;
        }

        // 自动计算并套用平滑的曲柄节点 (使各曲线段在连接点达到平滑过渡)
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

                controlPoints[i - 1] = anchor - dir * distPrev;
                controlPoints[i + 1] = anchor + dir * distNext;
            }
        }

        // 获取三次贝塞尔曲线上的点
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