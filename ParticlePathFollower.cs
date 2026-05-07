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

        /// <summary>
        /// 粒子沿路径的推进模式
        /// </summary>
        public enum PathTravelMode
        {
            OneShot,
            Loop,
            PingPong
        }

        [HideInInspector] public Vector3[] controlPoints =
        {
            new(0, 0, 0),
            new(0, 0, 1),
            new(0, 0, 4),
            new(0, 0, 5)
        };

        [Tooltip("选择另一个同组件对象作为控制点复制来源"),HideInInspector]
        public ParticlePathFollower copySource;

        [Tooltip("粒子沿路径移动的速度")]
        public float speed = 1.0f;

        [Tooltip("路径行进模式（单次、循环、往返）")]
        public PathTravelMode pathTravelMode = PathTravelMode.OneShot;

        [Tooltip("沿路径进度(0~1)的速度倍率曲线")]
        public AnimationCurve speedOverPath = AnimationCurve.Linear(0f, 1f, 1f, 1f);

        [Tooltip("自动生命周期是否计入速度曲线影响")]
        public bool includeCurveInLifetime = true;

        [Tooltip("速度曲线积分采样数（越大越精确）")] [Range(4, 128)]
        public int speedSampleCount = 32;

        [Tooltip("是否自动根据路径长和速度计算并修改粒子的生命周期")] public bool autoSetLifetime = true;

        [Tooltip("是否改变粒子的朝向（使其对齐路径的前进方向）")] public bool alignToPath = true;

        [Tooltip("对 alignToPath 结果统一施加的整体朝向补偿（欧拉角）")]
        public Vector3 overallRotationCompensation = Vector3.zero;
        
        [Tooltip("垂直于路径的偏移模式")]
        public OffsetMode offsetMode = OffsetMode.Random;

        [Tooltip("是否启用内部真空区（在此范围内不会生成和偏移粒子）")] public bool enableInnerVacuum;
        [Tooltip("是否将偏移范围与真空区映射为圆/椭圆形")] public bool circularShape;
        [Tooltip("重复/来回模式的频率")] public float offsetFrequency = 1.0f;

        [Tooltip("启用时进行预发射，使对象显示时已有粒子分布在路径上")]
        public bool prewarmOnEnable = true;

        [Tooltip("预发射时长（秒）。<=0 时自动使用一个生命周期")]
        public float prewarmDuration = 0f;


        [Serializable]
        public class PathOffsetData
        {
            [Tooltip("偏移范围（X/Y：左右最小最大，Z/W：上下最小最大）")]
            public Vector4 offset = new Vector4(1f, -1f, 1f, -1f);

            [Tooltip("水平方向的真空区百分比")] [Range(0f, 1f)]
            public float innerVacuumX;

            [Tooltip("垂直方向的真空区百分比")] [Range(0f, 1f)]
            public float innerVacuumY;
        }

        [HideInInspector]
        public PathOffsetData[] segmentOffsets = { new PathOffsetData() };

        private static readonly PathOffsetData FallbackOffsetData = new PathOffsetData();

        private ParticleSystem.Particle[] _particles;

        private ParticleSystem _particleSystem;
        private float[] _pathDistances;
        private Vector3[] _pathForwards;
        private Vector3[] _pathRights;
        private Vector3[] _pathUps;

        /// <summary>
        /// 缓存路径坐标数据表，用于实现平滑的匀速运动和距离查值。
        /// </summary>
        private Vector3[] _pathPoints;

        /// <summary>
        /// 贝塞尔曲线的实际物理累计总长。
        /// </summary>
        private float _totalPathLength;

        private float[] _motionTimeSamples;
        private float[] _motionDistanceSamples;
        private float _oneWayDuration;
        private int _pathCacheSignature = int.MinValue;
        private int _motionCacheSignature = int.MinValue;

        private void OnValidate()
        {
            speedSampleCount = Mathf.Clamp(speedSampleCount, 4, 128);
            speedOverPath = SanitizeCurve01(speedOverPath);
            if (prewarmDuration < 0f) prewarmDuration = 0f;
        }

        private void OnEnable()
        {
            if (!Application.isPlaying || !prewarmOnEnable)
                return;

            InitializeIfNeeded();
            UpdatePathCache();
            UpdateMotionCache();

            var main = _particleSystem.main;
            ApplyAutoLifetime(main);

            var warmupTime = prewarmDuration > 0f ? prewarmDuration : ResolveStartLifetime(main);
            if (warmupTime <= 0f)
                return;

            _particleSystem.Simulate(warmupTime, true, true, true);
            _particleSystem.Play(true);
        }

        private void LateUpdate()
        {
            InitializeIfNeeded();
            UpdatePathCache();
            UpdateMotionCache();
            var main = _particleSystem.main;
            var simulationSpace = main.simulationSpace;
            var isLocalSimulation = simulationSpace == ParticleSystemSimulationSpace.Local;
            var frameTime = Time.time;
            var compensationRotation = Quaternion.Euler(overallRotationCompensation);

            // 动态设置粒子的初始生命周期，使其刚好在到达路径终点时消失，避免过早消失或路径末尾堆积
            ApplyAutoLifetime(main);

            var count = _particleSystem.GetParticles(_particles);

            for (var i = 0; i < count; i++)
            {
                // 根据粒子年龄映射到路径距离，支持速度曲线与路径模式
                var age = _particles[i].startLifetime - _particles[i].remainingLifetime;
                var distance = GetDistanceAtAge(age, out var isForwardTravel);

                Vector3 forwardDir;
                Vector3 right;
                Vector3 up;
                float segIdx;
                var newPos = GetPointAtDistance(distance, out forwardDir, out right, out up, out segIdx);
                if (!isForwardTravel) forwardDir = -forwardDir;

                const float velocityDeltaTime = 0.02f;
                var needNextSample = alignToPath || offsetMode != OffsetMode.None || forwardDir == Vector3.zero;
                var nextBasePos = newPos;
                var nextRight = right;
                var nextUp = up;
                var baseDelta = Vector3.zero;

                if (needNextSample)
                {
                    var nextDistance = GetDistanceAtAge(age + velocityDeltaTime, out _);
                    nextBasePos = GetPointAtDistance(nextDistance, out _, out nextRight, out nextUp, out _);
                    baseDelta = nextBasePos - newPos;

                    if (forwardDir == Vector3.zero && baseDelta != Vector3.zero)
                        forwardDir = baseDelta.normalized;
                }

                if (offsetMode != OffsetMode.None)
                {
                    EvaluateSegmentOffsetData(segIdx, out var segOffset, out var segInnerVacuumX, out var segInnerVacuumY);

                    // 计算该粒子的相对固定出生时间，以此作为重复与来回的依据，保证单颗粒子一生中的偏移保持固定，而不是在路径上扭波
                    var spawnTime = frameTime - age;
                    var offset = GetOffsetSample(offsetMode, _particles[i].randomSeed, spawnTime, offsetFrequency, segOffset,
                        segInnerVacuumX, segInnerVacuumY);
                    var currentOffsetX = offset.x;
                    var currentOffsetY = offset.y;

                    if ((currentOffsetX != 0f || currentOffsetY != 0f) && forwardDir != Vector3.zero)
                    {
                        newPos += right * currentOffsetX + up * currentOffsetY;

                        var nextPos = nextBasePos + nextRight * currentOffsetX + nextUp * currentOffsetY;

                        // 偏移后的前进方向
                        var offsetForward = (nextPos - newPos).normalized;
                        if (offsetForward != Vector3.zero) forwardDir = offsetForward;
                    }
                }

                // 将本地坐标系转换到粒子系统的模拟空间，如果是世界空间则保持不变，如果是局部空间则转换为相对于粒子系统的坐标
                if (isLocalSimulation)
                    _particles[i].position = newPos;
                else
                    _particles[i].position = transform.TransformPoint(newPos);

                if (alignToPath && forwardDir != Vector3.zero)
                {
                    // alignToPath 启用后使用三维朝向：沿路径前向并使用运输帧的 up 保持稳定翻滚。
                    var renderForward = forwardDir.normalized;
                    var renderUp = up.sqrMagnitude > 0.000001f ? up.normalized : Vector3.up;
                    var currentEffectiveSpeed = baseDelta.magnitude / velocityDeltaTime;

                    if (simulationSpace != ParticleSystemSimulationSpace.Local)
                    {
                        renderForward = transform.TransformDirection(renderForward).normalized;
                        renderUp = transform.TransformDirection(renderUp).normalized;
                    }

                    // 在路径朝向基础上叠加统一补偿，便于修正粒子资源自身前向轴差异。
                    var lookRotation = Quaternion.LookRotation(renderForward, renderUp) * compensationRotation;
                    _particles[i].rotation3D = lookRotation.eulerAngles;
                    _particles[i].velocity = renderForward * currentEffectiveSpeed; // 更新速度向量，利用真实速度
                }
            }

            _particleSystem.SetParticles(_particles, count);
        }

        private static void BuildStableFrame(Vector3 forward, out Vector3 right, out Vector3 up)
        {
            if (forward.sqrMagnitude < 0.000001f)
            {
                right = Vector3.right;
                up = Vector3.up;
                return;
            }

            forward.Normalize();

            var referenceAxis = Mathf.Abs(Vector3.Dot(forward, Vector3.forward)) > 0.98f
                ? Vector3.up
                : Vector3.forward;

            right = Vector3.Cross(forward, referenceAxis).normalized;
            if (right == Vector3.zero)
                right = Vector3.right;

            up = Vector3.Cross(right, forward).normalized;
            if (up == Vector3.zero)
                up = Vector3.up;
        }

        private void ApplyAutoLifetime(ParticleSystem.MainModule main)
        {
            if (!autoSetLifetime || speed <= 0.001f)
                return;

            var travelModeMultiplier = pathTravelMode == PathTravelMode.PingPong ? 2f : 1f;

            if (includeCurveInLifetime)
                main.startLifetime = Mathf.Max(0.01f, _oneWayDuration * travelModeMultiplier);
            else
                main.startLifetime = Mathf.Max(0.01f, (_totalPathLength / speed) * travelModeMultiplier);
        }

        private float ResolveStartLifetime(ParticleSystem.MainModule main)
        {
            if (autoSetLifetime && speed > 0.001f)
            {
                var travelModeMultiplier = pathTravelMode == PathTravelMode.PingPong ? 2f : 1f;
                if (includeCurveInLifetime)
                    return Mathf.Max(0.01f, _oneWayDuration * travelModeMultiplier);

                return Mathf.Max(0.01f, (_totalPathLength / speed) * travelModeMultiplier);
            }

            var startLifetime = main.startLifetime;
            switch (startLifetime.mode)
            {
                case ParticleSystemCurveMode.Constant:
                    return Mathf.Max(0f, startLifetime.constant);
                case ParticleSystemCurveMode.TwoConstants:
                    return Mathf.Max(0f, startLifetime.constantMax);
                case ParticleSystemCurveMode.Curve:
                    return Mathf.Max(0f, startLifetime.curveMultiplier);
                case ParticleSystemCurveMode.TwoCurves:
                    return Mathf.Max(0f, startLifetime.curveMultiplier);
                default:
                    return 0f;
            }
        }

        private Vector2 GetOffsetSample(OffsetMode mode, uint randomSeed, float spawnTime, float frequency, Vector4 offset,
            float innerVacuumX, float innerVacuumY)
        {
            switch (mode)
            {
                case OffsetMode.Random:
                    return CalculateOffset2D(SeedToUnitFloat(randomSeed, 0), SeedToUnitFloat(randomSeed, 16), offset, innerVacuumX, innerVacuumY, true);
                case OffsetMode.Repeat:
                    return CalculateOffset2DFromPhase(spawnTime * frequency, offset, innerVacuumX, innerVacuumY, false);
                case OffsetMode.PingPong:
                    return CalculateOffset2DFromPhase(Mathf.PingPong(spawnTime * frequency, 1f), offset, innerVacuumX, innerVacuumY, false);
                default:
                    return Vector2.zero;
            }
        }

        private Vector2 CalculateOffset2DFromPhase(float phase, Vector4 offset, float innerVacuumX, float innerVacuumY, bool applyCircularShape)
        {
            var angle = Mathf.Repeat(phase, 1f) * Mathf.PI * 2f;
            return CalculateOffset2D(Mathf.Cos(angle) * 0.5f + 0.5f, Mathf.Sin(angle) * 0.5f + 0.5f, offset, innerVacuumX, innerVacuumY, applyCircularShape);
        }

        private static float SeedToUnitFloat(uint seed, int shift)
        {
            return ((seed >> shift) & 0xFFFFu) / 65535f;
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

            var signature = ComputePathCacheSignature(stepsPerCurve);
            if (signature == _pathCacheSignature &&
                _pathPoints != null && _pathPoints.Length > 1 &&
                _pathDistances != null && _pathForwards != null &&
                _pathRights != null && _pathUps != null &&
                _pathRights.Length == _pathPoints.Length && _pathUps.Length == _pathPoints.Length)
                return;

            var curveCount = (controlPoints.Length - 1) / 3;
            var totalSteps = curveCount * stepsPerCurve;

            if (_pathPoints == null || _pathPoints.Length != totalSteps + 1 ||
                _pathForwards == null || _pathForwards.Length != totalSteps + 1 ||
                _pathRights == null || _pathRights.Length != totalSteps + 1 ||
                _pathUps == null || _pathUps.Length != totalSteps + 1 ||
                _pathDistances == null || _pathDistances.Length != totalSteps + 1)
            {
                _pathPoints = new Vector3[totalSteps + 1];
                _pathForwards = new Vector3[totalSteps + 1];
                _pathRights = new Vector3[totalSteps + 1];
                _pathUps = new Vector3[totalSteps + 1];
                _pathDistances = new float[totalSteps + 1];
            }

            _pathDistances[0] = 0f;
            var length = 0f;

            for (var i = 0; i <= totalSteps; i++)
            {
                var t = (float)i / totalSteps;
                _pathPoints[i] = EvaluateSplineWithTangent(t, out var forwardVec);
                if (forwardVec == Vector3.zero && i > 0) forwardVec = _pathPoints[i] - _pathPoints[i - 1];
                _pathForwards[i] = forwardVec.normalized;

                if (i > 0)
                {
                    length += Vector3.Distance(_pathPoints[i - 1], _pathPoints[i]);
                    _pathDistances[i] = length;
                }
            }

            // 沿路径平行运输局部截面坐标系，减少偏移基向量在曲率变化处的扭转翻转。
            var firstForward = _pathForwards[0];
            if (firstForward == Vector3.zero && totalSteps > 0)
                firstForward = (_pathPoints[1] - _pathPoints[0]).normalized;
            if (firstForward == Vector3.zero)
                firstForward = Vector3.right;

            BuildStableFrame(firstForward, out _pathRights[0], out _pathUps[0]);

            for (var i = 1; i <= totalSteps; i++)
            {
                var prevForward = _pathForwards[i - 1];
                var currForward = _pathForwards[i];

                if (prevForward == Vector3.zero) prevForward = firstForward;
                if (currForward == Vector3.zero) currForward = prevForward;

                var rotation = Quaternion.FromToRotation(prevForward, currForward);
                var transportedRight = rotation * _pathRights[i - 1];

                var orthoRight = transportedRight - currForward * Vector3.Dot(transportedRight, currForward);
                if (orthoRight.sqrMagnitude < 0.000001f)
                    BuildStableFrame(currForward, out orthoRight, out _pathUps[i]);
                else
                {
                    orthoRight.Normalize();
                    _pathUps[i] = Vector3.Cross(orthoRight, currForward).normalized;
                }

                if (Vector3.Dot(orthoRight, _pathRights[i - 1]) < 0f)
                {
                    orthoRight = -orthoRight;
                    _pathUps[i] = -_pathUps[i];
                }

                _pathRights[i] = orthoRight;
            }

            _totalPathLength = length;
            _pathCacheSignature = signature;
        }

        /// <summary>
        /// 构建距离-时间查找表，用于支持沿路径的速度曲线积分与反查。
        /// </summary>
        private void UpdateMotionCache()
        {
            var sampleCount = Mathf.Max(4, speedSampleCount);
            var signature = ComputeMotionCacheSignature(sampleCount);

            if (signature == _motionCacheSignature && _motionTimeSamples != null && _motionTimeSamples.Length == sampleCount + 1 && _motionDistanceSamples != null && _motionDistanceSamples.Length == sampleCount + 1)
                return;

            if (_motionTimeSamples == null || _motionTimeSamples.Length != sampleCount + 1 ||
                _motionDistanceSamples == null || _motionDistanceSamples.Length != sampleCount + 1)
            {
                _motionTimeSamples = new float[sampleCount + 1];
                _motionDistanceSamples = new float[sampleCount + 1];
            }

            if (_totalPathLength <= 0.0001f || speed <= 0.0001f)
            {
                _oneWayDuration = 0f;
                for (var i = 0; i <= sampleCount; i++)
                {
                    _motionDistanceSamples[i] = 0f;
                    _motionTimeSamples[i] = 0f;
                }

                _motionCacheSignature = signature;

                return;
            }

            _motionDistanceSamples[0] = 0f;
            _motionTimeSamples[0] = 0f;

            var cumulativeTime = 0f;
            var stepDistance = _totalPathLength / sampleCount;

            for (var i = 1; i <= sampleCount; i++)
            {
                var t0 = (float)(i - 1) / sampleCount;
                var t1 = (float)i / sampleCount;
                var m0 = Mathf.Max(0.0001f, speedOverPath.Evaluate(t0));
                var m1 = Mathf.Max(0.0001f, speedOverPath.Evaluate(t1));
                var avgMultiplier = (m0 + m1) * 0.5f;

                cumulativeTime += stepDistance / (speed * avgMultiplier);

                _motionDistanceSamples[i] = stepDistance * i;
                _motionTimeSamples[i] = cumulativeTime;
            }

            _oneWayDuration = cumulativeTime;
            _motionCacheSignature = signature;
        }

        private int ComputePathCacheSignature(int stepsPerCurve)
        {
            unchecked
            {
                var hash = 17;
                hash = hash * 31 + stepsPerCurve;
                hash = hash * 31 + (controlPoints?.Length ?? 0);

                if (controlPoints != null)
                    for (var i = 0; i < controlPoints.Length; i++)
                        hash = hash * 31 + controlPoints[i].GetHashCode();

                return hash;
            }
        }

        private int ComputeMotionCacheSignature(int sampleCount)
        {
            unchecked
            {
                var hash = 17;
                hash = hash * 31 + sampleCount;
                hash = hash * 31 + speed.GetHashCode();
                hash = hash * 31 + _totalPathLength.GetHashCode();

                if (speedOverPath != null)
                {
                    hash = hash * 31 + speedOverPath.length;
                    hash = hash * 31 + speedOverPath.Evaluate(0.25f).GetHashCode();
                    hash = hash * 31 + speedOverPath.Evaluate(0.5f).GetHashCode();
                    hash = hash * 31 + speedOverPath.Evaluate(0.75f).GetHashCode();
                }

                return hash;
            }
        }

        /// <summary>
        /// 将曲线约束到 x/y 都为 0~1，并保证首尾关键帧覆盖整个横轴。
        /// </summary>
        private static AnimationCurve SanitizeCurve01(AnimationCurve curve)
        {
            if (curve == null || curve.length == 0)
                return AnimationCurve.Linear(0f, 1f, 1f, 1f);

            var keys = curve.keys;
            for (var i = 0; i < keys.Length; i++)
            {
                var key = keys[i];
                key.time = Mathf.Clamp01(key.time);
                key.value = Mathf.Clamp01(key.value);
                keys[i] = key;
            }

            Array.Sort(keys, (a, b) => a.time.CompareTo(b.time));

            if (keys.Length == 0)
            {
                keys = new[]
                {
                    new Keyframe(0f, 1f),
                    new Keyframe(1f, 1f)
                };
            }
            else
            {
                if (keys[0].time > 0f)
                    Array.Resize(ref keys, keys.Length + 1);

                if (keys[0].time > 0f)
                {
                    Array.Copy(keys, 0, keys, 1, keys.Length - 1);
                    keys[0] = new Keyframe(0f, Mathf.Clamp01(keys[1].value));
                }
                else
                {
                    var first = keys[0];
                    first.time = 0f;
                    first.value = Mathf.Clamp01(first.value);
                    keys[0] = first;
                }

                var lastIndex = keys.Length - 1;
                if (keys[lastIndex].time < 1f)
                {
                    Array.Resize(ref keys, keys.Length + 1);
                    keys[^1] = new Keyframe(1f, Mathf.Clamp01(keys[^2].value));
                }
                else
                {
                    var last = keys[lastIndex];
                    last.time = 1f;
                    last.value = Mathf.Clamp01(last.value);
                    keys[lastIndex] = last;
                }
            }

            var sanitized = new AnimationCurve(keys)
            {
                preWrapMode = WrapMode.ClampForever,
                postWrapMode = WrapMode.ClampForever
            };
            return sanitized;
        }

        private float GetDistanceAtAge(float age, out bool isForwardTravel)
        {
            isForwardTravel = true;
            if (_totalPathLength <= 0.0001f || speed <= 0.0001f || _oneWayDuration <= 0.0001f)
                return 0f;

            switch (pathTravelMode)
            {
                case PathTravelMode.Loop:
                {
                    var localTime = Mathf.Repeat(age, _oneWayDuration);
                    return GetDistanceAtTime(localTime);
                }
                case PathTravelMode.PingPong:
                {
                    var period = _oneWayDuration * 2f;
                    var localTime = Mathf.Repeat(age, period);

                    if (localTime <= _oneWayDuration)
                        return GetDistanceAtTime(localTime);

                    isForwardTravel = false;
                    return GetDistanceAtTime(period - localTime);
                }
                default:
                {
                    var clamped = Mathf.Clamp(age, 0f, _oneWayDuration);
                    return GetDistanceAtTime(clamped);
                }
            }
        }

        private float GetDistanceAtTime(float elapsedTime)
        {
            if (_motionTimeSamples == null || _motionTimeSamples.Length < 2)
                return 0f;

            if (elapsedTime <= 0f)
                return 0f;

            var last = _motionTimeSamples.Length - 1;
            if (elapsedTime >= _motionTimeSamples[last])
                return _motionDistanceSamples[last];

            var idx = Array.BinarySearch(_motionTimeSamples, elapsedTime);
            if (idx >= 0)
                return _motionDistanceSamples[idx];

            var right = ~idx;
            var left = Mathf.Max(0, right - 1);
            right = Mathf.Min(last, right);

            var t0 = _motionTimeSamples[left];
            var t1 = _motionTimeSamples[right];
            var lerp = (elapsedTime - t0) / Mathf.Max(0.0001f, t1 - t0);
            return Mathf.Lerp(_motionDistanceSamples[left], _motionDistanceSamples[right], lerp);
        }

        /// <summary>
        /// 基于全局二维极坐标系推演真实的横截面偏移量。
        /// 内置同心防形变量算法，解决了纯中心排除带来的十字形空缺伪影，并支撑了对于圆形缩放投影的挤压模拟。
        /// </summary>
        /// <param name="t1">第一轴向插值参数 (0~1)</param>
        /// <param name="t2">第二轴向插值参数 (0~1)</param>
        /// <returns>处理真空裁切及圆平滑折变后的最终 2D 实际偏移值</returns>
        private Vector2 CalculateOffset2D(float t1, float t2, Vector4 offset, float innerVacuumX, float innerVacuumY, bool applyCircularShape)
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
            if (circularShape && applyCircularShape)
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

            var signX = offset.x > offset.y ? -1f : 1f;
            var signY = offset.z > offset.w ? -1f : 1f;

            var finalX = centerX + px * extentX * signX;
            var finalY = centerY + py * extentY * signY;

            return new Vector2(finalX, finalY);
        }

        private void EvaluateSegmentOffsetData(float floatSegIdx, out Vector4 offset, out float innerVacuumX, out float innerVacuumY)
        {
            if (segmentOffsets == null || segmentOffsets.Length == 0)
            {
                offset = default;
                innerVacuumX = 0f;
                innerVacuumY = 0f;
                return;
            }
            if (segmentOffsets.Length == 1)
            {
                var single = segmentOffsets[0] ?? FallbackOffsetData;
                offset = single.offset;
                innerVacuumX = single.innerVacuumX;
                innerVacuumY = single.innerVacuumY;
                return;
            }

            // 整段连续平滑插值：每段区间 [i, i+1] 内都从 i 平滑过渡到 i+1，避免首段内部体感“顿挫”。
            var lastIndex = segmentOffsets.Length - 1;
            var clampedSeg = Mathf.Clamp(floatSegIdx, 0f, lastIndex);
            var leftIndex = Mathf.FloorToInt(clampedSeg);
            var rightIndex = Mathf.Min(leftIndex + 1, lastIndex);

            if (leftIndex == rightIndex)
            {
                var data = segmentOffsets[leftIndex] ?? FallbackOffsetData;
                offset = data.offset;
                innerVacuumX = data.innerVacuumX;
                innerVacuumY = data.innerVacuumY;
                return;
            }

            var leftData = segmentOffsets[leftIndex] ?? FallbackOffsetData;
            var rightData = segmentOffsets[rightIndex] ?? FallbackOffsetData;

            var t = clampedSeg - leftIndex;
            t = t * t * (3f - 2f * t); // SmoothStep

            offset = Vector4.Lerp(leftData.offset, rightData.offset, t);
            innerVacuumX = Mathf.Lerp(leftData.innerVacuumX, rightData.innerVacuumX, t);
            innerVacuumY = Mathf.Lerp(leftData.innerVacuumY, rightData.innerVacuumY, t);
        }

        /// <summary>
        /// 根据粒子的真实移动物理距离查询其所处的贝塞尔曲线确切空间坐标与路径朝向。
        /// </summary>
        /// <param name="targetDistance">预计推进的物理距离(米)</param>
        /// <param name="forward">输出：此时该点指向前方平滑推移的切线向量</param>
        /// <param name="right">输出：沿路径平行运输后的局部 right 轴</param>
        /// <param name="up">输出：沿路径平行运输后的局部 up 轴</param>
        /// <param name="floatSegIdx">输出：此时所处曲线段落的连续浮点进度索引，用于平滑插值管径</param>
        /// <returns>推算所处在物理世界中的实际位置标点</returns>
        private Vector3 GetPointAtDistance(float targetDistance, out Vector3 forward, out Vector3 right, out Vector3 up, out float floatSegIdx)
        {
            if (_pathRights == null || _pathUps == null || _pathForwards == null || _pathDistances == null)
            {
                UpdatePathCache();
            }

            if (_pathPoints == null || _pathPoints.Length < 2)
            {
                forward = Vector3.right;
                right = Vector3.right;
                up = Vector3.up;
                floatSegIdx = 0f;
                return Vector3.zero;
            }

            if (targetDistance <= 0f)
            {
                forward = _pathForwards[0];
                right = _pathRights[0];
                up = _pathUps[0];
                floatSegIdx = 0f;
                return _pathPoints[0];
            }

            var curveCount = (controlPoints.Length - 1) / 3;

            if (targetDistance >= _totalPathLength)
            {
                var last = _pathPoints.Length - 1;
                forward = _pathForwards[last];
                right = _pathRights[last];
                up = _pathUps[last];
                floatSegIdx = Mathf.Max(0, curveCount);
                return _pathPoints[last];
            }

            var totalSteps = _pathDistances.Length - 1;
            var stepPerCurve = totalSteps / (float)Mathf.Max(1, curveCount);

            var idx = Array.BinarySearch(_pathDistances, targetDistance);
            if (idx < 0) idx = ~idx;
            idx = Mathf.Clamp(idx, 1, _pathDistances.Length - 1);

            var d0 = _pathDistances[idx - 1];
            var d1 = _pathDistances[idx];
            var t = (targetDistance - d0) / Mathf.Max(0.0001f, d1 - d0);
            var pos = Vector3.Lerp(_pathPoints[idx - 1], _pathPoints[idx], t);
            // 核心修复点：将方向从“死板的折线段”改为插值平滑向量
            forward = Vector3.Lerp(_pathForwards[idx - 1], _pathForwards[idx], t).normalized;

            var right0 = _pathRights[idx - 1];
            var right1 = _pathRights[idx];
            var up1 = _pathUps[idx];
            if (Vector3.Dot(right0, right1) < 0f)
            {
                right1 = -right1;
                up1 = -up1;
            }

            right = Vector3.Lerp(right0, right1, t).normalized;
            up = Vector3.Cross(right, forward).normalized;
            if (up == Vector3.zero)
            {
                up = Vector3.Lerp(_pathUps[idx - 1], up1, t).normalized;
                if (up == Vector3.zero) up = Vector3.up;
            }

            var exactStep = (idx - 1) + t;
            floatSegIdx = exactStep / stepPerCurve;
            return pos;
        }

        /// <summary>
        /// 依据基础参数化演进率 t (0~1) 直接折算未经分布均化的第三次贝塞尔曲线估计点。
        /// </summary>
        /// <param name="t">整条串联曲线生命周期内的全长进度参数百分比</param>
        /// <returns>未进行修正的平滑折算顶点</returns>
        public Vector3 EvaluateSplineRaw(float t)
        {
            return EvaluateSplineWithTangent(t, out _);
        }

        private Vector3 EvaluateSplineWithTangent(float t, out Vector3 tangent)
        {
            if (controlPoints == null || controlPoints.Length < 4)
            {
                tangent = Vector3.zero;
                return Vector3.zero;
            }

            var curveCount = (controlPoints.Length - 1) / 3;
            t = Mathf.Clamp01(t);
            if (MathF.Abs(t - 1f) < 0.0001f)
            {
                var lastCurveIndex = (curveCount - 1) * 3;
                tangent = EvaluateCubicBezierTangent(controlPoints[lastCurveIndex], controlPoints[lastCurveIndex + 1], controlPoints[lastCurveIndex + 2],
                    controlPoints[lastCurveIndex + 3], 1f);
                return controlPoints[^1];
            }

            // 根据 t 找出处于哪一段平滑曲线中
            var floatIndex = t * curveCount;
            var curveIndex = Mathf.FloorToInt(floatIndex);
            var clampedCurveIndex = Mathf.Min(curveIndex, curveCount - 1);

            // 计算局部t
            var localT = clampedCurveIndex == curveIndex ? floatIndex - curveIndex : 1f;
            var i = clampedCurveIndex * 3;

            tangent = EvaluateCubicBezierTangent(controlPoints[i], controlPoints[i + 1], controlPoints[i + 2], controlPoints[i + 3], localT);

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
        /// 从另一个同组件对象复制 controlPoints，并同步段偏移数组长度。
        /// </summary>
        public bool CopyControlPointsFrom(ParticlePathFollower source)
        {
            if (source == null || source == this || source.controlPoints == null || source.controlPoints.Length < 4)
                return false;

            controlPoints = new Vector3[source.controlPoints.Length];
            Array.Copy(source.controlPoints, controlPoints, source.controlPoints.Length);

            var curveCount = Mathf.Max(1, (controlPoints.Length - 1) / 3);
            var oldSegments = segmentOffsets;
            var newSegments = new PathOffsetData[curveCount];

            for (var i = 0; i < curveCount; i++)
            {
                if (oldSegments != null && i < oldSegments.Length && oldSegments[i] != null)
                    newSegments[i] = oldSegments[i];
                else
                    newSegments[i] = new PathOffsetData();
            }

            segmentOffsets = newSegments;

            // 复制路径后强制重建缓存，保证运行时立即生效。
            _pathCacheSignature = int.MinValue;
            _motionCacheSignature = int.MinValue;

            return true;
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

        public static Vector3 EvaluateCubicBezierTangent(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {
            t = Mathf.Clamp01(t);
            var u = 1 - t;

            return 3f * u * u * (p1 - p0) + 6f * u * t * (p2 - p1) + 3f * t * t * (p3 - p2);
        }
    }
}