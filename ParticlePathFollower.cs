using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using System.Collections.Generic;
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

        private struct SegmentOffsetNative
        {
            public float4 offset;
            public float innerVacuumX;
            public float innerVacuumY;
        }

        private struct ParticleJobInput
        {
            public float age;
            public uint randomSeed;
        }

        private struct ParticleJobOutput
        {
            public float3 position;
            public float3 forward;
            public float3 up;
            public float speed;
        }

        // 这个 Job 只负责纯数学计算，不直接访问 Unity 对象。
        // 主线程先把粒子的年龄、随机种子、路径缓存和 motion 缓存准备好，
        // Job 再并行算出每颗粒子的目标位置、朝向和速度，最后由主线程写回 ParticleSystem。
        [BurstCompile]
        private struct ParticlePathFollowerJob : IJobParallelFor
        {
            // 路径采样缓存：这些数组都是只读输入，避免 Job 内部再去碰曲线求值或做昂贵的重建。
            [ReadOnly] public NativeArray<float3> PathPoints;
            [ReadOnly] public NativeArray<float3> PathForwards;
            [ReadOnly] public NativeArray<float3> PathRights;
            [ReadOnly] public NativeArray<float3> PathUps;
            [ReadOnly] public NativeArray<float> PathDistances;
            [ReadOnly] public NativeArray<float> MotionTimeSamples;
            [ReadOnly] public NativeArray<float> MotionDistanceSamples;
            [ReadOnly] public NativeArray<SegmentOffsetNative> SegmentOffsets;
            [ReadOnly] public NativeArray<ParticleJobInput> Inputs;

            // 输出缓存：Job 把结果写到这里，主线程再统一回填到 ParticleSystem.Particle。
            public NativeArray<ParticleJobOutput> Outputs;

            // 下面这些字段是每帧传入 Job 的“运行参数”，用于控制距离反查、偏移模式和朝向逻辑。
            public float TotalPathLength;
            public float OneWayDuration;
            public float Speed;
            public float OffsetFrequency;
            public float TimeNow;
            public float DeltaTime;
            public int CurveCount;
            public int PathTravelModeValue;
            public int OffsetModeValue;
            public int ShouldAlign;
            public int HasOffset;
            public int EnableInnerVacuum;
            public int CircularShape;

            public void Execute(int index)
            {
                // 每个粒子只读取自己的输入，再独立完成整条路径上的映射，因此可以安全并行。
                var input = Inputs[index];
                var age = input.age;
                var distance = GetDistanceAtAge(age, out var isForwardTravel, out var baseEffectiveSpeed);

                var needsFrame = ShouldAlign != 0 || HasOffset != 0;

                var forward = new float3(0f, 0f, 0f);
                var right = new float3(1f, 0f, 0f);
                var up = new float3(0f, 1f, 0f);
                var sampleIndex = 0;
                var segIdx = 0f;

                var position = needsFrame
                    ? GetPointAtDistance(distance, out forward, out right, out up, out segIdx, out sampleIndex)
                    : GetPointPositionAtDistance(distance);

                if (!isForwardTravel && math.lengthsq(forward) > 0.000001f)
                    forward = -forward;

                var currentOffsetX = 0f;
                var currentOffsetY = 0f;
                var nextBasePos = position;
                var nextRight = right;
                var nextUp = up;
                var currentEffectiveSpeed = baseEffectiveSpeed;

                if (HasOffset != 0)
                {
                    EvaluateSegmentOffsetData(segIdx, out var segOffset, out var segInnerVacuumX, out var segInnerVacuumY);
                    var spawnTime = TimeNow - age;
                    var offset = GetOffsetSample((ParticlePathFollower.OffsetMode)OffsetModeValue, input.randomSeed, spawnTime, OffsetFrequency, segOffset,
                        segInnerVacuumX, segInnerVacuumY);
                    currentOffsetX = offset.x;
                    currentOffsetY = offset.y;
                }

                if (ShouldAlign != 0 && (currentOffsetX != 0f || currentOffsetY != 0f))
                {
                    var nextDistance = PredictNextDistance(distance, currentEffectiveSpeed, DeltaTime, isForwardTravel);
                    nextBasePos = GetPointAtDistance(nextDistance, sampleIndex, out _, out nextRight, out nextUp, out _, out _);

                    var baseDelta = nextBasePos - position;
                    if (currentEffectiveSpeed <= 0f)
                        currentEffectiveSpeed = math.length(baseDelta) / math.max(0.0001f, DeltaTime);

                    if (math.lengthsq(forward) <= 0.000001f && math.lengthsq(baseDelta) > 0.000001f)
                        forward = NormalizeSafe(baseDelta, new float3(1f, 0f, 0f));
                }

                if (currentOffsetX != 0f || currentOffsetY != 0f)
                {
                    position += right * currentOffsetX + up * currentOffsetY;

                    if (ShouldAlign != 0 && math.lengthsq(forward) > 0.000001f)
                    {
                        var nextPos = nextBasePos + nextRight * currentOffsetX + nextUp * currentOffsetY;
                        var offsetForward = nextPos - position;
                        if (math.lengthsq(offsetForward) > 0.000001f)
                            forward = NormalizeSafe(offsetForward, forward);
                    }
                }

                var finalUp = math.lengthsq(up) > 0.000001f ? NormalizeSafe(up, new float3(0f, 1f, 0f)) : new float3(0f, 1f, 0f);
                Outputs[index] = new ParticleJobOutput
                {
                    position = position,
                    forward = forward,
                    up = finalUp,
                    speed = currentEffectiveSpeed
                };
            }

            private float GetDistanceAtAge(float age, out bool isForwardTravel, out float effectiveSpeed)
            {
                // 先根据路径模式把“年龄”折算成局部时间，再用 motion 缓存把时间映射到物理距离。
                isForwardTravel = true;
                effectiveSpeed = 0f;
                if (TotalPathLength <= 0.0001f || Speed <= 0.0001f || OneWayDuration <= 0.0001f)
                    return 0f;

                switch ((ParticlePathFollower.PathTravelMode)PathTravelModeValue)
                {
                    case ParticlePathFollower.PathTravelMode.Loop:
                    {
                        var localTime = Repeat(age, OneWayDuration);
                        return GetDistanceAtTime(localTime, out effectiveSpeed);
                    }
                    case ParticlePathFollower.PathTravelMode.PingPong:
                    {
                        var period = OneWayDuration * 2f;
                        var localTime = Repeat(age, period);

                        if (localTime <= OneWayDuration)
                            return GetDistanceAtTime(localTime, out effectiveSpeed);

                        isForwardTravel = false;
                        return GetDistanceAtTime(period - localTime, out effectiveSpeed);
                    }
                    default:
                    {
                        var clamped = math.clamp(age, 0f, OneWayDuration);
                        return GetDistanceAtTime(clamped, out effectiveSpeed);
                    }
                }
            }

            private float GetDistanceAtTime(float elapsedTime, out float effectiveSpeed)
            {
                // 速度曲线积分后的时间表是单调递增的，先找附近采样点再线性插值，避免每次全表扫描。
                effectiveSpeed = 0f;
                if (MotionTimeSamples.Length < 2)
                    return 0f;

                if (elapsedTime <= 0f)
                    return 0f;

                var last = MotionTimeSamples.Length - 1;
                if (elapsedTime >= MotionTimeSamples[last])
                    return MotionDistanceSamples[last];

                var idx = FindMotionSampleIndex(elapsedTime);
                var left = math.max(0, idx - 1);
                var rightIndex = math.min(last, idx);

                var t0 = MotionTimeSamples[left];
                var t1 = MotionTimeSamples[rightIndex];
                effectiveSpeed = (MotionDistanceSamples[rightIndex] - MotionDistanceSamples[left]) / math.max(0.0001f, t1 - t0);
                var lerp = (elapsedTime - t0) / math.max(0.0001f, t1 - t0);
                return math.lerp(MotionDistanceSamples[left], MotionDistanceSamples[rightIndex], lerp);
            }

            private int FindMotionSampleIndex(float elapsedTime)
            {
                // 先按比例猜一个位置，再向两侧微调；比每次完整二分更适合粒子这种局部连续访问模式。
                var last = MotionTimeSamples.Length - 1;
                var lastTime = MotionTimeSamples[last];

                if (lastTime <= 0.0001f)
                    return 1;

                var estimated = math.clamp((int)(elapsedTime / lastTime * last), 1, last);

                while (estimated > 1 && MotionTimeSamples[estimated - 1] > elapsedTime)
                    estimated--;

                while (estimated < last && MotionTimeSamples[estimated] < elapsedTime)
                    estimated++;

                return estimated;
            }

            private float PredictNextDistance(float distance, float effectiveSpeed, float deltaTime, bool isForwardTravel)
            {
                // 用当前速度预测下一小段距离，减少后续为了算朝向而做的重复查表。
                var step = math.max(0f, effectiveSpeed) * deltaTime;

                switch ((ParticlePathFollower.PathTravelMode)PathTravelModeValue)
                {
                    case ParticlePathFollower.PathTravelMode.Loop:
                        return Repeat(distance + step, math.max(0.0001f, TotalPathLength));
                    case ParticlePathFollower.PathTravelMode.PingPong:
                        if (TotalPathLength <= 0.0001f)
                            return 0f;

                        if (isForwardTravel)
                        {
                            var next = distance + step;
                            if (next <= TotalPathLength)
                                return next;

                            return math.max(0f, TotalPathLength - (next - TotalPathLength));
                        }

                        var prev = distance - step;
                        if (prev >= 0f)
                            return prev;

                        return math.min(TotalPathLength, -prev);
                    default:
                        return math.clamp(distance + step, 0f, TotalPathLength);
                }
            }

            private float3 GetPointPositionAtDistance(float targetDistance)
            {
                // 只需要位置时，不去算朝向和截面坐标，减少不必要的插值开销。
                if (PathPoints.Length < 2)
                    return float3.zero;

                if (targetDistance <= 0f)
                    return PathPoints[0];

                if (targetDistance >= TotalPathLength)
                    return PathPoints[PathPoints.Length - 1];

                var idx = FindDistanceSampleIndex(targetDistance, -1);
                var d0 = PathDistances[idx - 1];
                var d1 = PathDistances[idx];
                var t = (targetDistance - d0) / math.max(0.0001f, d1 - d0);
                return math.lerp(PathPoints[idx - 1], PathPoints[idx], t);
            }

            private float3 GetPointAtDistance(float targetDistance, out float3 forward, out float3 right, out float3 up, out float floatSegIdx, out int sampleIndex)
            {
                return GetPointAtDistance(targetDistance, -1, out forward, out right, out up, out floatSegIdx, out sampleIndex);
            }

            private float3 GetPointAtDistance(float targetDistance, int hintSampleIndex, out float3 forward, out float3 right, out float3 up, out float floatSegIdx, out int sampleIndex)
            {
                // 需要朝向或偏移时，读取离散采样点并在相邻采样之间做平滑插值。
                if (PathPoints.Length < 2)
                {
                    forward = new float3(1f, 0f, 0f);
                    right = new float3(1f, 0f, 0f);
                    up = new float3(0f, 1f, 0f);
                    floatSegIdx = 0f;
                    sampleIndex = 0;
                    return float3.zero;
                }

                if (targetDistance <= 0f)
                {
                    forward = PathForwards[0];
                    right = PathRights[0];
                    up = PathUps[0];
                    floatSegIdx = 0f;
                    sampleIndex = 0;
                    return PathPoints[0];
                }

                if (targetDistance >= TotalPathLength)
                {
                    var last = PathPoints.Length - 1;
                    forward = PathForwards[last];
                    right = PathRights[last];
                    up = PathUps[last];
                    floatSegIdx = math.max(0, CurveCount);
                    sampleIndex = last;
                    return PathPoints[last];
                }

                var totalSteps = PathDistances.Length - 1;
                var stepPerCurve = totalSteps / (float)math.max(1, CurveCount);

                var idx = FindDistanceSampleIndex(targetDistance, hintSampleIndex);
                sampleIndex = idx;

                var d0 = PathDistances[idx - 1];
                var d1 = PathDistances[idx];
                var t = (targetDistance - d0) / math.max(0.0001f, d1 - d0);
                var pos = math.lerp(PathPoints[idx - 1], PathPoints[idx], t);
                forward = NormalizeSafe(math.lerp(PathForwards[idx - 1], PathForwards[idx], t), new float3(1f, 0f, 0f));

                var right0 = PathRights[idx - 1];
                var right1 = PathRights[idx];
                var up1 = PathUps[idx];
                if (math.dot(right0, right1) < 0f)
                {
                    right1 = -right1;
                    up1 = -up1;
                }

                right = NormalizeSafe(math.lerp(right0, right1, t), new float3(1f, 0f, 0f));
                up = NormalizeSafe(math.cross(right, forward), new float3(0f, 1f, 0f));
                if (math.lengthsq(up) < 0.000001f)
                    up = NormalizeSafe(math.lerp(PathUps[idx - 1], up1, t), new float3(0f, 1f, 0f));

                var exactStep = (idx - 1) + t;
                floatSegIdx = exactStep / stepPerCurve;
                return pos;
            }

            private int FindDistanceSampleIndex(float targetDistance, int hintSampleIndex)
            {
                // 优先复用上一次命中的附近索引；如果没有 hint，再退回二分查找。
                var last = PathDistances.Length - 1;

                if (hintSampleIndex > 0 && hintSampleIndex < last)
                {
                    var hint = hintSampleIndex;
                    if (PathDistances[hint] >= targetDistance)
                    {
                        while (hint > 1 && PathDistances[hint - 1] >= targetDistance)
                            hint--;
                        return math.clamp(hint, 1, last);
                    }

                    while (hint < last && PathDistances[hint] < targetDistance)
                        hint++;

                    return math.clamp(hint, 1, last);
                }

                var idx = 0;
                var low = 1;
                var high = last;
                while (low <= high)
                {
                    var mid = (low + high) >> 1;
                    if (PathDistances[mid] < targetDistance)
                    {
                        idx = mid;
                        low = mid + 1;
                    }
                    else
                    {
                        high = mid - 1;
                    }
                }

                return math.clamp(idx + 1, 1, last);
            }

            private void EvaluateSegmentOffsetData(float floatSegIdx, out float4 offset, out float innerVacuumX, out float innerVacuumY)
            {
                // 段偏移支持按段平滑过渡，这里先定位左右段，再用 SmoothStep 生成更自然的过渡权重。
                if (SegmentOffsets.Length == 0)
                {
                    offset = default;
                    innerVacuumX = 0f;
                    innerVacuumY = 0f;
                    return;
                }

                if (SegmentOffsets.Length == 1)
                {
                    var single = SegmentOffsets[0];
                    offset = single.offset;
                    innerVacuumX = single.innerVacuumX;
                    innerVacuumY = single.innerVacuumY;
                    return;
                }

                var lastIndex = SegmentOffsets.Length - 1;
                var clampedSeg = math.clamp(floatSegIdx, 0f, lastIndex);
                var leftIndex = (int)math.floor(clampedSeg);
                var rightIndex = math.min(leftIndex + 1, lastIndex);

                if (leftIndex == rightIndex)
                {
                    var data = SegmentOffsets[leftIndex];
                    offset = data.offset;
                    innerVacuumX = data.innerVacuumX;
                    innerVacuumY = data.innerVacuumY;
                    return;
                }

                var leftData = SegmentOffsets[leftIndex];
                var rightData = SegmentOffsets[rightIndex];

                var t = clampedSeg - leftIndex;
                t = t * t * (3f - 2f * t);

                offset = math.lerp(leftData.offset, rightData.offset, t);
                innerVacuumX = math.lerp(leftData.innerVacuumX, rightData.innerVacuumX, t);
                innerVacuumY = math.lerp(leftData.innerVacuumY, rightData.innerVacuumY, t);
            }

            private float2 GetOffsetSample(OffsetMode mode, uint randomSeed, float spawnTime, float frequency, float4 offset,
                float innerVacuumX, float innerVacuumY)
            {
                // 偏移采样根据模式切换：随机、重复、往返三种方式都在 Job 内完成，避免主线程逐粒子判断。
                switch (mode)
                {
                    case OffsetMode.Random:
                        return CalculateOffset2D(SeedToUnitFloat(randomSeed, 0), SeedToUnitFloat(randomSeed, 16), offset, innerVacuumX, innerVacuumY, true);
                    case OffsetMode.Repeat:
                        return CalculateOffset2DFromPhase(spawnTime * frequency, offset, innerVacuumX, innerVacuumY, false);
                    case OffsetMode.PingPong:
                        return CalculateOffset2DFromPhase(PingPong(spawnTime * frequency, 1f), offset, innerVacuumX, innerVacuumY, false);
                    default:
                        return float2.zero;
                }
            }

            private float2 CalculateOffset2DFromPhase(float phase, float4 offset, float innerVacuumX, float innerVacuumY, bool applyCircularShape)
            {
                var angle = Repeat(phase, 1f) * math.PI * 2f;
                return CalculateOffset2D(math.cos(angle) * 0.5f + 0.5f, math.sin(angle) * 0.5f + 0.5f, offset, innerVacuumX, innerVacuumY, applyCircularShape);
            }

            private float2 CalculateOffset2D(float t1, float t2, float4 offset, float innerVacuumX, float innerVacuumY, bool applyCircularShape)
            {
                // 先把两个 0~1 参数映射到标准盒子，再做真空裁切、圆形化、最后投射到真实偏移范围。
                var px = math.lerp(-1f, 1f, t1);
                var py = math.lerp(-1f, 1f, t2);

                if (EnableInnerVacuum != 0)
                {
                    var vx = math.clamp(innerVacuumX, 0f, 1f);
                    var vy = math.clamp(innerVacuumY, 0f, 1f);

                    if (vx > 0f || vy > 0f)
                    {
                        var absPx = math.abs(px);
                        var absPy = math.abs(py);
                        var maxAbs = math.max(absPx, absPy);

                        if (maxAbs < 0.0001f)
                        {
                            px = vx;
                            py = 0f;
                        }
                        else
                        {
                            var tOuter = 1f / maxAbs;
                            var txInner = absPx > 0.0001f ? vx / absPx : float.MaxValue;
                            var tyInner = absPy > 0.0001f ? vy / absPy : float.MaxValue;
                            var tInner = math.min(txInner, tyInner);

                            var r = maxAbs;
                            var tNew = tInner + r * (tOuter - tInner);

                            px *= tNew;
                            py *= tNew;
                        }
                    }
                }

                if (CircularShape != 0 && applyCircularShape)
                {
                    var absPx = math.abs(px);
                    var absPy = math.abs(py);
                    var maxAbs = math.max(absPx, absPy);
                    if (maxAbs > 0.0001f)
                    {
                        var d = math.sqrt(px * px + py * py);
                        px = px * maxAbs / d;
                        py = py * maxAbs / d;
                    }
                }

                var minX = math.min(offset.x, offset.y);
                var maxX = math.max(offset.x, offset.y);
                var minY = math.min(offset.z, offset.w);
                var maxY = math.max(offset.z, offset.w);

                var centerX = (minX + maxX) * 0.5f;
                var centerY = (minY + maxY) * 0.5f;
                var extentX = (maxX - minX) * 0.5f;
                var extentY = (maxY - minY) * 0.5f;

                var signX = offset.x > offset.y ? -1f : 1f;
                var signY = offset.z > offset.w ? -1f : 1f;

                var finalX = centerX + px * extentX * signX;
                var finalY = centerY + py * extentY * signY;

                return new float2(finalX, finalY);
            }

            private static float SeedToUnitFloat(uint seed, int shift)
            {
                return ((seed >> shift) & 0xFFFFu) / 65535f;
            }

            private static float Repeat(float t, float length)
            {
                if (length <= 0f)
                    return 0f;

                return t - math.floor(t / length) * length;
            }

            private static float PingPong(float t, float length)
            {
                if (length <= 0f)
                    return 0f;

                var doubleLength = length * 2f;
                var value = Repeat(t, doubleLength);
                return length - math.abs(value - length);
            }

            private static float3 NormalizeSafe(float3 value, float3 fallback)
            {
                // Job 内部不能依赖 Vector3.normalized 的隐式行为，这里显式处理零长度向量。
                var lengthSq = math.lengthsq(value);
                if (lengthSq <= 0.000001f)
                    return fallback;

                return value * math.rsqrt(lengthSq);
            }
        }

        private NativeArray<SegmentOffsetNative> _nativeSegmentOffsets;
        private NativeArray<float3> _nativePathPoints;
        private NativeArray<float3> _nativePathForwards;
        private NativeArray<float3> _nativePathRights;
        private NativeArray<float3> _nativePathUps;
        private NativeArray<float> _nativePathDistances;
        private NativeArray<float> _nativeMotionTimeSamples;
        private NativeArray<float> _nativeMotionDistanceSamples;
        private NativeArray<ParticleJobInput> _nativeParticleInputs;
        private NativeArray<ParticleJobOutput> _nativeParticleOutputs;

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
        private float _lastAppliedAutoLifetime = float.NaN;
        private bool _pathCacheDirty = true;
        private bool _motionCacheDirty = true;

        private void OnValidate()
        {
            speedSampleCount = Mathf.Clamp(speedSampleCount, 4, 128);
            speedOverPath = SanitizeCurve01(speedOverPath);
            if (prewarmDuration < 0f) prewarmDuration = 0f;

            MarkCachesDirty();
        }

        private void OnEnable()
        {
            MarkCachesDirty();

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

        private void OnDisable()
        {
            DisposeNativeCaches();
        }

        private void OnDestroy()
        {
            DisposeNativeCaches();
        }

        private void LateUpdate()
        {
            // 主线程负责三件事：更新缓存、把当前粒子状态转成 Job 输入、以及把 Job 结果写回粒子系统。
            InitializeIfNeeded();
            if (_pathCacheDirty)
                UpdatePathCache();
            if (_motionCacheDirty)
                UpdateMotionCache();
            SyncNativeSegmentOffsets();

            var main = _particleSystem.main;
            var simulationSpace = main.simulationSpace;
            var isLocalSimulation = simulationSpace == ParticleSystemSimulationSpace.Local;
            var compensationRotation = alignToPath ? Quaternion.Euler(overallRotationCompensation) : default;

            // 动态设置粒子的初始生命周期，使其刚好在到达路径终点时消失，避免过早消失或路径末尾堆积
            ApplyAutoLifetime(main);

            var count = _particleSystem.GetParticles(_particles);
            if (count == 0)
                return;

            EnsureNativeParticleJobBuffers(count);

            // 确保路径与 motion 的 Native 缓存已同步并可安全被 Job 读取。
            if ((!_nativePathPoints.IsCreated || _nativePathPoints.Length < 2) || (!_nativeMotionTimeSamples.IsCreated || _nativeMotionTimeSamples.Length < 2))
            {
                // 尝试把托管缓存同步到 Native 缓存（若托管缓存可用）
                SyncNativePathCache();
                SyncNativeMotionCache();
            }

            if (!_nativePathPoints.IsCreated || _nativePathPoints.Length < 2 || !_nativeMotionTimeSamples.IsCreated || _nativeMotionTimeSamples.Length < 2)
            {
                // 路径或 motion 缓存不完整，跳过 Job 调度以避免运行时访问未初始化的 NativeArray 引发崩溃。
                // 直接把当前粒子写回，保持与之前主线程逻辑一致（粒子位置保持不变）。
                _particleSystem.SetParticles(_particles, count);
                return;
            }

            for (var i = 0; i < count; i++)
            {
                var particle = _particles[i];
                _nativeParticleInputs[i] = new ParticleJobInput
                {
                    age = particle.startLifetime - particle.remainingLifetime,
                    randomSeed = particle.randomSeed
                };
            }

            const float velocityDeltaTime = 0.02f;
            // 这里先调度并等待 Job 完成，再统一写回粒子数组；这样计算部分可以并行，而 Unity API 仍留在主线程。
            var job = CreateParticleJob(Time.time, velocityDeltaTime);
            var handle = job.Schedule(count, 32);
            handle.Complete();

            var worldMatrix = transform.localToWorldMatrix;
            var worldRotation = transform.rotation;

            for (var i = 0; i < count; i++)
            {
                var output = _nativeParticleOutputs[i];
                var position = output.position;

                if (!isLocalSimulation)
                    position = worldMatrix.MultiplyPoint3x4(position);

                _particles[i].position = position;

                if (alignToPath && math.lengthsq(output.forward) > 0.000001f)
                {
                    var renderForward = output.forward;
                    var renderUp = output.up;

                    if (!isLocalSimulation)
                    {
                        renderForward = (worldRotation * renderForward).normalized;
                        renderUp = (worldRotation * renderUp).normalized;
                    }

                    var lookRotation = Quaternion.LookRotation(renderForward, renderUp) * compensationRotation;
                    _particles[i].rotation3D = lookRotation.eulerAngles;
                    _particles[i].velocity = renderForward * output.speed;
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
            {
                _lastAppliedAutoLifetime = float.NaN;
                return;
            }

            var travelModeMultiplier = pathTravelMode == PathTravelMode.PingPong ? 2f : 1f;
            float desiredLifetime;

            if (includeCurveInLifetime)
                desiredLifetime = Mathf.Max(0.01f, _oneWayDuration * travelModeMultiplier);
            else
                desiredLifetime = Mathf.Max(0.01f, (_totalPathLength / speed) * travelModeMultiplier);

            if (!float.IsNaN(_lastAppliedAutoLifetime) && Mathf.Abs(_lastAppliedAutoLifetime - desiredLifetime) < 0.0001f)
                return;

            main.startLifetime = desiredLifetime;
            _lastAppliedAutoLifetime = desiredLifetime;
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

            EnsureNativeArray(ref _nativeParticleInputs, maxParticles);
            EnsureNativeArray(ref _nativeParticleOutputs, maxParticles);
        }

        private void DisposeNativeCaches()
        {
            // NativeArray 是托管外内存，组件销毁或停用时必须释放，避免编辑器和运行时泄漏。
            DisposeNativeArray(ref _nativeSegmentOffsets);
            DisposeNativeArray(ref _nativePathPoints);
            DisposeNativeArray(ref _nativePathForwards);
            DisposeNativeArray(ref _nativePathRights);
            DisposeNativeArray(ref _nativePathUps);
            DisposeNativeArray(ref _nativePathDistances);
            DisposeNativeArray(ref _nativeMotionTimeSamples);
            DisposeNativeArray(ref _nativeMotionDistanceSamples);
            DisposeNativeArray(ref _nativeParticleInputs);
            DisposeNativeArray(ref _nativeParticleOutputs);
        }

        private void EnsureNativeParticleJobBuffers(int count)
        {
            // 每帧粒子数会变，这里只负责保证输入/输出缓冲区容量足够，不做额外分配逻辑以外的工作。
            // 使用扩容策略：仅在容量不足时扩容，为避免频繁收缩造成分配抖动，当前不尝试减小容量。
            EnsureNativeArrayCapacity(ref _nativeParticleInputs, count);
            EnsureNativeArrayCapacity(ref _nativeParticleOutputs, count);
        }

        private void SyncNativePathCache()
        {
            // 把托管侧的路径采样结果同步到 NativeArray，Job 就可以直接读这份只读缓存。
            if (_pathPoints == null || _pathForwards == null || _pathRights == null || _pathUps == null || _pathDistances == null)
                return;

            EnsureNativeArray(ref _nativePathPoints, _pathPoints.Length);
            EnsureNativeArray(ref _nativePathForwards, _pathForwards.Length);
            EnsureNativeArray(ref _nativePathRights, _pathRights.Length);
            EnsureNativeArray(ref _nativePathUps, _pathUps.Length);
            EnsureNativeArray(ref _nativePathDistances, _pathDistances.Length);

            for (var i = 0; i < _pathPoints.Length; i++)
            {
                _nativePathPoints[i] = _pathPoints[i];
                _nativePathForwards[i] = _pathForwards[i];
                _nativePathRights[i] = _pathRights[i];
                _nativePathUps[i] = _pathUps[i];
                _nativePathDistances[i] = _pathDistances[i];
            }
        }

        private void SyncNativeMotionCache()
        {
            // motion 缓存决定“时间 -> 距离”的映射，必须与托管侧保持一致，Job 才能得到同样的轨迹推进结果。
            if (_motionTimeSamples == null || _motionDistanceSamples == null)
                return;

            EnsureNativeArray(ref _nativeMotionTimeSamples, _motionTimeSamples.Length);
            EnsureNativeArray(ref _nativeMotionDistanceSamples, _motionDistanceSamples.Length);

            for (var i = 0; i < _motionTimeSamples.Length; i++)
            {
                _nativeMotionTimeSamples[i] = _motionTimeSamples[i];
                _nativeMotionDistanceSamples[i] = _motionDistanceSamples[i];
            }
        }

        private void SyncNativeSegmentOffsets()
        {
            // 段偏移数据按帧同步到 NativeArray，避免 Job 直接读托管对象数组和类实例引用。
            var sourceLength = segmentOffsets == null || segmentOffsets.Length == 0 ? 1 : segmentOffsets.Length;
            EnsureNativeArray(ref _nativeSegmentOffsets, sourceLength);

            if (segmentOffsets == null || segmentOffsets.Length == 0)
            {
                _nativeSegmentOffsets[0] = default;
                return;
            }

            for (var i = 0; i < segmentOffsets.Length; i++)
            {
                var data = segmentOffsets[i] ?? FallbackOffsetData;
                _nativeSegmentOffsets[i] = new SegmentOffsetNative
                {
                    offset = data.offset,
                    innerVacuumX = data.innerVacuumX,
                    innerVacuumY = data.innerVacuumY
                };
            }
        }

        private ParticlePathFollowerJob CreateParticleJob(float timeNow, float deltaTime)
        {
            // 把当前帧所有开关、缓存和运行参数打包成一个 Job 实例，确保 Job 执行时不依赖外部状态。
            return new ParticlePathFollowerJob
            {
                PathPoints = _nativePathPoints,
                PathForwards = _nativePathForwards,
                PathRights = _nativePathRights,
                PathUps = _nativePathUps,
                PathDistances = _nativePathDistances,
                MotionTimeSamples = _nativeMotionTimeSamples,
                MotionDistanceSamples = _nativeMotionDistanceSamples,
                SegmentOffsets = _nativeSegmentOffsets,
                Inputs = _nativeParticleInputs,
                Outputs = _nativeParticleOutputs,
                TotalPathLength = _totalPathLength,
                OneWayDuration = _oneWayDuration,
                Speed = speed,
                OffsetFrequency = offsetFrequency,
                TimeNow = timeNow,
                DeltaTime = deltaTime,
                CurveCount = Mathf.Max(1, (controlPoints.Length - 1) / 3),
                PathTravelModeValue = (int)pathTravelMode,
                OffsetModeValue = (int)offsetMode,
                ShouldAlign = alignToPath ? 1 : 0,
                HasOffset = offsetMode != OffsetMode.None ? 1 : 0,
                EnableInnerVacuum = enableInnerVacuum ? 1 : 0,
                CircularShape = circularShape ? 1 : 0
            };
        }

        private static void EnsureNativeArray<T>(ref NativeArray<T> array, int length) where T : struct
        {
            // 统一的 NativeArray 容量管理：长度不够就重新分配，长度合适则复用，避免每帧重复申请内存。
            if (length <= 0)
            {
                DisposeNativeArray(ref array);
                return;
            }

            if (array.IsCreated && array.Length == length)
                return;

            DisposeNativeArray(ref array);
            array = new NativeArray<T>(length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        }

        private static void EnsureNativeArrayCapacity<T>(ref NativeArray<T> array, int requiredLength) where T : struct
        {
            if (requiredLength <= 0)
            {
                DisposeNativeArray(ref array);
                return;
            }

            if (array.IsCreated && array.Length >= requiredLength)
                return;

            // 扩容倍数：至少按 2 倍增长，或直接满足 requiredLength
            var newLen = requiredLength;
            if (array.IsCreated)
            {
                newLen = Math.Max(requiredLength, array.Length * 2);
            }

            DisposeNativeArray(ref array);
            array = new NativeArray<T>(newLen, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        }

        private static void DisposeNativeArray<T>(ref NativeArray<T> array) where T : struct
        {
            // 释放前先判断 IsCreated，避免对未初始化数组调用 Dispose。
            if (array.IsCreated)
                array.Dispose();

            array = default;
        }

        /// <summary>
        /// 预计算并构建距离分布缓存查找表，将基于曲线 T 值的控制参数转为线性均匀的物理分布，防止粒子在曲率极变处堆积。
        /// </summary>
        /// <param name="stepsPerCurve">单段贝塞尔曲线的前向采样细分段数</param>
        private void UpdatePathCache(int stepsPerCurve = 25)
        {
            if (controlPoints == null || controlPoints.Length < 4) return;
            if (!_pathCacheDirty &&
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
            _pathCacheDirty = false;
            _motionCacheDirty = true;
            SyncNativePathCache();
        }

        /// <summary>
        /// 构建距离-时间查找表，用于支持沿路径的速度曲线积分与反查。
        /// </summary>
        private void UpdateMotionCache()
        {
            var sampleCount = Mathf.Max(4, speedSampleCount);

            if (!_motionCacheDirty && _motionTimeSamples != null && _motionTimeSamples.Length == sampleCount + 1 && _motionDistanceSamples != null && _motionDistanceSamples.Length == sampleCount + 1)
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

                _motionCacheDirty = false;

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
            _motionCacheDirty = false;
            SyncNativeMotionCache();
        }

        /// <summary>
        /// 将曲线约束到 x/y 都为 0~1，并保证首尾关键帧覆盖整个横轴。
        /// </summary>
        private static AnimationCurve SanitizeCurve01(AnimationCurve curve)
        {
            if (curve == null || curve.length == 0)
                return AnimationCurve.Linear(0f, 1f, 1f, 1f);

            // 复制并规范化关键帧，避免原地移动导致的复杂条件分支错误
            var src = curve.keys;
            var list = new List<Keyframe>(src.Length);
            for (var i = 0; i < src.Length; i++)
            {
                var k = src[i];
                k.time = Mathf.Clamp01(k.time);
                k.value = Mathf.Clamp01(k.value);
                list.Add(k);
            }

            list.Sort((a, b) => a.time.CompareTo(b.time));

            if (list.Count == 0)
            {
                list.Add(new Keyframe(0f, 1f));
                list.Add(new Keyframe(1f, 1f));
            }

            // 确保首尾覆盖 [0,1]
            if (list[0].time > 0f)
                list.Insert(0, new Keyframe(0f, Mathf.Clamp01(list[0].value)));
            else
            {
                var first = list[0];
                first.time = 0f;
                first.value = Mathf.Clamp01(first.value);
                list[0] = first;
            }

            if (list[list.Count - 1].time < 1f)
                list.Add(new Keyframe(1f, Mathf.Clamp01(list[list.Count - 1].value)));
            else
            {
                var last = list[list.Count - 1];
                last.time = 1f;
                last.value = Mathf.Clamp01(last.value);
                list[list.Count - 1] = last;
            }

            var sanitized = new AnimationCurve(list.ToArray())
            {
                preWrapMode = WrapMode.ClampForever,
                postWrapMode = WrapMode.ClampForever
            };

            return sanitized;
        }

        private float GetDistanceAtAge(float age, out bool isForwardTravel)
        {
            return GetDistanceAtAge(age, out isForwardTravel, out _);
        }

        private float GetDistanceAtAge(float age, out bool isForwardTravel, out float effectiveSpeed)
        {
            isForwardTravel = true;
            effectiveSpeed = 0f;
            if (_totalPathLength <= 0.0001f || speed <= 0.0001f || _oneWayDuration <= 0.0001f)
                return 0f;

            switch (pathTravelMode)
            {
                case PathTravelMode.Loop:
                {
                    var localTime = Mathf.Repeat(age, _oneWayDuration);
                    return GetDistanceAtTime(localTime, out effectiveSpeed);
                }
                case PathTravelMode.PingPong:
                {
                    var period = _oneWayDuration * 2f;
                    var localTime = Mathf.Repeat(age, period);

                    if (localTime <= _oneWayDuration)
                        return GetDistanceAtTime(localTime, out effectiveSpeed);

                    isForwardTravel = false;
                    return GetDistanceAtTime(period - localTime, out effectiveSpeed);
                }
                default:
                {
                    var clamped = Mathf.Clamp(age, 0f, _oneWayDuration);
                    return GetDistanceAtTime(clamped, out effectiveSpeed);
                }
            }
        }

        private float GetDistanceAtTime(float elapsedTime)
        {
            return GetDistanceAtTime(elapsedTime, out _);
        }

        private float GetDistanceAtTime(float elapsedTime, out float effectiveSpeed)
        {
            effectiveSpeed = 0f;
            if (_motionTimeSamples == null || _motionTimeSamples.Length < 2)
                return 0f;

            if (elapsedTime <= 0f)
                return 0f;

            var last = _motionTimeSamples.Length - 1;
            if (elapsedTime >= _motionTimeSamples[last])
                return _motionDistanceSamples[last];

            var idx = FindMotionSampleIndex(elapsedTime);
            var left = Mathf.Max(0, idx - 1);
            var right = Mathf.Min(last, idx);

            var t0 = _motionTimeSamples[left];
            var t1 = _motionTimeSamples[right];
            effectiveSpeed = (_motionDistanceSamples[right] - _motionDistanceSamples[left]) / Mathf.Max(0.0001f, t1 - t0);
            var lerp = (elapsedTime - t0) / Mathf.Max(0.0001f, t1 - t0);
            return Mathf.Lerp(_motionDistanceSamples[left], _motionDistanceSamples[right], lerp);
        }

        private int FindMotionSampleIndex(float elapsedTime)
        {
            var last = _motionTimeSamples.Length - 1;
            var lastTime = _motionTimeSamples[last];

            if (lastTime <= 0.0001f)
                return 1;

            var estimated = Mathf.Clamp((int)(elapsedTime / lastTime * last), 1, last);

            while (estimated > 1 && _motionTimeSamples[estimated - 1] > elapsedTime)
                estimated--;

            while (estimated < last && _motionTimeSamples[estimated] < elapsedTime)
                estimated++;

            return estimated;
        }

        private float PredictNextDistance(float distance, float effectiveSpeed, float deltaTime, bool isForwardTravel)
        {
            var step = Mathf.Max(0f, effectiveSpeed) * deltaTime;

            switch (pathTravelMode)
            {
                case PathTravelMode.Loop:
                    return Mathf.Repeat(distance + step, Mathf.Max(0.0001f, _totalPathLength));
                case PathTravelMode.PingPong:
                    if (_totalPathLength <= 0.0001f)
                        return 0f;

                    if (isForwardTravel)
                    {
                        var next = distance + step;
                        if (next <= _totalPathLength)
                            return next;

                        return Mathf.Max(0f, _totalPathLength - (next - _totalPathLength));
                    }

                    var prev = distance - step;
                    if (prev >= 0f)
                        return prev;

                    return Mathf.Min(_totalPathLength, -prev);
                default:
                    return Mathf.Clamp(distance + step, 0f, _totalPathLength);
            }
        }

        /// <summary>
        /// 基于全局二维极坐标系推演真实的横截面偏移量。
        /// 处理流程分为三步：先把采样参数映射到标准二维坐标，再做真空裁切与圆形/椭圆形修正，最后投射到实际偏移范围。
        /// </summary>
        /// <param name="t1">第一轴向插值参数 (0~1)</param>
        /// <param name="t2">第二轴向插值参数 (0~1)</param>
        /// <returns>处理真空裁切及圆平滑折变后的最终 2D 实际偏移值</returns>
        private Vector2 CalculateOffset2D(float t1, float t2, Vector4 offset, float innerVacuumX, float innerVacuumY, bool applyCircularShape)
        {
            // 第一步：把 0~1 的随机/相位采样结果映射到 [-1, 1] 的标准盒子里。
            var px = Mathf.Lerp(-1f, 1f, t1);
            var py = Mathf.Lerp(-1f, 1f, t2);

            // 第二步：如果启用了内部真空区，就把中心区域沿射线方向外推，避免出现十字形空洞。
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
                        // 落在原点时没有方向可言，直接推到一个安全边缘位置，避免除零。
                        px = vx;
                        py = 0f;
                    }
                    else
                    {
                        // 先算出到外边界的比例，再把真空半径折算进去，让内外区域连续衔接。
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

            // 如果需要圆形或椭圆形分布，就把方形包围盒再压成圆形轮廓。
            if (circularShape && applyCircularShape)
            {
                var absPx = Mathf.Abs(px);
                var absPy = Mathf.Abs(py);
                var maxAbs = Mathf.Max(absPx, absPy);
                if (maxAbs > 0.0001f)
                {
                    // 保持射线方向不变，只调整到同心圆边界，避免方形采样的角落偏差。
                    var d = Mathf.Sqrt(px * px + py * py);
                    px = px * maxAbs / d;
                    py = py * maxAbs / d;
                }
            }

            // 第三步：把标准化结果缩放、平移到用户配置的真实偏移区间。
            var minX = Mathf.Min(offset.x, offset.y);
            var maxX = Mathf.Max(offset.x, offset.y);
            var minY = Mathf.Min(offset.z, offset.w);
            var maxY = Mathf.Max(offset.z, offset.w);

            // 中心点决定偏移的基准位置，extent 决定横竖两个方向的实际范围。
            var centerX = (minX + maxX) * 0.5f;
            var centerY = (minY + maxY) * 0.5f;
            var extentX = (maxX - minX) * 0.5f;
            var extentY = (maxY - minY) * 0.5f;

            // 如果用户把 min/max 以反向方式填写，这里保留方向语义，避免结果被翻转。
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

            // 整段连续平滑插值：每段区间 [i, i+1] 内都从 i 平滑过渡到 i+1。
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
        /// <param name="sampleIndex">输出：命中的距离采样索引，可用于复用附近查询</param>
        /// <returns>推算所处在物理世界中的实际位置标点</returns>
        private Vector3 GetPointAtDistance(float targetDistance, out Vector3 forward, out Vector3 right, out Vector3 up, out float floatSegIdx, out int sampleIndex)
        {
            return GetPointAtDistance(targetDistance, -1, out forward, out right, out up, out floatSegIdx, out sampleIndex);
        }

        private Vector3 GetPointAtDistance(float targetDistance, int hintSampleIndex, out Vector3 forward, out Vector3 right, out Vector3 up, out float floatSegIdx, out int sampleIndex)
        {
            if (_pathCacheDirty || _pathRights == null || _pathUps == null || _pathForwards == null || _pathDistances == null)
            {
                UpdatePathCache();
            }

            if (_pathPoints == null || _pathPoints.Length < 2)
            {
                forward = Vector3.right;
                right = Vector3.right;
                up = Vector3.up;
                floatSegIdx = 0f;
                sampleIndex = 0;
                return Vector3.zero;
            }

            if (targetDistance <= 0f)
            {
                forward = _pathForwards[0];
                right = _pathRights[0];
                up = _pathUps[0];
                floatSegIdx = 0f;
                sampleIndex = 0;
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
                sampleIndex = last;
                return _pathPoints[last];
            }

            var totalSteps = _pathDistances.Length - 1;
            var stepPerCurve = totalSteps / (float)Mathf.Max(1, curveCount);

            var idx = FindDistanceSampleIndex(targetDistance, hintSampleIndex);
            sampleIndex = idx;

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

        private Vector3 GetPointPositionAtDistance(float targetDistance)
        {
            if (_pathCacheDirty || _pathPoints == null || _pathDistances == null)
            {
                UpdatePathCache();
            }

            if (_pathPoints == null || _pathPoints.Length < 2)
                return Vector3.zero;

            if (targetDistance <= 0f)
                return _pathPoints[0];

            if (targetDistance >= _totalPathLength)
                return _pathPoints[^1];

            var idx = FindDistanceSampleIndex(targetDistance, -1);

            var d0 = _pathDistances[idx - 1];
            var d1 = _pathDistances[idx];
            var t = (targetDistance - d0) / Mathf.Max(0.0001f, d1 - d0);
            return Vector3.Lerp(_pathPoints[idx - 1], _pathPoints[idx], t);
        }

        private int FindDistanceSampleIndex(float targetDistance, int hintSampleIndex)
        {
            var last = _pathDistances.Length - 1;

            if (hintSampleIndex > 0 && hintSampleIndex < last)
            {
                var hint = hintSampleIndex;
                if (_pathDistances[hint] >= targetDistance)
                {
                    while (hint > 1 && _pathDistances[hint - 1] >= targetDistance)
                        hint--;
                    return hint;
                }

                while (hint < last && _pathDistances[hint] < targetDistance)
                    hint++;

                return Mathf.Clamp(hint, 1, last);
            }

            var idx = Array.BinarySearch(_pathDistances, targetDistance);
            if (idx < 0) idx = ~idx;
            return Mathf.Clamp(idx, 1, last);
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

            MarkCachesDirty();
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

            MarkCachesDirty();
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

            MarkCachesDirty();

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

            MarkCachesDirty();
        }

        public void MarkCachesDirty()
        {
            _pathCacheDirty = true;
            _motionCacheDirty = true;
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