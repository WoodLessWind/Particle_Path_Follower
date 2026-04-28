using Script.Particles;
using UnityEditor;
using UnityEngine;

namespace Editor
{
    [CustomEditor(typeof(ParticlePathFollower))]
    public class ParticlePathFollowerEditor : UnityEditor.Editor
    {
        private int _selectedIndex = -1;

        private void OnSceneGUI()
        {
            var pathFollower = (ParticlePathFollower)target;

            if (pathFollower.controlPoints == null || pathFollower.controlPoints.Length < 4) return;

            var handleTransform = pathFollower.transform;
            var handleRotation = Tools.pivotRotation == PivotRotation.Local
                ? handleTransform.rotation
                : Quaternion.identity;

            // 绘制控制点手柄
            for (var i = 0; i < pathFollower.controlPoints.Length; i++)
            {
                var p = handleTransform.TransformPoint(pathFollower.controlPoints[i]);

                // 绘制示意点：路径点为绿色，控制点为深绿色
                Handles.color = i % 3 == 0 ? Color.green : new Color(0f, 0.5f, 0f);

                // 当前选中的节点可以高亮显示
                if (_selectedIndex == i) Handles.color = Color.yellow;

                var dotSize = HandleUtility.GetHandleSize(p) * 0.1f;
                // 使用 Button 代替单纯的 SphereHandleCap，点击时可以选中该点
                if (Handles.Button(p, handleRotation, dotSize, dotSize, Handles.SphereHandleCap))
                {
                    _selectedIndex = i;

                    // 如果按住 Shift 选中且当前选中的是曲柄点（控制点），则以其自身为准将对面的曲柄强行拉回对称状态，恢复同步控制
                    if (Event.current.shift && i % 3 != 0)
                    {
                        var isForward = i % 3 == 1;
                        var anchorIdx = isForward ? i - 1 : i + 1;
                        var twinIdx = isForward ? i - 2 : i + 2;

                        if (twinIdx >= 0 && twinIdx < pathFollower.controlPoints.Length)
                        {
                            Undo.RecordObject(pathFollower, "Restore Smooth");
                            var anchorPos = pathFollower.controlPoints[anchorIdx];
                            var myPos = pathFollower.controlPoints[i];
                            var oppositeDir = (anchorPos - myPos).normalized;
                            var myDist = Vector3.Distance(anchorPos, myPos);
                            pathFollower.controlPoints[twinIdx] = anchorPos + oppositeDir * myDist;
                            EditorUtility.SetDirty(pathFollower);
                        }
                    }

                    Repaint();
                }

                // 只有被选中的控制点才显示位移坐标轴
                if (_selectedIndex == i)
                {
                    EditorGUI.BeginChangeCheck();

                    p = Handles.DoPositionHandle(p, handleRotation);

                    if (EditorGUI.EndChangeCheck())
                    {
                        Undo.RecordObject(pathFollower, "Move Path Point");

                        var newLocalPos = handleTransform.InverseTransformPoint(p);

                        // 按住 Shift 时，允许吸附到其他节点上
                        if (Event.current.shift)
                        {
                            const float snapRadius = 0.5f; // 吸附半径
                            for (var j = 0; j < pathFollower.controlPoints.Length; j++)
                            {
                                if (j == i) continue;
                                if (Vector3.Distance(newLocalPos, pathFollower.controlPoints[j]) < snapRadius)
                                {
                                    newLocalPos = pathFollower.controlPoints[j];
                                    break;
                                }
                            }
                        }

                        var delta = newLocalPos - pathFollower.controlPoints[i];

                        // 记录原始位置以推断操作前是否处于联动（平滑）状态
                        var originalPos = pathFollower.controlPoints[i];
                        var wasSmooth = false;
                        var anchorIndex = -1;
                        var twinIndex = -1;

                        if (i % 3 != 0)
                        {
                            var isForwardTangent = i % 3 == 1;
                            anchorIndex = isForwardTangent ? i - 1 : i + 1;
                            twinIndex = isForwardTangent ? i - 2 : i + 2;

                            if (twinIndex >= 0 && twinIndex < pathFollower.controlPoints.Length)
                            {
                                var anchorPos = pathFollower.controlPoints[anchorIndex];
                                var twinPos = pathFollower.controlPoints[twinIndex];

                                var originalDir = (anchorPos - originalPos).normalized;
                                var twinDir = (twinPos - anchorPos).normalized;

                                // 判别容差范围内（<1度且距离近似）视为两边是对称同步的
                                var angleSmooth = Vector3.Angle(originalDir, twinDir) < 1f;
                                var originalDist = Vector3.Distance(anchorPos, originalPos);
                                var twinDist = Vector3.Distance(anchorPos, twinPos);
                                var distSmooth = Mathf.Abs(originalDist - twinDist) < 0.05f;

                                wasSmooth = angleSmooth && distSmooth;
                            }
                        }

                        pathFollower.controlPoints[i] = newLocalPos;

                        // 如果移动的是锚点 (索引是3的倍数)，与其关联的控制点应该跟随移动
                        if (i % 3 == 0)
                        {
                            if (i - 1 >= 0)
                                pathFollower.controlPoints[i - 1] += delta;
                            if (i + 1 < pathFollower.controlPoints.Length)
                                pathFollower.controlPoints[i + 1] += delta;
                        }
                        else
                        {
                            // 如果没有按住 Shift且当前控制点原先处于联动状态，则继续应用联动
                            if (!Event.current.shift && wasSmooth && twinIndex >= 0 &&
                                twinIndex < pathFollower.controlPoints.Length)
                            {
                                var anchorPos = pathFollower.controlPoints[anchorIndex];
                                var movedPos = pathFollower.controlPoints[i];

                                // 对向曲柄的方向和距离应与当前曲柄完全对称
                                var oppositeDir = (anchorPos - movedPos).normalized;
                                var movedDist = Vector3.Distance(anchorPos, movedPos);

                                pathFollower.controlPoints[twinIndex] = anchorPos + oppositeDir * movedDist;
                            }
                        }

                        EditorUtility.SetDirty(pathFollower);
                    }
                }
            }

            // 绘制所有贝塞尔曲线段和辅助线
            Handles.color = Color.white;
            var curveCount = (pathFollower.controlPoints.Length - 1) / 3;

            for (var i = 0; i < curveCount; i++)
            {
                var startIndex = i * 3;
                var p0 = handleTransform.TransformPoint(pathFollower.controlPoints[startIndex]);
                var p1 = handleTransform.TransformPoint(pathFollower.controlPoints[startIndex + 1]);
                var p2 = handleTransform.TransformPoint(pathFollower.controlPoints[startIndex + 2]);
                var p3 = handleTransform.TransformPoint(pathFollower.controlPoints[startIndex + 3]);

                Handles.DrawLine(p0, p1);
                Handles.DrawLine(p2, p3);

                Handles.DrawBezier(p0, p3, p1, p2, Color.green, null, 2f);
            }
        }

        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            var pathFollower = (ParticlePathFollower)target;

            GUILayout.Space(10);
            if (GUILayout.Button("添加路径节点"))
            {
                Undo.RecordObject(pathFollower, "Add Segment");
                pathFollower.AddSegment();
                EditorUtility.SetDirty(pathFollower);
            }

            GUILayout.Space(5);
            if (GUILayout.Button("自动计算平滑"))
            {
                Undo.RecordObject(pathFollower, "Auto Smooth");
                pathFollower.AutoSmooth();
                EditorUtility.SetDirty(pathFollower);
            }
        }
    }
}