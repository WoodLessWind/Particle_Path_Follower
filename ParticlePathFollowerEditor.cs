using System;
using Script.Particles;
using UnityEditor;
using UnityEngine;

namespace Editor
{
    /// <summary>
    /// ParticlePathFollower 核心组件的自定义编辑器扩展。
    /// 提供了在 Scene 视图中直观操作贝塞尔曲线锚指、曲柄的联动与吸附能力，并扩展面板动作。
    /// </summary>
    [CustomEditor(typeof(ParticlePathFollower))]
    public class ParticlePathFollowerEditor : UnityEditor.Editor
    {
        /// <summary>
        /// 当前于 Scene 视图内处于聚焦选中状态的控制点索引（-1 为未选中物体）
        /// </summary>
        private int _selectedIndex = -1;

        /// <summary>
        /// 当前于 Scene 视图内选中的曲线跨度段落的索引。用于独立控制每一段的局部偏移数值。
        /// </summary>
        private int _selectedSegmentIndex = -1;

        private Vector4 _dragStartBounds;
        private int _draggingAxis = -1;

        private void OnEnable()
        {
            // 隐藏组件自带的 Transform 中心操作轴，避免与起点(0)的节点控制轴挤在一起导致难以拖拽
            Tools.hidden = true;
        }

        private void OnDisable()
        {
            // 取消选中或者销毁组件时，恢复默认的工具轴显示
            Tools.hidden = false;
        }

        /// <summary>
        /// Scene 视图环境下的周期轮询与渲染管线回调。
        /// 此处负责绘制可供视口直接交互的球形操作手柄，处理多态选点事件、连线张力修复以及贝塞尔线段的可视化描绘。
        /// </summary>
        private void OnSceneGUI()
        {
            if (GUIUtility.hotControl == 0)
            {
                _draggingAxis = -1;
            }

            var pathFollower = (ParticlePathFollower)target;

            if (pathFollower.controlPoints == null || pathFollower.controlPoints.Length < 4) return;

            var handleTransform = pathFollower.transform;
            var handleRotation = Tools.pivotRotation == PivotRotation.Local
                ? handleTransform.rotation
                : Quaternion.identity;

            // 监听键盘退格键与删除键事件，用于快捷删除当前选中的节点
            if (Event.current.type == EventType.KeyDown && (Event.current.keyCode == KeyCode.Backspace || Event.current.keyCode == KeyCode.Delete))
            {
                if (_selectedIndex >= 0 && pathFollower.controlPoints.Length > 4)
                {
                    Undo.RegisterCompleteObjectUndo(pathFollower, "Delete Path Node");

                    // 找到就近包含的锚点所对应的节点编号（每一个完整节点跨度为3）
                    var nodeIndex = Mathf.RoundToInt(_selectedIndex / 3f);
                    if (nodeIndex * 3 >= pathFollower.controlPoints.Length) 
                        nodeIndex = (pathFollower.controlPoints.Length - 1) / 3;

                    pathFollower.DeleteNode(nodeIndex);
                    _selectedIndex = -1;
                    _selectedSegmentIndex = -1;
                    EditorUtility.SetDirty(pathFollower);
                    Event.current.Use(); // 消耗当前事件避免触发 Unity 默认摧毁物体的热键操作
                }
            }

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

            // 绘制所有贝塞尔曲线段和辅助线及局部的横截面范围
            Handles.color = Color.white;
            var curveCount = (pathFollower.controlPoints.Length - 1) / 3;

            // 为了支持旧数据同步兼容
            if (pathFollower.segmentOffsets == null || pathFollower.segmentOffsets.Length < curveCount)
            {
                Array.Resize(ref pathFollower.segmentOffsets, Mathf.Max(1, curveCount));
                if (pathFollower.segmentOffsets[^1] == null) pathFollower.segmentOffsets[^1] = new ParticlePathFollower.PathOffsetData();
            }

            for (var i = 0; i < curveCount; i++)
            {
                var startIndex = i * 3;
                var p0 = handleTransform.TransformPoint(pathFollower.controlPoints[startIndex]);
                var p1 = handleTransform.TransformPoint(pathFollower.controlPoints[startIndex + 1]);
                var p2 = handleTransform.TransformPoint(pathFollower.controlPoints[startIndex + 2]);
                var p3 = handleTransform.TransformPoint(pathFollower.controlPoints[startIndex + 3]);

                Handles.color = Color.white;
                Handles.DrawLine(p0, p1);
                Handles.DrawLine(p2, p3);

                // 绘制段落的专属手柄（位于当前贝塞尔折弯的一半处, t=0.5f）
                var midT = 0.5f;
                var segmentCenter = handleTransform.TransformPoint(ParticlePathFollower.EvaluateCubicBezier(
                    pathFollower.controlPoints[startIndex], 
                    pathFollower.controlPoints[startIndex + 1], 
                    pathFollower.controlPoints[startIndex + 2], 
                    pathFollower.controlPoints[startIndex + 3], midT));

                Handles.color = _selectedSegmentIndex == i ? Color.cyan : new Color(0, 0.8f, 1f, 0.4f);
                var midSize = HandleUtility.GetHandleSize(segmentCenter) * 0.12f;

                if (Handles.Button(segmentCenter, handleRotation, midSize, midSize, Handles.CubeHandleCap))
                {
                    _selectedSegmentIndex = i;
                    _selectedIndex = -1; // 互斥撤销节点全选
                    Repaint();
                }

                // 只有处于选中状态或者是局部截面时，绘制这段截面真空范围圈
                if (_selectedSegmentIndex == i && pathFollower.offsetMode != ParticlePathFollower.OffsetMode.None)
                {
                    var fwLoc = ParticlePathFollower.EvaluateCubicBezier(
                        pathFollower.controlPoints[startIndex], pathFollower.controlPoints[startIndex + 1], 
                        pathFollower.controlPoints[startIndex + 2], pathFollower.controlPoints[startIndex + 3], midT + 0.01f);
                    var fwP = handleTransform.TransformPoint(fwLoc);
                    var segFwd = (fwP - segmentCenter).normalized;
                    if (segFwd == Vector3.zero) segFwd = Vector3.forward;

                    var segRight = Vector3.Cross(segFwd, Vector3.forward).normalized;
                    if (segRight == Vector3.zero) segRight = Vector3.right;
                    var segUp = Vector3.Cross(segRight, segFwd).normalized;

                    var segData = pathFollower.segmentOffsets[i];
                    var minX = Mathf.Min(segData.offset.x, segData.offset.y);
                    var maxX = Mathf.Max(segData.offset.x, segData.offset.y);
                    var minY = Mathf.Min(segData.offset.z, segData.offset.w);
                    var maxY = Mathf.Max(segData.offset.z, segData.offset.w);

                    var cX = (minX + maxX) * 0.5f;
                    var cY = (minY + maxY) * 0.5f;
                    var eX = (maxX - minX) * 0.5f;
                    var eY = (maxY - minY) * 0.5f;

                    var centerWorld = segmentCenter + segRight * cX + segUp * cY;

                    Handles.color = Color.cyan;
                    if (pathFollower.circularShape)
                    {
                        DrawEllipseWire(centerWorld, segFwd, segRight, segUp, eX, eY);
                        if (pathFollower.enableInnerVacuum)
                        {
                            Handles.color = Color.red;
                            DrawEllipseWire(centerWorld, segFwd, segRight, segUp, eX * segData.innerVacuumX, eY * segData.innerVacuumY);
                        }
                    }
                    else
                    {
                        DrawRectWire(centerWorld, segRight, segUp, eX, eY);
                        if (pathFollower.enableInnerVacuum)
                        {
                            Handles.color = Color.red;
                            DrawRectWire(centerWorld, segRight, segUp, eX * segData.innerVacuumX, eY * segData.innerVacuumY);
                        }
                    }

                    // 绘制可拖拽的外边界控制手柄
                    var rightPos = segmentCenter + segRight * segData.offset.y + segUp * cY;
                    var leftPos = segmentCenter + segRight * segData.offset.x + segUp * cY;
                    var topPos = segmentCenter + segRight * cX + segUp * segData.offset.w;
                    var bottomPos = segmentCenter + segRight * cX + segUp * segData.offset.z;

                    Handles.color = Color.yellow;

                    var handleSizeR = HandleUtility.GetHandleSize(rightPos) * 0.08f;
                    var handleSizeL = HandleUtility.GetHandleSize(leftPos) * 0.08f;
                    var handleSizeT = HandleUtility.GetHandleSize(topPos) * 0.08f;
                    var handleSizeB = HandleUtility.GetHandleSize(bottomPos) * 0.08f;

                    void ProcessSlider(int axis, Vector3 pos, Vector3 slideDir, float size, Vector3 projAxis, bool isNegativeAxis)
                    {
                        EditorGUI.BeginChangeCheck();
                        var newPos = Handles.Slider(pos, slideDir, size, Handles.CubeHandleCap, 0);
                        if (EditorGUI.EndChangeCheck())
                        {
                            Undo.RegisterCompleteObjectUndo(pathFollower, "Change Segment Bounds");

                            if (_draggingAxis != axis)
                            {
                                _draggingAxis = axis;
                                _dragStartBounds = segData.offset;
                            }

                            float absVal = Vector3.Dot(newPos - segmentCenter, projAxis);
                            float startVal = axis == 0 ? _dragStartBounds.x : (axis == 1 ? _dragStartBounds.y : (axis == 2 ? _dragStartBounds.z : _dragStartBounds.w));
                            float activeDelta = isNegativeAxis ? -(absVal - startVal) : (absVal - startVal);

                            // 松开 Shift 未松开鼠标时，使用 _dragStartBounds 恢复其他轴；按下则全部分配增量
                            if (Event.current.shift)
                            {
                                segData.offset.x = _dragStartBounds.x - activeDelta;
                                segData.offset.y = _dragStartBounds.y + activeDelta;
                                segData.offset.z = _dragStartBounds.z - activeDelta;
                                segData.offset.w = _dragStartBounds.w + activeDelta;
                            }
                            else
                            {
                                segData.offset = _dragStartBounds;
                                if (axis == 0) segData.offset.x = absVal;
                                else if (axis == 1) segData.offset.y = absVal;
                                else if (axis == 2) segData.offset.z = absVal;
                                else if (axis == 3) segData.offset.w = absVal;
                            }

                            EditorUtility.SetDirty(pathFollower);
                        }
                    }

                    ProcessSlider(1, rightPos, segRight, handleSizeR, segRight, false); // Right = y
                    ProcessSlider(0, leftPos, -segRight, handleSizeL, segRight, true); // Left = x
                    ProcessSlider(3, topPos, segUp, handleSizeT, segUp, false); // Top = w
                    ProcessSlider(2, bottomPos, -segUp, handleSizeB, segUp, true); // Bottom = z
                }

                Handles.DrawBezier(p0, p3, p1, p2, Color.green, null, 2f);
            }
        }

        private void DrawRectWire(Vector3 center, Vector3 right, Vector3 up, float extX, float extY)
        {
            var p1 = center + right * extX + up * extY;
            var p2 = center - right * extX + up * extY;
            var p3 = center - right * extX - up * extY;
            var p4 = center + right * extX - up * extY;
            Handles.DrawLine(p1, p2); Handles.DrawLine(p2, p3);
            Handles.DrawLine(p3, p4); Handles.DrawLine(p4, p1);
        }

        private void DrawEllipseWire(Vector3 center, Vector3 normal, Vector3 right, Vector3 up, float extX, float extY)
        {
            int segments = 36;
            Vector3 prev = Vector3.zero;
            for (int i = 0; i <= segments; i++)
            {
                float rad = Mathf.Deg2Rad * (i * 360f / segments);
                Vector3 p = center + right * (Mathf.Cos(rad) * extX) + up * (Mathf.Sin(rad) * extY);
                if (i > 0) Handles.DrawLine(prev, p);
                prev = p;
            }
        }

        /// <summary>
        /// 重载并定制 Inspector 检查器面板上的布局绘制。
        /// 挂载原组件属性的同时，尾部追加快捷编辑工具触发按钮，用于曲线伸展及连接处极速折角平滑化。
        /// </summary>
        public override void OnInspectorGUI()
        {
            var pathFollower = (ParticlePathFollower)target;

            GUILayout.Space(5);
            EditorGUILayout.LabelField("路径设置 (Path Nodes)", EditorStyles.boldLabel);

            // 仅提取并在面板上显示锚点（每三个点中的第一个），将控制点隐藏
            for (var i = 0; i < pathFollower.controlPoints.Length; i += 3)
            {
                EditorGUILayout.BeginHorizontal();

                // 同步高亮选中状态
                var isSelected = (_selectedIndex == i);
                if (isSelected) GUI.backgroundColor = Color.yellow;

                EditorGUI.BeginChangeCheck();
                var newPos = EditorGUILayout.Vector3Field($"节点 {i / 3}", pathFollower.controlPoints[i]);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(pathFollower, "Change Path Point");

                    // 当在 Inspector 中修改锚点坐标时，保证连带的控制扭柄等比跟随
                    var delta = newPos - pathFollower.controlPoints[i];
                    pathFollower.controlPoints[i] = newPos;
                    if (i - 1 >= 0) pathFollower.controlPoints[i - 1] += delta;
                    if (i + 1 < pathFollower.controlPoints.Length) pathFollower.controlPoints[i + 1] += delta;

                    EditorUtility.SetDirty(pathFollower);
                    SceneView.RepaintAll();
                }

                if (GUILayout.Button("选中", GUILayout.Width(50)))
                {
                    _selectedIndex = i;
                    _selectedSegmentIndex = -1;
                    SceneView.RepaintAll(); // 强制刷新 Scene 视图显式定位柄
                }

                if (pathFollower.controlPoints.Length > 4)
                {
                    GUI.backgroundColor = new Color(1f, 0.4f, 0.4f);
                    if (GUILayout.Button("删除", GUILayout.Width(50)))
                    {
                        Undo.RegisterCompleteObjectUndo(pathFollower, "Delete Path Node");
                        pathFollower.DeleteNode(i / 3);
                        if (_selectedIndex == i) _selectedIndex = -1;
                        if (_selectedSegmentIndex >= (pathFollower.controlPoints.Length - 1) / 3) _selectedSegmentIndex = -1;
                        EditorUtility.SetDirty(pathFollower);
                        SceneView.RepaintAll();
                        GUIUtility.ExitGUI(); // 打断当前 GUI 渲染管线，防止数组即时变小引发的越界渲染报错
                    }
                }

                GUI.backgroundColor = Color.white;
                EditorGUILayout.EndHorizontal();
            }

            // 绘制当前选中的特定路径段的局部横截面独立属性调控面板
            if (_selectedSegmentIndex >= 0 && pathFollower.segmentOffsets != null && _selectedSegmentIndex < pathFollower.segmentOffsets.Length)
            {
                GUILayout.Space(10);
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                EditorGUILayout.LabelField($"局部横截面设置 - {(_selectedSegmentIndex)} 号路径段", EditorStyles.boldLabel);
                var seg = pathFollower.segmentOffsets[_selectedSegmentIndex];

                EditorGUI.BeginChangeCheck();
                seg.offset = EditorGUILayout.Vector4Field("局部偏移范围", seg.offset);

                if (pathFollower.enableInnerVacuum)
                {
                    seg.innerVacuumX = EditorGUILayout.Slider("局部真空百分比 X", seg.innerVacuumX, 0f, 1f);
                    seg.innerVacuumY = EditorGUILayout.Slider("局部真空百分比 Y", seg.innerVacuumY, 0f, 1f);
                }

                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RegisterCompleteObjectUndo(pathFollower, "Change Segment Offset");
                    EditorUtility.SetDirty(pathFollower);
                    SceneView.RepaintAll();
                }
                EditorGUILayout.EndVertical();
            }

            GUILayout.Space(10);

            DrawDefaultInspector();

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