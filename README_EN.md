# Particle Path Follower

This is a Unity particle path-following extension script. It drives particles attached to a ParticleSystem along multi-segment cubic Bezier paths, and provides a custom editor for directly adjusting path nodes, cross-section offsets, vacuum cutouts, and segment transitions.

## Features

- Supports multi-segment cubic Bezier paths.
- Moves particles by real path length for smoother motion and fewer density spikes on curved sections.
- Supports a speed-over-path curve for varying the speed multiplier across progress.
- Supports three path travel modes: OneShot, Loop, and PingPong.
- Supports four offset modes: None, Random, Repeat, and PingPong.
- Supports circular or elliptical cross-section mapping.
- Supports inner vacuum cutouts to keep particles out of the center area.
- Supports independent editing for the start cross-section and per-segment cross-sections.
- Includes Scene view handles for dragging, snapping, and symmetric smoothing.

## Files

- [ParticlePathFollower.cs](ParticlePathFollower.cs): runtime logic for particle position, orientation, offsets, and path caching.
- [ParticlePathFollowerEditor.cs](ParticlePathFollowerEditor.cs): Unity editor extension for Scene view handles and the Inspector UI.

## Installation

1. Place both scripts anywhere in your Unity project's script folders.
2. Create or select a particle object in the scene and make sure it has a ParticleSystem component.
3. Add the ParticlePathFollower component to that object.
4. Place ParticlePathFollowerEditor.cs inside an Editor folder in your Unity project to enable the custom Scene view handles and Inspector UI.

## Basic Usage

1. Select the object with ParticlePathFollower attached.
2. Edit the path nodes in the Inspector. Every 3 points define one cubic Bezier segment.
3. Use Add Segment to extend the path and Auto Smooth to generate smoother connections.
4. Drag anchors and control handles in the Scene view to shape the curve.
5. Select a segment or the start section to edit its local offset range and vacuum settings.

## Parameters

### Motion Settings

- `speed`: movement speed along the path.
- `pathTravelMode`: how the particle advances along the path.
- `speedOverPath`: speed multiplier curve over normalized progress.
- `includeCurveInLifetime`: whether the lifetime should be affected by the speed curve.
- `speedSampleCount`: sample count used for speed-curve integration.
- `autoSetLifetime`: whether to auto-adjust particle lifetime from path length and speed.
- `alignToPath`: whether particles should face the path forward direction.

### Offset Settings

- `offsetMode`: cross-section offset mode.
- `enableInnerVacuum`: enables an inner vacuum area.
- `circularShape`: maps the offset range to a circular or elliptical shape.
- `offsetFrequency`: frequency used by Repeat and PingPong modes.

### Cross-Section Data

- `startOffsetData`: cross-section data for the path start.
- `applyInitialOffsetToWholePath`: makes the whole path use the start cross-section.
- `segmentOffsets`: array of per-segment cross-section data.

## Editor Actions

- Click anchors or tangents to select them.
- Drag the position handle after selection to move points.
- Hold Shift to snap or preserve smooth symmetric relationships.
- Select a segment center handle to edit that segment's local cross section.
- Select the start handle to edit the start cross section and start offset.
- Press Delete or Backspace to remove the currently selected node.

## Runtime Overview

The script precomputes path length and tangent caches, maps each particle's age to a distance along the path, and then applies cross-section offsets perpendicular to the path tangent. The goal is stable motion with smoother behavior on complex curves.

## Notes

- At least 4 control points are required, and paths are typically extended in groups of 3 points per new segment.
- If `alignToPath` is disabled, particles still move along the path but their facing direction and velocity vector are not updated.
- When `autoSetLifetime` is enabled, the script will modify ParticleSystem startLifetime at runtime.
- The script is primarily intended for 2D or planar path setups, and uses `Vector3.forward` as the basis for orientation calculations.

## Example Workflow

1. Create a ParticleSystem.
2. Add ParticlePathFollower.
3. Edit the control points to build a path.
4. Adjust `offsetMode` and cross-section values to control how particles spread around the path.
5. Enable `autoSetLifetime` and `alignToPath` to keep the particle motion stable.

## License

This project is licensed under the [MIT License](LICENSE).
