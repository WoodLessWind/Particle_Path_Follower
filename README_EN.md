# Particle Path Follower

A path-following component for the Unity Particle System. With a visual Bézier curve editor, it allows particles to move smoothly along custom 3D paths, while supporting rich displacement scattering and dynamic alignment controls.

## ✨ Core Features

- **Visual Path Editing**: Manipulate Bézier curves directly in the Scene view. Supports dragging, snapping, and symmetric control point tracking.
- **Dynamic Lifetime Matching**: Automatically calculates and assigns appropriate particle lifetimes based on the total path length and the specified speed.
- **Smart Tangent Alignment**: Particles' orientations and velocity vectors automatically align to the tangent direction of the current path during movement.
- **Advanced Displacement Control**:
  - Various offset modes: **Random**, **Repeat**, and **PingPong**.
  - Non-uniform scattering ranges (independent control over horizontal and vertical randomized minimum/maximum limits).
  - **Inner Vacuum** mechanism for easily creating hollow cylindrical or pipe-like particle effects.
  - **Circular Shape** constraint to force the scattering into circles or ellipses.

## 🛠️ Usage Guide

1. **Attach the Script**
   Attach the `ParticlePathFollower` script to a GameObject with a `Particle System` component (if none exists, it will be automatically added).
2. **Scene Visual Editing**
   - Once the object is selected, you can see nodes in the Scene view: green nodes (path points) and dark green nodes (control handles).
   - **Select Node**: Click a node to select it, then drag the axes to move it.
   - **Snap Node**: Hold down the **`Shift` key** while dragging to snap the node to other path points.
   - **Restore Smooth Handles**: If the control handles of a path point lose their symmetrical alignment making the curve unsmooth, select one of the handles and **Shift + Click** (or move it slightly with Shift pressed) to force the opposite handle back into a symmetrical position.
3. **Inspector Operations**
   - **Add Path Node**: Click the "添加路径节点" (Add Segment) button at the bottom of the Inspector panel to append a new segment to the end of the path.
   - **Auto Smooth**: Click the "自动计算平滑" (Auto Smooth) button to automatically calculate and apply smooth transitions to all path connecting points.

## ⚙️ Inspector Properties

### Movement Settings
- **Speed**: The base speed at which particles move along the path.
- **Auto Set Lifetime**: When enabled, the script dynamically overrides the particle system's start lifetime (`Total Length / Speed`), ensuring the particle exactly reaches the end of the path when its life ends.
- **Align To Path**: When enabled, the particle's render orientation (rotation) and actual moving direction (velocity) will constantly align with the current motion path tangent.

### Offset Settings
- **Offset Mode**: Determines how particles deviate from the main path.
  - `None`: No offset; strictly follows the center line.
  - `Random`: Gives particles a random birth offset, forming a 3D volumetric trajectory.
  - `Repeat`: Offsets particles in a repeating or helical pattern based on birth time.
  - `PingPong`: Offsets particles in an oscillating wave pattern based on time.
- **Offset**: The displacement ranges.
  - `X / Y`: Represent the minimum/maximum offsets in the **horizontal** direction.
  - `Z / W`: Represent the minimum/maximum offsets in the **vertical** direction.
- **Enable Inner Vacuum**: 
  - When enabled, removes particle distribution from a central "hollow" area to create hollow pipes and rings.
  - **Inner Vacuum X / Y** controls the percentage boundary (0.0 ~ 1.0) of the hollowed-out area in the horizontal and vertical domains respectively.
- **Circular Shape**: By default, the offset is based on a rectangular range. Enabling this forces the scattering constraint into a circular or elliptical shape.
- **Offset Frequency**: Active in `Repeat` and `PingPong` modes, controlling the rhythm/frequency of the offset cycles.
