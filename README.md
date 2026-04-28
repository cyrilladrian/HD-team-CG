# HD Team CG Walking Robot

A real-time 3D walking robot animation built with OpenGL/GLUT and Eigen. The robot is constructed from a hierarchical scene graph of boxes, animated with a procedural walking cycle, and features two distinctive accessories: a spinning antenna and a bouncing heart emblem.

---

## Build

**Dependencies:** CMake 3.10+, C++17 compiler, Eigen3, OpenGL, GLUT/FreeGLUT

```bash
cmake -B build
cmake --build build --target walking_robot
./build/walking_robot
```

---

## What Is Implemented

### 1. Scene Graph (`ModelNode`)

The entire robot is represented as a tree of `ModelNode` objects. Each node stores:

| Field | Type | Purpose |
|---|---|---|
| `name` | `string` | Identifies the part; drives color selection |
| `position` | `Vector3f` | Local translation relative to parent |
| `rotation` | `Vector3f` | Euler angles in degrees — applied Z → Y → X |
| `scale` | `Vector3f` | Per-axis scale applied to a unit cube |
| `children` | `vector<unique_ptr<ModelNode>>` | Owned child nodes |

`addChild(name, pos, scale, rot)` creates a child and returns a raw pointer to it for later animation access. Ownership stays in the tree via `unique_ptr`.

### 2. Robot Body Hierarchy

```
Pelvis  (root, walks along +Z)
├── Torso
│   ├── HeartPivot              ← bouncing heart accessory
│   │   ├── HeartLobeL
│   │   ├── HeartLobeR
│   │   └── HeartTip            (rotated 45° to form the bottom point)
│   ├── Neck
│   │   └── Head
│   │       ├── LeftEye
│   │       ├── RightEye
│   │       └── AntennaStick    ← spinning antenna accessory
│   │           └── AntennaBall
│   ├── LeftUpperArm
│   │   └── LeftForearm
│   └── RightUpperArm
│       └── RightForearm
├── LeftThigh
│   └── LeftShin
│       └── LeftFoot
└── RightThigh
    └── RightShin
        └── RightFoot
```

Every body part is a scaled `glutSolidCube(1.0)`. No mesh loading — the geometry is entirely procedural.

### 3. Colors (`setNodeColor`)

Colors are assigned by node name each frame before rendering the cube:

- **Heart nodes** (any name containing `"Heart"`) → vivid red `(0.90, 0.15, 0.18)`
- **`AntennaBall`** → gold `(1.00, 0.82, 0.10)`
- **`AntennaStick`** → silver `(0.75, 0.75, 0.82)`
- **Everything else** → hash of the name string mapped to a mid-range RGB value, so each body part gets a consistent, deterministic color without manual assignment

### 4. Renderer (`renderNode`)

`renderNode` is a recursive function that walks the scene graph depth-first using the OpenGL matrix stack:

```
glPushMatrix()
  glTranslatef(position)         ← move to local origin
  glRotatef(rotation.z, Z)       ← apply Z rotation
  glRotatef(rotation.y, Y)       ← apply Y rotation
  glRotatef(rotation.x, X)       ← apply X rotation
  glPushMatrix()
    glScalef(scale)              ← scale the cube
    setNodeColor(name)
    glutSolidCube(1.0)           ← draw
  glPopMatrix()
  for each child → renderNode()  ← recurse with transforms inherited
glPopMatrix()
```

The double push/pop pattern is key: the scale is isolated in the inner pair so it does not accumulate into child transforms. Children only inherit position and rotation from their parent.

### 5. Walking Animation (`updatePose`)

All animation is driven by a single time variable `t` (seconds). A phase angle is derived from it:

```cpp
float phase = 2.0f * PI * t / STRIDE_PERIOD;   // STRIDE_PERIOD = 1.0s
```

| Motion | Formula | Constants |
|---|---|---|
| Forward travel | `pelvis.z = WALK_SPEED * t` | `WALK_SPEED = 0.6` |
| Body bob | `pelvis.y = 0.80 + BODY_BOB * abs(sin(phase))` | `BODY_BOB = 0.04` |
| Torso rock | `torso.rotZ = BODY_ROCK * sin(2 * phase)` | `BODY_ROCK = 4°` |
| Left hip | `leftThigh.rotX = HIP_SWING * sin(phase)` | `HIP_SWING = 22°` |
| Right hip | `rightThigh.rotX = HIP_SWING * sin(phase + π)` | counter-phase |
| Knee bend | `shin.rotX = KNEE_BEND * max(0, sin(phase + π/4))` | `KNEE_BEND = 18°` — half-wave rectified so knees only bend, never hyperextend |
| Ankle | `foot.rotX = -ANKLE_COMP * sin(phase)` | `ANKLE_COMP = 10°` — keeps foot roughly level |
| Arm swing | `upperArm.rotX = -ARM_SWING * sin(phase)` (left), `+π` (right) | `ARM_SWING = 28°` — opposite leg counter-phase |
| **Antenna spin** | `antenna.rotY = fmod(255 * t, 360)` | 255°/s ≈ 1.4 s/revolution |
| **Heart bounce** | `heart.y = 0.08 + 0.010 * abs(sin(2π * 1.8 * t))` | 1.8 Hz independent of stride |

The right leg and left leg are always exactly half a stride period out of phase (`+ π`). The knee uses `max(0, ...)` (half-wave rectification) so it only bends during the swing phase and stays straight during stance.

### 6. Accessories

#### Spinning Antenna
- **Structure:** `AntennaStick` (thin silver box) attached to the top center of the `Head` node, with `AntennaBall` (gold cube) at its tip.
- **Animation:** `AntennaStick.rotY` increases linearly with time, giving one full revolution every ~1.4 seconds. Because the antenna is a child of `Head` (which is deep in the hierarchy), it naturally inherits body bob, torso rock, and neck position.

#### Bouncing Heart
- **Structure:** `HeartPivot` (near-zero-scale invisible anchor) on the front face of the `Torso`, with three children:
  - `HeartLobeL` and `HeartLobeR` — small cubes slightly offset left/right and upward for the top lobes of the heart
  - `HeartTip` — a cube rotated 45° around Z to form a diamond, positioned below the lobes to create the pointed bottom
- **Animation:** `HeartPivot.y` oscillates at 1.8 Hz using `abs(sin(...))`, which produces a double-peak per period mimicking a lub-dub heartbeat. The animation is fully independent of the stride cycle.
- **Color:** Any node whose name contains `"Heart"` renders red, so all three children are automatically colored.

### 7. Display Loop

| Setting | Value |
|---|---|
| Window | 1280 × 720 |
| Frame rate | 30 FPS via `glutTimerFunc` |
| Duration | 15 seconds, then freezes on last frame |
| Background | Dark blue-grey `(0.15, 0.15, 0.20)` |
| Ground plane | Grey quad from z = −2 to z = 50, rendered without lighting |
| Camera | Perspective 45°, tracks robot: eye at `(3.5, 2.5, pz+5)`, looks at `(0, 1.2, pz)` |
| Light | Single positional light at `(3, 6, 5)` — white diffuse, 0.3 grey ambient |

The camera `pz` is read live from `pelvis.position.z` each frame, so it follows the robot forward without any additional logic.

---

## File Structure

```
walking_robot.cpp          main source — scene graph, robot, animation, render loop
wall-e.cpp                 original static export program (unchanged)
CMakeLists.txt             builds both targets; links Eigen3, OpenGL, GLUT
docs/superpowers/plans/    implementation plans
```

---

## Animation Constants Quick Reference

```cpp
static const float STRIDE_PERIOD  = 1.0f;   // seconds per step cycle
static const float HIP_SWING      = 22.0f;  // degrees
static const float KNEE_BEND      = 18.0f;  // degrees (max bend)
static const float ANKLE_COMP     = 10.0f;  // degrees
static const float BODY_BOB       = 0.04f;  // world units (vertical)
static const float BODY_ROCK      = 4.0f;   // degrees (side-to-side)
static const float ARM_SWING      = 28.0f;  // degrees
static const float WALK_SPEED     = 0.6f;   // world units per second
```

Tweak these to change how the walk feels without touching any math.
