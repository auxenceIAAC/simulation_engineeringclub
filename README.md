# 2D Navigation Simulator

Autonomous boat simulator for NODE Engineering Club.
**Goal:** test GPS waypoint navigation algorithms without physical hardware.

No Gazebo — pure Python simulator running at 50Hz, displayed in RViz2.

---

## What is this project?

The **Asket EC** is a real autonomous catamaran built by the club. Before testing
navigation algorithms on the physical boat, we simulate them here.

The simulation includes the boat's physics, a camera sensor that detects buoy
gates, and a navigator that steers the boat through each gate automatically.
Everything runs in Python — no heavy simulation engine needed.

---

## Repository structure

```
simulation_engineeringclub/
├── asket_ec_sim_ws/              ← main ROS2 workspace  ✅ USE THIS
│   └── src/
│       ├── asket_ec_sim2d/       ← 2D simulator          ✅ ACTIVE
│       ├── asket_ec_control/     ← differential drive    ⚠️  legacy, unused
│       ├── asket_ec_description/ ← 3D model for Gazebo   ❌ NOT WORKING
│       └── asket_ec_gazebo/      ← Gazebo simulation     ❌ NOT WORKING
└── README.md
```

### Why are `asket_ec_gazebo` and `asket_ec_description` not working?

We originally planned to use **Gazebo Harmonic** (a 3D robotics simulator) to
visualise the boat. It works well on a dedicated Linux machine with a decent GPU,
but most club members run **WSL2 on Windows** with Intel integrated graphics —
and Gazebo Harmonic crashes or runs at 1 FPS in that environment.

We replaced it with `asket_ec_sim2d`: a lightweight 2D simulator written in pure
Python that runs at 50Hz on any machine and displays in RViz2. The Gazebo packages
are kept in the repo for reference and potential future use.

### What is inside `asket_ec_sim2d`?

```
asket_ec_sim2d/
├── asket_ec_sim2d/
│   ├── simulator_node.py           # boat physics at 50Hz
│   ├── buoy_simulator_node.py      # camera sensor — detects buoys and gates
│   ├── waypoint_navigator_node.py  # navigation brain — steers through gates
│   └── keyboard_teleop_node.py     # keyboard teleoperation — MANUAL/AUTO mode
├── config/
│   ├── waypoints.yaml              # GPS target points (fallback)
│   ├── buoys.yaml                  # gate definitions (red + green buoy pairs)
│   └── sim2d.rviz                  # RViz2 display configuration
├── launch/
│   └── sim2d.launch.py
├── setup.cfg
└── package.xml
```

---

## Prerequisites

| Requirement | Version | Notes |
|-------------|---------|-------|
| Ubuntu | 24.04 LTS | or WSL2 on Windows — [WSL2 install guide](https://learn.microsoft.com/en-us/windows/wsl/install) |
| ROS2 | Jazzy Jalisco | [ROS2 Jazzy install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) |
| Python | 3.12 | included with Ubuntu 24.04, nothing to install |

> **New to ROS2?** Think of ROS2 as a messaging system that lets separate
> programs (called *nodes*) talk to each other. One node simulates physics,
> another simulates the camera, another decides where to go — they all communicate
> through named *topics*.

---

## Step-by-step setup (for beginners)

### Step 1 — Clone the repository

**Cloning** means downloading a copy of all the code from GitHub onto your
computer. You only do this once.

```bash
git clone https://github.com/NODE-Engineering-Club/simulation_engineeringclub
cd simulation_engineeringclub
```

After this, a folder called `simulation_engineeringclub` exists on your machine
with all the project files inside.

### Step 2 — Build the ROS2 workspace

**Building** means compiling and registering the code so that ROS2 knows where
to find it. You need to do this once after cloning, and again whenever you modify
a `.py` file in the `asket_ec_sim2d` package.

```bash
cd asket_ec_sim_ws
colcon build --packages-select asket_ec_sim2d
source install/setup.bash
```

- `colcon build` — compiles the package
- `--packages-select asket_ec_sim2d` — only build the 2D simulator (faster)
- `source install/setup.bash` — tells your terminal where the built files are

> If you see warnings in yellow, that is usually fine. Errors in red need fixing.

### Step 3 — Install the Python nodes

This step makes all three simulator programs runnable as commands from any
terminal — not just from inside the project folder.

```bash
cd src/asket_ec_sim2d
pip install -e . --break-system-packages
cd ../../..
```

- `pip install -e .` — installs the package in *editable* mode, meaning changes
  you make to the `.py` files take effect immediately without reinstalling
- `--break-system-packages` — required on Ubuntu 24.04 to install into the
  system Python (safe in this context)

---

## Running the simulation

The simulation needs **3 separate terminal windows** running at the same time.
Open each terminal and run these commands in order.

> **Every time you open a new terminal**, you must source ROS2 first:
> ```bash
> source /opt/ros/jazzy/setup.bash
> ```
> This loads all ROS2 commands into the terminal session. Without it, commands
> like `rviz2` or `ros2` will not be found.

---

### Terminal 1 — Physics simulator

```bash
source /opt/ros/jazzy/setup.bash
~/.local/bin/simulator_node
```

**What it does:** the physics engine. It moves the boat based on commands it
receives, and publishes the boat's position and heading 50 times per second.
Think of it as the boat's hull and motors.

**Expected output:**
```
[INFO] Simulateur 2D Asket EC démarré
```

---

### Terminal 2 — Camera sensor + Navigator

```bash
source /opt/ros/jazzy/setup.bash
~/.local/bin/buoy_simulator_node &
~/.local/bin/waypoint_navigator_node
```

> The `&` runs `buoy_simulator_node` in the background so both nodes share
> the same terminal window.

**`buoy_simulator_node` — the camera sensor:**
reads `config/buoys.yaml`, places red and green buoys on the map, and at 10Hz
checks which buoys are currently in the boat's field of view (15m range, ±60°).
It computes the midpoint of each gate and sends those positions to the navigator.

**Expected output:**
```
[INFO] Buoy simulator started — 3 gates loaded
```

**`waypoint_navigator_node` — the navigation brain:**
decides where the boat should go next. If gate centres are available (from
`buoy_simulator_node`), it steers the boat through each gate in order using pure
pursuit. If no gates are published, it falls back to `config/waypoints.yaml`.

**Expected output:**
```
[INFO] Gate navigation activated — 3 gates received
[INFO] Gate 1 passed ✓
[INFO] Gate 2 passed ✓
...
```

---

### Terminal 3 — Visualisation in RViz2

```bash
source /opt/ros/jazzy/setup.bash
rviz2 -d ~/simulation_engineeringclub/asket_ec_sim_ws/src/asket_ec_sim2d/config/sim2d.rviz
```

**What it does:** opens RViz2 with a pre-configured layout showing:

| Colour / Shape | What you see | ROS2 topic |
|----------------|-------------|------------|
| Green line | Boat's actual trajectory so far | `/sim2d/path` |
| Red line | Planned route (waypoints fallback) | `/waypoints/path` |
| White line | Gate-to-gate navigation route | `/gates/centers` |
| Red spheres | Red buoys (bright = visible, dim = out of range) | `/buoys/all` |
| Green spheres | Green buoys (bright = visible, dim = out of range) | `/buoys/all` |
| Labels | "GATE X - RED/GREEN" above detected buoys | `/buoys/detected` |
| Arrow | Current position and heading of the boat | `/sim2d/pose` |
| Grid | Reference plane (Fixed Frame: `odom`) | — |

**How to read what you see:**
- The **green line** grows over time — it is the history of where the boat has been.
- The **white line** connects all gate centres in order — the planned slalom route.
- **Bright buoys** are currently in the camera's field of view. **Dim buoys** (alpha 0.2) exist in the world but are too far away or behind the boat.
- If nothing appears, check that Terminal 1 and 2 are running and that you sourced ROS2.

---

## Editing gates and waypoints

### Gate definitions (`config/buoys.yaml`)

Each gate has one red buoy (starboard / right side) and one green buoy (port /
left side). The boat navigates to the midpoint between them.

```yaml
gates:
  - id: 1
    red:   {lat: 41.3853, lon: 2.1733}   # keep to the right when passing
    green: {lat: 41.3853, lon: 2.1737}   # keep to the left  when passing
  - id: 2
    ...
```

To change the course, edit the GPS coordinates. `0.0001` degrees ≈ 11 metres.

### Fallback waypoints (`config/waypoints.yaml`)

Used only if `buoy_simulator_node` is not running.

```yaml
waypoints:
  - {lat: 41.3851, lon: 2.1734}
  - {lat: 41.3860, lon: 2.1750}
```

---

## Manual control (instead of Terminal 2)

Skip Terminal 2 and run this to steer the boat yourself:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10
```

| Parameter | Effect | Range |
|-----------|--------|-------|
| `linear.x` | **speed** — positive = forward, negative = backward | −1.5 to +1.5 |
| `angular.z` | **turning** — positive = left, negative = right | −1.0 to +1.0 |

---

## Manual control — keyboard teleoperation

Launch in a new terminal (while the simulator and navigator are already running):

```bash
source /opt/ros/jazzy/setup.bash
~/.local/bin/keyboard_teleop_node
```

### Key mapping

| Key        | Action      | linear.x | angular.z |
|------------|-------------|----------|-----------|
| Z / ↑      | Forward     | 1.5      | 0.0       |
| S / ↓      | Backward    | -1.0     | 0.0       |
| Q / ←      | Turn left   | 0.3      | 1.0       |
| D / →      | Turn right  | 0.3      | -1.0      |
| Space      | Stop        | 0.0      | 0.0       |
| M          | Toggle mode | —        | —         |
| Ctrl+C     | Quit        | —        | —         |

### How it works

- **M** toggles between MANUAL and AUTO mode by publishing on `/manual_mode`.
- In **AUTO** mode the navigator resumes from the **closest gate** to the current
  boat position — not from gate 1. Movement keypresses are ignored in AUTO mode.
- In **MANUAL** mode the navigator stops publishing `/cmd_vel` immediately and
  the keyboard takes over. The terminal prints the mode and last command at every
  keypress:

```
[MANUAL] linear=1.5 angular=0.0
[AUTO]   Navigator active — press M to switch to MANUAL
```

---

## How it works (technical overview)

### Differential drive physics

The Asket EC has two independent motors (left and right). Like a tank, it turns
by spinning one motor faster than the other. The simulator models this with:
- Motor spacing: 0.5 m
- Water drag coefficient: 0.6 (terminal velocity ~2.5 m/s at full thrust)
- Simulation step: 50Hz (every 20ms)

### Camera sensor model

`buoy_simulator_node` simulates what a real camera would see:
- **Range:** 15 m maximum detection distance
- **Field of view:** ±60° relative to the boat heading (120° total)
- **Noise:** Gaussian noise added to measured distance (σ = 0.3 m) and bearing
  (σ = 2°) to simulate real camera imprecision

### Pure pursuit navigation (in plain language)

1. Calculate the angle from the boat's current heading to the target gate centre.
2. If the target is to the left, turn left. If to the right, turn right.
   The further off-course, the sharper the turn.
3. Slow down when turning hard, go fast when pointed directly at the target.

No complex path planning — just "look at the target and steer toward it."

### TF transforms and RViz2

ROS2 uses a system called **TF2** to track the position of every part of a
robot relative to every other part. RViz2 needs a valid TF tree to know where
to draw things. The simulator publishes a transform from the `odom` frame
(world origin) to the `base_link` frame (the boat).

### GPS to local coordinates

The real world uses GPS (latitude/longitude), but physics simulations work in
metres on a flat plane (x/y). The simulator converts between the two using the
Barcelona reference point (41.3851°N, 2.1734°E) as the origin (0, 0).

---

## ROS2 Topics reference

| Topic | Type | Publisher | Subscriber | Description |
|-------|------|-----------|------------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | navigator or manual | `simulator_node` | Speed and turn commands |
| `/sim2d/pose` | `geometry_msgs/PoseStamped` | `simulator_node` | buoy\_simulator, navigator | Boat position and heading |
| `/sim2d/path` | `nav_msgs/Path` | `simulator_node` | RViz2 | Full trajectory history |
| `/sim2d/odom` | `nav_msgs/Odometry` | `simulator_node` | — | Odometry data |
| `/sim2d/navsat` | `sensor_msgs/NavSatFix` | `simulator_node` | — | Simulated GPS (Barcelona origin) |
| `/buoys/all` | `visualization_msgs/MarkerArray` | `buoy_simulator` | RViz2 | All buoys — bright if visible, dim if not |
| `/buoys/detected` | `visualization_msgs/MarkerArray` | `buoy_simulator` | RViz2 | Currently visible buoys with labels |
| `/gates/centers` | `nav_msgs/Path` | `buoy_simulator` | `waypoint_navigator` | Gate midpoints to navigate through |
| `/manual_mode` | `std_msgs/Bool` | `keyboard_teleop` | `waypoint_navigator` | true=MANUAL false=AUTO |
| `/current_mode` | `std_msgs/String` | `waypoint_navigator` | — | Current mode published at 1 Hz |

---

## From simulation to real hardware

The key principle of this simulation is **topic compatibility**. Every simulated
sensor publishes on the exact same ROS2 topic with the exact same message type as
the real sensor will use. This means the navigation algorithms
(`waypoint_navigator_node`) do not need to change at all when moving to real
hardware — only the data source changes.

| Simulated component | Real hardware equivalent |
|---------------------|--------------------------|
| `simulator_node` → `/sim2d/pose` | GPS + IMU driver on the real boat |
| `buoy_simulator_node` → `/buoys/detected` | OpenCV color detection node (real camera feed) |
| `buoy_simulator_node` → `/gates/centers` | Same node — no change needed |
| `waypoint_navigator_node` | Same node — no change needed |
| `/cmd_vel` topic | Motor controller node (converts Twist → PWM signals) |
| `config/waypoints.yaml` | QGroundControl mission upload via MAVLink |

When you run on real hardware, you swap out `simulator_node` for a real GPS/IMU
driver, and `buoy_simulator_node` for a real camera detection node. The navigator
keeps running unchanged because it only cares about topic names and message types,
not where the data comes from.

---

## Roadmap

### Done

- [x] 2D physics simulator (50Hz, differential drive)
- [x] GPS waypoint navigation (pure pursuit)
- [x] RViz2 visualisation (trajectory + waypoints)
- [x] Buoy gate simulation (red + green pairs)
- [x] Camera sensor simulation with field of view and Gaussian noise
- [x] Automatic gate-to-gate navigation

### Next steps — simulation

- [x] Manual / automatic mode switching via ROS2 topic
      → publish `True`/`False` on `/manual_mode` to override the navigator
- [x] Keyboard teleoperation node
      → control the boat with arrow keys in manual mode
- [ ] LIDAR obstacle simulation
      → publish `sensor_msgs/LaserScan` on `/scan` with simulated obstacles
- [ ] Obstacle avoidance algorithm
      → modify `waypoint_navigator` to detour around detected obstacles

### Next steps — real hardware integration

- [ ] Replace `simulator_node` with real GPS + IMU driver
      → same `/sim2d/pose` topic, real data
- [ ] Replace `buoy_simulator_node` with OpenCV color detection node
      → same `/buoys/detected` and `/gates/centers` topics, real camera feed
- [ ] Connect `/cmd_vel` to motor controller
      → ROS2 node that converts Twist messages to PWM signals
- [ ] QGroundControl integration via MAVLink + MAVROS
      → upload waypoint missions from QGC, receive on `/gates/centers`

---

## Contributing

This project is maintained by members of **NODE Engineering Club**.
If you fix a bug or add a feature, open a pull request with a clear description
of what changed and why.

If something is broken or confusing, open an issue — especially if you're new
to ROS2, your question probably helps others too.
