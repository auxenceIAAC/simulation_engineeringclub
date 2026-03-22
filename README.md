# Asket EC — 2D Navigation Simulator

Autonomous boat simulator for NODE Engineering Club.
**Goal:** test GPS waypoint navigation algorithms without physical hardware.

No Gazebo — pure Python simulator running at 50Hz, displayed in RViz2.

---

## What is this project?

The **Asket EC** is a real autonomous catamaran built by the club. Before testing
navigation algorithms on the physical boat, we simulate them here.

This simulator reproduces the boat's physics (propulsion, drag, heading) and
lets you define GPS waypoints the boat should reach automatically. Everything
runs in Python — no heavy simulation engine needed.

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
│   └── waypoint_navigator_node.py  # GPS navigation logic
├── config/
│   ├── waypoints.yaml              # GPS target points (edit this!)
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
> another reads waypoints, another draws the result — they all communicate
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

This step makes the two simulator programs (`simulator_node` and
`waypoint_navigator_node`) runnable as commands from any terminal — not just
from inside the project folder.

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

**What it does:** simulates the boat's position, heading, and speed at 50Hz
using a differential drive model (two motors, like a tank). It listens for
movement commands and publishes the updated state continuously.

**Expected output:**
```
[INFO] Simulateur 2D Asket EC démarré
[INFO] Publication à 50Hz sur /sim2d/pose, /sim2d/path, /sim2d/odom, /sim2d/navsat
```

**Topics published:**
| Topic | Description |
|-------|-------------|
| `/sim2d/pose` | current position and heading of the boat |
| `/sim2d/path` | full trajectory since start (the green trail) |
| `/sim2d/odom` | odometry (speed and displacement) |
| `/sim2d/navsat` | simulated GPS fix, anchored on Barcelona |

**Topic subscribed:**
| Topic | Description |
|-------|-------------|
| `/cmd_vel` | movement commands (speed and turn rate) |

---

### Terminal 2 — Waypoint navigator (automatic mode)

```bash
source /opt/ros/jazzy/setup.bash
~/.local/bin/waypoint_navigator_node
```

**What it does:** reads the GPS waypoints from `config/waypoints.yaml`, sorts
them from closest to furthest, then steers the boat toward each one
automatically by publishing to `/cmd_vel`. It moves to the next waypoint when
the boat is within 2 metres.

**Expected output:**
```
[INFO] Waypoints chargés : 2 points
[INFO] Sorted waypoints: [(41.3851, 2.1734), (41.3860, 2.1750)]
[INFO] Navigation vers waypoint 1/2
```

> **Note:** Terminal 2 and Terminal 1 must both be running. Terminal 2 sends
> commands, Terminal 1 executes the physics.

---

### Terminal 3 — Visualisation in RViz2

```bash
source /opt/ros/jazzy/setup.bash
rviz2 -d ~/simulation_engineeringclub/asket_ec_sim_ws/src/asket_ec_sim2d/config/sim2d.rviz
```

**What it does:** opens RViz2 with a pre-configured layout showing:

| Colour | What you see | ROS2 topic |
|--------|-------------|------------|
| Green line | Boat's actual trajectory so far | `/sim2d/path` |
| Red line | Planned route through all waypoints | `/waypoints/path` |
| Arrow | Current position and heading of the boat | `/sim2d/pose` |
| Grid | Reference plane (Fixed Frame: `odom`) | — |

**How to read what you see:**
- The **green line** grows over time as the boat moves — it is the history of
  where the boat has been.
- The **red line** is the planned route connecting all waypoints in order.
- The **arrow** shows where the boat is right now and which direction it faces.
- If nothing appears, make sure Terminal 1 is running and you sourced ROS2.

---

## Editing waypoints

Waypoints are GPS coordinates the boat navigates toward in order.
Open the file:

```
asket_ec_sim_ws/src/asket_ec_sim2d/config/waypoints.yaml
```

It looks like this:

```yaml
waypoints:
  # Each entry is one GPS target point.
  # lat = latitude  (north/south, between -90 and +90)
  # lon = longitude (east/west,  between -180 and +180)
  #
  # Barcelona reference point: lat 41.3851, lon 2.1734
  # Moving north increases lat. Moving east increases lon.
  # 0.0001 degrees ≈ 11 metres.

  - {lat: 41.3851, lon: 2.1734}   # starting area
  - {lat: 41.3860, lon: 2.1750}   # ~170m north-east
```

**To add a new waypoint**, copy one line and change the coordinates. The
navigator will automatically sort all waypoints from closest to the boat's
starting position to furthest.

> **GPS coordinates in simple terms:**
> Latitude tells you how far north or south you are. Longitude tells you how
> far east or west. Barcelona is at roughly 41.4°N, 2.2°E. A change of 0.0001
> in either coordinate is about 11 metres on the ground.

---

## Manual control (instead of Terminal 2)

If you want to steer the boat yourself instead of using the automatic navigator,
skip Terminal 2 and run this command:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10
```

Edit the two values to control the boat:

| Parameter | Effect | Range |
|-----------|--------|-------|
| `linear.x` | **speed** — positive = forward, negative = backward | −1.0 to +1.0 |
| `angular.z` | **turning** — positive = left, negative = right | −1.0 to +1.0 |

Examples:
```bash
# Go straight forward
"{linear: {x: 0.5}, angular: {z: 0.0}}"

# Turn left while moving
"{linear: {x: 0.3}, angular: {z: 0.6}}"

# Spin in place to the right
"{linear: {x: 0.0}, angular: {z: -1.0}}"

# Stop
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

## How it works (technical overview)

### Differential drive physics

The Asket EC has two independent motors (left and right). Like a tank, it turns
by spinning one motor faster than the other. The simulator models this with:
- Motor spacing: 0.5 m
- Water drag coefficient: 3.0
- Simulation step: 50Hz (every 20ms)

### Pure pursuit navigation (in plain language)

The waypoint navigator uses a simplified *pure pursuit* algorithm:

1. Calculate the angle from the boat's current heading to the target waypoint.
2. If the target is to the left, turn left. If to the right, turn right.
   The further off-course, the sharper the turn.
3. Slow down when turning hard, go fast when pointed directly at the target.

No complex path planning — just "look at the target and steer toward it."

### TF transforms and RViz2

ROS2 uses a system called **TF2** to track the position of every part of a
robot relative to every other part. RViz2 needs a valid TF tree to know where
to draw things. The simulator publishes a simple transform from the `odom` frame
(world origin) to the `base_link` frame (the boat), which is why RViz2 can
display the boat's position correctly.

### GPS to local coordinates

The real world uses GPS (latitude/longitude), but physics simulations work in
metres on a flat plane (x/y). The simulator converts between the two using the
Barcelona reference point (41.3851°N, 2.1734°E) as the origin (0, 0). Every
waypoint is converted to metres from that origin before navigation begins.

---

## Roadmap

- [x] 2D physics simulator (50Hz, differential drive)
- [x] GPS waypoint navigation (pure pursuit)
- [x] RViz2 visualisation (trajectory + waypoints)
- [ ] Manual / automatic mode switching via ROS2 topic
- [ ] QGroundControl integration via MAVLink + MAVROS
- [ ] Wind and current disturbances

---

## Contributing

This project is maintained by members of **NODE Engineering Club**.
If you fix a bug or add a feature, open a pull request with a clear description
of what changed and why.

If something is broken or confusing, open an issue — especially if you're new
to ROS2, your question probably helps others too.
