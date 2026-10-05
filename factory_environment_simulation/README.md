# Factory Environment Simulation

A two-robot factory hazard demonstration for **Gazebo Harmonic**. A black writer
robot maps events with permanent beacon tiles, then a gray executor robot follows
those tiles to extinguish fires and mark victim status.

The factory contains shelves, boxes, two pits, six fires, two stationary victims,
and an entrance gateway. Both robots use the known layout to avoid obstacles.
One command runs the complete scenario and leaves Gazebo open for inspection.

## Quick start

If Gazebo and its Python bindings are already installed, extract or clone the
repository and open a terminal in its root:

```bash
cd "$HOME/Desktop/factory_environment_simulation"
```

```bash
bash run.sh demo
```

Use your actual extraction or clone path if it differs. Close any previous
factory demo before starting a new run. No compilation or package installation
is needed to run this repository after its system dependencies are available.

## Scenario

1. The **black writer** enters, activates the wired blue entrance beacon, visits
   all 15 aisle sections, and drops event and communication beacons. It returns
   outside and stops.
2. The **gray executor** waits for the writer's completed mission and confirmed
   return pose. It follows safe paths to both white victim tiles first, then the
   orange fire tiles, standing above each target to perform its action.
3. The gray robot returns outside. All beacon tiles remain on the floor.

| Beacon | Writer behavior | Gray robot behavior |
|---|---|---|
| Blue | Activate the fixed entrance tile, 1 m inside the doorway; its cable connects to the outside blue gateway box | Avoid it and preserve the cable |
| Orange | Drop one tile when the robot edge is within 1 m of a fire region | Stand above the tile, pause 1 second, throw a white ball into the fire, then extinguish it |
| White, left victim | Drop one tile within 30 cm of the victim footprint | Stand above the same tile, pause 1 second, change it to green: alive |
| White, right victim | Drop one tile within 30 cm of the victim footprint | Stand above the same tile, pause 1 second, change it to red: dead |
| Yellow | Drop a relay only when the robot is more than 10 m from every confirmed interior beacon, counting all colors | Avoid it and leave it in place |

Each tile is **10 × 10 × 3 cm**. The writer pauses for one second at each drop or
entrance activation. The checked default layout uses five yellow relays. The
outside box provides the wired gateway; interior beacon messages reach it through
radio relay links and the entrance cable.

### Simulation approach

Robot movement uses a bounded kinematic animation with actual Gazebo pose and
clock feedback. Service and feedback workers run separately. Event detection uses
known locations and geometric proximity. Victim states are assigned to the two
existing scene models.

Balls follow visible arcs. After impact, the fire's flame, smoke and glow are
removed while its charred objects and collision geometry remain. Green/red changes
are applied to the original victim beacon visual. Communication is a logical
relay model. These mechanisms demonstrate the scenario rather than camera
recognition, autonomous SLAM, thermal dynamics or RF propagation.

## Requirements and versions

| Component | Version / platform | Purpose |
|---|---|---|
| Operating system | Linux Mint **22.3**, 64-bit; Ubuntu **24.04 LTS (Noble)**, amd64, is the matching Gazebo platform | Linux desktop runtime |
| Gazebo | **Harmonic**, Gazebo Sim **8.x**; known working simulator version **8.11.0** | World, renderer, services and feedback |
| Gazebo Python Transport | **13.x** (`python3-gz-transport13`) | `gz.transport13` bindings |
| Gazebo Python Messages | **10.x** (`python3-gz-msgs10`) | `gz.msgs10` message definitions |
| Python | System Python **3.12.x**, at `/usr/bin/python3` | Controllers, planners and offline checks |
| Protocol Buffers | Ubuntu's `python3-protobuf` package; native validation used **4.21.12** | Message serialization |
| Bash | **5.x** | Launcher |
| Graphics | Working desktop display and graphics driver for Gazebo's Ogre 2 renderer | GUI, meshes, flame and smoke |
| ROS 2 — optional | **Jazzy Jalisco** | Optional `ament_cmake` installation and ROS launch |

Linux Mint 22.x uses Ubuntu 24.04 packages. Harmonic includes Gazebo Messages 10,
Transport 13 and SDFormat 14. The launcher uses the system Python interpreter and
selects the Python protobuf parser automatically; install the Gazebo bindings
through APT rather than a separate pip environment.

The standalone launcher uses Gazebo Transport directly. ROS 2, `colcon`, NumPy,
Matplotlib and a ROS–Gazebo bridge are not required for that launch. All referenced
meshes and textures are bundled; the demo does not download models from Fuel.

## Install system dependencies

These commands target **Ubuntu 24.04 / Linux Mint 22.3**. If the working Gazebo
installation and Python bindings are already present, skip this section.
Run each command separately.

### 1. Install repository tools

```bash
sudo apt update
```

```bash
sudo apt install -y curl gnupg git unzip
```

### 2. Enable the Gazebo stable repository

```bash
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
```

The repository line deliberately uses `noble`, the Ubuntu base for both supported
systems, rather than the Linux Mint release codename.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
```

```bash
sudo apt update
```

### 3. Install the simulator and Python bindings

```bash
sudo apt install -y gz-harmonic python3 python3-protobuf python3-gz-transport13 python3-gz-msgs10
```

Gazebo's renderer, physics libraries and SDFormat dependencies are installed by
the `gz-harmonic` package. This command selects the compatible Harmonic library
families; patch versions depend on the package repository.

Check the installed simulator version:

```bash
gz sim --versions
```

```bash
/usr/bin/python3 --version
```

Official references: [Harmonic installation](https://gazebosim.org/docs/harmonic/install_ubuntu/),
[Harmonic library versions](https://gazebosim.org/docs/harmonic/install/),
[Gazebo Transport Python bindings](https://gazebosim.org/api/transport/13/python.html),
and [Linux Mint 22.3 release notes](https://www.linuxmint.com/rel_zena.php).

## Run and validate

From the repository root, run the complete demo:

```bash
bash run.sh demo
```

Or use the default mode:

```bash
bash run.sh
```

The launcher starts both Gazebo and the mission controller. The default motion
budgets are 90 seconds for the writer and at least 60 seconds for the gray robot,
plus drop pauses, ball flights and service overhead. The gray budget can extend
when its safe route requires more time at the 4 m/s speed limit.

After completion, Gazebo remains open with the final scene. Close its window or
press **Ctrl+C** in the launch terminal to stop the demo's processes.

Run the complete offline checks, plus native message/SDF checks when Gazebo is
installed:

```bash
bash run.sh check
```

Run only the offline checks; these need system Python and Bash, without Gazebo:

```bash
bash run.sh check --offline
```

Require native checks on a Gazebo-equipped machine:

```bash
bash run.sh check --native
```

Checks cover the route, aisle coverage, event distances, one-second pauses,
confirmed beacon creation, relay delivery through the cable, robot handoff,
blue/yellow avoidance, six fire actions, existing victim visual IDs, delayed
feedback, paused clocks and movement recovery. Native checks validate the
Gazebo message definitions and the SDF files; they do not run the full GUI mission.

The complete GUI scenario was confirmed working on Linux Mint 22.3 with Gazebo
Sim 8.11.0. Development checks also use Gazebo Sim 8.15.0, Messages 10.4.0 and
Transport 13.6.0 for native message/SDF validation. Optional ROS launch metadata
and install paths are checked structurally; a full ROS build was not run during
packaging. The GitHub workflow runs the
offline suite on Ubuntu 24.04.

### Runtime options

| Option | Default | Meaning |
|---|---|---|
| `--writer-seconds` | `90` | Writer motion budget, excluding event pauses |
| `--seconds` | `60` | Requested gray robot motion budget; extended if needed for its route |
| `--range` | `10` | Interior relay range and yellow drop gap, in metres |
| `--headless` | Off | Run the Gazebo server without the GUI |

Example:

```bash
bash run.sh demo --writer-seconds 90 --seconds 80 --range 10
```

```bash
bash run.sh demo --headless
```

```bash
bash run.sh --help
```

## Configuration and repository layout

| Path | Contents |
|---|---|
| `run.sh` | Single entry point for demo, validation and regeneration |
| `scripts/` | Controllers, shared helpers, route builders and regression checks |
| `worlds/` | Main scenario and source/intermediate worlds used by builders and validation |
| `models/` | Referenced scene meshes, textures and the primitive robot definition |
| `step1/` | Base geometry settings, safe aisle route and grid |
| `step2/` | Writer/event settings, extended route, grid and entrance beacon definition |
| `step3/` | Gray robot settings, scene manifest, fire-off definitions and ball example |
| `launch/`, `package.xml`, `CMakeLists.txt` | Optional ROS 2 resource package |
| `licenses/`, `NOTICE.txt` | Preserved third-party license texts and asset provenance |
| `.github/workflows/validate.yml` | Offline checks on pushes and pull requests |
| `logs/` | Generated runtime output; created automatically and ignored by Git |

The Step 1 and Step 2 directories and shared Python helpers remain because the
final scenario imports them and validates its source worlds. They are required
inputs, not archived versions. Generated logs, bytecode, validation reports,
old previews and unused source model bundles are excluded from the distribution.

Tune movement/geometry in `step1/config.json`, writer events in `step2/config.json`,
and gray actions in `step3/config.json`. Lengths use metres; angles use radians;
colors use normalized RGBA. After editing scene or route settings, regenerate all
stages in order:

```bash
bash run.sh plan
```

```bash
bash run.sh check
```

Precomputed routes and scenes are committed so a normal demo does not need
regeneration. Runtime results and fixture reports are ignored by `.gitignore`.

## Optional ROS 2 Jazzy installation

The standalone instructions above are sufficient. To install this repository
as a ROS 2 resource package, first install and source **ROS 2 Jazzy** using its
[official Ubuntu instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
The Gazebo and Python packages listed above are still required.

Install build tools:

```bash
sudo apt install -y cmake python3-colcon-common-extensions ros-jazzy-ament-cmake ros-jazzy-ament-index-python ros-jazzy-launch
```

Place this repository under `~/Documents/factory_ws/src/factory_environment_simulation`,
then build and source the workspace:

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
cd "$HOME/Documents/factory_ws"
```

```bash
colcon build --symlink-install --packages-select factory_environment_simulation
```

```bash
source install/setup.bash
```

```bash
ros2 launch factory_environment_simulation simulation.launch.py
```

This launches the same complete two-robot demo. The optional CMake installation
includes all three stage data directories and the final runner.

## Logs and troubleshooting

| Output | Information |
|---|---|
| `logs/step2_result.json` | Writer completion, aisle coverage and beacon records |
| `logs/step2_events.json` | Actual deployed beacon positions |
| `logs/gateway_inbox.json` | Event information delivered through the entrance cable |
| `logs/step3_route.json` | Gray route built from the current writer mission |
| `logs/step3_actions.json` | Fire actions and victim color changes |
| `logs/step3_result.json` | Overall mission result |
| `logs/gazebo-step3.log` | Simulator output |
| `logs/step2_*worker.log`, `logs/step3_*worker.log` | Transport worker diagnostics |

- **`gz` is missing or native imports fail:** install the exact Harmonic Python
  package families listed above, then use `bash run.sh check --native`.
- **The robot waits for pose/clock feedback:** ensure simulation playback is
  enabled. Resume with Gazebo's Play button.
- **A source/configuration hash check fails:** use `bash run.sh plan`, then check
  again. Keep the complete `models`, `worlds` and stage data directories together.
- **A mission stops:** inspect its result JSON and worker/Gazebo logs before
  restarting. The launcher only cleans up the processes it started.

## Licenses and provenance

Third-party warehouse assets retain their **BSD-3-Clause** notices. The Gazebo
victim source asset and existing navigation additions retain their
**Apache-2.0** notices. Full texts are in `licenses/`; source URLs, commit IDs and
adaptations are recorded in [NOTICE.txt](NOTICE.txt).
