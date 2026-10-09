# Firefighter Robot — Setup Guide
**Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · ESP32**

Two setups are covered:

- **[Simulation](#simulation-setup-laptoppc--wsl2)** — Gazebo + RViz on a Windows PC, inside WSL2.
- **[Real hardware](#real-hardware-setup-raspberry-pi-4)** — the Raspberry Pi 4 on the robot.

Day-to-day operation (launch arguments, mapping, the dashboard) is in
[`docs/runbook.md`](docs/runbook.md) and
[`src/firefighter_bringup/README.md`](src/firefighter_bringup/README.md); this
file only gets a machine to the point where those work.

---

## Simulation Setup (Laptop/PC — WSL2)

**Use WSL2, not a VMware/VirtualBox VM.** Gazebo Harmonic renders with Ogre2,
which flickers or crawls on a VM's virtual GPU. WSL2 passes the real GPU through
(WSLg), so Gazebo and RViz run at near-native speed with no workarounds.

### 1. Install WSL2 and Ubuntu 24.04

In an **administrator PowerShell** on Windows:

```powershell
wsl --install -d Ubuntu-24.04
```

Reboot if asked, then open **Ubuntu** from the Start menu. Everything below runs
inside that Ubuntu shell.

### 2. Install ROS 2 Jazzy Desktop and Gazebo

```bash
sudo apt update && sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-ros-gz cmake build-essential
```

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Clone the workspace

Clone **inside the Linux filesystem** (`~/`), not under `/mnt/c/`:

```bash
cd ~
git clone https://github.com/thechaelice/firefighter_ws.git
```

> **Do not copy or build the Windows checkout from `/mnt/c/...`.** Builds there
> are several times slower, and a Windows checkout can carry CRLF line endings
> that break `build.sh` (`$'\r': command not found`). A fresh clone in `~/` avoids
> both.

The launch files look for `config/` and `maps/` in `~/firefighter_ws`. If you
clone somewhere else, point them at it:

```bash
echo "export FIREFIGHTER_WS=/path/to/firefighter_ws" >> ~/.bashrc
```

### 4. Install workspace dependencies

```bash
cd ~/firefighter_ws
sudo rosdep init        # once per machine; "already initialized" is fine
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

That pulls in SLAM Toolbox, Nav2, `robot_localization`, `joint_state_publisher`,
`xacro` and the Gazebo bridge. If `rosdep` skips something, install the set by
hand:

```bash
sudo apt install -y \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization ros-jazzy-joint-state-publisher ros-jazzy-xacro \
  ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard
```

### 5. Build

`build.sh` defaults to one compiler job (it is tuned for the Pi). On a PC, raise it:

```bash
cd ~/firefighter_ws
JOBS=4 ./build.sh
source install/setup.bash
```

```bash
echo "source ~/firefighter_ws/install/setup.bash" >> ~/.bashrc
```

### 6. Run the simulation

```bash
ros2 launch firefighter_bringup sim.launch.py
```

This opens Gazebo with the `firehouse` world (three rooms, obstacles, a fire in
the far room), spawns the robot in the west room, and starts the same stack the
real robot runs — `rf2o`, the EKF, SLAM Toolbox, Nav2, the mission FSM — plus RViz.

| Variant | Command |
|---|---|
| Mapping only (no Nav2) | `ros2 launch firefighter_bringup sim.launch.py nav2_enabled:=false` |
| No RViz | `ros2 launch firefighter_bringup sim.launch.py rviz:=false` |
| No Gazebo window (lighter; RViz is the view) | `ros2 launch firefighter_bringup sim.launch.py gui:=false` |
| Different spawn pose | `ros2 launch firefighter_bringup sim.launch.py x:=2.5 y:=0.5 yaw:=1.57` |
| Different world | `ros2 launch firefighter_bringup sim.launch.py world:=/path/to/world.sdf` |

Drive it from a second terminal (click that terminal first so it has focus):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The bundled RViz config already has the robot model, `/scan` and `/map` displays
with **Fixed Frame** `map`; until SLAM starts publishing `map`, switch it to `odom`.

**What is and is not simulated**

| Real robot | In simulation |
|---|---|
| RPLIDAR A1 → `/scan` | Gazebo `gpu_lidar` → `/scan` |
| ESP32 bridge → `/wheel_odom`, takes `/cmd_vel` | Gazebo diff-drive → `/wheel_odom`, takes `/cmd_vel` |
| — | Gazebo → `/joint_states` (wheels turn in RViz) |
| MLX90641 reader → `/thermal/image`, `/flame_event` | Gazebo thermal camera → `/thermal/raw` → the same perception node |
| ESP-NOW beacons → `/beacon_event` | **not simulated** |

### 7. View the model only (no Gazebo)

```bash
ros2 launch firefighter_description description.launch.py
```

```bash
rviz2 -d $(ros2 pkg prefix firefighter_bringup)/share/firefighter_bringup/rviz/firefighter.rviz
```

Set **Fixed Frame** to `base_link`.

### 8. Verify Gazebo (optional)

```bash
gz sim -r shapes.sdf
```

### 9. Enable GPU rendering (optional)

WSL sometimes falls back to CPU rendering (`llvmpipe`), which makes Gazebo slow.

```bash
sudo apt install -y mesa-utils
glxinfo -B | grep "OpenGL renderer"
```

If that prints `llvmpipe`, force the Direct3D 12 driver. The last line names the
GPU vendor — change `NVIDIA` to `AMD` or `Intel` to match yours:

```bash
echo "export LIBGL_ALWAYS_SOFTWARE=false" >> ~/.bashrc
echo "export GALLIUM_DRIVER=d3d12" >> ~/.bashrc
echo "export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA" >> ~/.bashrc
source ~/.bashrc
glxinfo -B | grep "OpenGL renderer"     # should now show D3D12 (<your GPU>)
```

### 10. Run the unit tests (optional)

The FSM, protocol codec and flame-detection tests are pure Python and need no
running ROS graph:

```bash
cd ~/firefighter_ws
python3 -m pytest tests
```

---

## Real Hardware Setup (Raspberry Pi 4)

### Part 1: Flash Ubuntu Server 24.04

With **Raspberry Pi Imager**: choose *Raspberry Pi 4* → *Other general-purpose
OS* → *Ubuntu* → **Ubuntu Server 24.04 LTS (64-bit)**. In *Edit Settings* set a
hostname, username and password, add Wi-Fi if needed, and enable **SSH** on the
Services tab. First boot takes several minutes.

```bash
ssh your-username@<hostname>.local     # or the Pi's IP address
```

Ubuntu Server does not ship an mDNS responder, so `<hostname>.local` only works
after:

```bash
sudo apt install -y avahi-daemon libnss-mdns
```

### Part 2: Install ROS 2 Jazzy (base)

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release ca-certificates software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools cmake build-essential i2c-tools
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

> **Do not install Gazebo (`ros-jazzy-ros-gz`) on the Pi.** It is only needed for
> simulation. `firefighter_bringup` lists it as a dependency for `sim.launch.py`,
> so on the Pi tell `rosdep` to skip it (Part 5).

> **"Unmet dependencies / held broken packages"** during the ROS install usually
> means Ubuntu's security repo upgraded `liblz4-1`, `libzstd1` or `zlib1g` past
> the version the ROS `-dev` packages pin. Check that `noble-updates` is in
> `/etc/apt/sources.list.d/ubuntu.sources`, run `sudo apt --fix-broken install`,
> and retry.

### Part 3: Harden the Pi against build lockups

A plain parallel `colcon build` can wedge this Pi's SD/MMC bus and take Wi-Fi
down with it (the full story is in the [README](README.md#1-build-the-workspace)).
`build.sh` avoids triggering it; these make the board robust regardless:

```bash
sudo apt install -y zram-tools earlyoom
# set ALGO=zstd and PERCENT=50 in /etc/default/zramswap, then:
sudo systemctl restart zramswap
```

Prefer Ethernet over Wi-Fi, and a USB SSD over the SD card, if you can.

### Part 4: Enable the buses

The Pi talks to three devices:

| Device | Bus | Appears as |
|---|---|---|
| RPLIDAR A1 | USB serial adapter | `/dev/ttyUSB0` |
| Mobility ESP32 | UART on GPIO14 (TXD) / GPIO15 (RXD), 115200 8N1 | `/dev/ttyAMA0` |
| MLX90641 thermal camera | I²C1 on GPIO2 (SDA) / GPIO3 (SCL), address `0x33` | `/dev/i2c-1` |

Wiring tables: [`firmware/README.md`](firmware/README.md#5-pi--esp32-uart-link-option-a--active)
for the UART, [`docs/thermal-camera.md`](docs/thermal-camera.md) for the camera.

**1. Device-tree settings**

```bash
sudo tee -a /boot/firmware/config.txt <<'EOF'
dtparam=i2c_arm=on
enable_uart=1
dtoverlay=miniuart-bt
EOF
```

`miniuart-bt` moves Bluetooth to the mini-UART so the full PL011 UART
(`/dev/ttyAMA0`) is free on GPIO14/15.

**2. Take the serial console off that UART**

Edit `/boot/firmware/cmdline.txt` and delete the `console=serial0,115200` token
(leave the rest of the line intact), then:

```bash
sudo systemctl disable --now serial-getty@ttyAMA0.service
sudo systemctl mask serial-getty@ttyAMA0.service
```

**3. Permissions**

```bash
sudo usermod -aG dialout,i2c $USER
sudo reboot
```

**4. Verify after the reboot**

```bash
groups                         # includes dialout and i2c
ls -l /dev/ttyUSB0 /dev/ttyAMA0
i2cdetect -y 1                 # 0x33 is the thermal camera
```

### Part 5: Clone, install dependencies, build

```bash
cd ~
git clone https://github.com/thechaelice/firefighter_ws.git
cd ~/firefighter_ws

sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "ros_gz_sim ros_gz_bridge"
```

Build with the wrapper — **never a bare `colcon build` on the Pi**:

```bash
./build.sh
source install/setup.bash
echo "source ~/firefighter_ws/install/setup.bash" >> ~/.bashrc
```

Then build the thermal camera reader (clones the Melexis vendor library):

```bash
bash scripts/thermal/build.sh
```

### Part 6: Flash the ESP32s

The mobility controller and the fire beacons are Arduino sketches under
[`firmware/`](firmware/). Board settings, pinouts and beacon MAC pairing are in
[`firmware/README.md`](firmware/README.md#4-flashing--setup-instructions).

`WHEEL_RADIUS_M` and `ENCODER_TICKS_PER_REV` in `robot_mobility.ino` **must match**
`wheel_radius_m` / `ticks_per_rev` in
[`src/firefighter_bridge/config/bridge.yaml`](src/firefighter_bridge/config/bridge.yaml),
and the wheel radius and track width there must match
[`firefighter.gazebo.xacro`](src/firefighter_description/urdf/firefighter.gazebo.xacro).

### Part 7: Run the robot

Check the pieces one at a time first:

```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0
ros2 topic hz /scan                      # ~7-10 Hz
```

```bash
python3 scripts/motor_test.py --linear 0.10 --duration 2    # robot on blocks!
```

Then the whole stack:

```bash
ros2 launch firefighter_bringup bringup.launch.py rviz:=false
```

The Pi has no display, so leave RViz off there. Watch it from a browser with the
dashboard (`python3 scripts/robot_dashboard.py`, then `http://<pi-ip>:8080/`), or
run RViz on the PC — see below.

### Part 8: RViz on the PC, robot on the Pi

ROS 2 discovers nodes over the LAN by multicast, so a PC on the same network
sees the Pi's topics once both use the same domain ID (default `0`):

```bash
rviz2 -d $(ros2 pkg prefix firefighter_bringup)/share/firefighter_bringup/rviz/firefighter.rviz
```

From **WSL2** this needs mirrored networking, because the default NAT hides WSL
from the LAN. Put this in `C:\Users\<you>\.wslconfig` on Windows, then run
`wsl --shutdown` and reopen Ubuntu:

```ini
[wsl2]
networkingMode=mirrored
```

Phone hotspots and some routers drop multicast between clients; if `ros2 topic
list` on the PC shows nothing from the Pi while `ping <pi-ip>` works, that is why.

---

## Troubleshooting

### Build

| Symptom | Fix |
|---|---|
| `./build.sh: $'\r': command not found` | The checkout has Windows line endings. Clone fresh inside WSL/Linux instead of using a `/mnt/c` copy. |
| `colcon: command not found` | `sudo apt install python3-colcon-common-extensions` (included in `ros-dev-tools`). |
| `package 'slam_toolbox' not found` (or `nav2_bringup`, `robot_localization`, `joint_state_publisher`) at launch | Dependencies not installed — rerun the `rosdep install` step, or the explicit `apt install` list in Simulation §4. |
| Pi drops off SSH during a build | A parallel build wedged the SD/MMC bus. Power-cycle, then only ever use `./build.sh` (Part 3). |
| Launch cannot find `config/nav2_params.yaml` | The workspace is not at `~/firefighter_ws`; set `FIREFIGHTER_WS`. |

### Simulation

| Symptom | Fix |
|---|---|
| Gazebo is very slow | Software rendering — follow Simulation §9. |
| `/scan` is well under 8 Hz, map appears late | The PC cannot simulate in real time, and sensor rates follow simulation time. Check `gz topic -e -n 1 -t /stats` (`real_time_factor` should be near 1). Launch with `gui:=false`, or `nav2_enabled:=false` while mapping, and close other heavy programs. |
| Gazebo window opens but the robot is missing | Check the launch terminal for a `create` error; the robot is spawned from `/robot_description`, so `robot_state_publisher` must be up. |
| Robot appears without meshes | `GZ_SIM_RESOURCE_PATH` was overridden; launch through `sim.launch.py`, which sets it. |
| RViz shows "No transform from [laser] to [map]" for the first seconds | Normal — SLAM and Nav2 start `startup_delay` seconds (8 by default) after Gazebo. Raise it on a slow machine: `startup_delay:=15.0`. |
| Robot ignores teleop | Nav2 or the mission node is also publishing `/cmd_vel`. Launch with `nav2_enabled:=false`. |
| `no thermal frames for 5.x s` once at startup | Harmless — the simulated frame source takes a few seconds to connect. It is only a problem if `/thermal/image` never appears. |
| Thermal image is uniformly cold | The fire is not in view: the camera has a 55° lens and looks straight ahead. Drive into the east room and face the orange cylinder. |
| Everything is frozen at t=0 | `/clock` is not bridged — check `ros2 topic hz /clock`. |

### Robot

| Symptom | Fix |
|---|---|
| No `/dev/ttyUSB0` | Check the LiDAR's USB cable and power; `lsusb` should list a CP210x/CH340 device. |
| `/scan` is silent | Start the motor: `ros2 service call /start_motor std_srvs/srv/Empty "{}"`. |
| `Permission denied: '/dev/ttyAMA0'` or `/dev/ttyUSB0` | Not in `dialout` yet — `sudo usermod -aG dialout $USER`, then log out and back in. |
| Bridge reports the ESP32 link is down | Serial console still owns the UART (Part 4, step 2), TX/RX swapped, or the ESP32 is on UART1 instead of UART2 (GPIO16/17). |
| `i2cdetect -y 1` shows nothing at `0x33` | `dtparam=i2c_arm=on` missing or not rebooted; otherwise check SDA/SCL wiring. |
| Thermal frames are garbage | The sensor is an MLX9064**1**, not an MLX90640 — use the reader in `scripts/thermal/`, see [`docs/thermal-camera.md`](docs/thermal-camera.md). |
| TF tree unstable / `base_link` has two parents | Something else is publishing a transform bringup already owns — see the TF-ownership section of the [bringup README](src/firefighter_bringup/README.md#tf-ownership-rep-105). |
