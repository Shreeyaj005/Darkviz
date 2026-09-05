# Darkviz Simulation

This directory contains the ROS 2 and Gazebo simulation environment for the Darkviz project.

## Required Environment

**Use these exact versions. Do not use a different ROS 2 or Gazebo version unless you know what you are doing.**

| Software             | Required Version     |
| -------------------- | -------------------- |
| **Operating System** | Ubuntu **24.04 LTS** |
| **ROS 2**            | **Humble**    |
| **Gazebo**           | **Harmonic**         |
| **Gazebo Sim**       | **8.x**              |
| **Python**           | Python 3.x           |

ROS 2 Jazzy is officially paired with Gazebo Harmonic. Gazebo Harmonic uses the Gazebo Sim 8.x series.

> **Important:** Do **not** install Gazebo Classic 11 for this project. This project uses the newer **Gazebo Sim / Harmonic** ecosystem and the `ros_gz` integration packages.

### Check Your Versions

Before running the simulation, verify your installation:

```bash
# Check Ubuntu
lsb_release -a

# Check ROS 2
echo $ROS_DISTRO

# Check Gazebo
gz sim --version
```

You should see:

```text
Ubuntu: 24.04
ROS_DISTRO: humble
Gazebo: Harmonic / Gazebo Sim 8.x
```

If `echo $ROS_DISTRO` does not return:

```text
humble
```

**stop here and install ROS 2 humble.**

---

## Required ROS 2 Packages

After installing ROS 2 Humble, install the packages required by the simulation:

```bash
sudo apt update

sudo apt install \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-slam-toolbox
```

The `ros_gz` packages provide the integration between ROS 2 and Gazebo, including spawning models and bridging Gazebo topics to ROS 2.

---

## Getting the Simulation

Clone the repository:

```bash
git clone https://github.com/Shreeyaj005/Darkviz.git
cd Darkviz/simulation
```

The repository already contains the project-specific simulation files:

```text
kiwi_robot/
├── config/
├── launch/
├── scripts/
├── urdf/
└── worlds/
```

You **do not** need to separately download the robot model, URDF, Gazebo world, launch files, or controller scripts.

---

## Build

From the `simulation` directory:

```bash
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
```

Then:

```bash
source install/setup.bash
```

---

## Run

Launch the Gazebo simulation:

```bash
ros2 launch kiwi_robot gazebo.launch.py
```

In another terminal, source the environments again:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Darkviz/simulation/install/setup.bash
```

Then launch SLAM:

```bash
ros2 launch kiwi_robot slam.launch.py
```

---

Use:

**Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**

This is the expected development environment.

---

## Generated Files

The following directories are generated locally by `colcon`:

```text
build/
install/
log/
```

They do not need to be downloaded or copied between machines.

If they already exist and you want a completely clean build:

```bash
rm -rf build install log
colcon build --symlink-install
```
