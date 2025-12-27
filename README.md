# Manipulator Project - File Placement Guide

This workspace is set up for a 7-DOF manipulator with a 5-finger gripper, tactile sensors, and wrist-mounted depth camera.

## 📁 Where to Place Your Files

### URDF Files
Place your URDF/Xacro files in:
```
src/manipulator_description/urdf/
```

**Expected files:**
- Main robot URDF/Xacro (e.g., `manipulator.urdf.xacro`, `robot.urdf`, etc.)
- Any modular Xacro files for arm, gripper, sensors, etc.

### Mesh Files
Place your mesh files (.stl, .dae, .obj) in:
```
src/manipulator_description/meshes/
```

**Organized structure:**
- `meshes/arm/` - Manipulator arm meshes
- `meshes/gripper/` - Gripper and finger meshes
- `meshes/table/` - Table/base platform meshes

You can also organize with visual/ and collision/ subdirectories if needed.

---

## 📦 Package Structure

### manipulator_description
**Purpose:** Robot URDF/Xacro definition, meshes, and visualization

**Directories:**
- `urdf/` - URDF/Xacro robot description files
- `meshes/` - Visual and collision mesh files
- `config/` - Joint limits and other configurations
- `launch/` - RViz visualization launch files
- `rviz/` - RViz configuration files

---

### manipulator_gazebo
**Purpose:** Gazebo Ignition simulation

**Directories:**
- `worlds/` - Gazebo world files (.sdf)
- `config/` - Gazebo-ROS bridge configurations
- `launch/` - Simulation launch files

---

### manipulator_control
**Purpose:** ros2_control configuration

**Directories:**
- `config/` - Controller configuration files (.yaml)
- `launch/` - Controller spawning launch files

---

### manipulator_bringup
**Purpose:** High-level launch files

**Directories:**
- `launch/` - Complete system launch files



---

## 🔧 Build Command

Once files are in place, build with:
```bash
cd /home/parag/Hand_ws
colcon build --symlink-install
source install/setup.bash
```
