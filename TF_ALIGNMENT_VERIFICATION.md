# TF Axis Alignment Verification

## Transformation Chain Analysis

### Current Configuration:

**1. wrist_3_link → ee_link:**
```xml
<origin rpy="0 -1.57079632679 -1.57079632679" xyz="0 0 0"/>
```
- Roll: 0°
- Pitch: -90° (-π/2)
- Yaw: -90° (-π/2)

**2. ee_link → right_hand_base_link:**
```xml
<origin xyz="0 0 0" rpy="0 0 0"/>
```
- Roll: 0°
- Pitch: 0°
- Yaw: 0°

### Combined Transformation (wrist_3_link → right_hand_base_link):

Total rotation: `rpy="0 -1.57079632679 -1.57079632679"`

## Axis Alignment Result:

Using ROS RPY convention (ZYX rotation order):

| wrist_3_link Axis | → | right_hand_base_link Axis | Color Mapping |
|-------------------|---|---------------------------|---------------|
| 🔴 **X-axis (RED)** | → | 🟢 **Y-axis (GREEN)** | Red → Green |
| 🟢 **Y-axis (GREEN)** | → | 🔵 **Z-axis (BLUE)** | Green → Blue |
| 🔵 **Z-axis (BLUE)** | → | 🔴 **X-axis (RED)** | Blue → Red |

## What This Means:

**Current alignment with rpy="0 0 0" on gripper mount:**
- When wrist_3's **RED arrow** points forward → gripper's **GREEN arrow** points in that direction
- When wrist_3's **GREEN arrow** points left → gripper's **BLUE arrow** points in that direction  
- When wrist_3's **BLUE arrow** points up → gripper's **RED arrow** points in that direction

## To Make ALL Axes Aligned (Truly Coaxial):

To get **RED→RED, GREEN→GREEN, BLUE→BLUE**, compensate with:
```xml
<origin xyz="0 0 0" rpy="0 1.57079632679 1.57079632679"/>
```

This cancels out the ee_link rotation.

## Verification in RViz:

1. Launch: `ros2 launch manipulator_description display.launch.py`
2. Enable **TF** display in RViz
3. Look at the colored axes near the wrist
4. Compare the arrow colors between `wrist_3_link` and `right_hand_base_link`

**Current Setup:** Axes are rotated 90° relative to each other
**Needed for Coaxial:** All colors should match (parallel arrows same color)
