---
sidebar_position: 3
---

# Chapter 3: Humanoid Models with URDF

In the previous chapters, we explored how different software components communicate using ROS 2 Nodes, Topics, and Services, and how Python agents can integrate with this system. To bring AI to life in robotics, we need to represent the physical robot itself. This chapter introduces **URDF (Unified Robot Description Format)**, the standard way in ROS to describe a robot's physical structure, its links, joints, and their properties.

Understanding URDF is crucial for anyone working with ROS, especially when dealing with robots that have complex mechanical structures like humanoids. URDF allows tools like RViz and Gazebo to visualize and simulate the robot accurately.

## Introduction to URDF

URDF is an XML-based file format used by ROS to describe robot models. It defines:

-   **Links**: These represent the rigid physical components of the robot, such as the base, torso, arms, legs, head, etc. Each link can have visual, collision, and inertial properties.
-   **Joints**: These define the kinematic relationships between links, specifying how they can move relative to each other (e.g., rotational joints for arms and legs, fixed joints for rigid connections).

URDF acts as the blueprint for your robot, allowing ROS tools like RViz (for visualization) and Gazebo (for simulation) to understand and interact with the robot's physical model.

## URDF Structure and Syntax

A URDF file typically starts with a `<robot>` tag. Inside this, you'll find `<link>` and `<joint>` tags.

### Links

Each `<link>` tag defines a single rigid body.

```xml
<link name="base_link">
  <visual>
    <!-- Describes how the link looks -->
    <geometry>
      <box size="0.1 0.1 0.2"/> <!-- Example: a rectangular base -->
    </geometry>
    <!-- Position and orientation of the geometry relative to the link origin -->
    <origin xyz="0 0 0.1"/> 
    <material name="blue"> <!-- Optional: define color -->
      <color rgba="0 0 1 1"/> <!-- RGBA: Red, Green, Blue, Alpha -->
    </material>
  </visual>
  <collision>
    <!-- Defines the collision shape for physics simulations -->
    <geometry>
      <box size="0.1 0.1 0.2"/>
    </geometry>
    <origin xyz="0 0 0.1"/>
  </collision>
  <inertial>
    <!-- Crucial for physics simulations: defines mass and inertia -->
    <mass value="1.0"/> <!-- Mass in kilograms -->
    <!-- Inertia tensor (ixx, ixy, etc.) -->
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/> 
  </inertial>
</link>
```

-   **`<geometry>`**: Defines the shape using basic primitives (`box`, `cylinder`, `sphere`) or by referencing an external mesh file (e.g., `mesh filename="package://my_robot_description/meshes/my_part.stl"`).
-   **`<origin>`**: Specifies the position (`xyz`) and orientation (`rpy` - roll, pitch, yaw) of the geometry, collision, or joint relative to the link's origin.
-   **`<material>`**: Defines visual properties like color.
-   **`<collision>`**: Defines the shape used for physics interactions in simulations.
-   **`<inertial>`**: Essential for physics calculations, defining mass and inertia tensor.

### Joints

Joints connect two links, defining how they can move relative to each other.

```xml
<joint name="base_to_imu" type="fixed"> 
  <!-- Type can be 'fixed', 'revolute', 'continuous', 'prismatic', 'planar', 'floating' -->
  <parent link="base_link"/> <!-- The link this joint is attached to -->
  <child link="imu_link"/>  <!-- The link that moves relative to the parent -->
  <!-- Position and orientation of the child link relative to the parent's origin -->
  <origin xyz="0 0 0.15" rpy="0 0 0"/> 
</joint>

<joint name="neck_joint" type="revolute"> <!-- A revolute joint allows rotation -->
  <parent link="head"/>
  <child link="neck"/>
  <!-- Axis of rotation (e.g., Y-axis for neck pitch) -->
  <axis xyz="0 1 0"/> 
  <!-- Joint limits: lower/upper bounds, effort, velocity -->
  <limit lower="-1.57" upper="1.57" effort="1" velocity="1"/> 
</joint>
```

-   **`name`**: A unique identifier for the joint.
-   **`type`**: Specifies the joint's degree of freedom and motion type (e.g., `revolute` for rotation, `prismatic` for linear motion).
-   **`<parent>` / `<child>`**: Define the two links being connected.
-   **`<axis>`**: For revolute/continuous/prismatic joints, defines the axis of motion.
-   **`<limit>`**: Sets the joint's range of motion, maximum effort, and velocity.

## Creating a Simple Humanoid Model

URDF for a humanoid robot involves defining a chain of links and joints that mimic the human body's structure. This typically starts from a base link (like the torso or pelvis) and extends to the head, arms, and legs.

Here's a conceptual outline for a simplified humanoid URDF. This example is illustrative and not fully functional without defining all necessary links and joints.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link (Torso) -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.3 0.2 0.4"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.3 0.2 0.4"/></geometry>
    </collision>
    <inertial>
      <mass value="15"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual><geometry><sphere radius="0.15"/></geometry></visual>
    <collision><geometry><sphere radius="0.15"/></geometry></collision>
    <inertial><mass value="2"/><inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/></inertial>
  </link>
  <joint name="base_to_head" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/> <!-- Yaw axis -->
    <limit lower="-1.57" upper="1.57" effort="1" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual><geometry><cylinder length="0.25" radius="0.05"/></geometry></visual>
    <collision><geometry><cylinder length="0.25" radius="0.05"/></geometry></collision>
    <inertial><mass value="1"/><inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/></inertial>
  </link>
  <joint name="base_to_left_arm" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0.15 0.15" rpy="0 0 0"/> <!-- Attached to torso side -->
    <axis xyz="0 1 0"/> <!-- Rotation around Y axis -->
    <limit lower="-2.0" upper="2.0" effort="5" velocity="2"/>
  </joint>
  <!-- ... add left_lower_arm, left_hand ... -->

  <!-- Right Arm -->
  <!-- ... similar structure as left arm ... -->

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual><geometry><cylinder length="0.3" radius="0.07"/></geometry></visual>
    <collision><geometry><cylinder length="0.3" radius="0.07"/></geometry></collision>
    <inertial><mass value="3"/><inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.02"/></inertial>
  </link>
  <joint name="base_to_left_leg" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="0 -0.15 0.1" rpy="0 0 0"/> <!-- Attached to torso bottom side -->
    <axis xyz="0 1 0"/> <!-- Rotation around Y axis -->
    <limit lower="-1.57" upper="1.57" effort="15" velocity="1"/>
  </joint>
  <!-- ... add left_lower_leg, left_foot ... -->

  <!-- Right Leg -->
  <!-- ... similar structure as left leg ... -->

</robot>
```

**Important Considerations for Real Robots:**
-   **Mesh Files**: For realistic visuals, you'll use external mesh files (`.stl`, `.dae`, etc.) referenced in `<mesh filename="..." />`.
-   **`xacro`**: For complex robots like humanoids, `xacro` (XML Macros) is essential. It allows you to use variables, include files, and define macros to simplify the URDF, preventing repetition and making it more manageable.
-   **Physics Properties**: Accurate mass and inertia values are vital for simulations.

## Visualizing the URDF Model

Once you have a URDF file, you can visualize it using ROS tools.

### Using RViz2

RViz2 is a powerful 3D visualization tool for ROS.
1.  **Create a ROS 2 Package**: Your URDF file should reside within a ROS 2 package (e.g., `my_robot_description`).
2.  **Create a Launch File**: A launch file is used to load the robot model into RViz. It typically uses `xacro` to process the URDF (if using macros) and then starts RViz with a display configuration.
3.  **Run RViz**: Launch the file using `ros2 launch my_robot_description rviz_launch.py`.
4.  **Add Robot Model Display**: In RViz, add a "RobotModel" display and point it to your robot's URDF.

The RViz display will render your robot based on the link and joint definitions, showing the kinematic structure and (if defined) the visual properties and colors.

This chapter provided an overview of URDF, its structure, and how to represent a humanoid robot. While this example is simplified, it lays the groundwork for understanding how robots are described in ROS. In the next phase, we'll focus on the overall architecture of our AI-driven book creation system.