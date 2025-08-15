# turtlebot3-pid-motion-controller
Developed a custom ROS node to control TurtleBot3 using a PID controller based on real-time odometry, enabling smooth and accurate navigation to target positions.

# 🤖 TurtleBot3 PID Motion Controller using ROS

This project implements a **custom PID controller** for TurtleBot3 navigation using ROS 1. The robot receives goal coordinates and computes linear and angular velocity commands based on **odometry feedback** (`/odom` topic) to reach the goal smoothly and accurately. The control logic uses a full PID formulation (P + I + D) for both linear and angular movement.

---

## 📌 Features

- ✅ Custom PID control loop (Proportional, Integral, Derivative)
- ✅ Real-time feedback from `/odom` using `nav_msgs/Odometry`
- ✅ Converts quaternion to yaw angle using `tf` transformation
- ✅ Publishes `Twist` messages to `/cmd_vel` for robot motion
- ✅ Deadzone logic with `distance_tolerance` to stop at goal
- ✅ Fully modular ROS node using `rospy`

---

## 🧠 How It Works

1. The node subscribes to `/odom` to get the current `x`, `y`, and `yaw`.
2. Calculates distance and angle to the goal `(x_goal, y_goal)`.
3. Computes linear and angular speed using PID:
   - `linear_speed = Kp*d_error + Ki*∫d_error + Kd*d_diff`
   - `angular_speed = Kp*a_error + Ki*∫a_error + Kd*a_diff`
4. Publishes velocities via `/cmd_vel`.

---

## 🛠️ Requirements

- Ubuntu 20.04 / 22.04
- ROS Noetic or Melodic
- Python 3
- TurtleBot3 (real or simulation)

---

## 🚀 Setup & Run

### 1. Clone the Repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/<your-username>/turtlebot3-pid-motion-controller.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
````

### 2. Launch TurtleBot3 Simulation (if using Gazebo)

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

### 3. Run the Controller Node

```bash
rosrun turtlebot3_pid_controller control_pid.py
```

---

## 🧾 File Overview

| File             | Description                                                                      |
| ---------------- | -------------------------------------------------------------------------------- |
| `control_pid.py` | Main ROS node that implements the PID controller and publishes velocity commands |

---

## ⚙️ Tunable Parameters

You can adjust these PID coefficients and tolerance in `control_pid.py`:

```python
self.kp_linear = 0.2
self.ki_linear = 0.3
self.kd_linear = -0.2

self.kp_angular = 0.2
self.ki_angular = 0.1
self.kd_angular = -0.2

self.distance_tolerance = 0.2
```

---

Or if you'd like me to generate this as a downloadable file.
```
