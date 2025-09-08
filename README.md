# My Panda Sim — Week 1 (Day1–3)

## 📌 Overview
This document records the progress from **Day1 to Day3** of the 30-day robotics programming learning plan.

---

## ✅ Day1 — Environment Setup
- Installed **Ubuntu 22.04** in a VM environment.
- Installed **ROS2 Humble** and **Python3 environment**.
- Installed **PyBullet** (`pip install pybullet`).
- Cloned URDF models (Franka Panda as default, UR5 optional).

**Verification:**  
Able to start Python3, import `pybullet`, and confirm installation.

---

## ✅ Day2 — Load URDF in PyBullet
- Successfully loaded the Franka Panda URDF into PyBullet GUI.
- Observed the joint indices and names using `getJointInfo()`.
- Modified joint positions using `resetJointState()`.

**Verification:**  
Robot model appears in the PyBullet GUI window.  
Joints can be manually reset to new angles.

---

## ✅ Day3 — Controlled Motion & Stable Hold
- Implemented a control script in PyBullet:
  - Loads the Panda URDF.
  - Picks a target angle for a joint (e.g., 0.5 rad).
  - Uses `p.setJointMotorControl2(..., POSITION_CONTROL, targetPosition=...)` to move smoothly toward the target.
  - Once the target is reached, the joint **holds its position** instead of falling under gravity.

**Verification:**  
The robot arm joint moves toward the desired angle and stays there stably without sudden dropping.

---

## 🚀 How to Run (Day1–3 examples)

```bash
# Example: run the controlled motion script in PyBullet
python3 controlled_motion.py
```

Where `controlled_motion.py`:
- Loads Panda URDF.
- Commands a joint toward a target angle using POSITION_CONTROL.
- Steps the simulation to ensure smooth motion and stable hold.

---

## 🎯 Next Steps (Day4+)
- Build a minimal ROS2 package.
- Implement a ROS2 node skeleton (`rclpy`).
- Prepare to publish `/joint_states` in ROS2.

