import math
import pybullet as p
import pybullet_data
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

p.setGravity(0, 0, -9.81)
dt = 1.0 / 240.0

ctrl_idx = [0, 1, 2, 3, 4, 5, 6]

# 初始化關節狀態
for j in ctrl_idx:
    p.resetJointState(panda, j, 0.0)

def get_joint_positions(body, indices):
    states = p.getJointStates(body, indices)
    return np.array([s[0] for s in states], dtype=float)

def move_joints_trajectory(body, indices, q_goal, duration=2.0, force=40.0):
    step_size = 0.001
    q_curr_goal = q_goal.copy()
    for i in range(len(q_curr_goal)):
        q_curr_goal[i] = 0

    while True:
        for i in range(len(q_curr_goal)):
            if q_curr_goal[i] < q_goal[i]:
                q_curr_goal[i] += step_size
            elif q_curr_goal[i] > q_goal[i]:
                q_curr_goal[i] -= step_size

        print(q_curr_goal)
        p.setJointMotorControlArray(
            bodyUniqueId=body,
            jointIndices=indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=q_curr_goal.tolist(),
            forces=[20, 200, 80, 40, 20, 10, 10]
        )


        p.stepSimulation()
        time.sleep(dt)

    return q_goal

# ===== Main =====
q_now = get_joint_positions(panda, ctrl_idx)
q_goal = q_now.copy()
q_goal[0] = math.pi / 2.0
q_goal[1] = math.pi / 2.0

# final_target = move_joints_trajectory(panda, ctrl_idx, q_now, q_goal, duration=4.0, force=40.0)
move_joints_trajectory(panda,ctrl_idx, q_goal, duration=2.0, force=40.0)
