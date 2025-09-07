# learning how to apply basic move
# if step size = 0.01. joint 2 will hit the ground due to gravity and momentum
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#load floor and Panda(arm)
plane = p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

n_joints = p.getNumJoints(panda)
print("Number of joints:", n_joints)

p.setGravity(0, 0, -9.81)

#first 7 joints are the main (exclude the craw)
controlled_joints = list(range(7))

# initialize the angel of the joints
for j in controlled_joints:
    p.resetJointState(panda, j, 0)

#make joint 1 swing between rad = 0 to 1
target = 0.0
direction = 1   #pos dir
step_size = 0.001

while True:
    # update target
    target += direction * step_size
    if target > 1.5:
        direction = -1
    elif target < 0.0:
        direction = +1

    p.setJointMotorControl2(
        bodyUniqueId=panda,
        jointIndex=0,  # panda_joint1
        controlMode=p.POSITION_CONTROL,
        targetPosition=target,
        force=10  # max power of motor
    )
    p.setJointMotorControl2(
        bodyUniqueId=panda,
        jointIndex=1,  # panda_joint2
        controlMode=p.POSITION_CONTROL,
        targetPosition=target,
        force=50  # max power of motor
    )

    p.stepSimulation()
    time.sleep(1 / 240)
    joint_state = p.getJointState(panda, 1)
    actual_pos = joint_state[0]
    actual_vel = joint_state[1]
    applied_force = joint_state[3]

    # print joint status (angle, speed, reaction force)
    # joint_state = p.getJointState(panda, 0)
    # print(f"Joint1 pos={joint_state[0]:.3f}, vel={joint_state[1]:.3f}")
    print(f"Target={target:.2f} rad | Actual={actual_pos:.2f} rad | Vel={actual_vel:.2f} | Force={applied_force:.2f}")