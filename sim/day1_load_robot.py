# Day 1: u'n

import pybullet as p    #main api
import pybullet_data    #URDF/ model/ texture
import time             #sleep to make the while loop simulate in a constant interval

physics_client = p.connect(p.GUI)

p.setAdditionalSearchPath(         # PyBullet：using pybullet built-in urdf
    pybullet_data.getDataPath()
)

plane_id = p.loadURDF("plane.urdf")                                 # load florr URDF
robot_id = p.loadURDF("franka_panda/panda.urdf",           # loadFranka Panda 機械臂 URDF
                      useFixedBase=True)                            #Fixed the base of the robot arm

n_joints = p.getNumJoints(robot_id)     #get num of joints
print("Number of joints:", n_joints)

for idx in range(n_joints):
    info = p.getJointInfo(robot_id, idx)
    name = info[1].decode("utf-8")                    # print joint info and idx
    print(idx, name)

p.resetJointState(robot_id, 1, targetValue=0.5)       # set joint 1 to rad = 0.5, for testing


p.setGravity(0, 0, -9.81)                             # gravity

while True:
    p.stepSimulation()
    time.sleep(1/240)

p.disconnect()