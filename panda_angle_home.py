#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import numpy as np

def perform_trajectory():

    rospy.init_node('panda_trajectory_publisher')
    contoller_name='/panda_controller/command'
    trajectory_publihser = rospy.Publisher(contoller_name,JointTrajectory, queue_size=10)

    # Parameter setting
    dof = 7                                # degree of freedom
    argv = np.empty(dof, dtype=np.float64)

    # execute with 7 values: rosrun franka_gazebo panda_trajectory_node.py n0 n1 n2 n3 n4 n5 n6 (n# are your numbers)
    panda_joints = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

    # define input angles, radian
    argv[0] = 0
    argv[1] = -0.785398163
    argv[2] = 0 
    argv[3] = -2.35619449
    argv[4] = 0
    argv[5] = 1.57079632679
    argv[6] = 0.785398163397
    goal_positions = [ float(argv[0]) , float(argv[1]) , float(argv[2]) ,float(argv[3]), float(argv[4]) , float(argv[5]) , float(argv[6]) ]

    #
    rospy.loginfo("Goal position set, let's go!")
    rospy.sleep(0.1)

    #
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = panda_joints
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = goal_positions
    trajectory_msg.points[0].velocities = [0.0 for i in panda_joints]
    trajectory_msg.points[0].accelerations = [0.0 for i in panda_joints]
    trajectory_msg.points[0].time_from_start = rospy.Duration(3)
    rospy.sleep(0.1)
    trajectory_publihser.publish(trajectory_msg)

if __name__ == '__main__':
    perform_trajectory()