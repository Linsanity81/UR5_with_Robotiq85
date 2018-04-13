#!/usr/bin/env python

"""
    teleop_continue.py - Version 0.1 2018.4.2
    This file is created for UR5 with robotiq85 project.
    The maintainer is Rulin Chen(Linsanity81) from Shantou University in China.

    Through this file, you can control UR5 by keyborad input.

    usage:
    #roslaunch ur5_joint_limit_config ur5_bringup.launch
    #rosrun ur5_joint_limit_config teleop_continue.py
"""

import time
import rospy, sys
import moveit_commander
import select, termios, tty
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import roslib; roslib.load_manifest('ur_driver')

move_distance = 0.01

moveBindings = {
		'w':(0, move_distance),
		's':(0, -move_distance),
		'a':(1, move_distance),
		'd':(1, -move_distance),
		'q':(2, move_distance),
		'e':(2, -move_distance),
	       }

msg = """
Reading from the keyboard
---------------------------
Moving around:
   w : move forward 
   s : move back
   a : move left
   d : move right
   q : move up
   e : move dowm

anything else : stop
---------------------------
By default, the distance of each move is 0.05 meters

CTRL-C to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    settings = termios.tcgetattr(sys.stdin)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class MoveItDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_demo')

        global client

        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."

        client.wait_for_server()
        print "Connected to server" 
        # Initialize the move group for the arm
        arm = moveit_commander.MoveGroupCommander('arm')
                
        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()
                        
        # Set the reference frame for pose targets
        reference_frame = 'world'
        
        # Set the arm reference frame accordingly
        arm.set_pose_reference_frame(reference_frame)
                
        # Reject replanning to increase the odds of a solution
        arm.allow_replanning(False)
        
        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.05)
        
        # Start the arm in the "waiting" pose stored in the SRDF file
        arm.set_named_target('waiting')
        traj = arm.plan()

        g = FollowJointTrajectoryGoal()
        g.trajectory = traj.joint_trajectory
        client.send_goal(g)

        rospy.sleep(2)

        arm.set_start_state_to_current_state()

        print msg
        print arm.get_current_pose(end_effector_link).pose.position.x
	print arm.get_current_pose(end_effector_link).pose.position.y
	print arm.get_current_pose(end_effector_link).pose.position.z
	print arm.get_current_joint_values()

        time_start = rospy.Time.now()

	while(1):
	    key = getKey()
            time_from_start = rospy.Time.now() - time_start
	    if key in moveBindings.keys() and time_from_start >= rospy.Duration(0.3) :
                arm.shift_pose_target(moveBindings[key][0], moveBindings[key][1], end_effector_link)
		traj = arm.plan()

                g = FollowJointTrajectoryGoal()
                g.trajectory = traj.joint_trajectory
                client.send_goal(g)

        	print arm.get_current_pose(end_effector_link).pose.position.x
		print arm.get_current_pose(end_effector_link).pose.position.y
		print arm.get_current_pose(end_effector_link).pose.position.z 
		print arm.get_current_joint_values()
                time_start = rospy.Time.now()

            else:
                if (key == '\x03'):
                    break

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()