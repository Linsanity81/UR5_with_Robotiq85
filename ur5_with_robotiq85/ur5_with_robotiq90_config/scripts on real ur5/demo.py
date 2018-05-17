#!/usr/bin/env python

from copy import deepcopy
import time
import rospy, sys
import moveit_commander
import select, termios, tty
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import Point
from moveit_msgs.msg import Constraints, OrientationConstraint
from rbx2_arm_nav.arm_utils import scale_trajectory_speed
from geometry_msgs.msg import PoseStamped, Pose
import roslib; roslib.load_manifest('ur_driver')

class MoveItDemo:

    def __init__(self):
	
	global client

	rospy.init_node("moveit_demo", anonymous=True, disable_signals=True)

        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."

        client.wait_for_server()
        print "Connected to server"

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
	          
        # Initialize the move group for the arm
        arm = moveit_commander.MoveGroupCommander('arm')
                
        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()
                        
        # Set the reference frame for pose targets
        reference_frame = 'world'
        
        # Set the arm reference frame accordingly
        arm.set_pose_reference_frame(reference_frame)
                
        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)
        
        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.05)
        rospy.sleep(1)
        
	target_pose = PoseStamped()
	target_pose.header.frame_id = reference_frame
	target_pose.pose.position.x = 0.50
	target_pose.pose.position.y = 0
	target_pose.pose.position.z = 0
	target_pose.pose.orientation.x = -0.5
	target_pose.pose.orientation.y = -0.5
	target_pose.pose.orientation.z = 0.5
	target_pose.pose.orientation.w = 0.6

	arm.set_pose_target(target_pose, end_effector_link)

        # Plan the trajectory to the goal
        traj = arm.plan()

        # Scale the trajectory speed by a factor of 0.25
        new_traj = scale_trajectory_speed(traj, 0.20)

        g = FollowJointTrajectoryGoal()
        g.trajectory = new_traj.joint_trajectory
        client.send_goal(g)
                           
	rospy.sleep(1)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()
