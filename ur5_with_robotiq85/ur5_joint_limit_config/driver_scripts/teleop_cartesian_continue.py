#!/usr/bin/env python

"""
    teleop_cartesian.py - Version 0.1 2018.5.3
    This file is created for UR5 with robotiq85 project.
    The maintainer is Rulin Chen(Linsanity81) from Shantou University in China.

    Through this file, you can control real UR5 by keyborad input.

    usage:
    #roslaunch ur5_joint_limit_config ur5_bringup.launch robot_ip:=<IP>
    #rosrun ur5_joint_limit_config teleop_cartesian_continue.py
"""
from copy import deepcopy
import time
import rospy, sys
import moveit_commander
import select, termios, tty
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import roslib; roslib.load_manifest('ur_driver')
from rbx2_arm_nav.arm_utils import scale_trajectory_speed

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

move_distance = 0.60

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
   o : stop
---------------------------


CTRL-C to quit
"""

Q1 = [15,-76,54,-144, -84, -41]

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    settings = termios.tcgetattr(sys.stdin)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



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

        # Start the arm in the "waiting" pose stored in the SRDF file
        arm.set_named_target('waiting')
        traj = arm.plan()

        # Drive the real UR5
        g = FollowJointTrajectoryGoal()
        g.trajectory = traj.joint_trajectory
        client.send_goal(g)
	
        rospy.sleep(2)
        
        # Set the start state to the current state
        arm.set_start_state_to_current_state()

	print msg

	current_key = '\x24'



	while (1):
	
	    key = getKey()
	    if (key == '\x67'):
		client.cancel_all_goals()
		current_key = key
		rospy.loginfo("Stop.") 
		continue

	    if key in moveBindings.keys() and key != current_key:

		client.cancel_all_goals()
		rospy.sleep(0.8)
          	# Get the current pose so we can add it as a waypoint
        	start_pose = arm.get_current_pose(end_effector_link).pose
                
        	# Initialize the waypoints list
        	waypoints = []
                
            	waypoints.append(start_pose)
            
        	wpose = deepcopy(start_pose)
                
        	# Set the next waypoint
		if (key == '\x77' or key == '\x73'):
        		wpose.position.x += moveBindings[key][1]
		if (key == '\x61' or key == '\x64'):
        		wpose.position.y += moveBindings[key][1]
		if (key == '\x71' or key == '\x65'):
        		wpose.position.z += moveBindings[key][1]

		waypoints.append(deepcopy(wpose))

            	fraction = 0.0
            	maxtries = 50
            	attempts = 0
     
            	# Plan the Cartesian path connecting the waypoints
            	while fraction < 1.0 and attempts < maxtries:
                	(plan, fraction) = arm.compute_cartesian_path (
                                        	waypoints,   # waypoint poses
                                        	0.01,        # eef_step
                                        	0.0,         # jump_threshold
                                        	True)        # avoid_collisions
                
                	# Increment the number of attempts 
                	attempts += 1
                
                	# Print out a progress message
                	if attempts % 5 == 0:
				if (key == '\x77' or key == '\x73'):
					waypoints[1].position.x -= 0.05
				if (key == '\x61' or key == '\x64'):
					waypoints[1].position.y -= 0.05
				if (key == '\x71' or key == '\x65'):
					waypoints[1].position.z -= 0.05
                    		rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
            	# If we have a complete plan, execute the trajectory
            	if fraction == 1.0:
                	rospy.loginfo("Path computed successfully. Moving the arm.")
    
			new_traj = scale_trajectory_speed(plan, 0.3)
                	g = FollowJointTrajectoryGoal()
                	g.trajectory = new_traj.joint_trajectory
                	client.send_goal(g)
                            
                	rospy.loginfo("Path execution complete.")
            	else:
                	rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.") 
 
		current_key = key

	    else:
		if (key == '\x03'):
		    break

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()
