#! /usr/bin/python3.8

import rospy
import intera_interface
import time

from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

def limb_mov_to_coord(x:float, y:float, z:float):
	print('[DEBUG] ==> limb_mov_to_coord')
	robot_params = intera_interface.RobotParams()
	limb_names = robot_params.get_limb_names()
	joint_names = robot_params.get_joint_names(limb_names[0])
	limb = intera_interface.Limb(limb=limb_names[0])
	
	traj_options = TrajectoryOptions()
	traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
	traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
	
	way_point_opt = MotionWaypointOptions(
		max_linear_speed=0.05,
		max_linear_accel=0.05,
		max_rotational_speed=0.05,
		max_rotational_accel=0.05,
		max_joint_speed_ratio=0.05
	)
	
	waypoint = MotionWaypoint(options = way_point_opt.to_msg(), limb = limb)
	endpoint_state = limb.tip_state('right_hand')
	
	pose = endpoint_state.pose
	
	pose.position.x = x
	pose.position.y = y
	pose.position.z = z
	poseStamped = PoseStamped()
	poseStamped.pose = pose
	
	# get rotation of joint angeles
	joint_angles = limb.joint_ordered_angles()
	waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)
	
	traj.append_waypoint(waypoint.to_msg())
	result = traj.send_trajectory()
	if result is None:
		rospy.logerr('Trajectory FAILED to send')	
	

def limb_control():
	print('[DEBUG] ==> limb_control')
	
	robot_params = intera_interface.RobotParams()
	limb_names = robot_params.get_limb_names()
	joint_names = robot_params.get_joint_names(limb_names[0])
	limb = intera_interface.Limb(limb=limb_names[0])
	
	traj_options = TrajectoryOptions()
	traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
	traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
	
	way_point_opt = MotionWaypointOptions(
		max_linear_speed=0.05,
		max_linear_accel=0.05,
		max_rotational_speed=0.05,
		max_rotational_accel=0.05,
		max_joint_speed_ratio=0.05
	)
	waypoint = MotionWaypoint(options = way_point_opt.to_msg(), limb = limb)
	endpoint_state = limb.tip_state('right_hand')
	print(f'[DEBUG] endpoint state pose: {endpoint_state.pose}')
	
	pose = endpoint_state.pose
	
	# set pose
	pose.position.x = float(0.0)
	pose.position.y = float(0.99)
	pose.position.z = float(0.0)
	poseStamped = PoseStamped()
	poseStamped.pose = pose
	
	# get rotation of joint angeles
	joint_angles = limb.joint_ordered_angles()
	waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)
	
	# DEBUG by displaying the waypoint to be about to get send
	rospy.loginfo('[DEBUG] ending waypoint: \n%s', waypoint.to_string())
	
	traj.append_waypoint(waypoint.to_msg())
	result = traj.send_trajectory()
	if result is None:
		rospy.logerr('Trajectory FAILED to send')
	else:
		print(f'[DEBUG] endpoint state pose: {endpoint_state.pose}')
		print('[DEBUG] Success')
		
def limb_neutral_pos():
	print('[DEBUG] ==> limb_neutral_pos')
	robot_params = intera_interface.RobotParams()
	limb_names = robot_params.get_limb_names()
	joint_names = robot_params.get_joint_names(limb_names[0])
	limb = intera_interface.Limb(limb=limb_names[0])
	print(limb.endpoint_pose())
	# set joint speed 
	limb.set_joint_position_speed(0.1)
	# move to nutral positon
	limb.move_to_neutral(speed=0.2)
	
def limb_max_extention():
	print('[DEBUG] ==> limb_max_extention')
	robot_params = intera_interface.RobotParams()
	limb_names = robot_params.get_limb_names()
	joint_names = robot_params.get_joint_names(limb_names[0])
	limb = intera_interface.Limb(limb=limb_names[0])
	print(limb.endpoint_pose())
	# set joint speed 
	limb.set_joint_position_speed(0.01)
	
	joint_angles = {}
	# set all joint angels to zero
	for name in joint_names:
		joint_angles[name] = 0.0
	print(joint_angles)
	time.sleep(0.2)
	# move to meximum arme extentation
	limb.move_to_joint_positions(joint_angles)

def gripper_init():
	print('[DEBUG] ==> gripper_init')
	gripper = intera_interface.Gripper()
	gripper.reboot()
	time.sleep(5)
	gripper.calibrate()
	while not gripper.is_ready():
		print('.')
		time.sleep(0.1)
	if gripper.is_ready():
		print('[DEBUG] closing ...')
		gripper.close()
		time.sleep(10)
		print('[DEBUG] opening ...')
		gripper.open()
	else:
		print('[DEBUG] gripper not ready')	
	#print(gripper.get_position())
	
def gripper_controle(action:str):
	print('[DEBUG] ==> gripper_controle')
	gripper = intera_interface.Gripper()
	if action == 'open':
		gripper.open()
	if action == 'close':
		gripper.close()
	
def display_end_pos():
	print('[DEBUG] ==> display_end_pos')
	robot_params = intera_interface.RobotParams()
	limb_names = robot_params.get_limb_names()
	limb = intera_interface.Limb(limb=limb_names[0])
	endpoint_state = limb.tip_state('right_hand')
	print(f'[DEBUG] endpoint state pose: {endpoint_state.pose}')


def main():
	rospy.init_node('robot_mover')
	limb_neutral_pos()
	time.sleep(0.5)
	limb_mov_to_coord(-0.2, 0.6, 0.15)
	display_end_pos()
	time.sleep(0.5)
	limb_mov_to_coord(0.2,0.2,0.9)
	display_end_pos()
	gripper_init()
	rospy.spin()
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
		
###############################################
# define globale pos
#pose = {}
#for name in joint_names:
#	pose[name] = 0.0
#print(pose)
#limb.move_to_joint_positions(pose)
###############################################
