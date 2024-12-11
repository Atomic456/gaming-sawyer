#! /usr/bin/python3.8

import rospy
import intera_interface
import time
import math

from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

def limb_mov_to_coord(pos, ori):
	print('[DEBUG] ==> limb_mov_to_coord')
	robot_params = intera_interface.RobotParams()
	limb_names = robot_params.get_limb_names()
	joint_names = robot_params.get_joint_names(limb_names[0])
	limb = intera_interface.Limb(limb=limb_names[0])
	
	traj_options = TrajectoryOptions()
	traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
	traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
	
	way_point_opt = MotionWaypointOptions(
		max_linear_speed=0.2,
		max_linear_accel=0.2,
		max_rotational_speed=0.2,
		max_rotational_accel=0.2,
		max_joint_speed_ratio=0.2
	)
	
	waypoint = MotionWaypoint(options = way_point_opt.to_msg(), limb = limb)
	endpoint_state = limb.tip_state('right_hand')
	
	pose = endpoint_state.pose
	
	if pos is None or len(pos) != 3:
		rospy.logerr('No position specifyed failed to move the robot!')
		
	pose.position.x = pos[0]
	pose.position.y = pos[1]
	pose.position.z = pos[2]
	poseStamped = PoseStamped()
	poseStamped.pose = pose
	
	if ori is not None and len(ori) == 4:
		print("Setting oriententation ...")
		pose.orientation.x = ori[0]
		pose.orientation.y = ori[1]
		pose.orientation.z = ori[2]
		pose.orientation.w = ori[3]
		
	print(f'configured pose privided by paramters: {pose}')
	
	# get rotation of joint angeles
	joint_angles = limb.joint_ordered_angles()
	waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)
	
	traj.append_waypoint(waypoint.to_msg())
	result = traj.send_trajectory()
	if result is None:
		rospy.logerr('Trajectory FAILED to send')
	else: 
		print(f'Trajectory Executed: {result}')
		
def get_end_effector_orientation():
	print('[DEBUG] ==> get_end_effector_orientation')
	robot_params = intera_interface.RobotParams()
	limb_names = robot_params.get_limb_names()
	joint_names = robot_params.get_joint_names(limb_names[0])
	limb = intera_interface.Limb(limb=limb_names[0])
	
	endpoint_state = limb.tip_state('right_hand')
	print('============================= endpoint_state =============================')
	print(endpoint_state)
	print('==========================================================================')
	joint_angles = limb.joint_ordered_angles()
	print('============================== joint_angles ==============================')
	print(joint_angles)
	print('==========================================================================')
		
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
	limb.set_joint_position_speed(0.1)
	
	joint_angles = {}
	# set all joint angels to zero
	for name in joint_names:
		joint_angles[name] = 0.0
	print(joint_angles)
	time.sleep(0.2)
	# move to meximum arme extentation
	limb.move_to_joint_positions(joint_angles)
	
def set_joint_angels(angles):
	print('[DEBUG] ==> set_joint_angels')
	robot_params = intera_interface.RobotParams()
	limb_names = robot_params.get_limb_names()
	joint_names = robot_params.get_joint_names(limb_names[0])
	limb = intera_interface.Limb(limb=limb_names[0])
	print(limb.endpoint_pose())
	# set joint speed 
	limb.set_joint_position_speed(0.1)
	
	joint_angles = {}
	# set all joint angels to zero
	idx = 0
	for name in joint_names:
		if len(angles) > idx and angles[idx] is not None:
			joint_angles[name] = angles[idx]
		idx += 1
	print(joint_angles)
	# move to meximum arme extentation
	res = limb.move_to_joint_positions(joint_angles)
	print(res)

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
	pos = [0.8820733750820632, 0.1578053773120996, 0.31494917536003253]
	ori = [0.5401057038144683, 0.0000000000, 0.5428952585441672, -0.454491558906238]
	joint_angles = [-0.0017305944756964664, -0.00010714471693596971, 0.037955441591930435, -0.019549573823439508, -0.04093183928579047, 1.5824264201324811, -0.000494495177923493]
	display_end_pos()
	print("-------------------------------------------------")
	limb_neutral_pos()
	#limb_max_extention()
	set_joint_angels(joint_angles)
	#rospy.spin()
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
