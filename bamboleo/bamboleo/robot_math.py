#! /usr/bin/python3.8
import numpy as np
import math
from typing import List
from object_detection import calc_center_offset

def quaternion2matrix(x, y, z, w):
	return np.array([
		[1-2*(y**2+z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
		[2*(x*y + z*w), 1-2*(x**2 + z**2), 2*(y*z - x*w)],
		[2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x**2 + y**2)]
	])

def add_quaterion(quat1:List[float], quat2:List[float]):
	if len(quat1) == 4 and len(quat2) == 4:
		return ([quat1[0]+quat2[0], quat1[1]+quat2[1], quat1[2]+quat2[2], quat1[3]+quat2[3]])
	else:
		return []
	

def matrix2quaternion(r):
	quaternion = np.empty((4, ))
	# sum of diagonal values
	trace = np.trace(r)
	
	if trace > 0:
		s = np.sqrt(trace+1.0)*2
		w = 0.25 * s
		x = (r[2,1] - r[1,2])/s
		y = (r[0,2] - r[2,0])/s
		z = (r[1,0] - r[0,1])/s
	else:
		if (r[0,0] > r[1,1]) and (r[0,0] > r[2,2]):
			s = 2.0 * np.sqrt(1.0+ r[0,0]-r[1,1]-r[2,2])
			w = (r[2,1] - r[1,2])/s
			x = 0.25 * s 
			y = (r[0,1] + r[1,0])/s
			z = (r[0,2] + r[2,0])/s
		elif r[1,1] > r[2,2]:
			s = 2.0 * np.sqrt(1.0+ r[1,1]-r[0,0]-r[2,2])
			w = (r[0,2] - r[2,0])/s
			x = (r[0,1] + r[1,0])/s
			y = 0.25 * s
			z = (r[1,2] + r[2,1])/s
		else:
			s = 2.0 * np.sqrt(1.0+ r[2,2]-r[0,0]-r[1,1])
			w = (r[1,0] - r[0,1])/s
			x = (r[0,2] + r[2,0])/s
			y = (r[1,2] + r[2,1])/s
			z = 0.25 * s
	
	return np.array([x, y, z, w])
	
def angles2quaterion(roll_x, pitch_y, yaw_z):
	roll = (roll_x/180)*math.pi 
	pitch = (pitch_y/180)*math.pi
	yaw = (yaw_z/180)*math.pi
	
	cos_r = np.cos(roll / 2) 
	sin_r = np.sin(roll / 2) 
	cos_p = np.cos(pitch / 2) 
	sin_p = np.sin(pitch / 2) 
	cos_y = np.cos(yaw / 2) 
	sin_y = np.sin(yaw / 2)
	
	qw = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y 
	qx = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y 
	qy = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y 
	qz = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y
	
	return np.array([qx, qy, qz, qw])
	
def multiply_quaternions(quat1, quat2):
	x1, y1, z1, w1 = quat1
	x2, y2, z2, w2 = quat2
	
	qw = w1*w2 - x1*x2 - y1*y2 - z1*z2
	qx = w1*x2 + x1*w2 + y1*z2 - z1*y2
	qy = w1*y2 - x1*z2 + y1*w2 + z1*x2
	qz = w1*z2 + x1*y2 - y1*x2 + z1*w2
	
	return np.array([qx, qy, qz, qw])

def game_piece_picker(objects, center_x:float=0, center_y:float=0):
	obj_to_pick = {
		"idx": -1,
		"dx": 1000,
		"dy": 1000
	}
	total_weight = 0
	force_x = 0
	force_y = 0
	for obj in objects:
		total_weight += object_weights[obj['class']]
		d_x, d_y = calc_center_offset([center_x, center_y], [obj['x'], obj['y']])
		force_x = force_x + (d_x * object_weights[obj['class']])
		force_y = force_y + (d_y * object_weights[obj['class']])
	
	idx = 0
	for obj in objects:
		d_x, d_y = calc_center_offset([center_x, center_y], [obj['x'], obj['y']])
		changed_force_x = force_x - d_x * object_weights[obj['class']]
		changed_force_x = force_y - d_y * object_weights[obj['class']]
		changed_com_x = changed_force_x / (total_weight-object_weights[obj['class']])
		changed_com_y = changed_force_y / (total_weight-object_weights[obj['class']])
		new_com_x, new_com_y = calc_center_offset([changed_com_x, changed_com_y],[center_x, center_y])
		if abs(new_com_x)+abs(new_com_y) < abs(obj_to_pick['dx']) + abs(obj_to_pick['dy']):
			obj_to_pick['idx'] = idx
			obj_to_pick['dx'] = new_com_x
			obj_to_pick['dy'] = new_com_y
		idx += 1
	return objects[obj_to_pick['idx']]
	
