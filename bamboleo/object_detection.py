#! /usr/bin/python3.8
from ultralytics import YOLO
import numpy as np
import cv2
import time


def detect_objects(img):
	model_path = "/home/ubuntu/scripts/model/best.pt"
	model = YOLO(model_path)
	
	object_list = model(img)
	return object_list[0].boxes.data.numpy()
	
def calc_object_center(x_min, y_min, x_max, y_max):
	center_x = (x_max-x_min)/2 + x_min 
	center_y = (y_max-y_min)/2 + y_min
	return [center_x, center_y]
	
def calc_center_offset(x, y, hight, width):
	center_x = width/2
	center_y = hight/2
	
	dx = (x-center_x)
	dy = (y-center_y)
	return dx, dy
	
def get_pixel_size(img):
	img = cv2.GaussianBlur(img, (9,9), 2)
	circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=45, minRadius=0, maxRadius=0)
	board_center = []
	object_center = []
	pixel_size = 0
	if circles is not None:
		circles = np.round(circles[0, :]).astype("int")
		for i in range(len(circles)):
			x,y,r = circles[i]
			if i == 0:
				board_center = [x, y]
				pixel_size = 360/(2*r)
			elif i == 1:
				object_center = [x, y]
	
	return pixel_size, board_center

