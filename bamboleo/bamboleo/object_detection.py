#! /usr/bin/python3.8
from ultralytics import YOLO
import numpy as np
import cv2
import time


def detect_circle_and_blocks(image):

    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(image, (9, 9), 2)

    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=100,
        param1=100,
        param2=30,
        minRadius=150,
        maxRadius=275
    )

    if circles is not None:
        # Round circle parameters to integers
        circles = np.round(circles[0, :]).astype("int")

        # Choose the largest circle (assuming it's the wood disk)
        circles = sorted(circles, key=lambda c: c[2], reverse=True)
        main_circle = circles[0]
        center = (main_circle[0], main_circle[1])
        radius = main_circle[2]
        pixel_size = 360/2*radius

        # Adaptive Thresholding
        adaptive_thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
        )

        # Combine adaptive threshold with Canny edges
        edges = cv2.Canny(blurred, 30, 100)
        combined_edges = cv2.bitwise_or(adaptive_thresh, edges)

        # Apply morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        combined_edges = cv2.morphologyEx(combined_edges, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(combined_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        block_coordinates = []
        margin = 15  # Margin to avoid false detections near the circle boundary

        for contour in contours:
            # Calculate the moments to find the center of mass
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Calculate distance from the block center to the circle center
                distance = np.sqrt((cX - center[0])**2 + (cY - center[1])**2)

                # Check if the block is inside the circle and sufficiently far from the boundary
                area = cv2.contourArea(contour)
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = w / h

                if (distance <= radius - margin) and 65 < area < 20000 and 0.5 <= aspect_ratio <= 1.6:
                    # Calculate coordinates relative to the circle's center
                    rel_x = cX - center[0]
                    rel_y = cY - center[1]
                    block_coordinates.append((rel_x, rel_y))
            
            return center, block_coordinates, pixel_size

def detect_objects(img):
	model_path = "/home/ubuntu/scripts/model/best.pt"
	model = YOLO(model_path)
	
	object_list = model(img, imgsz=640)
	return object_list[0].boxes.data.numpy()
	
def calc_object_center(x_min, y_min, x_max, y_max):
	center_x = (x_max-x_min)/2 + x_min 
	center_y = (y_max-y_min)/2 + y_min
	return [center_x, center_y]
	
def calc_offset(x, y, hight, width):
	center_x = width/2
	center_y = hight/2
	
	dx = (x-center_x)
	dy = (y-center_y)
	return dx, dy

def calc_center_offset(board_center, item_center):
	dx = board_center[0]-item_center[0]
	dy = board_center[1]-item_center[1]
	return dx, dy
	
def get_pixel_size(img):
	img = cv2.GaussianBlur(img, (9,9), 2)
	circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, dp=1.2, minDist=100, param1=100, param2=30, minRadius=150, maxRadius=500)
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

