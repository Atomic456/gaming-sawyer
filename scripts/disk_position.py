import cv2
import numpy as np
import os

def detect_circle_and_blocks(image_path):
    # Read the image
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

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

    # Create an output image for visualization
    output = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    if circles is not None:
        # Round circle parameters to integers
        circles = np.round(circles[0, :]).astype("int")

        # Choose the largest circle (assuming it's the wood disk)
        circles = sorted(circles, key=lambda c: c[2], reverse=True)
        main_circle = circles[0]
        center = (main_circle[0], main_circle[1])
        radius = main_circle[2]

        # Draw the detected circle and center point
        cv2.circle(output, center, radius, (0, 255, 0), 2)  # Draw the main circle
        cv2.circle(output, center, 5, (0, 0, 255), -1)  # Draw the center point

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

                    # Draw the block center
                    cv2.circle(output, (cX, cY), 5, (255, 0, 0), -1)  # Draw block center
                    cv2.line(output, center, (cX, cY), (0, 255, 0), 1)  # Draw line to center

                    # Put the coordinates on the image
                    text = f"({rel_x}, {rel_y})"
                    cv2.putText(output, text, (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return center, block_coordinates, output

    else:
        print("No circles detected.")
        return None, [], output

# Process all images in a directory
input_folder = "input_images"
output_folder = "output_images"

# Create the output directory if it doesn't exist
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# Get all image file paths from the input directory
image_paths = [os.path.join(input_folder, file) for file in os.listdir(input_folder) if file.lower().endswith(('.png', '.jpg', '.jpeg'))]

# Process images
for path in image_paths:
    try:
        center, block_coordinates, output_image = detect_circle_and_blocks(path)
        if center:
            print(f"Detected circle center - Center coordinates: {center}")
            for coord in block_coordinates:
                print(f"Block relative coordinates: ({coord[0]}, {coord[1]})")

        # Save the result in the output folder
        output_path = os.path.join(output_folder, os.path.basename(path).replace(".jpeg", "_position.png"))
        cv2.imwrite(output_path, output_image)
    except Exception as e:
        print(f"Error processing image: {path}, Error message: {e}")

print("Processing completed, results have been saved.")

