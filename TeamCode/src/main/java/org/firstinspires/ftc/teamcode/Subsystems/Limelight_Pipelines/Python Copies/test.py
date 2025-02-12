import cv2
import numpy as np
from math import cos, sin

#useful variables
color = "Red"  # This should come from llrobot instead of being hardcoded
strafeDistance = 0
INCHtoPIXEL = 0.01875 # camera range inches / 640 (frame width)

def drawDecorations(image, distance, x=None, y=None, w=None, h=None):
    # Add the special text with a slightly larger font and yellow color
    cv2.putText(image,
                'Aaron & Stephen Goats',
                (000, 100),  # Positioned near top of screen
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5,  # Larger font scale
                (0, 255, 255),  # Yellow color
                3,  # Thicker text
                cv2.LINE_AA)

    # Original decorations
    cv2.putText(image,
                'Specimen Alignment!',
                (0, 700),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.putText(image,
                'Strafe Distance: ' + str(distance),
                (550, 700),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, (0, 255, 0), 2, cv2.LINE_AA)

    # Draw bounding box and crosshair if we have coordinates
    if all(v is not None for v in [x, y, w, h]):
        # Draw bounding box - separate color for debugging
        box_color = (0, 255, 255)  # Yellow for visibility
        cv2.rectangle(image, (x, y), (x + w, y + h), box_color, 3)  # Increased thickness

        # Add debug text
        cv2.putText(image,
                    f'Box: ({x},{y}) {w}x{h}',
                    (50, 650),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, box_color, 2, cv2.LINE_AA)

        # Draw crosshair at center of bounding box
        center_x = x + w//2
        center_y = y + h//2

        # Horizontal line
        cv2.line(image, (center_x - 10, center_y), (center_x + 10, center_y), (0, 255, 0), 2)
        # Vertical line
        cv2.line(image, (center_x, center_y - 10), (center_x, center_y + 10), (0, 255, 0), 2)

def runPipeline(image, llrobot):
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Get alliance color from llrobot
    color = "Blue" if llrobot[0] == 1 else "Blue"

    # Debug color text
    cv2.putText(image,
                f'Color Mode: {color}',
                (50, 600),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, (0, 255, 0), 2, cv2.LINE_AA)

    # Adjusted HSV ranges for better detection
    if color == "Red":
        # Use two ranges for red to handle the wrap-around in HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)
        img_threshold = cv2.bitwise_or(mask1, mask2)
    else:  # Blue
        # Adjusted blue range based on common blue values
        lower_blue = np.array([90, 150, 50])    # Broadened hue range, increased saturation minimum
        upper_blue = np.array([140, 255, 255])  # Increased maximum hue range
        img_threshold = cv2.inRange(img_hsv, lower_blue, upper_blue)

        # Apply morphological operations to clean up the mask
        kernel = np.ones((5,5), np.uint8)
        img_threshold = cv2.morphologyEx(img_threshold, cv2.MORPH_OPEN, kernel)
        img_threshold = cv2.morphologyEx(img_threshold, cv2.MORPH_CLOSE, kernel)

    # Save threshold image for debugging
    cv2.putText(img_threshold,
                f'Threshold Image',
                (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, (255, 255, 255), 2, cv2.LINE_AA)

    contours, _ = cv2.findContours(img_threshold,
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largestContour = np.array([[]])
    llpython = [0]  # Initialize with default value

    if len(contours) > 0:
        # Filter contours by minimum area
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]

        if valid_contours:
            # Draw all contours in white for debugging
            cv2.drawContours(image, valid_contours, -1, (255, 255, 255), 1)

            largestContour = max(valid_contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largestContour)

            # Calculate contour area for debugging
            area = cv2.contourArea(largestContour)
            cv2.putText(image,
                        f'Contour Area: {area:.0f}',
                        (50, 500),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.putText(image,
                        'Sample X:  ' + str(x),
                        (550, 650),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 0), 2, cv2.LINE_AA)

            #calculate Strafe Distance
            angle_in_radians = table.getEntry("tx").getDouble(0.0)
            cotangent = cos(angle_in_radians) / sin(angle_in_radians)
            strafeDistance = d * cotangent

            llpython = [strafeDistance]

            # Pass bounding box coordinates to drawDecorations
            drawDecorations(image, strafeDistance, x, y, w, h)
        else:
            drawDecorations(image, 0)
            cv2.putText(image,
                        'No valid contours found!',
                        (50, 550),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 0, 255), 2, cv2.LINE_AA)
    else:
        # Draw basic decorations without bounding box if no contours found
        drawDecorations(image, 0)
        cv2.putText(image,
                    'No contours found!',
                    (50, 550),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 0, 255), 2, cv2.LINE_AA)

    return largestContour, image, llpython, img_threshold  # Added threshold image for debugging