import cv2
import numpy as np
import math

def drawDecorations(image, distance, tx, ty, x=None, y=None, w=None, h=None):
    # (Keep your existing drawDecorations function, just add tx and ty to the parameters)
    # Add tx and ty to the display
    cv2.putText(image,
                f'tx: {tx:.2f}, ty: {ty:.2f}',
                (50, 550),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, (0, 255, 0), 2, cv2.LINE_AA)

def calculate_tx_ty(contour, image_width, image_height):
    M = cv2.moments(contour)

    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0, 0

    offset_x = cX - (image_width / 2)
    offset_y = cY - (image_height / 2)

    # Assuming a 54.505° horizontal FOV and 41.37° vertical FOV for Limelight 3
    degrees_per_pixel_x = 54.505 / image_width
    degrees_per_pixel_y = 41.37 / image_height

    tx = offset_x * degrees_per_pixel_x
    ty = -offset_y * degrees_per_pixel_y  # Negative because y increases downward in image

    return tx, ty

def runPipeline(image, llrobot):
    # Apply Gaussian blur for better performance
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    img_hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    strafeDistance = 0

    color = "Blue" if llrobot[0] == 0 else "Red"
    targetDistance = llrobot[1] if llrobot[1] != 0 else 4

    if color == "Red":
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)
        img_threshold = cv2.bitwise_or(mask1, mask2)
    else:  # Blue
        lower_blue = np.array([90, 150, 50])
        upper_blue = np.array([140, 255, 255])
        img_threshold = cv2.inRange(img_hsv, lower_blue, upper_blue)

    kernel = np.ones((5,5), np.uint8)
    img_threshold = cv2.morphologyEx(img_threshold, cv2.MORPH_OPEN, kernel)
    img_threshold = cv2.morphologyEx(img_threshold, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largestContour = np.array([[]])
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]  # Initialize with default values

    if contours:
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]

        if valid_contours:
            cv2.drawContours(image, valid_contours, -1, (255, 255, 255), 1)

            largestContour = max(valid_contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largestContour)

            tx, ty = calculate_tx_ty(largestContour, image.shape[1], image.shape[0])

            # Convert tx to radians for strafeDistance calculation
            angle_rad = math.radians(tx)
            strafeDistance = (targetDistance * math.sin(math.pi/2 - angle_rad)) / math.sin(angle_rad)

            cv2.putText(image,
                        "Strafe Distance: " + str(strafeDistance),
                        (350, 550),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 0), 2, cv2.LINE_AA)

            llpython = [tx, ty, strafeDistance, x, y, w, h, cv2.contourArea(largestContour)]

            drawDecorations(image, strafeDistance, tx, ty, x, y, w, h)
        else:
            drawDecorations(image, 0, 0, 0)
            cv2.putText(image, 'No valid contours found!', (50, 550), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    else:
        drawDecorations(image, 0, 0, 0)
        cv2.putText(image, 'No contours found!', (50, 550), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    return largestContour, image, llpython