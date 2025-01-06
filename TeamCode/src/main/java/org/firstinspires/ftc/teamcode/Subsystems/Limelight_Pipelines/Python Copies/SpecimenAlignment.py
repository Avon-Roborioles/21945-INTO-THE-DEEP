#----21945 SPECIMEN ALIGNMENT PIPELINE---
# Written by Stephen O. on 1/05/25
# Theory and Formula derived by Aaron M.

# llrobot is array of info sent from front end (bot)
#llpython is array of info sent from back end (vision)

#llrobot array data - {int color} 0 is red, 1 is blue
#llpython array data - {int strafeDistance}

import cv2
import numpy as np

#useful variables
color = "Red"
strafeDistance = 0
INCHtoPIXEL = 0.01875 # camera range inches / 640 (frame width)


def drawDecorations(image, distance):
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


# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #TODO get alliance color from llrobot
    # if llrobot[0] == 1:
    #     color = "Red"
    # else:
    #     color = "Blue"

    #cv2.inRange((hsv_image),(low color range hsv), (high color range hsv))
    if color == "Red":
        img_threshold = cv2.inRange(img_hsv, (0, 100, 100), (10, 255, 255))
    elif color == "Blue":
        img_threshold = cv2.inRange(img_hsv, (100, 100, 100), (140, 255, 255))


    contours, _ = cv2.findContours(img_threshold,
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largestContour = np.array([[]])
    llpython = [0,0,0,0,0,0,0,0]

    if len(contours) > 0:
        cv2.drawContours(image, contours, -1, 255, 2)
        largestContour = max(contours, key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(largestContour)

        cv2.putText(image,
                    'Sample X:  ' + str(x),
                    (550, 650),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 0), 2, cv2.LINE_AA)

        #calculate Strafe Distance
        frameDistance = x - 320
        strafeDistance = (frameDistance * INCHtoPIXEL)

        llpython = [strafeDistance]

        drawDecorations(image, strafeDistance)





    # make sure to return a contour,
    # an image to stream,
    # and optionally an array of up to 8 values for the "llpython"
    # networktables array
    return largestContour, image, llpython