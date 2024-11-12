import cv2
import numpy as np


def drawDecorations(image):
    cv2.putText(image, 
        'Specimen Alignment!', 
        (0, 455), 
        cv2.FONT_HERSHEY_SIMPLEX, 
        .7, (0, 255, 0), 1, cv2.LINE_AA)
    
# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, (60, 70, 70), (85, 255, 255))
   
    contours, _ = cv2.findContours(img_threshold, 
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
    largestContour = np.array([[]])
    llpython = [0,0,0,0,0,0,0,0]

    if len(contours) > 0:
        cv2.drawContours(image, contours, -1, 255, 2)
        largestContour = max(contours, key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(largestContour)

        llpython = [1,x,y,w,h,9,8,7]  
  
    
    drawDecorations(image)
       
    # make sure to return a contour,
    # an image to stream,
    # and optionally an array of up to 8 values for the "llpython"
    # networktables array
    return largestContour, image, llpython