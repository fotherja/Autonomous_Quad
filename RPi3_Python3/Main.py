# 1) Broad HSV filtering of the line
# 2) Top and bottom segmentation
# 3) Identify main mass 

import Frame_Grabber_Class
import Serial_Stream_Class
import Support
import time
import numpy as np
import cv2
import sys
import math


ss = Serial_Stream_Class.SerialStream().start()
vs = Frame_Grabber_Class.PiVideoStream((400,400), 30).start()
time.sleep(2.0)

# A function that aims to read the frame and generate a mask of everything but the the red line
def Obtain_and_Filter():
    frame = vs.read()
    #frameT = cv2.transpose(frame)
    
    blur1 = cv2.GaussianBlur(frame, (5, 5), 2, 2)
    blur = cv2.bilateralFilter(blur1, 9, 75, 75)

    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    
    lower_thres = np.array([150,80,40])                     # ([154,128,45])
    upper_thres = np.array([220,255,255])                   # ([218,255,255])

    mask = cv2.inRange(hsv, lower_thres, upper_thres)

    kernel = np.ones((5,5),np.uint8)
    erodeImg = cv2.erode(mask, kernel)
    dilateImg = cv2.dilate(erodeImg, kernel) 

    return dilateImg

# Function that takes a mask and finds the angle and offset of the line
def Find_Line(filtered_mask):
    im2,contours,hierarchy = cv2.findContours(filtered_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    r, c = 400, 400
    dp = cv2.cvtColor(filtered_mask, cv2.COLOR_GRAY2BGR)
    cv2.line(dp, (c//2, 1), (c//2, r), (255, 0, 0), 5)

    midPoint = c//2

    maxArea = 0
    offset = 0.0
    angle = 0.0

    for cntrs in contours:
        mu = cv2.moments(cntrs, False)
        x,y,w,h = cv2.boundingRect(cntrs)

        #if mu['m00'] > 120.0:
        if True:
            cx = x + (w//2) # centre point
            area = mu['m00']
            if area > maxArea:
                maxArea = area
                offset = midPoint - cx
                #offset = cx
                angle = math.atan2(w if offset >= 0 else -w, h)
            cv2.rectangle(dp,(x,y),(x+w,y+h),(0,255,0),2)
        else:
            cv2.rectangle(dp,(x,y),(x+w,y+h),(0,0,255),2)

        #print(area, cx, maxArea)

    cv2.imshow("frame", dp)

    angle = int(Support.remap(angle, -math.pi/2, math.pi/2, 0, 255))
    offset = int(127 - Support.remap(offset, 0, c, 0, 255))

    return offset, angle



#----------------------------------Main-------------------------------------------
while(True):
    mask = Obtain_and_Filter()
    Find_Line(mask[10:20,:])

    #cv2.imshow("Frame", dilateImg)   
    cv2.waitKey(5)


cv2.destroyAllWindows()
vs.stop()
ss.stop()







framesSinceStart = 0
startTime = time.time()
while(True):
    ret, frame = cap.read()
    frame = cv2.resize(frame, (400, 300))

    offset, angle = processing(frame)
    print("Offset: " , offset)
    print("Angle: ", angle)

    """
    currentTime = time.time()
    fps = np.int8(np.round(framesSinceStart / (currentTime - startTime)))
    cv2.putText(out, "FPS: " + str(fps), (5, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
    framesSinceStart += 1
    """
    #cv2.imshow('frame',out)
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break

cap.release()
cv2.destroyAllWindows()




    # Take slices of the image at the top and bottom of the frame:
    #Top_Slice = mask[10:20,:]
    #Bottom_Slice = mask[380:390,:]

    # Obtain a greyscale image
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #res = cv2.bitwise_and(gray,gray, mask=mask) 

    # Output to serial port
    #xy = (180, 9)
    #ss.write(xy)
