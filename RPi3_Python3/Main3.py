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


# A function that reads the frame and returns top and bottom strip 20px high
def Obtain_and_Strip():
    frame = vs.read()

    Top_Slice = frame[10:30,:]
    Bottom_Slice = frame[370:390,:]

    return Top_Slice, Bottom_Slice, frame


# Generate a mask of everything but the red line
def Filter_and_Mask(img):
    blur1 = cv2.GaussianBlur(img, (5, 5), 2, 2)
    blur = cv2.bilateralFilter(blur1, 9, 75, 75)

    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    lower_thres = np.array([150,80,40])                                                                     # ([154,128,45])
    upper_thres = np.array([220,255,255])                                                                   # ([218,255,255])

    mask = cv2.inRange(hsv, lower_thres, upper_thres)

    kernel = np.ones((5,5),np.uint8)
    erodeImg = cv2.erode(mask, kernel)
    dilateImg = cv2.dilate(erodeImg, kernel)

    return dilateImg


# Function that takes a mask and finds the largest bounding box and returns the centre x coord
def Bounding_Box(filtered_mask):
    im2,contours,hierarchy = cv2.findContours(filtered_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    Box_Center = 200
    maxArea = 0

    for cntrs in contours:
        mu = cv2.moments(cntrs, False)
        x,y,w,h = cv2.boundingRect(cntrs)

        cx = x + (w//2)                                                                                   	# x centre point
        area = mu['m00']

        if area > maxArea and area > 10.0:
            maxArea = area
            Box_Center = cx                                                                               	# store the center coords of the bigest box

    found_flag = False
    if maxArea > 10.0:																						# Return whether a box is found
        found_flag = True

    return Box_Center, found_flag


# return angle and offset given midpoints of bounding box in top & bottom strips
def Get_Angle_and_Offset(X_TS, X_BS):
    X_diff = X_TS - X_BS

    angle = math.atan2(X_diff,360)                                                                          # Opposite / Adjacent

    offset = ((X_TS + X_BS) / 2) - 200

    return angle, offset

def Filter_Coords(X_Coord, Last_Valid_X, Valid):

    if Valid == True:
        Last_Valid_X = X_Coord

    return Last_Valid_X

def Constrain_Values(x, min, max):
    if x < min:
        x = min
    elif x > max:
        x = max

    return x

#----------------------------------Main-------------------------------------------
Last_Valid_Top_X = 200
Last_Valid_Bot_X = 200

while(True):
    Top_Slice, Bottom_Slice, frame = Obtain_and_Strip()                                                     # Get a top and bottom stip of the frame

    Filtered_TS = Filter_and_Mask(Top_Slice)                                                                # Filter and mask these stips
    Filtered_BS = Filter_and_Mask(Bottom_Slice)

    Top_X, Found_Top_X = Bounding_Box(Filtered_TS)                                                          # Find the biggest bulk of unmasked area and return centre x coords
    Bottom_X, Found_Bot_X = Bounding_Box(Filtered_BS)

    Last_Valid_Top_X = Filter_Coords(Top_X, Last_Valid_Top_X, Found_Top_X)
    Last_Valid_Bot_X = Filter_Coords(Bottom_X, Last_Valid_Bot_X, Found_Bot_X)

    angle, offset = Get_Angle_and_Offset(Last_Valid_Top_X, Last_Valid_Bot_X)                                # Calculate angles and offsets

    angle_byte = int(Support.remap(angle, -math.pi/4, math.pi/4, 0, 255))                                   # Remap values for serial port transmission
    offset_byte = int(127 - Support.remap(offset, 0, 400, 0, 255))

    angle_byte_con = Constrain_Values(angle_byte, 0, 254)
    offset_byte_con = Constrain_Values(offset_byte, 0, 254)

    Tx_data = (offset_byte_con, angle_byte_con)                                                             # Output to serial port
    ss.write(Tx_data)

    # Debug code:
    #Debug_Output = np.zeros((400,400), np.uint8)
    #cv2.line(frame, (200, 1), (200, 400), 255, 5)
    #cv2.line(frame, (Last_Valid_Top_X, 20), (Last_Valid_Bot_X, 380), 255, 5)

    #cv2.imshow("Lines", frame)
    #cv2.imshow("Top_Strip", Filtered_TS)
    #cv2.imshow("Bot_Strip", Filtered_BS)

    #print(offset_byte, angle_byte)
    #print(Last_Valid_Top_X, Last_Valid_Bot_X, Found_Top_X, Found_Bot_X)
    cv2.waitKey(1)

cv2.destroyAllWindows()
vs.stop()
ss.stop()
