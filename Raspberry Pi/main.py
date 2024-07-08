from djitellopy import Tello
import cv2, math, time
import numpy as np
import os
import serial
from Uart_Comminication import*
from Face_Detection import*
from detect import*
# Initialize and connect to the Tello drone
tello = Tello()
tello.connect()

# Start video stream from Tello
tello.streamon()

# Initialize serial communication
ser = serial.Serial("/dev/ttyS0", 115200)
frame_read = tello.get_frame_read()

# Font settings for displaying text on the video frame
font = cv2.FONT_HERSHEY_SIMPLEX 
fontScale = 1
color = (222, 250, 52) 
thickness = 4

# State variables
state = "ok"
Take_Off = 0

# Function to handle different modes of operation
ser = serial.Serial ("/dev/ttyS0",115200 )
frame_read = tello.get_frame_read()
# tello.takeoff()
font = cv2.FONT_HERSHEY_SIMPLEX 
fontScale = 1
color = (222, 250, 52) 
thickness = 4
state="ok"
Take_Off = 0
#Funtion Definations
def Mode_Selector(mode,img,Uart_Data):
    global state
    global Take_Off
    cv2.putText(img,("Mode:"+str(mode)), (0,25), font,fontScale, color, 3, cv2.LINE_AA)
    if mode == "Land":
        # Handle landing mode
        if Take_Off == 1 and state == "ok":
            state = tello.land()
            Take_Off = 0
            if state =="error":
                Take_Off=1
                print("Land")
        else:
            cv2.putText(img,("ERROR"), (125,125), font,fontScale, (0,0,255), 3, cv2.LINE_AA)
    elif mode == "Take_Off":
        cv2.putText(img,("Mode:"+str(mode)), (0,25), font,fontScale, color, 3, cv2.LINE_AA)
        if Take_Off == 0 :
            # Handle takeoff mode
            state = tello.takeoff()
            Take_Off=1
            if state =="error" :
                Take_Off=0
                print("Take Off")
                cv2.putText(img,("ERROR"), (125,125), font,fontScale, (0,0,255), 3, cv2.LINE_AA)
    elif mode == "Manuel_mode" and Take_Off == 1 :
        # Handle manual control mode
        print("Manuel Mode")
        manuel_mode(Uart_Data)
    elif mode == "Face_Track" :
        # Handle face tracking mode
        print("Face Track")
        Face_Detection(img)
    elif mode == "Object_Track":
        # Handle object tracking mode
        Objectdetection(img)
        print("Object Detection") 
    elif mode == "Flip" and  Take_Off == 1 :
        # Handle flip mode
        print("Flip")
        flip(Uart_Data)
# Function to handle flipping of the drone       
def flip(Uart_Data):
    f=""
    x = Uart_Data
    up = int(x[0][4:7])
    Down = int(x[1][4:7])
    CCW = int(x[2][8:11])
    CW = int(x[3][7:10])
    if up >= 140:
        print("front Flip")
        f = "Front"
        #tello.flip("f")
    elif Down >= 140:
        print("Back Flip")
        f = "Back"
        #tello.flip("b")
    elif CCW >= 140:
        print("Left Flip")
        f = "Left"
        #tello.flip("l")
    elif CW >= 140:
        print("Right Flip")
        f = "Right"
        #tello.flip("r")
    cv2.putText(img,("Flip:"+str(f)), (10,245), font,fontScale, color, 3, cv2.LINE_AA) 

while True:
    # Read data from the serial port
    txt = str(ser.readline())
    x = txt.split("/")
    Uart_Data = x
    
    # Get battery status from the drone
    Battery = tello.get_battery()
    
    # Get video frame from the drone and resize it
    img = frame_read.frame
    img = cv2.resize(img, (368, 250))
    
    # Extract mode from UART data
    mode = (x[8][4:-5])
    
    # Handle the selected mode
    Mode_Selector(mode, img, Uart_Data)
    
    # Display battery status on the video frame
    cv2.putText(img, ("Bat=%" + str(Battery)), (225, 245), font, fontScale, color, 3, cv2.LINE_AA)
    
    # Transform image for LCD display
    rotated_fra = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    resized_fra = cv2.resize(rotated_fra, (128, 160))
    framebuffer = "/dev/fb0"
    img_rgb565 = cv2.cvtColor(resized_fra, cv2.COLOR_BGR2RGB)
    img_rgb565 = np.array(img_rgb565, dtype=np.uint16)
    img_rgb565 = ((img_rgb565[:, :, 0] & 0xF8) << 8) | ((img_rgb565[:, :, 1] & 0xFC) << 3) | (img_rgb565[:, :, 2] >> 3)
    img_rgb565 = img_rgb565.astype(np.uint16).tobytes()
    
    # Write the transformed image to the framebuffer
    with open(framebuffer, "wb") as f:
        f.write(img_rgb565)
    
    # Check for ESC key to exit the loop
    #cv2.imshow("img",img)
    key = cv2.waitKey(1) & 0xff
    if key == 27:  # ESC
        break
tello.land()
