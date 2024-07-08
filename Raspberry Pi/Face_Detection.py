from djitellopy import Tello
import cv2
import numpy as np
import Uart_Test
import Face_Detection

# Initialize Tello drone
tello = Tello()

# Load Haar cascade for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Center coordinates for the face and the video frame
X_Face_center = 176
Y_Face_center = 132
X_frame_center = 176
Y_frame_center = 132

# Target dimensions for the face
Target_w = 75
Target_h = 60

# Font settings for displaying text on the video frame
font = cv2.FONT_HERSHEY_SIMPLEX 
org = (10, 245)
org_2 = (110, 245)  
fontScale = 1
color = (222, 250, 52) 
thickness = 5

# Function to adjust the drone's position based on face offset
def adjust_tello_position(offset_x, offset_y, offset_z, w, h):
    # Adjust horizontal position
    if not -40 <= offset_x <= 40 and offset_x != 0:
        if offset_x < 0:
            tello.move_right(10)
        elif offset_x > 0:
            tello.move_left(10)
    
    # Adjust vertical position
    if not -10 <= offset_y <= 10 and offset_y != -10:
        if offset_y < 0:
            tello.move_up(10)
        elif offset_y > 0:
            tello.move_down(10)
    
    # Adjust forward/backward position based on size of the detected face
    if not 3000 <= offset_z <= 6000 and offset_z != 0:
        if offset_z < 3000:
            tello.move_forward(20)
        elif offset_z > 6000:
            tello.move_backward(20)

# Function to perform face detection and control the drone
def Face_Detection(img):
    # Convert the frame to gray scale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
  
    # Detect faces in the frame
    faces = face_cascade.detectMultiScale(gray, 1.3, 5) 
    
    # Reset face center coordinates
    X_Face_center = 176
    Y_Face_center = 132

    # Iterate over detected faces
    for (x, y, w, h) in faces:
        # Draw a rectangle around the face
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
        
        # Calculate the center of the face
        X_Face_center = x + int(w / 2)
        Y_Face_center = y + int(h / 2)
        
        # Draw a circle at the center of the face
        cv2.circle(img, (X_Face_center, Y_Face_center), 5, (0, 127, 255), 2)
        
        # Calculate offsets from the frame center to the face center
        offset_x = X_frame_center - X_Face_center
        offset_y = Y_frame_center - Y_Face_center
        offset_z = (Target_w - w) * (Target_h - h)
        
        # Adjust drone's position based on the offsets
        adjust_tello_position(offset_x, offset_y, offset_z, w, h)
    
    # Draw a circle at the center of the frame
    cv2.circle(img, (X_frame_center, Y_frame_center), 5, (255, 127, 0), 2)
    
    # Draw a line from the frame center to the face center
    cv2.line(img, (X_frame_center, Y_frame_center), (X_Face_center, Y_Face_center), (0, 255, 0), 2)
    
    # Resize the frame for display
    cv2.resize(img, (128, 160))
    
    # Display direction text based on the offsets
    if (X_frame_center - X_Face_center) > 0:
        cv2.putText(img, 'Right,', org, font, fontScale, color, thickness, cv2.LINE_AA)
    elif (X_frame_center - X_Face_center) < 0:
        cv2.putText(img, 'Left ,', org, font, fontScale, color, thickness, cv2.LINE_AA)
    
    if (Y_frame_center - Y_Face_center) > 0:
        cv2.putText(img, 'Up', org_2, font, fontScale, color, thickness, cv2.LINE_AA)
    elif (Y_frame_center - Y_Face_center) < 0:
        cv2.putText(img, 'Down', org_2, font, fontScale, color, thickness, cv2.LINE_AA)
