from djitellopy import Tello
import cv2, math, time
import numpy as np
import os
import Uart_Test
import serial

tello = Tello()
# Function to convert raw UART values to movement values
def val_to_move(x) :
    x = (int(x)-100)/5
    if x >= 30:
        x = 30
    return x,
# Function to control the Tello drone in manual mode based on UART data
def manuel_mode(Uart_Data):
    x=Uart_Data
    #print(x)
    #["b'", 'Up150', 'Down150', 'CCW_Turn153', 'CW_Turn150', 'Front100', 'Back100', 'Left100', "Right100,modeWaiting\\r\\n'"]
    #print("up :", (x[0][4:7]))
    #print("Down :",(x[1][4:7]))
    #print("CCW :",(x[2][8:11]))
    #print("CW :",(x[3][7:10]))
    #print("Back :",(x[4][5:8]))
    #print("Front :",(x[5][4:7]))
    #print("Left:",(x[6][4:7]))
    #print("Right :",(x[7][5:8]))
    #print("Mode :",(x[8][4:-5]))
    print(val_to_move((x[1][2:5])))
    if val_to_move((x[6][4:7]))>=2 :
        tello.move_forward(x[6][4:7])
    elif(val_to_move((x[5][5:8])))>=2 :
        tello.move_back(x[5][5:8])
    elif(val_to_move((x[7][4:7])))>=2 :
        tello.move_left(x[7][4:7])
    elif(val_to_move((x[8][5:8])))>=2 :
        tello.move_right(x[8][5:8])
    elif(val_to_move((x[4][7:10])))>=2 :
        tello.rotate_clockwise(x[4][7:10])    
    elif(val_to_move((x[3][8:11])))>=2 :
        tello.rotate_counter_clockwise(x[3][8:11])   
    elif(val_to_move((x[1][2:5])))>=2 :
        tello.move_up(x[1][2:5])
    elif(val_to_move((x[2][4:7])))>=2 :
        tello.move_down(x[2][4:7])
