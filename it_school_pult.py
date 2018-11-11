#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#базовые библиотеки
import os
import time
import sys
import socket
import pickle

#библиотека СКТБ
sys.path.append('../Joystick')
import joystick

IP_ROBOT = '192.168.1.184'
CONTROL_PORT = 8000
SPEED = 100

packetCount = 0 #счетчик пакетов
leftSpeed = 0
rightSpeed = 0
liftYawState = 0
liftPitchState = 0
cameraPosState = 0
lightState = False
parkState = False

controlClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

joy = joystick.Joystick()
joy.open("/dev/input/js0")
print(joy)
time.sleep(2)
joy.start()

def LiftCCW(state):
    global liftYawState
    liftYawState = 2*state

def LiftCW(state):
    global liftYawState
    liftYawState = 1*state

def LiftUp(state):
    global liftPitchState
    liftPitchState = 1*state

def LiftDown(state):
    global liftPitchState
    liftPitchState = 2*state

def LightOnOff(state):
    global lightState
    if state:
        lightState = not lightState
    
def ParkOnOff(state):
    global parkState
    if state:
        parkState = not parkState

joy.onButtonClick('x', LiftCCW)
joy.onButtonClick('b', LiftCW)
joy.onButtonClick('y', LiftUp)
joy.onButtonClick('a', LiftDown)
joy.onButtonClick('mode', ParkOnOff) #центрльная кнопка Logitech
joy.onButtonClick('tr', LightOnOff) #кнопка RB

try:
    while True:
        yAxis = -1*joy.axis['hat0y']
        xAxis = joy.axis['hat0x']
        
        leftSpeed = yAxis*SPEED + xAxis*SPEED/2
        rightSpeed = yAxis*SPEED - xAxis*SPEED/2
        
        msg = pickle.dumps((packetCount, leftSpeed, rightSpeed, liftYawState, liftPitchState, cameraPosState, lightState, parkState))
        #print(len(msg), msg)
        controlClient.sendto(msg, (IP_ROBOT, CONTROL_PORT))
        packetCount += 1
        time.sleep(0.1)
except (KeyboardInterrupt, SystemExit):
    print('Ctrl+C pressed')

controlClient.close()
joy.exit()
print('End program')
