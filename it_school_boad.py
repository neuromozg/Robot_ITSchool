#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#базовые библиотеки
import os
import time
import sys
import socket
import pickle

#библиотека СКТБ
sys.path.append('../RPiPWM')
import RPiPWM

SERVO_YAW_CHANNEL = 9
SERVO_PITCH_CHANNEL = 10
SERVO_CAMERA_CHANNEL = 11
LIGHT_CHANNEL = 8

MOTOR_LEFT_CHANNEL = 14
MOTOR_RIGHT_CHANNEL = 15

IP_ROBOT = str(os.popen('hostname -I | cut -d\' \' -f1').readline().replace('\n','')) #получаем IP, удаляем \n
CONTROL_PORT = 8000
CONTROL_TIMEOUT = 30

LIFT_PITCH_MAX_POS = 120
LIFT_PITCH_MIN_POS = 55
LIFT_PITCH_PARK_POS = 55

LIFT_YAW_MAX_POS = 172
LIFT_YAW_MIN_POS = 37
LIFT_YAW_PARK_POS = 108

CAMERA_MAX_POS = 165
CAMERA_MIN_POS = 108
CAMERA_PARK_POS = 165

SERVO_STEP = 2

running = True
liftPitchPos = LIFT_PITCH_PARK_POS
liftYawPos = LIFT_YAW_PARK_POS
cameraPos = CAMERA_PARK_POS

cameraParking = False

# функция, которая будет срабатывать при нажатии на кнопку
def ButtonEvent(a):     # обязательно должна иметь один аргумент
    global running
    running = False
    print("Somebody pressed button!")

def SetSpeed(leftSpeed, rightSpeed):
    #print(leftSpeed, rightSpeed)
    motorLeft.setValue(leftSpeed)
    motorRight.setValue(rightSpeed)

def SetLight(state):
    #print('Light', state)
    pass

def SetYaw(state):
    global liftYawPos
    if state == 1: #подъем
        liftYawPos += SERVO_STEP
        if liftYawPos > LIFT_YAW_MAX_POS:
            liftYawPos = LIFT_YAW_MAX_POS
    elif state == 2:
        liftYawPos -= SERVO_STEP
        if liftYawPos < LIFT_YAW_MIN_POS:
            liftYawPos = LIFT_YAW_MIN_POS
    servoYaw.setValue(liftYawPos)
    #print('Yaw', liftYawPos) 


def SetPitch(state):
    global liftPitchPos
    if state == 1: #подъем
        liftPitchPos += SERVO_STEP
        if liftPitchPos > LIFT_PITCH_MAX_POS:
            liftPitchPos = LIFT_PITCH_MAX_POS
    elif state == 2:
        liftPitchPos -= SERVO_STEP
        if liftPitchPos < LIFT_PITCH_MIN_POS:
            liftPitchPos = LIFT_PITCH_MIN_POS
    servoPitch.setValue(liftPitchPos)
    #print('Pitch', liftPitchPos)

def SetCameraPos(state):
    global cameraPos
    if state == 1: #подъем
        cameraPos += SERVO_STEP
        if cameraPos > CAMERA_MAX_POS:
            cameraPos = CAMERA_MAX_POS
    elif state == 2:
        cameraPos -= SERVO_STEP
        if cameraPos < CAMERA_MIN_POS:
            cameraPos = CAMERA_MIN_POS
    servoCamera.setValue(cameraPos)
    #print('Camera', cameraPos)

def CameraParking():
    global cameraParking
    global liftYawPos
    global liftPitchPos
    global cameraPos
    servoYaw.setValue(LIFT_YAW_PARK_POS)
    liftYawPos = LIFT_YAW_PARK_POS
    servoPitch.setValue(LIFT_PITCH_PARK_POS)
    liftPitchPos = LIFT_PITCH_PARK_POS
    servoCamera.setValue(CAMERA_PARK_POS)
    cameraPos = CAMERA_PARK_POS
    cameraParking = True
    
    
#создаем сервориводы
servoYaw = RPiPWM.Servo180(SERVO_YAW_CHANNEL, extended=True) # серва 180 градусов, почему-то моей потребовался широкий диапазон
servoPitch = RPiPWM.Servo180(SERVO_PITCH_CHANNEL, extended=True)
servoCamera = RPiPWM.Servo180(SERVO_CAMERA_CHANNEL, extended=True)

#моторы
motorLeft = RPiPWM.ReverseMotor(MOTOR_LEFT_CHANNEL) # мотор с реверсом
motorRight = RPiPWM.ReverseMotor(MOTOR_RIGHT_CHANNEL) # мотор с реверсом

SetSpeed(0, 0) #останов моторов
CameraParking() #камера в парковочное состояние

#подсветка
light = RPiPWM.Switch(LIGHT_CHANNEL)       # на этом канале будут просто чередоваться высокий и низкий уровни

# создаем объект, который будет работать с АЦП
# указываем опорное напряжение, оно замеряется на первом пине Raspberry (обведено квадратом на шелкографии)
adc = RPiPWM.Battery(vRef=3.28)
adc.start()

# создаем объект для работы с кнопкой и светодиодом
gpio = RPiPWM.Gpio()
gpio.buttonAddEvent(ButtonEvent) # связываем нажатие на кнопку с функцией

controlServer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #создаем UDP сервер
controlServer.bind((IP_ROBOT, CONTROL_PORT))
controlServer.settimeout(CONTROL_TIMEOUT)
print('UDP server listening %s:%d' % (IP_ROBOT, CONTROL_PORT))

try:
    while running:

        try:
            data = controlServer.recvfrom(1024)
        except (socket.timeout):
            print('UDP timeout!')
            running = False
            continue

        #разбираем по переменным принятый пакет
        packetCount, leftSpeed, rightSpeed, liftYawState, liftPitchState, cameraPosState, lightState, parkState = pickle.loads(data[0])

        SetSpeed(leftSpeed, rightSpeed) #задем скорость

        if not parkState: #если не запаркована камера
            if cameraParking:
                cameraParking = False
            if liftYawState:
                SetYaw(liftYawState) #крутим по/против часовой стрелке
            if liftPitchState:
                SetPitch(liftPitchState) #поднимаем/опускаем
                if liftPitchState == 1:
                    SetCameraPos(2)
                else:
                    SetCameraPos(1)
        else:
            if not cameraParking:
                CameraParking() #камера в парковочное состояние

        SetLight(lightState) #подсветка

        voltage = adc.getVoltageFiltered() # получаем напряжение аккумулятора

        print('Packet: %d, Speed: %d-%d, Yaw: %d, Pitch: %d, Camera: %d, Park: %d, Voltage: %0.2fV' %
              (packetCount, leftSpeed, rightSpeed, liftYawPos, liftPitchPos, cameraPos, parkState, voltage))
    
        gpio.ledToggle()    # переключаем светодиод
        
except (KeyboardInterrupt, SystemExit):
    print('Ctrl+C pressed')

controlServer.close()
SetSpeed(0, 0) #останов моторов
CameraParking() #камера в парковочное состояние
adc.stop() # останов замеров напряжения

print('End program')


