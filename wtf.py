#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np
import psutil
import threading
import os
import sys
from pyzbar import pyzbar

#библиотеки для работы с i2c монитором питания INA219
from ina219 import INA219
from ina219 import DeviceRangeError

#библиетека для работы с OLED дисплеем
import Adafruit_SSD1306

#библиотеки для работы с изображениями Python Image Library
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw

#библиотеки СКТБ
sys.path.append('RPicam-Streamer')
import rpicam
import cv_stream

sys.path.append('EduBot/EduBotLibrary')
import edubot

#настройки видеопотока
#FORMAT = rpicam.VIDEO_H264 #поток H264
FORMAT = rpicam.VIDEO_MJPEG #поток MJPEG
WIDTH, HEIGHT = 640, 360
RESOLUTION = (WIDTH, HEIGHT)
FRAMERATE = 30

STATE_READY = 0
STATE_LINE = 1
STATE_LABYRINTH = 2
STATE_SUMO = 3
STATE_CANCEL = 4

#сетевые параметры
#IP = '10.42.0.1' #IP адрес куда отправляем видео
#IP = '173.1.0.41' #IP адрес куда отправляем видео
IP = '192.168.1.76'
RTP_PORT = 5000 #порт отправки RTP видео

SHUNT_OHMS = 0.01 #значение сопротивления шунта на плате EduBot
MAX_EXPECTED_AMPS = 2.0

#Чувтвительность алгоритма определения линии
LINE_SENSITIVITY = 120

#Базовая скорость
BASE_SPEED = 100

def setSpeed(leftSpeed, rightSpeed):
    robot.leftMotor.SetSpeed(leftSpeed)
    robot.rightMotor.SetSpeed(rightSpeed)

#поток для обработки кадров
#параметр 
class FrameHandlerThread(threading.Thread):
    
    def __init__(self, camStream, rtpStream, width, height):
        super(FrameHandlerThread, self).__init__()
        self.daemon = True
        self.rpiCamStream = camStream #получение видео с RPi камеры
        self.rtpStream = rtpStream #отправка RTP потока
        self._frame = None
        self._frameCount = 0
        self._stopped = threading.Event() #событие для остановки потока
        self._newFrameEvent = threading.Event() #событие для контроля поступления кадров
        
        self.debug = False
        self.sensitivity = LINE_SENSITIVITY #чувствительность алгоритма определения линии (0..255)
        self.width = width
        if width > WIDTH:
            self.width = WIDTH
        self.height = height
        if height > HEIGHT:
            self.height = HEIGHT
        self._top = HEIGHT - height #верхняя точка среза
        self._left = (WIDTH - width)//2 #левая точка среза
        self._right = self._left + width #правая точка среза
        self._bottom = HEIGHT #нижняя точка среза
        self.state = STATE_READY
        
    def run(self):
        print('Frame handler started')
        while not self._stopped.is_set():
            if self.rpiCamStream.frameRequest(): #отправил запрос на новый кадр
                self._newFrameEvent.wait() #ждем появления нового кадра
                if not (self._frame is None): #если кадр есть
                
                    #--------------------------------------
                    # тут у нас обрабока кадра self._frame средствами OpenCV
                    gray = cv2.cvtColor(self._frame, cv2.COLOR_BGR2GRAY)  #преобразуем в градации серого
                    barcodes = pyzbar.decode(gray)
                    if len(barcodes) > 0:
                        stateRaw = barcodes[0].data.decode("utf-8")

                        if stateRaw == 'START':
                            if self.state != STATE_LINE:
                                self.state = STATE_LINE
                        elif stateRaw == 'CANCEL':
                            if self.state != STATE_READY:
                                self.state = STATE_READY
                        elif stateRaw == 'SUMO':
                            if self.state != STATE_SUMO:
                                self.state = STATE_SUMO
                        else:
                            print('Unknown barcode:', stateRaw)
                            
                        print('State: %d' % self.state)
                        
                            
                    if self.state == STATE_LINE:
                        # берем нижнюю часть кадра
                        crop = gray[self._top:self._bottom, self._left:self._right]    #обрезаем кадр

                         #вызываем функцию определения линии
                        lineFound, direction, resImg = self.detectLine(crop)

                        leftSpeed = 0
                        rightSpeed = 0
                        if lineFound: # если линия была обнаружена, задем скорости
                            leftSpeed = round(-BASE_SPEED + direction*60)
                            rightSpeed = round(BASE_SPEED + direction*60)

                        setSpeed(leftSpeed, rightSpeed)     #задаем скорости на роботе    
                        print('%.2f\t%d\t%d' % (direction, -leftSpeed, rightSpeed))
                    
                        
                        self.rtpStream.sendFrame(resImg) #помещаем кадр в поток
                        self._frameCount += 1
                        #--------------------------------------
                
                self._newFrameEvent.clear() #сбрасываем событие
            
        print('Frame handler stopped')
        
    def detectLine(self, frame):
        blur = cv2.GaussianBlur(frame, (5, 5), 0)   # размываем изображение blur
        ret, thresh = cv2.threshold(blur, self.sensitivity, 255, cv2.THRESH_BINARY_INV) #бинаризация в ч/б (исходное изобр, порог, максимальное знач., тип бинаризации)
        #_, contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE) #нахождение контуров
        _, contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        direction = 0.0 #курс нашей посудины
        cx = self.width//2 #на случай если не удастся вычислить cx
        cy = self.height
        lineFound = False
        if len(contours) > 0:   # если есть хоть один контур
            lineFound = True
            mainContour = max(contours, key = cv2.contourArea)    # берем максимальный
            M = cv2.moments(mainContour)  # берем его
            if M['m00'] != 0:   # если нет деления на ноль
                cx = int(M['m10']/M['m00'])     # смотрим координаты центра наибольшего черного пятна
                cy = int(M['m01']/M['m00'])     # они получаются в пикселях кадра

                if self.debug:
                    cv2.line(frame, (cx, 0), (cx, self.height), (255, 0, 0), 1)    # рисуем перекрестье на контуре
                    cv2.line(frame, (0, cy), (self.width, cy), (255, 0, 0), 1)
                    
            if self.debug:
                cv2.circle(frame, (self.width//2, self.height//2), 3, (0, 0, 255), -1) #отрисовываем центральную точку
                cv2.drawContours(frame, mainContour, -1, (0, 255, 0), 2, cv2.FILLED) #отображаем контуры на изображении

        direction = cx / (self.width/2) - 1  # преобразуем координаты от 0 до ширина кадра -> от -1 до 1

        gain = cy / self.height + 1 #усиление от 1 до 2
        return lineFound, direction*gain, frame

    def stop(self): #остановка потока
        self._stopped.set()
        if not self._newFrameEvent.is_set(): #если кадр не обрабатывается
            self._frame = None
            self._newFrameEvent.set()
        self.join()

    def setFrame(self, frame): #задание нового кадра для обработки
        if not self._newFrameEvent.is_set(): #если обработчик готов принять новый кадр
            self._frame = frame
            self._newFrameEvent.set() #задали событие
            return True
        return False


def onFrameCallback(frame): #обработчик события 'получен кадр'
    #print('New frame')
    frameHandlerThread.setFrame(frame) #задали новый кадр

robot = edubot.EduBot(1)
assert robot.Check(), 'EduBot not found!!!'
robot.Start() #обязательная процедура, запуск потока отправляющего на контроллер EduBot онлайн сообщений

print ('EduBot started!!!')

ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS) #создаем обект для работы с INA219
ina.configure(ina.RANGE_16V)

disp = Adafruit_SSD1306.SSD1306_128_64(rst = None) #создаем обект для работы c OLED дисплеем 128х64
disp.begin() #инициализируем дисплей

disp.clear() #очищаем дисплей
disp.display() #обновляем дисплей

#проверка наличия камеры в системе
assert rpicam.checkCamera(), 'Raspberry Pi camera not found'
print('Raspberry Pi camera found')

print('OpenCV version: %s' % cv2.__version__)

#создаем трансляцию с камеры (тип потока h264/mjpeg, разрешение, частота кадров, хост куда шлем, функция обрабтчик кадров)
rpiCamStreamer = rpicam.RPiCamStreamer(FORMAT, RESOLUTION, FRAMERATE, (IP, RTP_PORT), onFrameCallback)
#robotCamStreamer.setFlip(False, True) #отражаем кадр (вертикальное отражение, горизонтальное отражение)
#rpiCamStreamer.setRotation(180) #поворачиваем кадр на 180 град, доступные значения 90, 180, 270
rpiCamStreamer.start() #запускаем трансляцию

#отправка служебного cv потока
rtpStreamer = cv_stream.OpenCVRTPStreamer(resolution = RESOLUTION, framerate = FRAMERATE, host = (IP, RTP_PORT+50))
rtpStreamer.start()

#поток обработки кадров    
frameHandlerThread = FrameHandlerThread(rpiCamStreamer, rtpStreamer, int(WIDTH/2), int(HEIGHT/4))
frameHandlerThread.start() #запускаем обработку

#главный цикл программы
try:
    while True:
        # Отрисовываем на картинке черный прямоугольник, тем самым её очищая
        draw.rectangle((0, 0, self._disp.width, self._disp.height), outline=0, fill=0)
            
        #Отрисовываем строчки текста с текущими значениями напряжения, сылы тока и мощности
        draw.text((0, 0), "State: %d" % frameHandlerThread.state, font=font, fill=255)
        draw.text((0, 10), "Voltage: %.2fV" % self._ina.voltage(), font=font, fill=255)
        draw.text((0, 20), "CPU temp: %.2f°C" % rpicam.getCPUtemperature(), font=font, fill=255)
        draw.text((0, 30), "CPU use: %.2f%%" % psutil.cpu_percent(), font=font, fill=255)
        time.sleep(1)
except (KeyboardInterrupt, SystemExit):
    print('Ctrl+C pressed')


#останавливаем поток отправки онлайн сообщений в контроллер EduBot
robot.Release()

#останавливаем обработку кадров
frameHandlerThread.stop()

#останов трансляции c камеры
rpiCamStreamer.stop()
rpiCamStreamer.close()

#останов передачи CV потока
#rtpStreamer.stop()

disp.clear() #очищаем дисплей
disp.display() #обновляем дисплей

print('End program')
