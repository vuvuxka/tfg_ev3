#!/usr/bin/env python3
import time
import sys
from collections import deque
from ev3dev2.motor import LargeMotor, OUTPUT_D, OUTPUT_A
from ev3dev.core import GyroSensor
import ev3dev.ev3 as ev3
import importlib
import json
import math
import inout

def offset(): # Funcion para medir el offset inicial
    gMax = -32767
    gMin = 32767
    while abs(gMax - gMin) > 2:
        gyro_suma = 0.0
        offset_gyro = 0.0
        motorLeft.reset()
        motorRight.reset()
        time.sleep(0.5)
        gMax = -32767
        gMin = 32767

        for i in range(40):
            g = math.radians(Gyro.angle) # Offset (rad)
            offset_gyro += g
            time.sleep(0.05)
            if gMax < g:
                gMax = g
            if gMin > g:
                gMin = g
        offset_gyro = offset_gyro/40

    time.sleep(1)
    return offset_gyro


# EV3 Brick
buttons = ev3.Button()

# Inicializacion del Giroscopio
try: # Se prueba un modelo LEGO gyro to Gyro Rate mode
    gyroSensor          = ev3.GyroSensor()
    gyroSensor.mode     = gyroSensor.MODE_GYRO_RATE
    gyroType            = 'LEGO-EV3-Gyro'
except: # Se asume HiTechnic Gyro si LEGO Gyro no se encontro
    gyroSensor          = ev3.Sensor(address="ev3-ports:in2")
    gyroType            = 'HITECHNIC-NXT-Gyro'


# Inicializacion del sensor de contacto
touchSensor         = ev3.TouchSensor()
touchSensorValueRaw = open(touchSensor._path + "/value0", "rb")

# Configuracion de los motores
motorLeft  = LargeMotor(OUTPUT_D)
motorRight = LargeMotor(OUTPUT_A)
motorLeft.reset()
motorRight.reset()

# Abrimos el archivo de lectura del Giroscopio
Gyro = GyroSensor()
offset_gyro = offset()
Gyro.mode = 'GYRO-ANG'

velocidad = 0
aceleracion = 50
rotacion = 0.0
dt = 0.010 #tiempo de iteracion del bucle de control
v = 0.0
k = 0.0

# Inicializaciones de equilibrio

# offset_gyro = offset()

# Valores **
kp = 0.00442
ki = 0.0481
kd = 0.000000190

# Ganancias
g_th = 26
g_dth = 0.200
g_x = 750
g_dx = 24

dtheta = 0 # Velocidad angular
theta = 0 # Angulo

xdes = 0.0 # Error con la posicion del destino
n_max = 7
n = 0
x = 0.0
dx = 0
n_ant= 0
encoder = [0] * n_max

g = 0.0
e = 0.0
e_prev = 0.0
e_rot = 0.0
rot_prev = 0.0
de_dt = 0.0
iedt = 0.0
u = 0.0
u_rot = 0.0

# CONSTANTES
diametro = 42.0 # diametro de la rueda en mm
radio = diametro / 2000 # en metros
gra2rad = math.pi / 180.0

