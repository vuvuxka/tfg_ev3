#!/usr/bin/env python3
import time
import sys
from collections import deque
import ev3dev.ev3 as ev3
import importlib
import json
import inout
from equilibrio import offset

# EV3 Brick
powerSupply = ev3.PowerSupply()
buttons = ev3.Button()

# Inicializacion del Giroscopio
try: # Se prueba un modelo LEGO gyro to Gyro Rate mode
    gyroSensor          = ev3.GyroSensor()
    gyroSensor.mode     = gyroSensor.MODE_GYRO_RATE
    gyroType            = 'LEGO-EV3-Gyro'
except: # Se asume HiTechnic Gyro si LEGO Gyro no se encontro
    gyroSensor          = ev3.Sensor(address="ev3-ports:in2")
    gyroType            = 'HITECHNIC-NXT-Gyro'

# Abrimos el archivo de lectura del Giroscopio
gyroSensorValueRaw  = open(gyroSensor._path + "/value0", "rb")

# Inicializacion del sensor de contacto
touchSensor         = ev3.TouchSensor()
touchSensorValueRaw = open(touchSensor._path + "/value0", "rb")

# Configuracion de los motores
motorLeft  = ev3.LargeMotor('outD')
motorRight = ev3.LargeMotor('outA')

velocidad = 0
aceleracion = 50
rotacion = 0.0
dt = 0.010 #tiempo de iteracion del bucle de control
v = 0.0
k = 0.0

# Inicializaciones de equilibrio
offset_gyro = offset()
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
n= 0
n_ant= 0
encoder[n_max]

