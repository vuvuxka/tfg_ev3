#!/usr/bin/env python3
import time
import sys
from collections import deque
import ev3dev.ev3 as ev3
import parameters
import importlib
from ev3dev.core import GyroSensor
from ev3dev2.motor import LargeMotor, OUTPUT_D, OUTPUT_A, SpeedDPS
import os

def eprint(*args, **kwargs): # Funcion para sacar por terminal
    print(*args, file=sys.stderr, **kwargs)

def offset(): # Funcion para medir el offset inicial
    gMax = -32767
    gMin = 32767
    Gyro.mode = 'GYRO-ANG'
    while abs(gMax - gMin) > 2:
        gyro_suma = 0.0
        offset_gyro = 0.0
        time.sleep(0.5)
        gMax = -32767
        gMin = 32767

        for i in range(40):
            g = Gyro.angle
            offset_gyro += g
            time.sleep(0.05)
            if gMax < g:
                gMax = g
            if gMin > g:
                gMin = g
        offset_gyro = offset_gyro/40

    time.sleep(1)
    Gyro.mode = 'GYRO-RATE'
    return offset_gyro


Gyro = GyroSensor()
Gyro.mode = 'GYRO-RATE'
Gyro.mode = 'GYRO-ANG'
Gyro.mode = 'GYRO-RATE'
os.system('setfont Lat15-TerminusBold32x16')
o = offset()
eprint(str(o))
motorLeft  = LargeMotor(OUTPUT_D)
motorRight = LargeMotor(OUTPUT_A)
motorLeft.position = 0
motorRight.position = 0

angle = 0
n_max = 5
historico = [0] * n_max
tloop = 1.5
posicion = 0
vel = 0
k1 = 26
k2 = 0.2
k3 = 0.275
k4 = 0.0088
s_prev = 0
is_dt = 0

while True:
    tstart= time.time()
    rate = Gyro.rate # Velocidad Angular del Giroscopio (grados/s)

    print(str(rate))
    time.sleep(0.2)
    kp = 0.00442
    ki = 0.0481
    kd = 0.000000190

    angle = angle + rate*tloop # Angulo aproximado (grados)
    posicion = (motorLeft.position + motorRight.position)/2 # Posicion de los motores (grados)
    historico.append(posicion)
    vel = (posicion - historico.pop(0))/n_max*tloop # Velocidad de los motores (grados/s)
    s =  k1*angle + k2*rate + k3*posicion + k4*vel
    if s > 1050.0:
        s = 1050.0
    elif s < -1050.0:
        s = 1050.0
    eprint("k1*"+str(angle)+" + k2*"+str(rate)+" + k3*"+str(posicion)+" + k4*"+str(vel)+" = "+str(s))
    s_dt = (s - s_prev)/tloop
    is_dt = is_dt + s*tloop
    u = (kp*s + ki*s_dt + kd*is_dt)
    eprint("kp*"+str(s)+" + kd*"+str(s_dt)+" + ki*"+str(is_dt)+" = "+str(u))
    motorLeft.on(u)
    motorRight.on(u)
    tloop = (time.time() - tstart) # Tiempo en hacer un bucle

