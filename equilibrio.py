#!/usr/bin/env python3
import time
import sys
from collections import deque
import ev3dev.ev3 as ev3
import importlib
import json
import ini
import inout
from ev3dev2.motor import SpeedDPS
import math

########################################################################
## ARCHIVO PARA EQUILIBRAR A WALLY
########################################################################

def equilibrar():
    ini.motorRight.reset()
    g = ini.Gyro.rate # Velocidad Angular del Giroscopio (grados/s)
    time.sleep(0.002)
    g = g + ini.Gyro.rate

    ini.dtheta = g/2.0 - ini.offset_gyro # Velocidad Angular (grados/s)
    ini.offset_gyro = ini.offset_gyro*0.999 + (0.001*(ini.dtheta + ini.offset_gyro)) # Actualizamos el offset
    ini.theta = ini.theta + ini.dtheta*ini.dt # Angulo (grados)
    ini.theta = ini.theta*0.999 - ini.theta*0.001
    inout.eprint('\r' + "Angulo: " + str(ini.theta))
    inout.eprint("Velocidad Angular: " + str(ini.dtheta))

    ini.n = ini.n + 1
    if ini.n == ini.n_max:
        ini.n = 0
    ini.xdes = 0
    ini.x = ini.motorLeft.position + ini.motorRight.position # Posición (deg)
    ini.n_ant = ini.n + 1
    if ini.n_ant == ini.n_max:
        ini.n_ant = 0
    ini.encoder[ini.n] = ini.x # Posicion (rotaciones)
    average = 0
    for i in range(ini.n_max):
        average = average + ini.encoder[i]
        inout.eprint("ini.encoder[" + str(i) + "] = " + str(ini.encoder[i]))
    average = average / ini.n_max
    inout.eprint("average = " + str(average))
    ini.dx = average / ini.dt #Posicion

    # contralador PID
    k1 = 26.5
    k2 = 1.56
    k3 = 0.15
    k4 = 0.13


    ini.e = (k1*ini.theta + k2*ini.dtheta + k3*ini.x + k4*ini.dx)
    ini.de_dt = (ini.e - ini.e_prev)/ini.dt
    inout.eprint("Rotacion del motor = " + str(ini.x))
    inout.eprint("Velocidad del motor = " + str(ini.dx))
    ini.iedt = ini.iedt + ini.e*ini.dt
    ini.e_prev = ini.e
    inout.eprint("p = " + str(ini.e))
    ini.rot_prev = ini.rotacion

    if ini.e > 1050:
        ini.e = 1050
    elif ini.e < -1050:
        ini.e = -1050
    v = int(ini.e/ini.motorLeft.max_speed*100)
    #ini.motorLeft.on(speed = ini.e)
    ini.motorLeft.on(speed=SpeedDPS(ini.e))
    ini.motorRight.on(speed=SpeedDPS(ini.e))
    inout.eprint("speed = " + str(ini.motorLeft.speed))
    inout.eprint("speed = " + str(ini.e))
