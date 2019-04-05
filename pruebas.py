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

# CONSTANTES #
diametro = 42 # (milimetros)
n_max = 7 # tamaño maximo del array de historico de posiciones
Kp = 0.6
Ki = 14
Kd = 0.005

K_angle = 25
K_rate = 1.3
K_pos = 350
K_vel = 75

deg2rad = 0.0174533

def eprint(*args, **kwargs): # Funcion para sacar por terminal
    print(*args, file=sys.stderr, **kwargs)

def offset(): # Funcion para medir el offset inicial
    Sound.tone(440,1000).wait()
    m = 0
    for i in range(20):
        m = m + Gyro.rate
    m = m/20
    time.sleep(0.1)
    Sound.tone(440,1000).wait()
    time.sleep(0.1)
    Sound.tone(440,1000).wait()
    return m

dt = (time.time() - 2)/2000 # (milisegundos)
radio = diametro/(2*1000) # (metros)
historico = [0] * n_max # (metros)
pos_rel = 0

nowError = False
preError = False
error_cont = 0
error = 0

Gyro = GyroSensor()
Gyro.mode = 'GYRO-RATE'
angle = 0 # (deg)
rate = 0
time.sleep(0.1)
media_angle = offset() # (deg)

motorLeft  = LargeMotor(OUTPUT_D)
motorRight = LargeMotor(OUTPUT_A)
vel = 0
pos = 0
avance = 0
max_acel = 0


while True:
    pos_rel = pos_rel + vel*dt*0.002
    val = (motorLeft.position + motorRight.position)/2 # (deg)
    historico.append(val)
    val_ant = historico.pop()
    deg_vel = (val - val_ant)/n_max*dt
    vel = deg_vel*radio*deg2rad # (metros / s)
    pos = radio*val*deg2rad # (metros)

    val = 0
    for i in range(5):
        val = val + Gyro.rate
    val = val/5
    media_angle = media_angle*(1-dt*0.2) + val*dt*0.2
    rate = val - media_angle # (deg)
    angle = angle + rate*dt # (deg)

    ganancia = K_angle*angle + K_rate*rate + K_pos*(pos-pos_rel) + K_vel*vel

    error_p = ganancia - pos_rel
    error_d = error_d + error_t*dt
    error_i = (error_p - error_0)/dt

    out = error_p*Kp + error_d*Kd + error_i*Ki

    nowError = (abs(out) > 100)
    if nowError & preError:
        error_cont = error_cont + 1
    else:
        error_cont = 0
    if error_cont > 20:
        time.sleep(0.1)
        motorLeft.stop()
        motorRight.stop()
        Sound.tone(800,0.1)
        Sound.tone(600,0.1)
        Sound.tone(300,0.1)
        wait(4)
        exit()
    else:
        preError = nowError

    nuevo_avance = min(max(-50, avance), 50)
    if nuevo_avance == 0:
        if avance == 0:
            sync_0 = motorLeft.position -





    time.sleep(0.002)
    rate = (rate + Gyro.rate)/2 - o


    angle = angle + rate*tloop
    angle = 0.999*angle - 0.001*angle # Angulo aproximado (grados)

    o = o*0.999 + (0.001*(rate+o))
    posicion = (motorLeft.position + motorRight.position)/2 # Posicion de los motores (grados)
    historico.append(posicion)
    vel = (posicion - historico.pop(0))/n_max*tloop # Velocidad de los motores (grados/s)
    s =  k1*angle + k2*rate + k3*posicion + k4*vel
    eprint("k1*"+str(angle)+" + k2*"+str(rate)+" + k3*"+str(posicion)+" + k4*"+str(vel)+" = "+str(s))
    s_dt = (s - s_prev)/tloop
    is_dt = is_dt + s*tloop
    u = (kp*s + ki*s_dt + kd*is_dt)
    if u > 100:
        u = 100
    elif u < -100:
        u = -100
    eprint("kp*"+str(s)+" + kd*"+str(s_dt)+" + ki*"+str(is_dt)+" = "+str(u))
    motorLeft.on(u)
    motorRight.on(u)
    tloop = (time.time() - tstart) # Tiempo en hacer un bucle

