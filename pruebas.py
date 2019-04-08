#!/usr/bin/env python3
import time
import sys
import math
from collections import deque
import ev3dev.ev3 as ev3
import parameters
import importlib
from ev3dev.core import GyroSensor
from ev3dev2.motor import LargeMotor, OUTPUT_D, OUTPUT_A, SpeedPercent
import os
from ev3dev.ev3 import *
import statistics

# CONSTANTES #
diametro = 42 # (milimetros)
n_max = 7 # tamaño maximo del array de historico de posiciones
Kp = 0.6
Ki = 14
Kd = 0.005
error_d = 0
error_0 = 0
error_i = 0

K_angle = 25
K_rate = 1.3
K_pos = 350
K_vel = 75

deg2rad = 0.0174533

motorLeft  = LargeMotor(OUTPUT_D)
motorRight = LargeMotor(OUTPUT_A)
Gyro = open(ev3.GyroSensor()._path + "/value0", "rb")
motorR = open(motorRight._path + "/position", "rb")
motorL = open(motorRight._path + "/position", "rb")
setMotorR = open(motorRight._path + "/duty_cycle_sp", "w")
setMotorL = open(motorLeft._path + "/duty_cycle_sp", "w")


def eprint(*args, **kwargs): # Funcion para sacar por terminal
    print(*args, file=sys.stderr, **kwargs)
def FastRead(infile):
    infile.seek(0)
    return(int(infile.read().decode().strip()))
def FastWrite(outfile,value):
    outfile.truncate(0)
    outfile.write(str(int(value)))
    outfile.flush()

def offset(): # Funcion para medir el offset inicial
    Sound.tone(440,100).wait()
    time.sleep(0.1)
    g = []
    for i in range(20):
        g.append(FastRead(Gyro))
        time.sleep(0.005)
    m = statistics.mean(g)
    time.sleep(0.1)
    Sound.tone(440,100).wait()
    time.sleep(0.1)
    Sound.tone(440,100).wait()
    return m

dt = (22 - 2)/2000 # (milisegundos)
radio = diametro/(2*1000) # (metros)
historico = [0] * n_max # (metros)
pos_rel = 0
eprint("dt = " + str(dt))

nowError = False
preError = False
error_cont = 0
error = 0

angle = 0 # (deg)
rate = 0 # (deg/s)
time.sleep(0.01)
media_angle = offset() # (deg)

motorLeft.reset()
motorRight.reset()
motorLeft.duty_cycle_sp = 0
motorRight.duty_cycle_sp = 0
motorLeft.run_direct()
motorRight.run_direct()

vel = 0
pos = 0
avance = 0
max_acel = 0
extra_pwr = 0

tiempo = time.time()
t1 = []
t2 = []
t3 = []

while True:
    pos_rel = pos_rel + vel*dt*0.002
    val = (FastRead(motorL) + FastRead(motorR))/2 # (deg)
    historico.append(val)
    val_ant = historico.pop()
    deg_vel = (val - val_ant)/n_max*dt
    vel = deg_vel*radio*deg2rad # (metros / s)
    pos = radio*val*deg2rad # (metros)

    t1.append(time.time()-tiempo)

    val = statistics.mean([FastRead(Gyro) for i in range(5)])
    t2.append(time.time()-tiempo)

    media_angle = media_angle*(1-dt*0.2) + val*dt*0.2
    rate = val - media_angle # (deg)
    angle = angle + rate*dt # (deg)


    ganancia = K_angle*angle + K_rate*rate + K_pos*(pos-pos_rel) + K_vel*vel
    #eprint("k1*"+str(angle)+" + k2*"+str(rate)+" + k3*"+str(pos-pos_rel)+" + k4*"+str(vel)+" = "+str(ganancia))

    error_p = ganancia - pos_rel
    error_i = error_i + error_p*dt
    error_d = (error_0-error_p)/dt
    error_0 = ganancia

    out = error_p*Kp + error_d*Kd + error_i*Ki
    #eprint("kp*"+str(error_p)+" + kd*"+str(error_d)+" + ki*"+str(error_i)+" = "+str(out))

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
        time.sleep(4)
        eprint("t1 = " + str(statistics.mean(t1)))
        eprint("t2 = " + str(statistics.mean(t2)))
        eprint("t3 = " + str(statistics.mean(t3)))
        exit()
    else:
        preError = nowError

    nuevo_avance = min(max(-50, avance), 50)
    if nuevo_avance == 0:
        if avance != 0:
            sync_0 = FastRead(motorR) - FastRead(motorL)
        else:
            sync_0 = 0
        extra_pwr = (FastRead(motorL) - FastRead(motorR) - sync_0)*0.05

    #eprint("extra_pwr = " + str(extra_pwr))
    power1 = out - extra_pwr
    power2 = out + extra_pwr
    avance = nuevo_avance
    p2 = max(min((power2*0.021/radio), 100), -100)
    p1 = max(min((power1*0.021/radio), 100), -100)
    #eprint("powerA = " + str(p2))
    #eprint("powerD = " + str(p1))
    # motorRight.run_forever(speed_pc)
    # motorLeft.run_forever(SpeedPercent(50))
    FastWrite(setMotorR,p2)
    FastWrite(setMotorL,p1)
    now = time.time()
    t3.append((now-tiempo))
    #eprint("tiempo = " + str(now-tiempo))
    while (now - tiempo) < dt:
        time.sleep(0.001)
        #eprint("Esperando...")
    tiempo = time.time()



"""
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

"""
