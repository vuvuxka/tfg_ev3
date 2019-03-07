#!/usr/bin/env python3
import time
import sys
from collections import deque
import ev3dev.ev3 as ev3
import importlib
import json
import ini
#from ini import *
import inout

########################################################################
## ARCHIVO PARA EQUILIBRAR A WALLY
########################################################################

def equilibrar():
    g = ini.Gyro.angle
    time.sleep(0.002)
    g = g + ini.Gyro.angle

    ini.dtheta = g/2.0 - ini.offset_gyro
    ini.offset_gyro = ini.offset_gyro*0.999 + (0.001*(ini.dtheta + ini.offset_gyro)) # Actualizamos el offset
    ini.theta = ini.theta + ini.dtheta*ini.dt
    ini.theta = ini.theta*0.999 - ini.theta*0.001
    inout.eprint("ini.theta: " + str(ini.theta))

    if ini.v > ini.velocidad*10.0:
        ini.v = ini.v + ini.aceleracion*10.0*ini.dt
    elif ini.v > ini.velocidad*10.0:
        ini.v = ini.v - ini.aceleracion*10.0*ini.dt
    ini.xdes = ini.xdes + ini.v*ini.dt

    ini.n = ini.n + 1
    if ini.n == ini.n_max:
        ini.n = 0
    ini.encoder[ini.n] = ini.motorLeft.position + ini.motorRight.position + ini.xdes
    ini.n_ant = ini.n + 1
    if ini.n_ant == ini.n_max:
        ini.n_ant = 0
    ini.x = ini.encoder[ini.n]*ini.radio*ini.gra2rad
    ini.dx = (ini.encoder[ini.n] - ini.encoder[ini.n_ant]) / (ini.dt*(ini.n_max - 1))*ini.radio*ini.gra2rad

    # contralador PID
    if ini.velocidad == 0:
        ini.g_dx = 24
        ini.g_x = 700
    else:
        ini.g_dx = 62
        ini.g_x = 750


    ini.e = ini.g_th*ini.theta + ini.g_dth*ini.dtheta + ini.g_x*ini.x + ini.g_dx*ini.dx
    ini.de_dt = (ini.e - ini.e_prev)/ini.dt
    ini.iedt = ini.iedt + ini.e*ini.dt
    ini.e_prev = ini.e
    inout.eprint("ini.e = " + str(ini.e))
    inout.eprint("ini.iedt = " + str(ini.iedt))
    inout.eprint("ini.de_dt = " + str(ini.de_dt))
    ini.u = (ini.kp*ini.e + ini.ki*ini.iedt + ini.kd*ini.de_dt)/ini.radio
    ini.u_rot = ini.rotacion / ini.radio
    ini.rot_prev = ini.rotacion

    #inout.SetDuty(ini.motorDutyCycleRight, ini.u + ini.u_rot)
    #inout.SetDuty(ini.motorDutyCycleLeft, ini.u + ini.u_rot)
    duty = (ini.u + ini.u_rot)*10
    ini.motorLeft.run_forever(speed_sp=duty)
    ini.motorRight.run_forever(speed_sp=duty)

    inout.eprint("ini.u = " + str(ini.u))
    inout.eprint("ini.u_rot = " + str(ini.u_rot))

    """
    //lectura del giroscopio

            motor[motorA] = u + u_rot;
            motor[motorC] = u - u_rot;

            //controlador de rotacion
            if (m &lt; 100) {
                e_rot = of - o;

                //para buscar el menor sentido de giro
                if (e_rot &gt; PI) e_rot = e_rot-(2*PI);
                else if ( e_rot &lt; -PI) e_rot = e_rot + (2*PI);

                //Constante Kp del controlador P
                kp_rot = 6.0;
                if(abs(e_rot) &gt; gra2rad*5) kp_rot = 0.4;
                if(m == 10 || m == 20 || m == 30) kp_rot = 0.20;

                rotacion = e_rot*kp_rot;
            }

            while (time1[T1] &lt; dt*1000.0) {
                wait1Msec(1);
            }

            clearTimer(T1);
            k++;

            if (abs(theta) &gt; 60 || abs(u) &gt; 2000) stopAllTasks();
    """

