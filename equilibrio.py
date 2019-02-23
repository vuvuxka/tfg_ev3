#!/usr/bin/env python3
import time
import sys
from collections import deque
import ev3dev.ev3 as ev3
import importlib
import json
import ini
from ini import *
import inout

########################################################################
## ARCHIVO PARA EQUILIBRAR A WALLY
########################################################################

def offset():
    gyro_suma = 0.0
    offset_gyro = 0.0
    ini.motorLeft.stop()
    ini.motorRight.stop()
    time.sleep(0.5)

    for i in range(40):
        offset_gyro += inout.FastRead(ini.gyroSensorValueRaw)
        time.sleep(0.05)
    offset_gyro = offset_gyro/40

    time.sleep(1)
    return offset_gyro

ini.gyroSensorValueRaw = ini.gyroSensorValueRaw + inout.FastRead(gyroSensorValueRaw)
ini.dtheta = g/2.0 - ini.offset_gyro
ini.offset_gyro = ini.offset_gyro*0.999 + (0.001*(ini.dtheta + ini.offset_gyro)) # Actualizamos el offset
ini.theta = ini.theta + ini.dtheta*ini.dt
ini.theta = ini.theta*0.999 - ini.theta*0.001

if ini.v > ini.velocidad*10.0:
    ini.v = ini.v + ini.aceleracion*10.0*ini.dt
elif ini.v > ini.velocidad*10.0:
    ini.v = ini.v - ini.aceleracion*10.0*ini.dt
ini.xdes = ini.xdes + ini.v*ini.dt


"""
//lectura del giroscopio
		theta = theta + dtheta*dt;
		theta = theta*0.999-theta*0.001;

		//introducimos modelo de movimiento uniforme
		if (v &lt; velocidad*10.0) v = v + aceleracion*10.0*dt;
		else if (v &gt; velocidad*10.0) v = v - aceleracion*10.0*dt;
		xdes =  xdes + v*dt;

		//obtencion de la posicion y la velocidad lineal del movil
		n++;
		if (n==n_max) n=0;
		encoder[n]= (nMotorEncoder(motorA) + nMotorEncoder(motorC) + xdes);
		n_ant = n+1;
		if (n_ant == n_max) n_ant = 0;

		x = encoder[n]*radio*gra2rad;
		dx = (encoder[n] - encoder [n_ant]) / (dt*(n_max-1.0))*radio*gra2rad;


		//controlador PID
		if (velocidad == 0) {
			g_dx = 24;
			g_x = 700;
		}
		else {
			g_dx = 62;
			g_x = 750;
		}


		e= g_th*theta + g_dth*dtheta + g_x*x + g_dx*dx;
		de_dt = (e - e_prev)/dt;
		iedt = iedt + e*dt;
		e_prev = e;
		u = (kp*e + ki*iedt + kd*de_dt)/radio;
		u_rot = rotacion/(radio);
		rot_prev = rotacion;

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

