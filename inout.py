#!/usr/bin/env python3
import time
import sys
from collections import deque
import ev3dev.ev3 as ev3
import importlib
import json

########################################################################
## ARCHIVO FUNCIONES ENTRADA Y SALIDA
########################################################################

def eprint(*args, **kwargs): # Funcion para sacar por pantalla
    print(*args, file=sys.stderr, **kwargs)

def FastRead(infile):
    infile.seek(0)
    return(int(infile.read().decode().strip()))
