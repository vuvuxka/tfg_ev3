#!/usr/bin/env python3
import time
import sys
from collections import deque
import ev3dev.ev3 as ev3
import importlib
import json
import ini
from equilibrio import equilibrar
from inout import eprint
from ev3dev.ev3 import *

########################################################################
## ARCHIVO INICIAL PRINCIPAL
########################################################################

eprint("- Inicializacion completada")
Sound.beep()
time.sleep(0.1)
while True:
    time.sleep(1)
    equilibrar()



