#!/usr/bin/env python3
import time
import sys
from collections import deque
import ev3dev.ev3 as ev3
import importlib
import json
import ini
import equilibrio
from inout import eprint
from ev3dev.ev3 import *

########################################################################
## ARCHIVO INICIAL PRINCIPAL
########################################################################

ini
eprint("- Inicializacion completada")
Sound.beep()

while True:
    equilibrio
    eprint("A")



