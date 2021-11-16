import ServoProjectModules.Communication as ServoComModule
from ServoProjectModules.Communication import createRobot
from ServoProjectModules.Communication import pi

import numpy as np
import scipy.signal
import threading
import time
import math
import random
import matplotlib.pyplot as plt
from matplotlib.backends.backend_gtk3agg import (
    FigureCanvasGTK3Agg as FigureCanvas)
from matplotlib.figure import Figure
import numba
import re
import ServoProjectModules.GuiHelper as GuiFunctions
from ServoProjectModules.GuiHelper import GLib, Gtk

def shrinkArray(a, size, median = False):
    if len(a) <= size:
        return a

    newA = []

    indexScale = len(a) / size

    index = 0

    while index < len(a):
        nextIndex = index + indexScale
        if nextIndex > len(a):
            nextIndex = len(a)

        sortedSubA = sorted(a[int(index): int(nextIndex)])
        if median and len(sortedSubA) > 1:
            i = int(len(sortedSubA) / 2 - 0.49)
            sortedSubA = sortedSubA[i: -i]

        newA.append(sum(sortedSubA) / len(sortedSubA))
        index = nextIndex

    return newA

def intArrayToString(a):
    string = "{"
    first = True
    for v in a:
        if not first:
            string += ", "
        first = False
        string += str(int(round(v)))
    string += "};"
    return string

def printAsEigenInit(mat, indent = ""):
    string = ""
    firstI = True
    for i in mat:
        if firstI:
            firstI = False
        else:
            string += ",\n" + indent
        firstJ = True
        for j in i:
            if firstJ:
                firstJ = False
            else:
                string += ", "
            string += str(j) + "f"
    string += ";\n"
    return string

def sign(v):
    if v >= 0:
        return 1.0
    return -1.0
