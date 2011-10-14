#!/usr/bin/python
import sys, subprocess, time
from math import pi, sin, cos
import math

def g(t):
    g = 1./(2+4.142*t+3.492*t*t+6.67*t*t*t)
    return g

def f(t):
    f = (1.+0.926*t)/(2+1.792*t+3.104*t*t)
    return f

def fresnelS(t):
    return 0.5-f(t)*cos(pi*t*t/2.)-g(t)*sin(pi*t*t/2.)

def fresnelC(t):
    return 0.5+f(t)*cos(pi*t*t/2.)-g(t)*sin(pi*t*t/2.)

def main(argv):
    n = 256

    ctrlpoints = [(50,50), (80, 200), (200, 45)]
    img = [None]*n
    for i in xrange(len(img)):
        img[i] = [None]*n
        for j in xrange(n):
            img[i][j] = [0,0,0]

    t = 0
    while t < 20:
        y = n*fresnelS(t)
        x = n*fresnelC(t)
        if x >=256: x = 255
        if y >=256: y = 255
        t += 0.01
        img[int(y)][int(x)] = [255,0,0]


    for c in ctrlpoints:
        img[c[1]][c[0]] = [0,255,0]

    p = subprocess.Popen(["gnuplot"], stdin=subprocess.PIPE)
    
    p.stdin.write("set size square\n")
    p.stdin.write("set xrange [%d:%d]\n" % (0, n-1))
    p.stdin.write("set yrange [%d:%d]\n" % (0, n-1))
    p.stdin.write("plot [0:%d][0:%d] \"-\" binary array=(%d,%d) format='%%uchar' with rgbimage\n" % (n-1, n-1, n, n))
    for i in xrange(n):
        for j in xrange(n):
            p.stdin.write("%c%c%c" % (img[i][j][0], img[i][j][1], img[i][j][2]));
   
    raw_input("Press enter.\n")
    p.communicate()

    return 0

if __name__=="__main__":
    exit(main(sys.argv))
