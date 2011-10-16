#!/usr/bin/python
import sys, subprocess, time
from math import pi, sin, cos
import math
import png

fn = [0.318309844, 9.34626e-8, -0.09676631, 0.000606222, 0.325539361, 0.325206461, -7.450551455, 32.20380908, -78.8035274, 118.5343352, -102.4339798, 39.06207702]
gn = [0, 0.101321519, -4.07292e-05, -0.152068115, -0.046292605, 1.622793598, -5.199186089, 7.477942354, -0.695291507, -15.10996796, 22.28401942, -10.8996849]

taylor_terms = 11
cs_terms = 12

cn = [None]*taylor_terms
cn[0] = 1
sn = [None]*taylor_terms
sn[0] = pi/6.


for n in xrange(taylor_terms-1):
    cn[n+1] = -(pi**2) * (4*n+1)*cn[n] / (4*(2*n+1)*(2*n+2)*(4*n+5))
    sn[n+1] = -(pi**2) * (4*n+3)*sn[n] / (4*(2*n+2)*(2*n+3)*(4*n+7))

def g(t):
    ans = 0
    for i in xrange(cs_terms):
        ans += gn[i] * (t**(-2*i-1))
#    g = 1./(2+4.142*t+3.492*t*t+6.67*t*t*t)

    
    return ans

def f(t):
#    f = (1.+0.926*t)/(2+1.792*t+3.104*t*t)
    ans = 0
    for i in xrange(cs_terms):
        ans += fn[i] * (t**(-2*i-1))
    return ans

def fresnelS(t):
    s = 0
    if t < 1.6:
        for i in xrange(taylor_terms):
            s += sn[i]*(t**(4*i+3))
        return s
    else:
        return 0.5-f(t)*cos(pi*t*t/2.)-g(t)*sin(pi*t*t/2.)
    

def fresnelC(t):
    c = 0
    if t < 1.6:
        for i in xrange(taylor_terms):
            c += cn[i]*(t**(4*i+1))
        return c
    else:
        return 0.5-g(t)*cos(pi*t*t/2.)+f(t)*sin(pi*t*t/2.)

def main(argv):
    n = 1024

    ctrlpoints = [(50,50), (80, 200), (200, 45)]
    img = [None]*n
    for i in xrange(len(img)):
        img[i] = [255]*n*3

    t = 0.0
    while t < 200.:
        y = float(n)*fresnelS(t)
        x = float(n)*fresnelC(t)
        t += 0.0001
        if x > n-1 or y > n-1 or x < 0 or y < 0: 
            print x,y
            continue
        for i in xrange(3):
            img[int(y)][int(x)*3+i] = 0

    for c in ctrlpoints:
        img[c[1]][c[0]*3] = 0
        img[c[1]][c[0]*3+1] = 0
#        img[c[1]][c[0]*3+2] = 255

    fout = open("cornu.png", "wb")
    w = png.Writer(n, n)
    w.write(fout, img)
    fout.close()
    
    print "Rendering to gnuplot..."

    p = subprocess.Popen(["gnuplot"], stdin=subprocess.PIPE)
    
    p.stdin.write("set size square\n")
    p.stdin.write("set xrange [%d:%d]\n" % (0, n-1))
    p.stdin.write("set yrange [%d:%d]\n" % (0, n-1))
    p.stdin.write("plot [0:%d][0:%d] \"-\" binary array=(%d,%d) format='%%uchar' with rgbimage\n" % (n-1, n-1, n, n))
    for i in xrange(n):
        for j in xrange(n):
            p.stdin.write("%c%c%c" % (img[i][j*3], img[i][j*3+1], img[i][j*3+2]));
   
    raw_input("Press enter.\n")
    p.communicate()

    return 0

if __name__=="__main__":
    exit(main(sys.argv))
