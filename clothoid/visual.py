#!/usr/bin/python
import sys, subprocess, time
from math import pi, sin, cos, sqrt, acos, tan
from time import sleep
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

def fresnelG(t):
    ans = 0
    for i in xrange(cs_terms):
        ans += gn[i] * (t**(-2*i-1))
#    g = 1./(2+4.142*t+3.492*t*t+6.67*t*t*t)

    
    return ans

def fresnelF(t):
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
        return 0.5-fresnelF(t)*cos(pi*t*t/2.)-fresnelG(t)*sin(pi*t*t/2.)
    

def fresnelC(t):
    c = 0
    if t < 1.6:
        for i in xrange(taylor_terms):
            c += cn[i]*(t**(4*i+1))
        return c
    else:
        return 0.5-fresnelG(t)*cos(pi*t*t/2.)+fresnelF(t)*sin(pi*t*t/2.)

def fresnelC2(t):
    X = sqrt(2.*t/pi)
    return fresnelC(X)

def fresnelS2(t):
    X = sqrt(2.*t/pi)
    return fresnelS(X)

def norm(v):
    return sqrt(v[0]*v[0]+v[1]*v[1])

def dot(v1,v2):
    return v1[0]*v2[0]+v1[1]*v2[1]

def fx(x, k,alpha):
    S = fresnelS2
    C = fresnelC2
#    print x,k,alpha
    return sqrt(x)*(C(x)*sin(alpha) - S(x)*(k + cos(alpha))) + sqrt(alpha-x) * (S(alpha-x)*(1.+k*cos(alpha)) - k*C(alpha-x)*sin(alpha))

def dfx(x,k,alpha):
    S = fresnelS2
    C = fresnelC2
    return (C(x)*sin(alpha) - S(x)*(k+cos(alpha)))/(2.*sqrt(x)) + (k*C(alpha-x)*sin(alpha) - S(alpha-x)*(1+k*cos(alpha)))/(2.*sqrt(alpha-x))

# Find the root of y = f(x) = 0
def newton(y, dy, init, err = 1e-5):
    xnext = init
    xold = None
    while abs(y(xnext)) > err:
#        print abs(y(xnext)), xnext
        xnext, xold = xold, xnext
        xnext = xold - y(xold)/dy(xold)

    return xnext

def transrot(p,dp,rot):
    return (cos(rot)*(p[0]+dp[0]) - sin(rot)*(p[1]+dp[1]), sin(rot)*(p[0]+dp[0]) + cos(rot)*(p[1]+dp[1]))

def rottrans(p,dp,rot):
    return (cos(rot)*(p[0]) - sin(rot)*(p[1])+dp[0], sin(rot)*(p[0]) + cos(rot)*(p[1])+dp[1])

def main(argv):
    C = fresnelC2
    S = fresnelS2
    n = 256
#    ctrlpoints = [(50,50), (200, 50), (60, 200)]
    ctrlpoints = [(0,0), (200, 0), (100, 200)]
    #ctrlpoints = [(50,50), (80, 200), (200, 45)]
    img = [None]*n
    for i in xrange(len(img)):
        img[i] = [255]*n*3

    connectors = [ctrlpoints[0]]
    for i in xrange(1,len(ctrlpoints)-1):
        connectors.append(((ctrlpoints[i+1][0] - ctrlpoints[i][0])/2., (ctrlpoints[i+1][1]- ctrlpoints[i][1])/2.))
    connectors.append(ctrlpoints[-1])
    print connectors

    for i in xrange(1, len(ctrlpoints)-1):
        va = (ctrlpoints[i][0]-ctrlpoints[i-1][0], ctrlpoints[i][1]-ctrlpoints[i-1][1])
        vb = (ctrlpoints[i+1][0]-ctrlpoints[i][0], ctrlpoints[i+1][1]-ctrlpoints[i][1])
        normva = norm(va)
        normvb = norm(vb)
        g = max(normva, normvb)
        h = min(normva, normvb)

        dp = (-ctrlpoints[i+1][0], -ctrlpoints[i+1][1])
        p0 = (0,0)
        if normva < normvb:
            p1 = va
        else:
            p1 = vb

        p1 = transrot(ctrlpoints[i+1], dp, acos(dot(ctrlpoints[i-1], ctrlpoints[i])/normva))

        va = (va[0]/normva, va[1]/normva)
        vb = (vb[0]/normvb, vb[1]/normvb)

        cosalpha= min(max(va[0]*vb[0]+va[1]*vb[1], -1),1)
        alpha = acos(cosalpha)
        
        k=max(normva, normvb)/min(normva, normvb) #normva/normvb
        print "k: %f %f < %f?" % (k, (k+cos(alpha))/sin(alpha), C(alpha)/S(alpha))
        print alpha/2

        fxx = lambda x: fx(x,k,alpha)
        dfxx = lambda x: dfx(x,k,alpha)

        t0 = newton(fxx, dfxx, 0.5*alpha, 1e-10)
        a0 = (g+h*cos(alpha))/(C(t0)+sqrt((alpha-t0)/t0)*(C(alpha-t0)*cos(alpha)+S(alpha-t0)*sin(alpha)))
        a1 = a0*sqrt((alpha-t0)/t0)
        print a0,a1
        print rottrans(transrot((50,20), (-10,-10), -pi/2), (10,10), pi/2)

        t = 0.0
        while t < t0:
            x = a0*fresnelC2(t)
            y = a0*fresnelS2(t)

            t += 0.0001
            if x > n-1 or y > n-1 or x < 0 or y < 0: 
                print x,y
                continue

            for j in xrange(3):
                img[int(y)][int(x)*3+j] = 0
            img[int(y)][int(x)*3] = 255

    for c in ctrlpoints:
        img[c[1]][c[0]*3] = 0
        img[c[1]][c[0]*3+1] = 0
        img[c[1]][c[0]*3+2] = 0

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
   
    sys.stdout.flush()
    raw_input("Press enter.\n")
    p.communicate()

    return 0

if __name__=="__main__":
    exit(main(sys.argv))
