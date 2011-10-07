#!/usr/bin/python

# Converts files from 16bit raw heightmap to 24bit rgb normal map

import argparse, sys, struct
from math import sqrt
from time import sleep

def main(argv):
    parser = argparse.ArgumentParser(description="Converts a 16bit unsigned short heightmap to a 24bit normal map.")
    parser.add_argument("-f", "--inputfile", type=str, help="Input 16bit RAW file")
    parser.add_argument("-o", "--outputfile", type=str, help="Output normal map file")
    parser.add_argument("-x", "--width", type=int, help="Width of height map in points.")
    parser.add_argument("-y", "--height", type=int, help="Height of height map in points.")
    parser.add_argument("--minheight", default=0, type=int, help="Height represented by the value 0 in the heightmap")
    parser.add_argument("--maxheight", default=65535, type=int, help="Height represented by the value 65535 in the heightmap")
    parser.add_argument("--resx", type=int, default=1, help="Distance between columns")
    parser.add_argument("--resy", type=int, default=1, help="Distance between rows")

    args = vars(parser.parse_args(argv))
 
    must_close_input = False
    must_close_output = False
    outputstream = sys.stdout
    inputstream = sys.stdin

    if args['inputfile']:
        inputstream = file(args['inputfile'], "r")
        must_close_input = True

    if args['outputfile']:
        outputstream = file(args['outputfile'], "wb")
        must_close_output = True

    width = args["width"]
    height = args["height"]

    if not width or not height:
        sys.stderr.write("Height and width must be specified.\n")
        return 1

    minheight = args["minheight"]
    maxheight = args["maxheight"]

    resx = args["resx"]
    resy = args["resy"]

    # Read heightmap data
    heightmap = []
    normalmap = []
    data = inputstream.read(height*width*2)
    for i in xrange(height):
        heightmap.append([])
        normalmap.append([])
        for j in xrange(width):
            base = i*width*2+j*2
            e = float(struct.unpack("H",data[base:base+2])[0])
#            print e
#            sleep(0.2)
            e = (e / 65536.) * (maxheight-minheight) + minheight
            heightmap[i].append(e)
            normalmap[i].append([0, 0, 1])

    for i in xrange(height-1):
        for j in xrange(width-1):
            p1 = (0, 0, heightmap[i][j])
            p2 = (resx, 0, heightmap[i+1][j])
            p3 = (0, resy, heightmap[i][j+1])

            v1 = (p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2])
            v2 = (p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2])
            
            #Take the cross product
            v3 = (v1[1]*v2[2]-v1[2]*v2[1], v1[2]*v2[0]-v1[0]*v2[2], v1[0]*v2[1]-v1[1]*v2[0])
            lenv3 = sqrt(v3[0]**2 + v3[1]**2 + v3[2]**2)
            v3 = [v3[0]/lenv3, v3[1]/lenv3, v3[2]/lenv3]
            #normalmap[i][j] = [heightmap[i][j]/maxheight for k in xrange(3)]
            #normalmap[i][j] = [(i%256)/256., (j%256)/256., 0]
            normalmap[i][j] = v3


    # Write output
    for i in xrange(0, height):
        for j in xrange(0, width):
            n = normalmap[i][j]
            for k in xrange(len(n)):
                n[k] = int((n[k]/2.+0.5)*256.)
#                n[k] = min(max(n[k], -128), 127)
                n[k] = min(max(n[k], 0), 255)

            #print n
#            rgb = struct.pack("ccc", n[0], n[1], n[2])
            rgb = "%c%c%c" % (n[0], n[1], n[2])

            # Pack height into 3 8 bit integers and write it
            outputstream.write(rgb)

            #e *= 256
            #if e > 255:
            #    e = 255
#            outputstream.write('%c%c%c' % (e,e,e))

#    print max_y-min_y, max_x-min_x

    if must_close_output:
        outputstream.close()
    if must_close_input:
        inputstream.close()

    return 0;

if __name__ == "__main__":
    if len(sys.argv) > 1:
        exit(main(sys.argv[1:]))
    else:
        exit(main([]))
