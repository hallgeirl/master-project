#!/usr/bin/python

# Converts files from 16bit raw heightmap to 24bit rgb normal map

import array,argparse, sys, struct, operator
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
    data = array.array('H')
    data.fromfile(inputstream, width*height)
    heightmap = array.array('d', data)
    
    elev_factor = (maxheight-minheight)/65536.
    normalmap = array.array('B', [0]*3*(width-1))

    for i in xrange(height-1):
        if i % 100 == 0:
            sys.stderr.write("Writing line %d...\r" % i);

        base = i*width
        for j in xrange(width-1):
            e1 = float(heightmap[base+j]) * elev_factor + minheight
            e2 = float(heightmap[base+width+j]) * elev_factor + minheight
            e3 = float(heightmap[base+j+1]) * elev_factor + minheight


            p1 = (0, 0, e1)
            p2 = (resx, 0, e2)
            p3 = (0, resy, e3)

            v1 = (p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2])
            v2 = (p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2])
            
            #Take the cross product
            v3 = (v1[1]*v2[2]-v1[2]*v2[1], v1[2]*v2[0]-v1[0]*v2[2], v1[0]*v2[1]-v1[1]*v2[0])
            lenv3 = sqrt(v3[0]**2 + v3[1]**2 + v3[2]**2)
            v3 = [v3[0]/lenv3, v3[1]/lenv3, v3[2]/lenv3]
            print v3[0]*128+128, v3[1]*128+128, v3[2]*128+128
            normalmap[j*3] = int(max(min(v3[0]*128+128, 255),0))
            normalmap[j*3+1] = int(max(min(v3[1]*128+128, 255),0))
            normalmap[j*3+2] = int(max(min(v3[2]*128+128, 255),0))
        normalmap.tofile(outputstream)

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
