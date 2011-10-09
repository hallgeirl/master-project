#!/usr/bin/python

# Converts files from 16bit raw heightmap to 24bit rgb normal map

import argparse, sys, struct, array
from math import sqrt
from time import sleep

def main(argv):
    parser = argparse.ArgumentParser(description="Converts a 16bit unsigned short heightmap to a 24bit normal map.")
    parser.add_argument("-f", "--inputfile", type=str, help="Input 16bit RAW file")
    parser.add_argument("-o", "--outputfile", type=str, help="Output normal map file")
    parser.add_argument("-x", "--width", type=int, help="Width of height map in points.")
    parser.add_argument("-y", "--height", type=int, help="Height of height map in points.")

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

    # Read heightmap data
    heightmap = array.array('H')
    heightmap.fromfile(inputstream, width*height)
    tmp = array.array('B', [0] * width*3)

    # Write output
    for i in xrange(height):
        ii = 0
        if i % 100 == 0:
            sys.stderr.write("Writing line %d...\r" % i);
        base = i*width
        for j in xrange(width):
           # heightmap[i*width+j] = int(float(heightmap[i*width+j])/65536.*256.)
            #c = chr(heightmap[base+j] >> 8)
            c = heightmap[base+j] >> 8
            tmp[ii*3] = c
            tmp[ii*3+1] = c
            tmp[ii*3+2] = c
            ii+=1

        tmp.tofile(outputstream)

    sys.stderr.write("\n")

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
