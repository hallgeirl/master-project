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
    heightmap = []
    data = inputstream.read(height*width*2)
    for i in xrange(height):
        heightmap.append([])
        for j in xrange(width):
            base = i*width*2+j*2
            e = float(struct.unpack("H",data[base:base+2])[0]) / 65536. * 256.;
            heightmap[i].append(e)

    del data

    # Write output
    for i in xrange(0, height):
        for j in xrange(0, width):
            h = min(heightmap[i][j], 255);
            rgb = "%c%c%c" % (h, h, h)

            # Pack height into 3 8 bit integers and write it
            outputstream.write(rgb)

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
