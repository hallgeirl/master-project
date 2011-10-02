#!/usr/bin/python

# Converts files from the USGS DEM format to 16 bit RAW 

import argparse, sys, struct

# Some unit conversion constants
foot = 0.3048
arcsec = 30.0

def parse_A_record(record):
    # Units for the planar coordinates and elevation. 1 unit distance is 1 meter.
    unit_plane = 1.0
    unit_elevation = 1.0
    elevation_min = 0
    elevation_max = 0
    resolution = (0,0,0)
    nrows = 0
    ncols = 0
    quadrangle = [(0,0), (0,0), (0,0), (0,0)]

    # Get the planar units
    tmp = int(record[528:534])
    if tmp == 1:
        unit_plane = foot
    elif tmp == 3:
        unit_plane = arcsec

    # Get the elevation units
    tmp = int(record[528:534])
    if tmp == 1:
        unit_elevation = foot

    # Get quadrangle corners
    tmp = record[546:738].replace("D", "E").split()
    quadrangle[0] = (float(tmp[0]), float(tmp[1]))
    quadrangle[1] = (float(tmp[2]), float(tmp[3]))
    quadrangle[2] = (float(tmp[4]), float(tmp[5]))
    quadrangle[3] = (float(tmp[6]), float(tmp[7]))

    # Get minimum and maximum elevation
    tmp = record[738:786].replace("D", "E").split()
    elevation_min = float(tmp[0])
    elevation_max = float(tmp[1])

    # Get spatial resolution
    tmp = record[816:852]
    resolution = (float(tmp[0:12]), float(tmp[12:24]), float(tmp[24:36]))

    tmp = record[852:864].split()
    nrows = int(tmp[0])
    ncols = int(tmp[1])
    
    return {"unit_plane": unit_plane, "unit_elevation": unit_elevation, "elevation_bounds": (elevation_min, elevation_max), "resolution": resolution, "dimensions": (nrows, ncols), "quadrangle": quadrangle}

def parse_B_record(a_record_dict, file_ptr):
    elevations_read = 0
    index = (0, 0)
    dimensions = (0, 0)
    elevation_bounds = (0, 0)
    data = []
    elevation_datum = 0
    first_point = (0,0)

    record = file_ptr.read(1024)

    # Get index of this profile (b record)
    tmp = record[0:12].split()
    index = (int(tmp[0]), int(tmp[1]))
    
    # Get the number of rows of this B record
    tmp = record[12:24].split()
    dimensions = (int(tmp[0]), int(tmp[1]))

    # Get the x,y coordinates of the first point in the profile
    tmp = record[24:72].replace("D","E").split()
    first_point = (float(tmp[0]), float(tmp[1]))

    elevation_datum = float(record[72:96].replace("D", "E"))*a_record_dict["unit_elevation"]

    # Get the elevation bounds
    tmp = record[96:144].replace("D", "E").split()
    elevation_bounds = (float(tmp[0]), float(tmp[1]))

    # Read the elevation data
    elevations = record[144:].split()
    while elevations_read < dimensions[0]*dimensions[1]:
        for e in elevations:
            data.append(float(e)*a_record_dict["resolution"][2]*a_record_dict["unit_elevation"] + elevation_datum)
        elevations_read += len(elevations)        

        if elevations_read < dimensions[0]*dimensions[1]:
            record = file_ptr.read(1024)
            elevations = record.split()

    return {"index": index, "dimensions": dimensions, "first_point": first_point, "elevation_bounds": elevation_bounds, "data": data}    

def main(argv):
    parser = argparse.ArgumentParser(description="Converts the USGS DEM format to a 16bit (unsigned short) RAW format.")
    parser.add_argument("-f", "--inputfile", type=str, nargs=1)
    parser.add_argument("-o", "--outputfile", type=str, nargs=1)
    parser.add_argument("--format", type=str, nargs=1)

    args = vars(parser.parse_args(argv))
 
    must_close_input = False
    must_close_output = False
    outputstream = sys.stdout
    inputstream = sys.stdin

    if args['inputfile']:
        inputstream = file(args['inputfile'][0], "r")
        must_close_input = True

    if args['outputfile']:
        outputstream = file(args['outputfile'][0], "wb")
        must_close_output = True

    #Parse A record (header)
    a_record_dict = parse_A_record(inputstream.read(1024))
    b_records = []

    #Read and parse all B records
    for i in xrange(a_record_dict["dimensions"][1]):
        b_records.append(parse_B_record(a_record_dict, inputstream))

    #Place each elevation point in a grid. But first figure out how many grid points we need to store everything, based on the min and max y coordinates given by the quadrangle.
    #We already know how many columns we have.
    min_ypos = 1e127
    max_ypos = -1e127

    for v in a_record_dict["quadrangle"]:
        if v[1] > max_ypos: max_ypos = v[1]
        if v[1] < min_ypos: min_ypos = v[1]
    
    heightmap = []
    for i in xrange(int((max_ypos - min_ypos)/a_record_dict["resolution"][1])+1):
        heightmap.append([])
        for j in xrange(a_record_dict["dimensions"][1]):
            heightmap[i].append(-1)

    # Organize the data in a height map
    for b in b_records:
        col = b["index"][1] - 1
        row = int((b["first_point"][1]-min_ypos)/a_record_dict["resolution"][1])
        for e in b["data"]:
            heightmap[row][col] = e
            row += 1
    
    # Crop incomplete rows and columns
    min_x = min_y = 0 
    max_x = len(heightmap[0])
    max_y = len(heightmap)

    cropped = False
    while not cropped:
        not_cropped = False
        data_top = data_bottom = data_left = data_right = 0

        # Find the row or column with the lowest #data elements/#empty elements ratio.
        for j in xrange(min_x, max_x):
            if heightmap[min_y][j] == -1: not_cropped = True
            else: data_top += 1
            if heightmap[max_y-1][j] == -1: not_cropped = True
            else: data_bottom += 1

        for i in xrange(min_y, max_y):
            if heightmap[i][min_x] == -1: not_cropped = True
            else: data_left += 1
            if heightmap[i][max_x-1] == -1: not_cropped = True
            else: data_right += 1

        if not_cropped: 
            ratio_top = float(data_top) / float(max_x-min_x)
            ratio_bottom = float(data_bottom) / float(max_x-min_x)
            ratio_left = float(data_left) / float(max_y-min_y)
            ratio_right = float(data_right) / float(max_y-min_y)

            ratios = [(ratio_top,0), (ratio_bottom, 1), (ratio_left, 2), (ratio_right,3)]
            ratios = sorted(ratios, key=lambda r: r[0])

            crop = ratios[0][1]

            if crop == 0: min_y += 1
            elif crop == 1: max_y -= 1
            elif crop == 2: min_x += 1
            elif crop == 3: max_x -= 1

        cropped = not not_cropped

    # Write output
    for i in xrange(min_y, max_y):
        for j in xrange(min_x, max_x):
            #e = int(heightmap[i][j]/a_record_dict["elevation_bounds"][1]*256.)
            e = (heightmap[i][j]-a_record_dict["elevation_bounds"][0])/(a_record_dict["elevation_bounds"][1] - a_record_dict["elevation_bounds"][0])
            # Map to a 16 bit float
            e *= 2**16
            if e >= 2**16:
                e = 2**16-1
            outputstream.write(struct.pack("H", int(e)))
            # Pack height into 16 bit float
#            outputstream.write('%c%c%c' % (e,e,e))

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
