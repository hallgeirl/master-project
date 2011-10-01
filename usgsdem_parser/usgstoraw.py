#!/usr/bin/python

# Converts files from the USGS DEM format to 16 bit RAW 

import argparse, sys

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

    # Get minimum and maximum elevation
    tmp = record[738:786].split()
    elevation_min = float(tmp[0].replace("D", "E"))
    elevation_max = float(tmp[1].replace("D", "E"))

    # Get spatial resolution
    tmp = record[816:852]
    resolution = (float(tmp[0:12]), float(tmp[12:24]), float(tmp[24:36]))

    tmp = record[852:864].split()
    nrows = int(tmp[0])
    ncols = int(tmp[1])
    
    return {"unit_plane": unit_plane, "unit_elevation": unit_elevation, "elevation_bounds": (elevation_min, elevation_max), "resolution": resolution, "dimensions": (nrows, ncols)}

def parse_B_record(a_record_dict, file_ptr):
    elevations_read = 0
    index = (0, 0)
    dimensions = (0, 0)
    elevation_bounds = (0, 0)
    data = []
    elevation_datum = 0

    record = file_ptr.read(1024)

    # Get index of this profile (b record)
    tmp = record[0:12].split()
    index = (int(tmp[0]), int(tmp[1]))
    
    # Get the number of rows of this B record
    tmp = record[12:24].split()
    dimensions = (int(tmp[0]), int(tmp[1]))

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

    return {"index": index, "dimensions": dimensions, "elevation_bounds": elevation_bounds, "data": data}    

def main(argv):
    parser = argparse.ArgumentParser(description="Converts the USGS DEM format to a 16bit (unsigned short) RAW format.")
    parser.add_argument("-f", "--inputfile", type=str, nargs=1)
    parser.add_argument("-o", "--outputfile", type=str, nargs=1)

    args = vars(parser.parse_args(argv))
    
    outputstream = sys.stdout
    inputstream = sys.stdin

    if args['inputfile']:
        inputstream = file(args['inputfile'][0], "r")

    if args['outputfile']:
        outputstream = file(args['outputfile'][0], "r")

#    if (args['inputfile'] != None):
#        print("Reading from file %s." % args['inputfile'][0])

    #Parse A record (header)
    a_record_dict = parse_A_record(inputstream.read(1024))
    b_records = []

    for i in xrange(a_record_dict["dimensions"][1]):
        b_records.append(parse_B_record(a_record_dict, inputstream))

    return 0;

if __name__ == "__main__":
    if len(sys.argv) > 1:
        exit(main(sys.argv[1:]))
    else:
        exit(main([]))
