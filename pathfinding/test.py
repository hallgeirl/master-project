#!/usr/bin/python

import sqlite3, subprocess, sys, re, os
from datetime import datetime

class Test:
    def __init__(self):
        self.name = ""
        self.executable = ""
        self.path = "./"
        self.makeflags = ""
        self.inputs = [] #Tuples of (tx, ty, probsize, iters)

    def __str__(self):
        return str(self.__dict__)

class Input:
    def __init__(self, width, height, heightmap, density, spacing, testset, _file):
        self.file = _file   #Set to filename if output is already stored in file
        self.density = density
        self.width = width
        self.height = height
        self.heightmap = heightmap
        self.spacing = spacing
        self.testset = testset

    def __str__(self):
        return str(self.__dict__)

class Result:
    def __init__(self, _input, runningtime, width, height, heightmap, cost):
        self.input = _input
        self.rt = runningtime
        self.cost = cost

    def __str__(self):
        return str(self.__dict__)

def run_tests(test, reps):
    results = []
    test_func = None
    make_func = None

    test_func = test_pathfind
    make_func = make_pathfind

    print("Test \"%s\" starts at %s" % (test.name, str(datetime.now())))

    oldinput = None
    
    for i in test.inputs:
        if must_make(i, oldinput, test):
            make_func(test, i)
            oldinput = i
        r = test_func(test, i, os.path.join(test.path, test.executable), reps)
        r.input = i
        results.append(r) 
    print("Test ends at " + str(datetime.now()))

    return results

# Determine if we need to remake, given the next and previous input
def must_make(inext, iprev, test):
    return False

def make_pathfind(test, _input):
    make(test.path, test.makeflags)

def make(path, makeopts):
    print("Building...")
    print("Options: " + "".join(makeopts))

    #clean
    args = ("make -C %s clean" % path).split() 
    p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output,errors = p.communicate()

    if errors != None and errors.strip() != "":
        sys.stderr.write("Make error: " + errors + ". Command: " + " ".join(args) + "\n")

    args = ("make -j4 ").split() + makeopts.split() + ("-C " + path).split()
    p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output,errors = p.communicate()

    if errors != None and errors.strip() != "":
        sys.stderr.write("Make error: " + errors + ". Command: " + " ".join(args) + "\n")

def test_pathfind(test, _input, executable, reps):
 #29     ./$(PROGRAM) -x 4096 -y 4096 -a "-5" -b 842 trondheim4096.raw trondheim
    return test_common(test, _input, reps, "(\S+)\s+(\S+)", os.path.join(test.path, test.executable) + " -x " + str(_input.width) + " -y " + str(_input.height) + " -d " + str(_input.density) + " -s " + str(_input.spacing) + " " + _input.heightmap + " " + _input.heightmap)

def test_common(test, _input, reps, ex, executable):
    if _input.file == None:
        print("Testing " + executable + " with width=" + str(_input.width) + ", height=" + str(_input.height) + ", density=" + str(_input.density) + " on map " + _input.heightmap + ", " + str(reps) + " repetitions.")
    else:   
        print("Fetching output from file " + _input.file)

    runtime_best = 1e15
    cost = 0

    #Regex for timings
    regex = re.compile(ex)

    if _input.file == None:
        args = executable.split(" ")
    else:
        args = ("cat %s " % (_input.file)).split()
        reps = 1
        _input.i = 1

    for j in xrange(reps):
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        output,errors = p.communicate()
        if errors != None and errors.strip() != "":
            sys.stderr.write("Rep " + str(j) + ": " + errors.strip() + "\n")

        matches = regex.findall(output)
        offs = 0
#        print matches
        if _input.file != None:
            offs = 1
            _input.n = int(matches[0][0])

        t = float(matches[0][0+offs])
        c = float(matches[0][1+offs])

        best = False

        if t < runtime_best:
            runtime_best = t
            cost = c
            best = True

        print("Running time: %8.4f, road cost: %8.1f " % (t, c)),
        print("[New best]" if best else "")

    return Result(_input, runtime_best, _input.width, _input.height, _input.heightmap, cost)

def print_help():
    print("Usage:")
    print("./test.py OPTIONS")
    print("Options:")
    print("-o <file>, set output file to <file>")
    print("-i <file>, set input file to <file>")
    print("-r <#>, run for # iterations, use minimum time")
    print("-nots, disable timestamping of output files")

def read_include(filename, width, height, heightmap, spacing, testset):
    inputdata = open(filename, "r")
    out = []

    for i in inputdata:
        if len(i.strip()) == 0 or i.strip()[0] == "#": continue
        inp = i.split()
        inp[0] = inp[0].strip()
        if inp[0] == "width": width = int(inp[1])
        elif inp[0] == "height": height = int(inp[1])
        elif inp[0] == "heightmap": heightmap = ''.join(inp[1:])
        elif inp[0] == "spacing": spacing = int(inp[1])
#        elif inp[0] == "file": out.append( Input(geom[0], geom[1], int(inp[0]), int(inp[1]), perthread, inp[1]))
        else: out.append( Input(width, height, heightmap, int(inp[0]), spacing, testset, None) )

    inputdata.close()

    return out

def main(args):
    inputs = []
    output = None # Output filename
    program = None
    reps = 5
    ignore_timestamp = False
    tx = 16
    ty = 16
    inputfile = "input.in"

    try:
        i = 0
        while i < len(args):
            if args[i] == "-p":
                if args[i+1].strip() not in ["summa", "mm"]:
                    print_help()
                    exit(1)
                program = args[i+1]
                i += 1
            elif args[i] == "-r":
                reps = int(args[i+1])
                i += 1
            elif args[i] == "-i":
                inputfile = args[i+1]
                i += 1
            elif args[i] == "-o":
                output = args[i+1]
                i += 1
            elif args[i] == "-nots":
                ignore_timestamp = True
            else:
                print_help()
                exit(1)
            i += 1
    except Exception as e:
        print e
        print_help()
        exit(1)

    today = datetime.now()

    t = Test()
    inputf = open(inputfile, "r")
    geom = (1,1)
    perthread = 1
    width = 0
    height = 0
    spacing = 0
    heightmap = ""
    testset = "default_set"
    for l in inputf:
        if len(l.strip()) == 0 or l.strip()[0] == "#": continue
        inp = l.strip().split()
        inp[0] = inp[0].strip()
        inp[1] = inp[1].strip()
        if inp[0] == "test":
            t.name = inp[1]
        elif inp[0] == "path":
            t.path = inp[1]
        elif inp[0] == "exec":
            t.executable = inp[1]
        elif inp[0] == "makeflags":
            t.makeflags = " ".join(inp[1:])
        elif inp[0] == "width":
            width = int(inp[1])
        elif inp[0] == "height":
            height = int(inp[1])
        elif inp[0] == "heightmap":
            heightmap = ''.join(inp[1:])
        elif inp[0] == "spacing":
            spacing = int(inp[1])
        elif inp[0] == "testset":
            testset = ''.join(inp[1:])
#        elif inp[0] == "file":
#            t.inputs.append( Input(geom[0], geom[1], 0, 0, perthread, inp[1]))
        elif inp[0] == "include":
            ins = read_include(''.join(inp[1:]), width, height, heightmap, spacing, testset)
            t.inputs += ins
#            print [s.__dict__ for s in t.inputs]
        else:
            t.inputs.append( Input(width, height, heightmap, int(inp[1]), spacing, testset, None) )
    inputf.close()
    
#    print [ s.__dict__ for s in t.inputs]
    if output == None:
        output = "results.db"

    # Redirect stderr to file
    stderr_old = sys.stderr
    sys.stderr = open("errors.out", "w")

    # Run the tests
    results = run_tests(t, reps)

    sys.stderr.close()
    sys.stderr = stderr_old

    # Write results to database
    db = sqlite3.connect(output)
    c = db.cursor()
    c.execute("create table if not exists results (resultid int primary key, name varchar(255), testset varchar(255), width int, height int, heightmap varchar(255), density int, runningtime real, cost real, optimal real, speedup real)")

    # Delete old test results
    c.execute("delete from results where name = '%s'" % (t.name))
    
    for r in results:
        c.execute("insert into results (name, testset, width, height, heightmap, density, runningtime, cost) values ('%s', '%s', %d, %d, '%s', %d, %f, %f)" % (t.name, r.input.testset, r.input.width, r.input.height, r.input.heightmap, r.input.density, r.rt, r.cost))

    db.commit()

    c.close()
    db.close()

if __name__ == "__main__":
    main(sys.argv[1:])
