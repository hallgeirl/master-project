#!/usr/bin/python
import sqlite3, sys, os

def print_help():
    print("Usage:")
    print("./maketable.py OPTIONS")
    print("Options:")
    print("-i <file>, set input file to <file>")

class File:
    def __init__(self, filename, separator, newline, query):
        self.filename = filename
        self.separator = separator
        self.newline = newline
        self.query = query

def main(args):
    inputfile = None
    dbfile = "results.db"
    preprocess = False

    try:
        i = 0
        while i < len(args):
            if args[i] == "-d":
                dbfile = args[i+1]
                i += 1
            elif args[i] == "-i":
                inputfile = args[i+1]
                i += 1
            elif args[i] == '-p':
                preprocess = True
            else:
                print_help()
                exit(1)
            i += 1
    except Exception as e:
        print e
        print_help()
        exit(1)

    #preprocess db file
    if preprocess:
        db = sqlite3.connect(dbfile)
        c = db.cursor()

        print("Preprocessing DB file...")
    
        # Precision
        #c.execute("update results set precision='double' where name like '%double%' or name='summa'")
        #c.execute("update results set precision='single' where name like '%single%'")

        # Other fields
        #c.execute("update results set speedup=gflops/(select r.gflops from results r where r.tx=1 and r.ty=1 and name=r.name and type=r.type)")
        c.execute("select distinct name, testset, cost, runningtime from results where density=1")
        rows = c.fetchall()
        for row in rows:
            c.execute("update results set speedup=%lf/runningtime where name='%s' and testset='%s'" % (row[3], row[0], row[1]));
            c.execute("update results set optimal=%lf where name='%s' and testset='%s'" % (row[2], row[0], row[1]));

        db.commit()
        db.close()

    if inputfile == None: 
        print("No input file specified.")
        exit(1)

    files = []

    # Read input file
    inputf = open(inputfile, "r")
    for l in inputf:
        if len(l.strip()) == 0 or l.strip()[0] == "#": continue
        inp = l.split()

        # New file
        if inp[0] == "file":
            f = None
            if len(inp) >= 2: 
                f = "".join(inp[1:])
            files.append(File(f, "\t", "", "select * from results"))
        elif inp[0] == "separator":
            files[-1].separator = " ".join(["\t"] + inp[1:] + [" "])
        elif inp[0] == "newline":
            files[-1].newline = " ".join(inp[1:])
        elif inp[0] == "query":
            files[-1].query = " ".join(inp[1:])
        else:
            raise ValueError("Invalid line in input file: " + l)
    inputf.close()


    db = sqlite3.connect(dbfile)
    c = db.cursor()


    for f in files:
        outf = sys.stdout
        if f.filename != None: 
            outf = open(f.filename, "w")
            print "Writing %s..." % f.filename;
        c.execute(f.query)
        for row in c:
            first = True
            for field in row:
                if not first:
                    outf.write(f.separator)
                first = False
                outf.write(str(field))
            outf.write(f.newline + "\n")

    db.close()

if __name__ == "__main__":
    main(sys.argv[1:])
