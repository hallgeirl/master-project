RESULTS = results.db
PLOTS = $(subst .p,.pdf,$(shell ls *.p))

.PHONY: tables
.PHONY: clean

.SUFFIXES: .pdf .p

all: tables $(PLOTS)

tables: $(RESULTS)
	./maketable.py -p -i tables.in


.p.pdf:
	gnuplot $<|epstopdf -f > $@

$(RESULTS): ../../pathfinding/results.db
	cp ../../pathfinding/results.db ./

clean:
	rm -f *.tex results.db *.eps *.pdf *.dat
