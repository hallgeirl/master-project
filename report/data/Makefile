RESULTS = results.db
PLOTS = $(subst .p,.pdf,$(shell ls *.p))
INPUTS = tables.in
FILES = $(shell cat tables.in |egrep ^file|sed s/file\ //g)

.PHONY: tables
.PHONY: clean

.SUFFIXES: .pdf .p

all:  $(PLOTS) $(FILES)

$(PLOTS): $(FILES)

$(FILES): $(INPUTS) $(RESULTS)
	./maketable.py -p -i $(INPUTS)

.p.pdf: $(subts .p,.dat,$<)
	gnuplot $<|epstopdf -f > $@

$(RESULTS): ../../pathfinding/results.db
	cp ../../pathfinding/results.db ./

clean:
	rm -f results.db $(FILES) $(PLOTS)
