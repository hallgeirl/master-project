RESULTS = results.db

.PHONY: tables
.PHONY: clean

tables: $(RESULTS)
	./maketable.py -p -i tables.in


$(RESULTS): ../../pathfinding/results.db
	cp ../../pathfinding/results.db ./

clean:
	rm -f *.tex results.db
