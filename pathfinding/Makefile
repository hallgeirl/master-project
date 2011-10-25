OBJ=$(subst .cpp,.o,$(shell ls *.cpp 2> /dev/null))
LIBS=-lfreeimage
LDFLAGS=$(LIBS)
CXXFLAGS+=-g -Wall -Wno-deprecated -std=c++0x -O3
PROGRAM=pathfind
PUB=$(patsubst %,pub/%,$(shell ls *.cpp *.h 2> /dev/null))

all:$(PROGRAM) $(PUB)

$(PROGRAM): $(OBJ)
	$(CXX) $(OBJ) -o $(PROGRAM) $(LDFLAGS)

pub/%:./% 
	@cat fileheader.txt > $@
	@cat $< >> $@
	@echo "$< saved with header as $@."

$(PUB): pub

pub:
	mkdir pub

publish: $(PUB)

clean:
	rm -f $(OBJ) $(PROGRAM) $(PUB)

test: all
	make -C raw_crop
	./raw_crop/raw_crop test4.raw test4_crop.raw
	./$(PROGRAM) test4.raw mntroad
	eog mntroad.png
