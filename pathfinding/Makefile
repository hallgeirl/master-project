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
#	./$(PROGRAM) -x 4096 -y 4096 -a "-5" -b 842 trondheim4096.raw trondheim
#	eog trondheim.png
	./$(PROGRAM) -x 768 -y 768 -s 10 -d 4 helens768.raw helens
	eog helens.png
#	./$(PROGRAM) test4.raw test4
	#make -C raw_crop
	#./raw_crop/raw_crop test4.raw test4_crop.raw
