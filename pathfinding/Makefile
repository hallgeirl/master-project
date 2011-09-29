OBJ=$(subst .cpp,.o,$(shell ls *.cpp 2> /dev/null))
LIBS=-lfreeimage
LDFLAGS=$(LIBS)
CXXFLAGS+=-g -Wall -Wno-deprecated -std=c++0x
PROGRAM=pathfind

all:$(PROGRAM)

$(PROGRAM): $(OBJ)
	$(CXX) $(OBJ) -o $(PROGRAM) $(LDFLAGS)

clean:
	rm -f $(OBJ) $(PROGRAM)

test: all
	make -C raw_crop
	./raw_crop/raw_crop test4.raw test4_crop.raw
	time ./$(PROGRAM) test4.raw test4.png
	eog test4.png
	#gdb --args ./$(PROGRAM) test4_crop.raw test4.png
