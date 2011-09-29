OBJ=$(subst .cpp,.o,$(shell ls *.cpp 2> /dev/null))
LIBS=-lfreeimage
LDFLAGS=$(LIBS)
CXXFLAGS+=-g -Wall -Wno-deprecated -std=c++0x
PROGRAM=raw_crop

all:$(PROGRAM)

$(PROGRAM): $(OBJ)
	$(CXX) $(OBJ) -o $(PROGRAM) $(LDFLAGS)

clean:
	rm -f $(OBJ) $(PROGRAM)

test: all
	gdb --args ./$(PROGRAM) test4.raw test4.png
