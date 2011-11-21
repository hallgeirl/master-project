TEXFILES=$(shell find . -type f -name '*.tex')
BIBFILES=$(shell ls *.bib)
REPORT=main
BBLFILES=$(REPORT).bbl
TEXFLAGS = -interaction nonstopmode

TEX=pdflatex $(TEXFLAGS)

.PHONY: all
.PHONY: figures
.PHONY: tables

all: $(REPORT).pdf

%.pdf: %.tex
	rm -f $(*F).stdout
	@echo "Compiling $(*F).tex..."
	@- $(TEX) $(*F) > $(*F).stdout
	@if [ -n "`cat $(*F).stdout|egrep -e '^\!'`" ]; then\
		echo "\nErrors found!";\
		echo "-------------";\
		cat $(*F).stdout|egrep -A2 -B2 -e '^!' 1>&2;\
		rm $@;\
	else\
		echo "Compiling bibliography...";\
		bibtex $(*F) > /dev/null; \
		echo "Rerunning pdflatex to fix references...";\
		$(TEX) $(*F) > /dev/null;\
		$(TEX) $(*F) > $(*F).stdout;\
		if [ -n "`cat $(*F).stdout|egrep -e '^\!'`" ]; then\
			echo "\nErrors found!";\
			echo "-------------";\
			cat $(*F).stdout|egrep -A2 -B2 -e '^!' 1>&2;\
			rm $@;\
	    elif [ -n "`cat $(*F).stdout|grep 'Warning'`" ]; then\
			echo "\nWarnings:";\
			echo "---------";\
			cat $(*F).stdout|grep -B2 'Warning';\
		else\
		    echo "-------------------------------------";\
		    echo "Completed with no errors or warnings!";\
        fi\
    fi

figures:
	$(MAKE) -C figure/

tables:
	make -j1 -C data/

.PHONY: clean
.PHONY: purge

purge: clean
	$(MAKE) -C figure/ clean
	$(MAKE) -C data/ clean

clean:
	rm -f *.aux *.nav *.out *.snm *.toc *.log *.bbl *.idx *.ind *.ilg *.blg *.lot *.lof *~ *.ist *.acn *.glo *.gls *.acr *.alg *.backup *.pdf *.loa

test: all
	evince main.pdf

#Dependencies
$(REPORT).tex: $(TEXFILES) figures tables
$(REPORT).aux: $(TEXFILES) figures tables
$(REPORT).bbl: $(TEXFILES) $(BIBFILES) figures tables
$(REPORT).pdf: $(TEXFILES) $(BIBFILES) 
