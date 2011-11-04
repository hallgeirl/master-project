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

#figure/crop/%.ps: figure/crop/%.pdf
#	pdf2ps $< $@

#figure/%.pdf: figure/crop/%.ps
#	cat $< | ps2eps -c -n -P | epstopdf --filter > $@

%.pdf: %.tex
	@echo "Compiling $(*F).tex..."
	@- $(TEX) $(*F) > $(*F).stdout
	@if [ -n "`cat $(*F).stdout|egrep -e '^!'`" ]; then\
		echo "\nErrors found!";\
		echo "-------------";\
		cat $(*F).stdout|egrep -A2 -B2 -e '^!' 1>&2;\
	else\
		echo "Compiling bibliography...";\
		bibtex $(*F) > /dev/null; \
		echo "Rerunning pdflatex to fix references...";\
		$(TEX) $(*F) > /dev/null;\
		$(TEX) $(*F) > $(*F).stdout;\
	    if [ -n "`cat $(*F).stdout|grep 'Warning'`" ]; then\
			echo "\nWarnings:";\
			echo "---------";\
			cat $(*F).stdout|grep -B1 'Warning';\
		else\
		    echo "-------------------------------------";\
		    echo "Completed with no errors or warnings!";\
        fi\
    fi

#while grep "Rerun" main.log; do pdflatex $(*F) > /dev/null; done
#pdflatex main
#makeglossaries main
#pdflatex main
#while grep "Rerun" main.log; do pdflatex main; done
#bibtex main
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
