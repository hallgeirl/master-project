TEXFILES=$(shell find . -type f -name '*.tex')
BIBFILES=$(shell ls *.bib)
REPORT=main
BBLFILES=$(REPORT).bbl

.PHONY: all
.PHONY: figures
.PHONY: tables

all: $(REPORT).pdf

#figure/crop/%.ps: figure/crop/%.pdf
#	pdf2ps $< $@

#figure/%.pdf: figure/crop/%.ps
#	cat $< | ps2eps -c -n -P | epstopdf --filter > $@

%.pdf: %.tex
	$(MAKE) figures
	$(MAKE) tables
	pdflatex $(*F)
	bibtex $(*F)
	@echo "Rerunning pdflatex to fix references..."
	@pdflatex $(*F) > /dev/null
	@while grep "Rerun" main.log; do pdflatex $(*F) > /dev/null; done
	@#pdflatex main
	@#makeglossaries main
	@#pdflatex main
	@#while grep "Rerun" main.log; do pdflatex main; done
	@#bibtex main
figures:
	$(MAKE) -C figure/

tables:
	make -j1 -C data/

.PHONY: clean
clean:
	rm -f *.aux *.nav *.out *.snm *.toc *.log *.bbl *.idx *.ind *.ilg *.blg *.lot *.lof *~ *.ist *.acn *.glo *.gls *.acr *.alg *.backup *.pdf *.loa
	$(MAKE) -C figure/ clean
	$(MAKE) -C data/ clean

test: all
	evince main.pdf

#Dependencies
$(REPORT).tex: $(TEXFILES) figures tables
$(REPORT).aux: $(TEXFILES) figures tables
$(REPORT).bbl: $(TEXFILES) $(BIBFILES) figures tables
$(REPORT).pdf: $(TEXFILES) $(BIBFILES)
