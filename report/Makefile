TEXFILES=$(shell find . -type f -name '*.tex')
BIBFILES=$(shell ls *.bib)

.PHONY: all
.PHONY: figures
.PHONY: tables

all: main.pdf

#figure/crop/%.ps: figure/crop/%.pdf
#	pdf2ps $< $@

#figure/%.pdf: figure/crop/%.ps
#	cat $< | ps2eps -c -n -P | epstopdf --filter > $@

main.pdf: $(TEXFILES) $(BIBFILES) figures tables
	pdflatex main
	makeglossaries main
	pdflatex main
	while grep "Rerun" main.log; do pdflatex main; done
	bibtex main
	pdflatex main
	while grep "Rerun" main.log; do pdflatex main; done

figures:
	$(MAKE) -C figure/

tables:
	$(MAKE) -C data/

.PHONY: clean

clean:
	rm -f *.aux *.nav *.out *.snm *.toc *.log *.bbl *.idx *.ind *.ilg *.blg *.lot *.lof *~ *.ist *.acn *.glo *.gls *.acr *.alg *.backup *.pdf *.loa
	$(MAKE) -C figure/ clean
	$(MAKE) -C data/ clean

test: all
	evince main.pdf
