TEXFILES=$(shell find . -type f -name '*.tex')
#FIGURES=$(shell find figure/crop/ -type f -name '*.pdf' | awk '{gsub(/crop\//,"")}; 1')

PDF=$(subst .tex,.pdf,$(TEXFILES))

.SUFFIXES: .pdf .tex

all: $(PDF)

#figure/crop/%.ps: figure/crop/%.pdf
#	pdf2ps $< $@

#figure/%.pdf: figure/crop/%.ps
#	cat $< | ps2eps -c -n -P | epstopdf --filter > $@

.tex.pdf:
	pdflatex $<
	pdflatex $<

.PHONY: clean

clean:
	rm -f *.aux *.nav *.out *.snm *.toc *.log *.bbl *.idx *.ind *.ilg *.blg *.lot *.lof *~ *.ist *.acn *.glo *.gls *.acr *.alg *.backup *.pdf

test:
	evince $(PDF)
