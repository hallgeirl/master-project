TEXFILES=$(shell find . -type f -name '*.tex')
#FIGURES=$(shell find figure/crop/ -type f -name '*.pdf' | awk '{gsub(/crop\//,"")}; 1')

.PHONY: all

all: main.pdf

#figure/crop/%.ps: figure/crop/%.pdf
#	pdf2ps $< $@

#figure/%.pdf: figure/crop/%.ps
#	cat $< | ps2eps -c -n -P | epstopdf --filter > $@

main.pdf: $(TEXFILES) $(FIGURES)
	pdflatex main
	makeglossaries main
	pdflatex main
	while grep "Rerun" main.log; do pdflatex main; done
	bibtex main
	pdflatex main
	while grep "Rerun" main.log; do pdflatex main; done

.PHONY: clean

clean:
	rm -f *.aux *.nav *.out *.snm *.toc *.log *.bbl *.idx *.ind *.ilg *.blg *.lot *.lof *~ *.ist *.acn *.glo *.gls *.acr *.alg *.backup *.pdf

test:
	evince main.pdf
