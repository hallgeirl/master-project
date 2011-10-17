TEX=pdflatex
OBJ=slides.pdf
TRASH=$(shell ls *.aux *.idx *.log *.toc *.ilg *.ind *.backup *.snm *.out *.nav 2>/dev/null 2>/dev/null)

.SUFFIXES : .tex .pdf

all:${OBJ}

.tex.pdf: style.sty document.tex
	make -C gfx
	${TEX} $<
	${TEX} $<

handout.pdf: document.tex
slides.pdf: document.tex


view:all
	evince  ${OBJ}&

clean:
	rm -f $(OBJ) $(TRASH)

test:view
