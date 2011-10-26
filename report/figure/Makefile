SVGPDF=$(subst .svg,.pdf,$(shell ls *.svg))

.SUFFIXES: .pdf .svg

all: $(SVGPDF)


.svg.pdf:
	inkscape --export-pdf=$(subst .svg,.pdf,$<) --export-dpi=600 --export-area-drawing $<

clean:
	rm -f $(SVGPDF)
