SVGPDF=$(subst .svg,.pdf,$(shell ls *.svg))

.SUFFIXES: .pdf .svg

all: $(SVGPDF)


.svg.pdf:
	inkscape --export-eps=$(subst .svg,.eps,$<) --export-dpi=600 $<
	epstopdf --res=600 $(subst .svg,.eps,$<)
	rm $(subst .svg,.eps,$<)

clean:
	rm -f $(SVGPDF)
