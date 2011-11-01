SVGPDF=$(subst .svg,.pdf,$(shell ls *.svg))

.SUFFIXES: .pdf .svg

.PHONY: clean
.PHONY: trajectories

all: $(SVGPDF) $(TRAJECTORIES)

trajectories:
	cp ../../pathfinding/helens256b.raw.png trajectory_helens_before.png 
	cp ../../pathfinding/helens768.raw.png trajectory_helens_after.png 
	cp ../../pathfinding/test4.raw.png trajectory_mountains.png 
	cp ../../pathfinding/trondheim4096.raw.png trajectory_trondheim.png 

.svg.pdf:
	inkscape --export-pdf=$(subst .svg,.pdf,$<) --export-dpi=600 --export-area-drawing $<

clean:
	rm -f $(SVGPDF)
