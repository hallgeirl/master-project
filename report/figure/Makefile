SVGPDF=$(subst .svg,.pdf,$(shell ls *.svg))

.SUFFIXES: .pdf .svg

.PHONY: clean
.PHONY: trajectories
.PHONY: screenshots

all: $(SVGPDF) $(TRAJECTORIES) plots screenshots

screenshots:
	$(MAKE) -C screenshots/

trajectories:
	cp ../../pathfinding/helens256b.raw.png trajectory_helens_before.png 
	cp ../../pathfinding/helens768.raw.png trajectory_helens_after.png 
	cp ../../pathfinding/test4.raw.png trajectory_mountains.png 
	cp ../../pathfinding/trondheim4096.raw.png trajectory_trondheim.png 

.svg.pdf:
	inkscape --export-pdf=$(subst .svg,.pdf,$<) --export-dpi=600 --export-area-drawing $<

plots:
	$(MAKE) -C ../data
	cp ../data/*.pdf ./

clean:
	rm -f $(SVGPDF) $(subst ../data/,,$(shell ls ../data/*.pdf))
	
