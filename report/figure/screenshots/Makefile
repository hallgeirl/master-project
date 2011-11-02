RESAMPLED_IMAGES = $(subst .png,_resampled.png,$(shell ls *.png))

all: $(RESAMPLED_IMAGES)

%_resampled.png: %.png
	convert $< -resize 800x512 $@
