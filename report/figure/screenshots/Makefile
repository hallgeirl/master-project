RESAMPLED_IMAGES = $(subst .png,_resampled.png,$(shell ls *.png|grep -v _resampled.png))

.PHONY: clean

all: $(RESAMPLED_IMAGES)

%_resampled.png: %.png
	convert $< -resize 800x512 $@

clean:
	rm -f $(RESAMPLED_IMAGES)
