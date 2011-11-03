RESAMPLED_IMAGES = $(patsubst %.png,resampled/%.png,$(shell ls *.png))

.PHONY: clean

all: $(RESAMPLED_IMAGES)

resampled/%.png: %.png
	convert $< -resize 800x512 -quality 105 $@

clean:
	rm -f $(RESAMPLED_IMAGES)

resampled:
	mkdir -p resampled

$(RESAMPLED_IMAGES): resampled
