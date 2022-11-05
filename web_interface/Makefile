.PHONY: clean all dist http serve


DIST_FILES=		   \
	assets/site/favicon.ico             \
	assets/site/chota.min.css           \
	assets/site/chart.min.js            \
	assets/site/jquery-3.6.1.min.js     \
	assets/site/logo_white.png          \
	assets/mlx-i2c-stick-overview.png   \
	index.html              \
	interface.css           \
	interface.js            \
	mlx-i2c-stick.html      \
	firmware.html           \
	products.html        

PANDOC_FILES=               \
	products                \
	firmware                \
	mlx-i2c-stick

DIST_DIR=../dist


all: ${DIST_FILES} dist

clean:
	@rm -rfv ${DIST_DIR}
	@rm -f $(patsubst %, %.html, ${PANDOC_FILES})

dist:
	@mkdir -p ${DIST_DIR}/assets/site
	for file in $(DIST_FILES); do \
		cp -vrf $$file ${DIST_DIR}/$$file; \
	done

products.html: products.md
	@echo "pandoc $< => $@"
	@pandoc $< -f markdown+lists_without_preceding_blankline+autolink_bare_uris+hard_line_breaks+smart --mathjax -o $@

mlx-i2c-stick.html: mlx-i2c-stick.md
	@echo "pandoc $< => $@"
	@pandoc $< -f markdown+lists_without_preceding_blankline+autolink_bare_uris+hard_line_breaks+smart --mathjax -o $@

firmware.html: firmware.md firmware_list.md
	@echo "pandoc $^ => $@"	
	@cat $^ | pandoc -f markdown+lists_without_preceding_blankline+autolink_bare_uris+hard_line_breaks+smart --mathjax -o $@


serve:
	python -m http.server

http:
	python -m http.server
