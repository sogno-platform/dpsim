BUILDDIR=build

IPYNBS = $(shell find . -name "*.ipynb")

HTMLS = $(patsubst %.ipynb,$(BUILDDIR)/%.html,$(IPYNBS))

all: $(HTMLS)

$(BUILDDIR)/%.html: %.ipynb
	jupyter nbconvert --to html --output-dir $(dir $@) $<
