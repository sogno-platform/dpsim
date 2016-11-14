NAME=DPsimDoc
TEX=pdflatex
.PHONY: all clean veryclean

all: $(NAME).tex
	$(TEX) $(NAME).tex
	$(TEX) $(NAME).tex
	
clean:
	-rm -rf $(NAME).aux $(NAME).out $(NAME).log
	-rm -rf $(NAME).bbl $(NAME).bcf $(NAME).blg $(NAME).run.xml
	-rm -rf $(NAME).auxlock
	-rm -rf img/*.aux
	
veryclean: clean
	-rm -f $(NAME).pdf
	-rm -f cache/*
