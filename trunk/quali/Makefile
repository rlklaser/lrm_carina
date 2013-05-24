LATEX = latex
BIBTEX = bibtex
DVIPS = dvips -t A4 -z -Ppdf -G0
#DVIPS = dvips -t Letter -z -Ppdf -G0
PSPDF = ps2pdf
RM = rm -f
FILE = document

all:
	$(LATEX) $(FILE)
	$(BIBTEX) $(FILE)
	$(LATEX) $(FILE)
	$(LATEX) $(FILE)
	$(DVIPS) $(FILE).dvi -o
	$(PSPDF) $(FILE).ps
	@echo ---------------------------------------------------
	@echo Ps: done.
	rm -f *.aux *.bbl *.blg *.dvi *.lof *.log *.lot *.toc *.tmp
#	acroread $(FILE).pdf
