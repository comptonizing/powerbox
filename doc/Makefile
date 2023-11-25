all: manual/manual.html manual.pdf

manual/manual.html: manual.tex manual.pdf
	latex2html -split 0 manual.tex

manual.pdf: manual.tex
	pdflatex manual.tex
	pdflatex manual.tex
	pdflatex manual.tex

clean:
	rm -rf manual manual.pdf *.aux *.log *.toc
