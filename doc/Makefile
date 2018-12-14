robot.pdf: robot.tex
	pdflatex robot
	bibtex robot
	pdflatex robot
	pdflatex robot

clean:
	rm *-converted-*
	rm *.aux *.bbl *.blg *.log *.out *.toc
	