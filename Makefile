report: 
	@pandoc -o rendu.pdf -f markdown+implicit_figures -V colorlinks=true -V linkcolor=blue  README.md 
