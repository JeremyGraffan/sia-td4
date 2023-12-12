report: 
	@pandoc -o rendu.pdf -V colorlinks=true -V linkcolor=blue  README.md
