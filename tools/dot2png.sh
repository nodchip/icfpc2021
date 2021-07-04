for f in *.dot; do
	echo $f
	dot $f -Tpng > ${f%%.dot}.png
done
