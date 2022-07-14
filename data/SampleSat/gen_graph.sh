#/bin/bash

csv="$1"
for graph in $(cat graph.toml | tomlq -c '.graph[]'); do
	#echo "$graph" | jq -c ".value[]"
	title=$(echo "$graph" | jq -rc ".title")
	typ=$(echo "$graph" | jq -rc ".type")
	unit=$(echo "$graph" | jq -rc ".unit")

	for v in $(echo "$graph" | jq -rc ".value[]"); do
		cname="$typ($v)" # [$unit]"
		header=$(head -n1 "$csv")

		# get column number
		cnum=$(echo "$header" | sed 's/,/\n/g' | sed -n "/$cname/=")
		# generate each plot command
		echo "\"${csv}\" using 1:$cnum with lines title \"${title}($v)\"" >> tmp
	done
	echo "" >> tmp

	# generate script
	f="${title}.plt"
	echo 'set datafile separator ","' > $f
	## concatinate to single plot
	echo -n 'plot ' >> $f
	cat tmp | sed -z 's/\n\n//g' | sed -z 's/\n/, \\\n  /g' >> $f
	rm tmp

	# generate png
	echo "set term png" > tmp
	echo "set output \"imgs/${title}.png\"" >> tmp
	cat tmp $f > tmp.plt
	rm tmp
	cat tmp.plt

	mkdir -p ./imgs
	gnuplot tmp.plt
	rm tmp.plt
done
