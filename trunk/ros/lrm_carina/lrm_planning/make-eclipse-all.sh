#!/bin/bash
basedir=$(pwd)
for file in $(find . -name 'CMakeLists.txt'); do
	makedir=$(dirname "$file")
	echo $file - $makedir
	cd "$basedir"
	cd "$makedir"
	make eclipse-project
	rm CATKIN_IGNORE
done
