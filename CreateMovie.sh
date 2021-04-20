#!/bin/bash

set -e # fail on error

for f in Out/*.ppm
do
    convert $f $f.png
    rm $f
    echo -ne "Converted $f \r"
done
echo -e "\nRendering Movie..."
convert Out/*.png Out/Movie.mp4
echo -e "...Done!"
# clean directory
for f in Out/*.png
do
    rm $f
done
