#!/bin/bash

set -e # fail on error

for f in $(ls Out/*.ppm | sort -V)
do
    convert $f $f.png
    rm $f
    echo -ne "Converted $f \r"
done
echo -e "\nRendering Movie..."
# natural sort
convert $(ls Out/*.png | sort -V) Out/Movie.mp4
echo -e "...Done!"
# clean directory
# for f in Out/*.png
# do
#     rm $f
# done