#!/bin/bash

mkdir -p PNGs

for f in ./*.ppm
do
    convert $f $f.png
    mv $f.png ./PNGs/
    echo -ne "Converted $f \r"
done
echo -e "\nDone!\n"