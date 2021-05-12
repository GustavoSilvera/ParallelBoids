#!/bin/bash

# run the entire pipeline for generating a simulator movie

# fail on any error
set -e

# go to the ParallelBoids/ directory
cd ..

# make with 4 processors
make -j4 -B --silent

# ensure an Out directory is made before writing to it
mkdir -p out

# main executable is in ParallelBoids/ directory
./Simulator

# scripts should be run in scripts/
cd scripts 
./CreateMovie.sh

