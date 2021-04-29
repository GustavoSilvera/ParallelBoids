#!/bin/bash

# run the entire pipeline for generating a simulator movie

set -e

# don't need to remove old out directory
# rm -rf Out/ || true

make -j4 -B --silent

mkdir -p Out

./Simulator

./CreateMovie.sh

