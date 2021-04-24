#!/bin/bash

# run the entire pipeline for generating a simulator movie

# don't need to remove old out directory
# rm -rf Out/ || true

make --silent

mkdir -p Out

./Boids

./CreateMovie.sh

