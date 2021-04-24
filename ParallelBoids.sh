#!/bin/bash

# run the entire pipeline for generating a simulator movie

rm -rf Out/ || true

make --silent

mkdir -p Out

./Boids

./CreateMovie.sh

