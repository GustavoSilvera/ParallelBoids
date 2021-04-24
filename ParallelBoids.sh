#!/bin/bash

# run the entire pipeline for generating a simulator movie

make --silent

mkdir -p Out

./Boids

./CreateMovie.sh

