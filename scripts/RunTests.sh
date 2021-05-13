#!/bin/bash

set -e

cd ..

make -j4 -B --silent

echo -e "Boids + Global"
./Simulator tests/ParBoids/GlobalN.ini

echo -e "Boids + Local"
./Simulator tests/ParBoids/LocalN.ini

echo -e "Flocks + Global"
./Simulator tests/ParFlocks/GlobalN.ini

echo -e "Flocks + Local"
./Simulator tests/ParFlocks/LocalN.ini