# ParallelProject

## Project for 15-418 (Spring 2021)
### Gustavo Silvera & Elan Biswas
#### Carnegie Mellon University

## Install
You will need the following installed:
```bash
sudo apt install g++
sudo apt install build-essential
# using openmp for parallelism
sudo apt install libomp-dev
# to convert the .ppm's to .mp4
sudo apt install ffmpeg
```

## Building
```bash
# in ParallelBoids/
make -j4 
# run executable
./Simulator
```

## Using scripts
```bash
# build, init, and run the program
cd scripts/
./ParallelBoids.sh
```

## Editing Params
Parameters to the program (such as #boids & #threads) can be tuned at runtime (does not require recompilation) by editing `params.ini` in `params/params.ini`

