# ParallelBoids

## Project for 15-418 (Spring 2021)
### Gustavo Silvera & Elan Biswas
#### Carnegie Mellon University


## Demo output with Boids coloured by FlockID
![DemoImage](writeup/demo.gif)

## Read our reports
- The [Final Report](writeup/final/final_report.pdf) discusses all major findings and technologies
- The [Checkpoint](writeup/checkpoint/checkpoint.pdf) showcases our transition to this new Boids project
- The [Initial Proposal (Irrelevant)](writeup/proposal/proposal.pdf) was for our initial idea which we switched from

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

## Using Cuda
We also provide a `CUDA` implementation of the simulator, you'll need `cuda` installed and `nvcc` available
```bash
# in ParallelBoids/
make -j4 cuda
# run executable
./CudaSimulator
```

## Editing Params
Parameters to the program (such as #boids & #threads) can be tuned at runtime (does not require recompilation) by editing `params.ini` in `params/params.ini`

An example is shown below:
```ini
[Simulator]
num_boids=10000 # edit the number of boids
num_iters=200   # edit how long the simulator runs
num_threads=8   # how many threads are running the code
render=true     # whether or not to render the scene (adds overhead)
timestep=0.55   # global timestep for all boids (increase to make time faster)
par_flocks=true # whether or not to parallelize across flocks (vs boids)

[Boids]
boid_radius=2.0         # how large (in pixels) the boids are
cohesion=0.5            # the cohesion parameter of the boids
alignment=0.5           # the alignment parameter of the boids
separation=0.5          # the separation parameter of the boids
collision_radius=5      # the distance where boids start separating
neighbourhood_radius=10 # the distance where boids consider other boids as neighbours
max_vel=20              # maximum velocity of the boids
colour_mode=flock       # colour the boids by flock idx or thread idx

[Flocks]
use_flocks=true             # whether or not to update the flocks
max_size=500                # maximum size of the flocks
max_flock_delegation=3      # max flock delegation is depracated
is_local_neighbourhood=true # store boids in local vectors per flock (vs one giant shared one)
weight_flock_size=0.1       # how much boids weigh flock sizes when transisioning
weight_flock_dist=0.9       # how much boids weigh flock distance when transisioning

[Image]
window_x=1000 # horizontal size of the output image 
window_y=1000 # vertical size of the output image 

[Trace]
track_mem=false        # whether the tracer should track memory (broken)
track_tick_t=true      # whether the tracer should track tick timing 
track_flock_sizes=true # whether the tracer should track flock sizes 

```


## Using scripts
```bash
# build, init, and run the program
cd scripts/
./ParallelBoids.sh
```

