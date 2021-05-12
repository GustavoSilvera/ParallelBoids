#include "Flock.hpp"  // Flocks
#include "Tracer.hpp" // Tracer
#include "Utils.hpp"  // Params
#include "Vec.hpp"    // Vec3D
#include <chrono>     // timing threads
#include <omp.h>      // OpenMP
#include <string>     // cout
#include <vector>     // std::vector

#include <cuda.h>
#include <cuda_runtime.h>
#include <driver_functions.h>

struct GlobalConstants {
    size_t numBoids;

    float* position;
    float* velocity;
    int* flockID; 
    int* flockSize;
};

__constant__ GlobalConstants cuConstTickParams;
class Simulator
{
  public:
    float* cudaDevicePosition;
    float* cudaDeviceVelocity;
    int* cudaDeviceFlockID;
    int* cudaDeviceFlockSize;

    float* position;
    float* velocity;
    int* flockID;
    int* flockSize;
    Simulator()
    {
        Params = GlobalParams.SimulatorParams;
        // Print out status
        std::cout << "Running on " << Params.NumBoids << " boids for " << Params.NumIterations << " iterations in a ("
                  << GlobalParams.ImageParams.WindowX << ", " << GlobalParams.ImageParams.WindowY << ") world with "
                  << Params.NumThreads << " threads" << std::endl;

        // Initialize neighbourhood layout for flocks before use
        Flock::InitNeighbourhoodLayout();
        // Spawn flocks
        for (size_t i = 0; i < Params.NumBoids; i++)
        {
            AllFlocks.push_back(Flock(i, 1));
        }

        // begin tracking which flocks communicate with which
        Tracer::InitFlockMatrix(AllFlocks.size());

        // initialize image frame
        if (Params.RenderingMovie)
        {
            // only allocate memory if we're gonna use it
            I.Init();
        }
    }
    static SimulatorParamsStruct Params;
    std::vector<Flock> AllFlocks;
    std::vector<int> FlockSizes;
    Image I;

    void Simulate()
    {
        double ElapsedTime = 0;
        for (size_t i = 0; i < Params.NumIterations; i++)
        {
            ElapsedTime += Tick();
            std::cout << "Tick: " << i << "\r" << std::flush; // carriage return, no newline
        }
        std::cout << "Finished simulation! Took " << ElapsedTime << "s" << std::endl;
    }

    /**
     * Inserts the value v in the float2 array represented by arr
     * @pre i < |arr|/2
     */
    void insertFloat2(float* arr, Vec2D v, size_t i) {
        arr[2*i] = v[0];
        arr[2*i+1] = v[1];
    }

    void InitBoidDataArrays() {
        std::vector<Boid> &AllBoids = *(AllFlocks.begin()->Neighbourhood.GetAllBoidsPtr());
        size_t numBoids = AllBoids.size();
        size_t vecSize = sizeof(float) * 2 * numBoids;
        size_t intSize = sizeof(int) * numBoids;

        position = (float*)malloc(vecSize);
        velocity = (float*)malloc(vecSize);
        flockID = (int*)malloc(intSize);
        flockSize = (int*)malloc(intSize);

        for (size_t i=0; i < AllBoids.size(); i++)
        {
            Boid boid = AllBoids[i];
            Vec2D pos = boid.Position;
            Vec2D vel = boid.Velocity;
            int fID = boid.FlockID;
            int fSize = 1;

            insertFloat2(position, pos, i);
            insertFloat2(velocity, vel, i);
            flockID[i] = fID;
            flockSize[i] = fSize;
        }
    }
    /** 
     * Sets up the memory required for the cuda device code
     *
     * @return The time it took to setup. 
     */
    double Setup() {
        auto StartTime = std::chrono::system_clock::now();

        // allocate boid and flock memory
        std::vector<Boid> &AllBoids = *(AllFlocks.begin()->Neighbourhood.GetAllBoidsPtr());
        size_t numBoids = AllBoids.size();
        size_t vecSize = sizeof(float) * 2 * numBoids;
        size_t intSize = sizeof(int) * numBoids;
        cudaMalloc(&cudaDevicePosition, vecSize);
        cudaMalloc(&cudaDeviceVelocity, vecSize);
        cudaMalloc(&cudaDeviceFlockID, intSize);
        cudaMalloc(&flockSize, intSize);

        InitBoidDataArrays();
        
        cudaMemcpy(cudaDevicePosition,position,vecSize,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceVelocity,velocity,vecSize,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceFlockID,flockID,intSize,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceFlockSize,flockSize,intSize,cudaMemcpyHostToDevice);

        // set up constants struct for copying into device memory struct
        GlobalConstants constParams;
        constParams.numBoids = numBoids;
        constParams.position = cudaDevicePosition; 
        constParams.velocity = cudaDeviceVelocity;
        constParams.flockID = cudaDeviceFlockID;
        constParams.flockSize = flockSize;

        cudaMemcpyToSymbol(cuConstTickParams, &constParams, sizeof(GlobalConstants));

        auto EndTime = std::chrono::system_clock::now();
        std::chrono::duration<double> ElapsedTime = EndTime - StartTime;

        return ElapsedTime.count(); // return wall clock time diff
    }

    double Tick()
    {
        // Run our actual problem (boid computation)
        auto StartTime = std::chrono::system_clock::now();
        auto EndTime = std::chrono::system_clock::now();
        std::chrono::duration<double> ElapsedTime = EndTime - StartTime;
        // save tracer data
        Tracer::AddTickT(ElapsedTime.count());

        if (Params.RenderingMovie)
        {
            // Rendering is not part of our problem
            Render();
        }

        return ElapsedTime.count(); // return wall clock time diff
    }
    
    void Render()
    {
        // draw all the boids onto the frame
#pragma omp parallel for num_threads(Params.NumThreads) schedule(static)
        for (size_t i = 0; i < AllFlocks.size(); i++)
        {
            AllFlocks[i].Draw(I);
        }
        // draw the target onto the frame
        I.ExportPPMImage();
        I.Blank();
    }
};

// declaring static variables
SimulatorParamsStruct Simulator::Params;
ImageParamsStruct Image::Params;
TracerParamsStruct Tracer::Params;


// global params struct
ParamsStruct GlobalParams;

int main()
{
    std::srand(0); // consistent seed
    ParseParams("params/params.ini");
    Simulator Sim;
    Sim.Simulate();
    // Dump all tracer data
    Tracer::Dump();
    return 0;
}