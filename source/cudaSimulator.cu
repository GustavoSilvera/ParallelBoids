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
    int numBoids;

    float* position;
    float* velocity;
    float* flockID; 

    float* flockCOM; 
    float* flockSize;
}


__constant__ cudaGlobalConstants;
float* cudaDevicePosition;
float* cudaDeviceVelocity;
float* cudaDeviceFlockID;
float* cudaDeviceFlockCOM;
float* cudaDeviceFlockSize;

class Simulator
{
  public:
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
     * Sets up the memory required for the cuda device code
     *
     * @return The time it took to setup. 
     */
    double Setup() {

    }

    double Tick()
    {
        // Run our actual problem (boid computation)
        auto StartTime = std::chrono::system_clock::now();

        /// TODO: use omp for (spawns threads) and omp barrier/single

#ifndef NDEBUG
        size_t BoidCount = 0;
        for (auto A : AllFlocks)
        {
            BoidCount += A.Size();
        }
        assert(Params.NumBoids == BoidCount);
#endif
        if (!GlobalParams.FlockParams.UseParFlocks)
        {
            // get all the boids
            std::vector<Boids> &Boids = AllFlocks[0].Neighbourhood;  
            int N = Boids.size();


        }
#pragma omp parallel num_threads(Params.NumThreads) // spawns threads
        {
#pragma omp for schedule(static)
            for (size_t i = 0; i < AllFlocks.size(); i++)
            {
                AllFlocks[i].SenseAndPlan(omp_get_thread_num(), AllFlocks);
            }
#pragma omp barrier
#pragma omp for schedule(static)
            for (size_t i = 0; i < AllFlocks.size(); i++)
            {
                AllFlocks[i].Act(Params.DeltaTime);
            }
#pragma omp barrier
#pragma omp for schedule(static)
            for (size_t i = 0; i < AllFlocks.size(); i++)
            {
                AllFlocks[i].Delegate(omp_get_thread_num(), AllFlocks);
            }
#pragma omp barrier
#pragma omp for schedule(static)
            for (size_t i = 0; i < AllFlocks.size(); i++)
            {
                AllFlocks[i].AssignToFlock(omp_get_thread_num(), AllFlocks);
            }
        }
        // convert flock data to processor communications
        Tracer::SaveFlockMatrix(AllFlocks);
        // remove empty (invalid) flocks
        Flock::CleanUp(AllFlocks);
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