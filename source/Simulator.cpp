#include "Flock.hpp"  // Flocks
#include "Tracer.hpp" // Tracer
#include "Utils.hpp"  // Params
#include "Vec.hpp"    // Vec3D
#include <chrono>     // timing threads
#include <omp.h>      // OpenMP
#include <string>     // cout
#include <vector>     // std::vector

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

    double Tick()
    {
        if (Params.RenderingMovie)
        {
            // Rendering is not part of our problem
            Render();
        }

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
                AllFlocks[i].Act(omp_get_thread_num(), Params.DeltaTime);
            }
            // if we don't parallelize across flocks, then every boid will remain in
            // their initial (singleton) flock
            if (Params.ParallelizeAcrossFlocks)
            {
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
        }
        // convert flock data to processor communications
        Tracer::SaveFlockMatrix(AllFlocks);
        // remove empty (invalid) flocks
        Flock::CleanUp(AllFlocks);
        auto EndTime = std::chrono::system_clock::now();
        std::chrono::duration<double> ElapsedTime = EndTime - StartTime;
        // save tracer data
        Tracer::AddTickT(ElapsedTime.count());
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

/// PSEUDOCODE: http://www.vergenet.net/~conrad/boids/pseudocode.html
/// OVERVIEW: https://cs.stanford.edu/people/eroberts/courses/soco/projects/2008-09/modeling-natural-systems/boids.html
/// EXAMPLE: https://eater.net/boids