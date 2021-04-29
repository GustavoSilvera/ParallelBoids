#include "Boid.hpp"  // Boids
#include "Utils.hpp" // Params
#include "Vec.hpp"   // Vec3D
#include <chrono>    // timing threads
#include <cstdlib>   //
#include <omp.h>     // OpenMP
#include <string>    // cout
#include <vector>    // std::vector

class Simulator
{
  public:
    Simulator()
    {
        Params = GlobalParams.SimulatorParams;
        for (size_t i = 0; i < Params.NumBoids; i++)
        {
            const double x0 = RandD(0, GlobalParams.ImageParams.WindowX, 3);
            const double y0 = RandD(0, GlobalParams.ImageParams.WindowY, 3);
            const double dx0 = RandD(0, GlobalParams.ImageParams.WindowX, 3);
            const double dy0 = RandD(0, GlobalParams.ImageParams.WindowY, 3);
            Boid NewBoid(x0, y0, dx0, dy0, i, i);
            AllBoids.push_back(NewBoid);
            FlockSizes.push_back(1); // every flock is of size 1
        }
        // Print out status
        std::cout << "Running on " << Params.NumBoids << " boids for " << Params.NumIterations << " iterations in a ("
                  << GlobalParams.ImageParams.WindowX << ", " << GlobalParams.ImageParams.WindowY << ") world with "
                  << Params.NumThreads << " threads" << std::endl;

        // initialize image frame
        if (Params.RenderingMovie)
        {
            // only allocate memory if we're gonna use it
            I.Init();
        }
    }
    SimulatorParamsStruct Params;
    std::vector<Boid> AllBoids;
    std::vector<int> FlockSizes;
    Image I;

    void Simulate()
    {
        double ElapsedTime = 0;
        for (size_t i = 0; i < Params.NumIterations; i++)
        {
            ElapsedTime += Tick();
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

#pragma omp parallel for num_threads(Params.NumThreads) schedule(static)
        for (size_t i = 0; i < Params.NumBoids; i++)
        {
            AllBoids[i].Sense(AllBoids, omp_get_thread_num());
        }
#pragma omp parallel for num_threads(Params.NumThreads) schedule(static)
        for (size_t i = 0; i < Params.NumBoids; i++)
        {
            AllBoids[i].Plan(FlockSizes, omp_get_thread_num());
        }
#pragma omp parallel for num_threads(Params.NumThreads) schedule(static)
        for (size_t i = 0; i < Params.NumBoids; i++)
        {
            AllBoids[i].Act(Params.DeltaTime, omp_get_thread_num());
        }

        auto EndTime = std::chrono::system_clock::now();
        std::chrono::duration<double> ElapsedTime = EndTime - StartTime;
        return ElapsedTime.count(); // return wall clock time diff
    }

    void Render()
    {
        // draw all the boids onto the frame
        for (const Boid &B : AllBoids)
        {
            B.Draw(I);
        }
        // draw the target onto the frame
        I.ExportPPMImage();
        I.Blank();
    }
};

int main()
{
    std::srand(0); // consistent seed
    ParseParams("params/params.ini");
    Simulator Sim;
    Sim.Simulate();
    return 0;
}

/// NOTE: resources:
/// PSEUDOCODE: http://www.vergenet.net/~conrad/boids/pseudocode.html
/// OVERVIEW: https://cs.stanford.edu/people/eroberts/courses/soco/projects/2008-09/modeling-natural-systems/boids.html
/// EXAMPLE: https://eater.net/boids