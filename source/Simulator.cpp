#include "Boid.hpp"
#include "Utils.hpp"
#include "Vec.hpp"
#include <array>
#include <chrono>
#include <cstdlib>
#include <omp.h>
#include <string>
#include <vector>

class Simulator
{
  public:
    Simulator(const size_t N, const size_t P, const double TB, const double DT, const Vec2D &WindowSize)
    {
        NumBoids = N;
        NumThreads = P;
        DeltaTime = DT;
        Time = 0;
        for (size_t i = 0; i < N; i++)
        {
            const double x0 = std::rand() % int(WindowSize[0]);
            const double y0 = std::rand() % int(WindowSize[1]);
            const double dx0 = std::rand() % int(WindowSize[0]);
            const double dy0 = std::rand() % int(WindowSize[1]);
            Boid NewBoid(x0, y0, dx0, dy0, i, i, WindowSize);
            AllBoids.push_back(NewBoid);
            FlockSizes.push_back(1); // every flock is of size 1
        }
        // set maximum time for simulator
        TimeBudget = TB;
        // initialize image frame
        I = I.Init(WindowSize[0], WindowSize[1]);
    }
    size_t NumBoids, NumThreads;
    double Time, TimeBudget, DeltaTime;
    std::vector<Boid> AllBoids;
    std::vector<int> FlockSizes;
    const bool RenderingMovie = false;
    Image I;

    void Simulate()
    {
        double ElapsedTime = 0;
        while (Time <= TimeBudget)
        {
            ElapsedTime += Tick();
        }
        std::cout << "Finished simulation! Took " << ElapsedTime << "s" << std::endl;
    }

    double Tick()
    {
        if (RenderingMovie)
        {
            // Rendering is not part of our problem
            Render();
        }

        // Run our actual problem (boid computation)
        auto StartTime = std::chrono::system_clock::now();

#pragma omp parallel for num_threads(NumThreads)
        for (size_t i = 0; i < AllBoids.size(); i++)
        {
            AllBoids[i].Sense(AllBoids);
        }
#pragma omp parallel for num_threads(NumThreads)
        for (size_t i = 0; i < AllBoids.size(); i++)
        {
            AllBoids[i].Plan(FlockSizes);
        }
#pragma omp parallel for num_threads(NumThreads)
        for (size_t i = 0; i < AllBoids.size(); i++)
        {
            AllBoids[i].Act(DeltaTime);
        }

        auto EndTime = std::chrono::system_clock::now();
        std::chrono::duration<double> ElapsedTime = EndTime - StartTime;
        Time += DeltaTime;          // update simulator time
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
    /// TODO: Add params for rendering and window size and other vars
    const size_t NumBoids = 10000;
    const size_t NumThreads = 16;
    const double MaxT = 8.0;
    const double DeltaT = 0.05;
    const Vec2D ScreenDim(1000, 1000);

    std::srand(0); // consistent seed
    Simulator Sim(NumBoids, NumThreads, MaxT, DeltaT, ScreenDim);
    Sim.Simulate();
    return 0;
}

/// NOTE: resources:
/// PSEUDOCODE: http://www.vergenet.net/~conrad/boids/pseudocode.html
/// OVERVIEW: https://cs.stanford.edu/people/eroberts/courses/soco/projects/2008-09/modeling-natural-systems/boids.html
/// EXAMPLE: https://eater.net/boids