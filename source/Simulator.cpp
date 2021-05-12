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

        // Print out important params
        std::string ParAxis = "FLOCKS";
        if (!Params.ParallelizeAcrossFlocks)
            ParAxis = "BOIDS";
        std::string NeighMode = "LOCAL";
        if (!GlobalParams.FlockParams.UseLocalNeighbourhoods)
            NeighMode = "GLOBAL";
        std::cout << "Parallelizing across " << ParAxis << " with a " << NeighMode << " neighbourhood layout"
                  << std::endl;

        // Initialize neighbourhood layout for flocks before use
        Flock::InitNeighbourhoodLayout();
        // Spawn flocks
        for (size_t i = 0; i < Params.NumBoids; i++)
        {
            AllFlocks[i] = Flock(i, 1);
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
    /// TODO: we can use the SenseAndPlan flock optimization if we change this vector
    // to an associative container (unordered_map) so we can get O(1) access regardless
    // of the resizing
    std::unordered_map<size_t, Flock> AllFlocks;
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
        // Run our actual problem (boid computation)
        auto StartTime = std::chrono::system_clock::now();

#ifndef NDEBUG
        size_t BoidCount = 0;
        for (auto It = AllFlocks.begin(); It != AllFlocks.end(); It++)
        {
            assert(It != AllFlocks.end());
            const Flock &F = It->second;
            BoidCount += F.Size();
            for (const Boid *B : F.Neighbourhood.GetBoids())
            {
                assert(B->IsValid());
            }
        }
        assert(Params.NumBoids == BoidCount);
#endif
        std::vector<Flock *> AllFlocksVec = GetAllFlocksVector();

        if (!Params.ParallelizeAcrossFlocks)
            ParallelBoids(AllFlocksVec);
        else
            ParallelFlocks(AllFlocksVec);
        UpdateFlocks(AllFlocksVec);

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

    std::vector<Flock *> GetAllFlocksVector() const
    {
        std::vector<Flock *> AllFlocksVec;
        for (auto It = AllFlocks.begin(); It != AllFlocks.end(); It++)
        {
            assert(It != AllFlocks.end());
            const Flock &F = It->second;
            AllFlocksVec.push_back(const_cast<Flock *>(&F));
        }
        assert(AllFlocksVec.size() == AllFlocks.size());
        return AllFlocksVec;
    }

    void ParallelBoids(std::vector<Flock *> AllFlocksVec)
    {
        /// NOTE: the following parallel operations are per-boids
#pragma omp parallel num_threads(Params.NumThreads) // spawns threads
        {
            if (!GlobalParams.FlockParams.UseLocalNeighbourhoods)
            {
                std::vector<Boid> &AllBoids = *(AllFlocks.begin()->second.Neighbourhood.GetAllBoidsPtr());
#pragma omp for schedule(static)
                for (size_t i = 0; i < AllBoids.size(); i++)
                {
                    AllBoids[i].SenseAndPlan(omp_get_thread_num(), AllFlocks);
                }
#pragma omp barrier
#pragma omp for schedule(static)
                for (size_t i = 0; i < AllBoids.size(); i++)
                {
                    AllBoids[i].Act(Params.DeltaTime);
                }
            }
            else
            {
                std::vector<Boid *> AllBoids;
                for (const Flock *F : AllFlocksVec)
                {
#pragma omp critical
                    {
                        std::vector<Boid *> LocalBoids = F->Neighbourhood.GetBoids();
                        AllBoids.insert(AllBoids.end(), LocalBoids.begin(), LocalBoids.end());
                    }
                }
#pragma omp barrier
#pragma omp for schedule(static)
                for (size_t i = 0; i < AllBoids.size(); i++)
                {
                    AllBoids[i]->SenseAndPlan(omp_get_thread_num(), AllFlocks);
                }
#pragma omp barrier
#pragma omp for schedule(static)
                for (size_t i = 0; i < AllBoids.size(); i++)
                {
                    AllBoids[i]->Act(Params.DeltaTime);
                }
            }
        }
    }

    void ParallelFlocks(std::vector<Flock *> AllFlocksVec)
    {
#pragma omp parallel num_threads(Params.NumThreads) // spawns threads
        {
            // parallelizing across flocks
#pragma omp for schedule(static)
            for (size_t i = 0; i < AllFlocksVec.size(); i++)
            {
                AllFlocksVec[i]->SenseAndPlan(omp_get_thread_num(), AllFlocks);
            }
#pragma omp barrier
#pragma omp for schedule(static)
            for (size_t i = 0; i < AllFlocksVec.size(); i++)
            {
                AllFlocksVec[i]->Act(Params.DeltaTime);
            }
        }
    }

    void UpdateFlocks(std::vector<Flock *> AllFlocksVec)
    {
#pragma omp parallel num_threads(Params.NumThreads) // spawns threads
        {
            /// NOTE: the following parallel operations are per-flocks, not per-boids
#pragma omp barrier
#pragma omp for schedule(static)
            for (size_t i = 0; i < AllFlocksVec.size(); i++)
            {
                AllFlocksVec[i]->Delegate(omp_get_thread_num(), AllFlocks);
            }
#pragma omp barrier
#pragma omp for schedule(static)
            for (size_t i = 0; i < AllFlocksVec.size(); i++)
            {
                AllFlocksVec[i]->AssignToFlock(omp_get_thread_num(), AllFlocks);
            }
#pragma omp barrier
#pragma omp for schedule(static)
            for (size_t i = 0; i < AllFlocksVec.size(); i++)
            {
                AllFlocksVec[i]->ComputeBB();
            }
        }
        // convert flock data to processor communications
        Tracer::SaveFlockMatrix(AllFlocks);
        // remove empty (invalid) flocks
        Flock::CleanUp(AllFlocks);
    }

    void Render()
    {
        // draw all the boids onto the frame
        std::vector<Flock *> AllFlocksVec;
        for (auto It = AllFlocks.begin(); It != AllFlocks.end(); It++)
        {
            assert(It != AllFlocks.end());
            Flock &F = It->second;
            AllFlocksVec.push_back(&F);
        }
#pragma omp parallel for num_threads(Params.NumThreads) schedule(static)
        for (size_t i = 0; i < AllFlocksVec.size(); i++)
        {
            AllFlocksVec[i]->Draw(I);
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
    Tracer::Initialize();
    Simulator Sim;
    Sim.Simulate();
    // Dump all tracer data
    Tracer::Dump();
    return 0;
}

/// PSEUDOCODE: http://www.vergenet.net/~conrad/boids/pseudocode.html
/// OVERVIEW: https://cs.stanford.edu/people/eroberts/courses/soco/projects/2008-09/modeling-natural-systems/boids.html
/// EXAMPLE: https://eater.net/boids