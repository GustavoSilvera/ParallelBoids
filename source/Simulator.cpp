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

    void Finish()
    {
        for (auto It = AllFlocks.begin(); It != AllFlocks.end(); It++)
        {
            assert(It != AllFlocks.end());
            Flock &F = It->second;
            F.Destroy();
            assert(F.Size() == 0);
        }
    }

    void Simulate()
    {
        double ElapsedTime = 0;
        double SenseTime = 0;
        double ActTime = 0;
        double DelegateTime = 0;
        double AssignTime = 0;
        for (size_t i = 0; i < Params.NumIterations; i++)
        {
            ElapsedTime += Tick(SenseTime, ActTime, DelegateTime, AssignTime);
            std::cout << "Tick: " << i << "\r" << std::flush; // carriage return, no newline
        }
        std::cout << "Finished simulation! Took " << ElapsedTime << "s" << std::endl;
        std::cout << "Total SenseT " << SenseTime << std::endl;
        std::cout << "Total ActT " << ActTime << std::endl;
        std::cout << "Total DelegateT " << DelegateTime << std::endl;
        std::cout << "Total AssignT " << AssignTime << std::endl;
    }

    double Tick(double &SenseTime, double &ActTime, double &DelegateTime, double &AssignTime)
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
            ParallelBoids(AllFlocksVec, SenseTime, ActTime);
        else
            ParallelFlocks(AllFlocksVec, SenseTime, ActTime);
        UpdateFlocks(AllFlocksVec, DelegateTime, AssignTime);

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
            Tracer::AddFlockSize(F.Size());
            AllFlocksVec.push_back(const_cast<Flock *>(&F));
        }
        assert(AllFlocksVec.size() == AllFlocks.size());
        return AllFlocksVec;
    }

    void ParallelBoids(std::vector<Flock *> AllFlocksVec, double &SenseT, double ActT)
    {
        /// NOTE: the following parallel operations are per-boids
        std::chrono::time_point<std::chrono::system_clock> StartTime1;
        std::chrono::time_point<std::chrono::system_clock> EndTime1;
        std::chrono::time_point<std::chrono::system_clock> EndTime2;
#pragma omp parallel num_threads(Params.NumThreads) // spawns threads
        {
            if (!GlobalParams.FlockParams.UseLocalNeighbourhoods)
            {
                std::vector<Boid> &AllBoids = *(AllFlocks.begin()->second.Neighbourhood.GetAllBoidsPtr());
#pragma omp single
                {
                    StartTime1 = std::chrono::system_clock::now();
                }
#pragma omp for schedule(dynamic)
                for (size_t i = 0; i < AllBoids.size(); i++)
                {
                    AllBoids[i].SenseAndPlan(omp_get_thread_num(), AllFlocks);
                }
#pragma omp barrier
#pragma omp single
                {
                    EndTime1 = std::chrono::system_clock::now();
                    std::chrono::duration<double> ElapsedTime1 = EndTime1 - StartTime1;
                    SenseT += ElapsedTime1.count();
                }
#pragma omp for schedule(dynamic)
                for (size_t i = 0; i < AllBoids.size(); i++)
                {
                    AllBoids[i].Act(Params.DeltaTime);
                }
#pragma omp single
                {
                    EndTime2 = std::chrono::system_clock::now();
                    std::chrono::duration<double> ElapsedTime2 = EndTime2 - EndTime1;
                    ActT += ElapsedTime2.count();
                }
            }
            else
            {
                std::vector<Boid *> AllBoids;
#pragma omp single
                {
                    StartTime1 = std::chrono::system_clock::now();
                }
                for (const Flock *F : AllFlocksVec)
                {
#pragma omp critical
                    {
                        std::vector<Boid *> LocalBoids = F->Neighbourhood.GetBoids();
                        AllBoids.insert(AllBoids.end(), LocalBoids.begin(), LocalBoids.end());
                    }
                }
#pragma omp barrier
#pragma omp for schedule(dynamic)
                for (size_t i = 0; i < AllBoids.size(); i++)
                {
                    AllBoids[i]->SenseAndPlan(omp_get_thread_num(), AllFlocks);
                }
#pragma omp barrier
#pragma omp single
                {
                    EndTime1 = std::chrono::system_clock::now();
                    std::chrono::duration<double> ElapsedTime1 = EndTime1 - StartTime1;
                    SenseT += ElapsedTime1.count();
                }
#pragma omp for schedule(dynamic)
                for (size_t i = 0; i < AllBoids.size(); i++)
                {
                    AllBoids[i]->Act(Params.DeltaTime);
                }
#pragma omp single
                {
                    EndTime2 = std::chrono::system_clock::now();
                    std::chrono::duration<double> ElapsedTime2 = EndTime2 - EndTime1;
                    ActT += ElapsedTime2.count();
                }
            }
        }
    }

    void ParallelFlocks(std::vector<Flock *> AllFlocksVec, double &SenseT, double &ActT)
    {
        std::chrono::time_point<std::chrono::system_clock> StartTime1;
        std::chrono::time_point<std::chrono::system_clock> EndTime1;
        std::chrono::time_point<std::chrono::system_clock> EndTime2;
#pragma omp parallel num_threads(Params.NumThreads) // spawns threads
        {
            // parallelizing across flocks
#pragma omp single
            {
                StartTime1 = std::chrono::system_clock::now();
            }
#pragma omp for schedule(dynamic)
            for (size_t i = 0; i < AllFlocksVec.size(); i++)
            {
                AllFlocksVec[i]->SenseAndPlan(omp_get_thread_num(), AllFlocks);
            }
#pragma omp barrier
#pragma omp single
            {
                EndTime1 = std::chrono::system_clock::now();
                std::chrono::duration<double> ElapsedTime1 = EndTime1 - StartTime1;
                SenseT += ElapsedTime1.count();
            }
#pragma omp for schedule(dynamic)
            for (size_t i = 0; i < AllFlocksVec.size(); i++)
            {
                AllFlocksVec[i]->Act(Params.DeltaTime);
            }
#pragma omp single
            {
                EndTime2 = std::chrono::system_clock::now();
                std::chrono::duration<double> ElapsedTime2 = EndTime2 - EndTime1;
                ActT += ElapsedTime2.count();
            }
        }
    }

    void UpdateFlocks(std::vector<Flock *> AllFlocksVec, double &DelegateT, double &AssignT)
    {
        std::chrono::time_point<std::chrono::system_clock> StartTime1;
        std::chrono::time_point<std::chrono::system_clock> EndTime1;
        std::chrono::time_point<std::chrono::system_clock> EndTime2;
#pragma omp parallel num_threads(Params.NumThreads) // spawns threads
        {
            if (GlobalParams.FlockParams.UseFlocks)
            {
                /// NOTE: the following parallel operations are per-flocks, not per-boids
#pragma omp single
                {
                    StartTime1 = std::chrono::system_clock::now();
                }
#pragma omp for schedule(dynamic)
                for (size_t i = 0; i < AllFlocksVec.size(); i++)
                {
                    AllFlocksVec[i]->Delegate(omp_get_thread_num(), AllFlocksVec);
                }
#pragma omp barrier
#pragma omp single
                {
                    EndTime1 = std::chrono::system_clock::now();
                    std::chrono::duration<double> ElapsedTime1 = EndTime1 - StartTime1;
                    DelegateT += ElapsedTime1.count();
                }
#pragma omp for schedule(dynamic)
                for (size_t i = 0; i < AllFlocksVec.size(); i++)
                {
                    AllFlocksVec[i]->AssignToFlock(omp_get_thread_num());
                }
#pragma omp single
                {
                    EndTime2 = std::chrono::system_clock::now();
                    std::chrono::duration<double> ElapsedTime2 = EndTime2 - EndTime1;
                    AssignT += ElapsedTime2.count();
                }
            }
#pragma omp barrier
#pragma omp for schedule(dynamic)
            for (size_t i = 0; i < AllFlocksVec.size(); i++)
            {
                AllFlocksVec[i]->ComputeBB();
            }
        }
        // convert flock data to processor communications
        Tracer::SaveFlockMatrix(AllFlocks);
        // compute avg flock size
        Tracer::ComputeFlockAverageSize();
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
#pragma omp parallel for num_threads(Params.NumThreads) schedule(dynamic)
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

void RunSimulation()
{
    Tracer::Initialize();
    Simulator Sim;
    Sim.Simulate();
    // Dump all tracer data
    Tracer::Dump();
    Sim.Finish();
}

int main(int argc, char *argv[])
{
    std::srand(0); // consistent seed
    if (argc == 1)
    {
        ParseParams("params/params.ini");
    }
    else
    {
        const std::string ParamFile(argv[1]);
        ParseParams("params/" + ParamFile);
    }
    if (GlobalParams.SimulatorParams.NumThreads > 0)
    {
        // specify a legal thread amnt
        RunSimulation();
    }
    else
    {
        // run on all threads
        const std::vector<size_t> AllProcTests = {2, 4, 8, 12, 16, 24, 32};
        for (const size_t P : AllProcTests)
        {
            // overwrite NumThreads
            GlobalParams.SimulatorParams.NumThreads = P;
            RunSimulation();
            std::cout << std::endl;
        }
    }
    // if (GlobalParams.FlockParams.MaxSize > 0)
    // {
    //     // specify a legal thread amnt
    //     RunSimulation();
    // }
    // else
    // {
    //     // run on all threads
    //     // const std::vector<size_t> AllProcTests = {2, 4, 8, 12, 16, 24, 32};
    //     // const std::vector<size_t> AllProcTests = {10, 50, 100, 200, 500, 1000, 2000};
    //     const std::vector<size_t> AllProcTests = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    //     for (const size_t P : AllProcTests)
    //     {
    //         // overwrite NumThreads
    //         GlobalParams.FlockParams.MaxSize = P;
    //         RunSimulation();
    //         std::cout << std::endl;
    //     }
    // }
    return 0;
}

/// PSEUDOCODE: http://www.vergenet.net/~conrad/boids/pseudocode.html
/// OVERVIEW: https://cs.stanford.edu/people/eroberts/courses/soco/projects/2008-09/modeling-natural-systems/boids.html
/// EXAMPLE: https://eater.net/boids