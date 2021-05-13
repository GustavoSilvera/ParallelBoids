#include "Tracer.hpp"
#include <algorithm>
#include <cassert>
#include <iostream>

void Tracer::Initialize()
{
#ifndef NTRACE
    Instance();
#else
    (void)0;
#endif
}

void Tracer::InitFlockMatrix(const size_t NumFlocks)
{
#ifndef NTRACE
    if (!Params.TrackMem)
        return; // do nothing
    Tracer *T = Instance();
    for (size_t i = 0; i < NumFlocks; i++)
    {
        // allocate empty dictionary
        T->CommunicationMatrix.push_back(std::vector<FlockOps>(NumFlocks));
    }
    assert(T->CommunicationMatrix.size() * T->CommunicationMatrix[0].size() == sqr(NumFlocks));
#else
    (void)0;
#endif
}

void Tracer::SaveFlockMatrix(const std::unordered_map<size_t, Flock> &AllFlocks)
{
#ifndef NTRACE
    if (!Params.TrackMem)
        return; // do nothing
    Tracer *T = Instance();
    assert(T->CommunicationMatrix.size() > 0);
    for (size_t FID = 0; FID < T->CommunicationMatrix.size(); FID++)
    {
        auto It = AllFlocks.find(FID);
        assert(It != AllFlocks.end());
        const Flock &F = It->second;
        // for FID being the requestor flock ID
        for (size_t FID2 = 0; FID2 < T->CommunicationMatrix[FID].size(); FID2++)
        {
            auto It2 = AllFlocks.find(FID2);
            assert(It2 != AllFlocks.end());
            const Flock &F2 = It2->second;
            // for FID2 being the holder flock ID
            FlockOps &FO = T->CommunicationMatrix[FID][FID2];
            /// NOTE: assigning thread ID's can only be done AFTER all ops have completed
            FO.RequestorTIDs = F.TIDs;
            FO.HolderTIDs = F2.TIDs;
            Tracer::AddFlockOps(FO);
        }
    }
    // clear matrix
    const size_t NumFlocks = T->CommunicationMatrix.size(); // initial matrix size
    for (auto &Row : T->CommunicationMatrix)
    {
        // reset to all 0's
        Row = std::vector<FlockOps>(NumFlocks);
    }
#else
    (void)0;
#endif
}

void Tracer::AddRead(const size_t F_Requestor, const size_t F_Holder, const Flock::FlockOp F)
{
    if (!Params.TrackMem)
        return; // do nothing
#ifndef NTRACE
    Tracer *T = Instance();
    assert(T->CommunicationMatrix.size() > 0);
    assert(0 <= F_Requestor && F_Requestor <= T->CommunicationMatrix.size());
    assert(0 <= F_Holder && F_Holder <= T->CommunicationMatrix[0].size());
    switch (F)
    {
    case Flock::SenseAndPlanOp:
#pragma omp atomic
        T->CommunicationMatrix[F_Requestor][F_Holder].SenseAndPlan.Reads++;
    case Flock::DelegateOp:
#pragma omp atomic
        T->CommunicationMatrix[F_Requestor][F_Holder].Delegate.Reads++;
    case Flock::AssignToFlockOp:
#pragma omp atomic
        T->CommunicationMatrix[F_Requestor][F_Holder].AssignToFlock.Reads++;
    }
#else
    (void)0;
#endif
}

void Tracer::AddReads(const size_t T_Requestor, const size_t T_Holder, const size_t Amnt = 1)
{
    if (!Params.TrackMem)
        return; // do nothing
#ifndef NTRACE
    Tracer *T = Instance();
    assert(T->MemoryOpMatrix.size() == GlobalParams.SimulatorParams.NumThreads);
    assert(T->MemoryOpMatrix[0].size() == GlobalParams.SimulatorParams.NumThreads);
    assert(0 <= T_Requestor && T_Requestor <= T->MemoryOpMatrix.size());
    assert(0 <= T_Holder && T_Holder <= T->MemoryOpMatrix[0].size());
    T->MemoryOpMatrix[T_Requestor][T_Holder].Reads += Amnt;
#else
    (void)0;
#endif
}

void Tracer::AddFlockOps(const Tracer::FlockOps &FO)
{
    if (!Params.TrackMem)
        return; // do nothing
#ifndef NTRACE
    // sense & plan
    AddReads(FO.RequestorTIDs.SenseAndPlan, FO.HolderTIDs.SenseAndPlan, FO.SenseAndPlan.Reads);
    // delegate
    AddReads(FO.RequestorTIDs.Delegate, FO.HolderTIDs.Delegate, FO.Delegate.Reads);
    // assign to flock
    AddReads(FO.RequestorTIDs.AssignToFlock, FO.HolderTIDs.AssignToFlock, FO.AssignToFlock.Reads);
#else
    (void)0;
#endif
}

void Tracer::AddTickT(const double ElapsedTime)
{
    if (!Params.TrackTickT)
        return; // do nothing
#ifndef NTRACE
    Tracer *T = Instance();
    T->TickTimes.push_back(ElapsedTime);
#else
    (void)0;
#endif
}

void Tracer::AddFlockSize(const size_t FS)
{
    if (!Params.TrackFlockSizes)
        return; // do nothing
#ifndef NTRACE
    Tracer *T = Instance();
    T->TmpFlockSizes.push_back(FS);
#else
    (void)0;
#endif
}

void Tracer::ComputeFlockAverageSize()
{
    if (!Params.TrackFlockSizes)
        return; // do nothing
#ifndef NTRACE
    Tracer *T = Instance();
    double avg = 0;
    for (size_t s : T->TmpFlockSizes)
    {
        avg += s;
    }
    if (T->TmpFlockSizes.size() > 0)
        avg /= T->TmpFlockSizes.size();
    T->TmpFlockSizes.clear();
    T->AvgFlockSizes.push_back(avg);
#else
    (void)0;
#endif
}

void Tracer::Dump()
{
#ifndef NTRACE
    Tracer *T = Instance();
    if (Params.TrackMem)
    {
        std::cout << "Comms Matrix:" << std::endl;
        for (const std::vector<MemoryOps> &MemoryRow : T->MemoryOpMatrix)
        {
            std::cout << "[ ";
            for (const MemoryOps &M : MemoryRow)
            {
                std::cout << M.Reads << ", ";
            }
            std::cout << "]" << std::endl;
        }
        T->MemoryOpMatrix.clear();
    }
    if (Params.TrackTickT)
    {
        std::cout << "Tick Timings" << std::endl << "[";
        for (const double t : T->TickTimes)
        {
            std::cout << t << ", ";
        }
        std::cout << "]" << std::endl;
        T->TickTimes.clear();
    }
    if (Params.TrackFlockSizes)
    {
        std::cout << "Flock Sizes" << std::endl << "[";
        for (const double t : T->AvgFlockSizes)
        {
            std::cout << t << ", ";
        }
        std::cout << "]" << std::endl;
        T->AvgFlockSizes.clear();
    }
#else
    std::cout << "Trace not executing (compiled with -DNTRACE)" << std::endl;
#endif
}
