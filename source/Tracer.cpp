#include "Tracer.hpp"
#include <cassert>
#include <iostream>

void Tracer::InitFlockMatrix(const size_t NumFlocks)
{
    Tracer *T = Instance();
    for (size_t i = 0; i < NumFlocks; i++)
    {
        T->CommunicationMatrix.push_back(std::vector<FlockOps>(NumFlocks));
    }
    assert(T->CommunicationMatrix.size() * T->CommunicationMatrix[0].size() == sqr(NumFlocks));
}

void Tracer::SaveFlockMatrix(const std::vector<Flock> &AllFlocks)
{
    Tracer *T = Instance();
    assert(T->CommunicationMatrix.size() > 0);
    assert(T->CommunicationMatrix.size() == T->CommunicationMatrix[0].size());
    for (size_t FID = 0; FID < T->CommunicationMatrix.size(); FID++)
    {
        // for FID being the requestor flock ID
        for (size_t FID2 = 0; FID2 < T->CommunicationMatrix.size(); FID2++)
        {
            // for FID2 being the holder flock ID
            FlockOps &FO = T->CommunicationMatrix[FID][FID2];
            /// NOTE: assigning thread ID's can only be done AFTER all ops have completed
            FO.RequestorTIDs = AllFlocks[FID].TIDs;
            FO.HolderTIDs = AllFlocks[FID2].TIDs;
            Tracer::AddFlockOps(FO);
        }
    }
    // clear matrix
    for (std::vector<FlockOps> &Row : T->CommunicationMatrix)
    {
        Row.clear();
    }
    T->CommunicationMatrix.clear();
    assert(T->CommunicationMatrix.size() == 0); // capacity is still large, fast allocation
}

void Tracer::AddWrite(const size_t F_Requestor, const size_t F_Holder, const Flock::FlockOp F)
{
    Tracer *T = Instance();
    assert(T->CommunicationMatrix.size() > 0);
    assert(T->CommunicationMatrix.size() == T->CommunicationMatrix[0].size());
    switch (F)
    {
    case Flock::SenseAndPlanOp:
        T->CommunicationMatrix[F_Requestor][F_Holder].SenseAndPlan.Writes++;
    case Flock::ActOp:
        T->CommunicationMatrix[F_Requestor][F_Holder].Act.Writes++;
    case Flock::DelegateOp:
        T->CommunicationMatrix[F_Requestor][F_Holder].Delegate.Writes++;
    case Flock::AssignToFlockOp:
        T->CommunicationMatrix[F_Requestor][F_Holder].AssignToFlock.Writes++;
    }
}

void Tracer::AddRead(const size_t F_Requestor, const size_t F_Holder, const Flock::FlockOp F)
{
    Tracer *T = Instance();
    assert(T->CommunicationMatrix.size() > 0);
    assert(T->CommunicationMatrix.size() == T->CommunicationMatrix[0].size());
    switch (F)
    {
    case Flock::SenseAndPlanOp:
        T->CommunicationMatrix[F_Requestor][F_Holder].SenseAndPlan.Reads++;
    case Flock::ActOp:
        T->CommunicationMatrix[F_Requestor][F_Holder].Act.Reads++;
    case Flock::DelegateOp:
        T->CommunicationMatrix[F_Requestor][F_Holder].Delegate.Reads++;
    case Flock::AssignToFlockOp:
        T->CommunicationMatrix[F_Requestor][F_Holder].AssignToFlock.Reads++;
    }
}

void Tracer::AddReads(const size_t T_Requestor, const size_t T_Holder, const size_t Amnt = 1)
{
    Tracer *T = Instance();
    assert(T->MemoryOpMatrix.size() == Tracer::Params.NumThreads);
    assert(T->MemoryOpMatrix[0].size() == Tracer::Params.NumThreads);
    T->MemoryOpMatrix[T_Requestor][T_Holder].Reads += Amnt;
}

void Tracer::AddWrites(const size_t T_Requestor, const size_t T_Holder, const size_t Amnt = 1)
{
    Tracer *T = Instance();
    assert(T->MemoryOpMatrix.size() == Tracer::Params.NumThreads);
    assert(T->MemoryOpMatrix[0].size() == Tracer::Params.NumThreads);
    T->MemoryOpMatrix[T_Requestor][T_Holder].Writes += Amnt;
}

void Tracer::AddFlockOps(const Tracer::FlockOps &FO)
{
    // sense & plan
    AddReads(FO.RequestorTIDs.SenseAndPlan, FO.HolderTIDs.SenseAndPlan, FO.SenseAndPlan.Reads);
    AddWrites(FO.RequestorTIDs.SenseAndPlan, FO.HolderTIDs.SenseAndPlan, FO.SenseAndPlan.Writes);
    // act
    AddReads(FO.RequestorTIDs.Act, FO.HolderTIDs.Act, FO.Act.Reads);
    AddWrites(FO.RequestorTIDs.Act, FO.HolderTIDs.Act, FO.Act.Writes);
    // delegate
    AddReads(FO.RequestorTIDs.Delegate, FO.HolderTIDs.Delegate, FO.Delegate.Reads);
    AddWrites(FO.RequestorTIDs.Delegate, FO.HolderTIDs.Delegate, FO.Delegate.Writes);
    // assign to flock
    AddReads(FO.RequestorTIDs.AssignToFlock, FO.HolderTIDs.AssignToFlock, FO.AssignToFlock.Reads);
    AddWrites(FO.RequestorTIDs.AssignToFlock, FO.HolderTIDs.AssignToFlock, FO.AssignToFlock.Writes);
}

void Tracer::AddTickT(const double ElapsedTime)
{
    Tracer *T = Instance();
    T->TickTimes.push_back(ElapsedTime);
}

void Tracer::Dump()
{
    const Tracer *T = Instance();
    std::cout << "Comms Matrix:" << std::endl;
    for (const std::vector<MemoryOps> &MemoryRow : T->MemoryOpMatrix)
    {
        std::cout << "{ ";
        for (const MemoryOps &M : MemoryRow)
        {
            std::cout << " R:" << M.Reads << " W:" << M.Writes << " |";
        }
        std::cout << "}" << std::endl;
    }
    std::cout << "Tick Timings" << std::endl << "[";
    for (const double t : T->TickTimes)
    {
        std::cout << t << ", ";
    }
    std::cout << "]" << std::endl;
}