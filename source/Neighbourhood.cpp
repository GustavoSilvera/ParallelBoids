#include "Neighbourhood.hpp"
#include "Vec.hpp"
#include <omp.h>

// default layout is invalid until assigned
NLayout::Layout NLayout::UsingLayout = NLayout::Invalid;
// boid struct of arrays is empty
std::vector<Boid> NLayout::BoidsGlobal;
// boid sizes hash map is empty
std::unordered_map<size_t, NLayout::FlockData> NLayout::BoidsGlobalData;

void NLayout::SetType(const Layout L)
{
    assert(L == Local || L == Global);
    UsingLayout = L;
    // reserve into vector to save on reallocating
    BoidsGlobal.reserve(GlobalParams.SimulatorParams.NumBoids);
}

NLayout::Layout NLayout::GetType()
{
    return UsingLayout;
}

bool NLayout::IsValid() const
{
    // ensure layout is local or global
    if (UsingLayout == Invalid)
        return false;
    /// GLOBAL:
    // ensure all boids don't move
    for (size_t i = 0; i < BoidsGlobal.size(); i++)
    {
        if (BoidsGlobal[i].BoidID != i)
            return false;
    }
    // ensure all flockmates are in the same flocks
    for (size_t bID : BoidsGlobalData[FlockID].BoidIDs)
    {
        if (BoidsGlobal[bID].FlockID != FlockID)
            return false;
    }
    // no boid left behind (VERY EXPENSIVE, only need to be done once)
    if (FlockID == 0) // arbitrary random FlockID
    {
        size_t NumBoids = 0;
        for (auto FD : BoidsGlobalData)
        {
            NumBoids += FD.second.Size();
        }
        if (NumBoids != BoidsGlobal.size())
            return false;
    }
    /// LOCAL:
    // ensure all flockmates are in the same local flock
    for (const Boid &B : BoidsLocal)
    {
        if (B.FlockID != FlockID)
        {
            return false;
        }
    }
    return true;
}

void NLayout::NewBoid(const size_t FID)
{
    Boid NewBoidStruct(FID);
    FlockID = FID;
    if (UsingLayout == Local)
    {
        BoidsLocal.push_back(NewBoidStruct);
    }
    else
    {
        assert(UsingLayout == Global);
        BoidsGlobal.push_back(NewBoidStruct);
        // need to manually manage boid data
        BoidsGlobalData[FlockID].Add(NewBoidStruct);
    }
    assert(IsValid());
}

size_t NLayout::Size() const
{
    assert(IsValid());
    if (UsingLayout == Local)
    {
        return BoidsLocal.size();
    }
    assert(UsingLayout == Global);
    return BoidsGlobalData[FlockID].Size();
}

Boid *NLayout::GetBoidF(const size_t Idx) const
{
    if (UsingLayout == Global)
    {
        // since Idx is local to the flock, we'll need to find the flock's local
        // neighbourhood boids
        assert(Idx < BoidsGlobalData.at(FlockID).Size());
        size_t GlobalIdx = BoidsGlobalData.at(FlockID).GetBoidIdx(Idx); // BoidID of global boid
        assert(GlobalIdx < BoidsGlobal.size());
        return (*this)[GlobalIdx];
    }
    return (*this)[Idx];
}

Boid *NLayout::operator[](const size_t Idx) const
{
    /// NOTE: using const-cast to keep function marked const
    // but allow edits to the underlying boids afterwards
    if (UsingLayout == Local)
    {
        assert(Idx < BoidsLocal.size());
        return const_cast<Boid *>(&(BoidsLocal[Idx]));
    }
    assert(UsingLayout == Global);
    assert(Idx < BoidsGlobal.size());
    return const_cast<Boid *>(&(BoidsGlobal[Idx]));
}

void NLayout::ClearLocal()
{
    assert(IsValid());
    if (UsingLayout == Local)
    {
        BoidsLocal.clear();
    }
}

void NLayout::Append(const std::vector<Boid> &Immigrants)
{
    if (UsingLayout == Local)
    {
        // don't need a critical section bc writing to local, reading from remote
        BoidsLocal.insert(BoidsLocal.end(), Immigrants.begin(), Immigrants.end());
    }
    else
    {
        assert(UsingLayout == Global);
        for (const Boid &B : Immigrants)
        {
            const size_t Idx = B.BoidID;
            if (BoidsGlobal[Idx].FlockID == FlockID)
            {
                continue; // don't need to remove/readd them
            }
            assert(Idx < BoidsGlobal.size());
            assert(Idx < BoidsGlobalData.size());
#pragma omp critical
            {
                /// NOTE: one option to have O(1) random iterator accesses is to
                // guarantee that Immigrants is always sorted by BoidID, then instead
                // of adding/removing one at a time, it can be done all at once in O(N)
                // which is O(1) for each boid in Immigrants
                BoidsGlobalData.at(BoidsGlobal[Idx].FlockID).Remove(B); // remove old
                BoidsGlobal[Idx].FlockID = FlockID;                     // assign new FlockID to Boid
                BoidsGlobalData.at(FlockID).Add(B);                     // add new to my flock
            }
        }
    }
    assert(IsValid());
}