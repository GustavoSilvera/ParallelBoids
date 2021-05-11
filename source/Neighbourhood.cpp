#include "Neighbourhood.hpp"
#include "Vec.hpp"
#include <omp.h>

// default layout is invalid until assigned
NLayout::Layout NLayout::UsingLayout = NLayout::Invalid;
// boid struct of arrays is empty
std::vector<Boid> NLayout::BoidsGlobal;
// boid sizes hash map is empty
std::unordered_map<size_t, size_t> NLayout::BoidsGlobalSizes;

void NLayout::SetType(const Layout L)
{
    assert(L == Local || L == Global);
    UsingLayout = L;
}

NLayout::Layout NLayout::GetType()
{
    return UsingLayout;
}

void NLayout::NewBoid(const size_t FlockID)
{
    Boid NewBoidStruct(FlockID);
    if (UsingLayout == Local)
    {
        BoidsLocal.push_back(NewBoidStruct);
    }
    else
    {
        assert(UsingLayout == Global);
        BoidsGlobal.push_back(NewBoidStruct);
        // need to manually manage boid sizes
        BoidsGlobalSizes[NewBoidStruct.FlockID]++;
    }
}

size_t NLayout::Size(const size_t FlockID) const
{
    if (UsingLayout == Local)
    {
        return BoidsLocal.size();
    }
    assert(UsingLayout == Global);
    return BoidsGlobalSizes[FlockID];
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
            assert(Idx < BoidsGlobal.size());
            assert(Idx < BoidsGlobalSizes.size());
#pragma omp critical
            {
                BoidsGlobalSizes[BoidsGlobal[Idx].FlockID]--; // removing from original FlockID
                BoidsGlobal[Idx].FlockID = B.FlockID;         // assigning new flockID
                BoidsGlobalSizes[B.FlockID]++;                // adding to new flockID
            }
        }
    }
}