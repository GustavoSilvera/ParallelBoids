#include "Neighbourhood.hpp"
#include "Vec.hpp"
#include <omp.h>

// default layout is invalid until assigned
static NLayout::Layout UsingLayout = NLayout::Invalid;
// boid struct of arrays is empty
static std::vector<Boid> NLayout::BoidsGlobal;
// boid sizes hash map is empty
static unordered_map<size_t, size_t> NLayout::BoidsGlobalSizes;

void NLayout::SetType(const Layout L)
{
    assert(L == Local || L == Global);
    UsingLayout = L;
}

void NLayout::Layout GetType() const
{
    return UsingLayout;
}

void NLayout::NewBoid(const size_t FlockID)
{
    Boid NewBoidStruct(FlockID);
    if (UsingLayout == Local)
    {
        BoidsLocal.push_back(NewBoidStruct)
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
        return BoidsAoS.size();
    }
    assert(UsingLayout == Global);
    return BoidsGlobalSizes[FlockID];
}

Boid *NLayout::operator[](const size_t Idx) const
{
    if (UsingLayout == Local)
    {
        assert(Idx < Size());
        return &(BoidsLocal[Idx]);
    }
    assert(UsingLayout == Global);
    assert(Idx < Size());
    return &(BoidsGlobal[Idx]);
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
#pragma omp critical
            {
                BoidsGlobal[Idx].FlockID = B.FlockID;
            }
        }
    }
}