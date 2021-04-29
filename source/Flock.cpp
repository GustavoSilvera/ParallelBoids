#include "Flock.hpp"
#include <algorithm>
#include <cassert>
#include <omp.h>  // OpenMP
#include <vector> // std::vector

int Flock::Size() const
{
    int S = Neighbourhood.size();
    assert(S >= 0);
    return S;
}

void Flock::SenseAndPlan(const size_t ThreadID, std::vector<Flock> &AllFlocks)
{
    for (size_t i = 0; i < Neighbourhood.size(); i++)
    {
        Neighbourhood[i].SenseAndPlan(AllFlocks, ThreadID);
    }
}

void Flock::Act(const double DeltaTime)
{
    for (size_t i = 0; i < Neighbourhood.size(); i++)
    {
        Neighbourhood[i].Act(DeltaTime);
    }
}

void Flock::Recruit(Boid &B, std::vector<Flock> &AllFlocks)
{
    const size_t TheirFlockID = B.FlockID;
    if (TheirFlockID == FlockID)
    {
        // do nothing
        return;
    }
    // find position of boid in other neighbourhood
    std::vector<Boid> &OtherNeighbourhood = AllFlocks[B.FlockID].Neighbourhood;
    // update the newcomer's flock id
    B.FlockID = FlockID;
    // move B over to our flock
    Neighbourhood.push_back(std::move(B));
    // O(1) swap B with end of the other flock's neighbourhood for fast pop
    std::swap(B, OtherNeighbourhood.back());
    // destroy the last element (unusable after std::move) in OtherNeighbourhood
    OtherNeighbourhood.pop_back(); // destructive
}

void Flock::Draw(Image &I) const
{
    /// TODO: check if can-parallelize?
    for (const Boid &B : Neighbourhood)
    {
        B.Draw(I);
    }
}
