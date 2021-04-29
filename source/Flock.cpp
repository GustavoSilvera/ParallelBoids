#include "Flock.hpp"
#include <omp.h>  // OpenMP
#include <vector> // std::vector

size_t Flock::Size() const
{
    return Neighbourhood.size();
}

void Flock::Tick(const size_t ThreadID, std::vector<Flock> &AllFlocks)
{
    /// TODO: fix race condition by having an old-pos
    for (size_t i = 0; i < Neighbourhood.size(); i++)
    {
        Neighbourhood[i].Sense(AllFlocks, ThreadID);
    }
    for (size_t i = 0; i < Neighbourhood.size(); i++)
    {
        Neighbourhood[i].Plan((AllFlocks), ThreadID);
    }
    for (size_t i = 0; i < Neighbourhood.size(); i++)
    {
        Neighbourhood[i].Act(Params.DeltaTime, ThreadID);
    }
}

void Flock::Recruit(Boid &B, std::vector<Flock> &AllFlocks)
{
    /// TODO: some kind of union-find here
    const size_t TheirFlockID = B.FlockID;
    if (TheirFlockID == FlockID)
    {
        // do nothing
        return;
    }
    // remove from their neighbourhood
    AllFlocks[B.FlockID].Leave(B);
    // update the newcomer's flock id
    B.FlockID = FlockID;
    // add to our neighbourhood
    Neighbourhood.push_back(B);
}

void Flock::Leave(Boid &Emigrant)
{
    /// TODO: this is very bad and naive
    std::vector<Boid> Remaining;
    for (const Boid &B : Neighbourhood)
    {
        if (B.BoidID != Emigrant.BoidID)
        {
            Remaining.push_back(B);
        }
    }
    // relpace old neighbourhood with new one
    Neighbourhood = Remaining;
}

void Flock::Draw(Image &I) const
{
    /// TODO: check if can-parallelize?
    for (const Boid &B : Neighbourhood)
    {
        B.Draw(I);
    }
}
