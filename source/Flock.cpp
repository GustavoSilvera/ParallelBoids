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

void Flock::SenseAndPlan(const size_t ThreadID, const std::vector<Flock> &AllFlocks)
{
    for (size_t i = 0; i < Neighbourhood.size(); i++)
    {
        Neighbourhood[i].SenseAndPlan(AllFlocks, ThreadID);
    }
}

void Flock::Act(const double DeltaTime)
{
    // all boids advance one timestep, can be done asynrhconously bc indep
    if (Neighbourhood.size() > 0)
    {
        COM = Vec2D(0, 0);
        for (size_t i = 0; i < Neighbourhood.size(); i++)
        {
            Boid &B = Neighbourhood[i];
            B.Act(DeltaTime);
            COM += B.Position; // updates COM based off the most up-to-date boid positions
        }
        COM /= Neighbourhood.size();
    }
}

void Flock::Delegate(std::vector<Flock> &AllFlocks)
{
    // update flock decisions for others who are not in the same flocks
    Flock &F = AllFlocks[NearestFlockId(AllFlocks)];
    if (F.FlockID == FlockID)
        return; // skip self (this should never happen)
    for (Boid &B : Neighbourhood)
    {
        for (Boid &FB : F.Neighbourhood)
        {
            /// NOTE: can do cool stuff like if the dist to their flock's COM is less
            // than the distance to this own flock's COM
            if (B.DistanceLT(FB, B.Params.CollisionRadius))
            {
                /// NOTE: this is a very simple rule... only checking if
                if (Size() >= F.Size())
                {
                    // their flock is smaller, I recruit
                    // may be easier to atomically swap pointers
                    // rather than a big ol' critical section

#pragma omp critical
                    {
                        Recruit(B, F);
                    }
                }
                break; // don't need to check the rest bc they are all in the same flock
            }
        }
    }
}

void Flock::Recruit(Boid &B, Flock &BsFlock)
{
    const size_t TheirFlockID = B.FlockID;
    if (TheirFlockID == FlockID || Size() > Params.MaxSize)
    {
        // do nothing
        return;
    }
    // find position of boid in other neighbourhood
    std::vector<Boid> &OtherNeighbourhood = BsFlock.Neighbourhood;
    // update the newcomer's flock id
    B.FlockID = FlockID;
    // move B over to our flock
    Neighbourhood.push_back(std::move(B));
    // O(1) swap B with end of the other flock's neighbourhood for fast pop
    std::swap(B, OtherNeighbourhood.back());
    // destroy the last element (unusable after std::move) in OtherNeighbourhood
    OtherNeighbourhood.pop_back(); // destructive
}

size_t Flock::NearestFlockId(std::vector<Flock> &AllFlocks)
{
    // finds the flock physically nearest to this one
    size_t Idx = -1;            // should be a good number as long as >1 flocks exist
    double NearestDist = 1e300; // big num
    for (const Flock &F : AllFlocks)
    {
        double FDist = (COM - F.COM).Size();
        if (FDist < NearestDist && F.FlockID != FlockID)
        {
            Idx = F.FlockID;
            NearestDist = FDist;
        }
    }
    return Idx;
}

void Flock::Draw(Image &I) const
{
    /// TODO: check if can-parallelize?
    for (const Boid &B : Neighbourhood)
    {
        B.Draw(I);
    }
}
