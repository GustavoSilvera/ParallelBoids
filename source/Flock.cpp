#include "Flock.hpp"
#include <algorithm>
#include <cassert>
#include <omp.h> // OpenMP

int Flock::Size() const
{
    int S = Neighbourhood.size();
    assert(S >= 0);
    return S;
}

void Flock::SenseAndPlan(const size_t ThreadID, const std::vector<Flock> &AllFlocks)
{
    if (!Valid) // make sure this flock is valid
        return;

    for (size_t i = 0; i < Neighbourhood.size(); i++)
    {
        Neighbourhood[i].SenseAndPlan(AllFlocks, ThreadID);
    }
}

void Flock::Act(const double DeltaTime)
{
    // all boids advance one timestep, can be done asynrhconously bc indep
    if (!Valid) // make sure this flock is valid
        return;
    COM = Vec2D(0, 0);
    for (size_t i = 0; i < Neighbourhood.size(); i++)
    {
        Boid &B = Neighbourhood[i];
        B.Act(DeltaTime);
        COM += B.Position; // updates COM based off the most up-to-date boid positions
    }
    COM /= Neighbourhood.size();
}

void Flock::Delegate(std::vector<Flock> &AllFlocks)
{
    if (!Valid) // make sure this flock is valid
        return;
    // update flock decisions for local neighbourhood based off nearby flocks
    const Flock &F = AllFlocks[NearestFlockId(AllFlocks)]; // can also maybe get "top 5 closest"
    if (F.FlockID == FlockID)
        return; // skip self (this should never happen)

    // clear buckets from last Delegation
    Buckets.clear();

    // technically don't need dictionary rn, since we're sending to the same flock
    Buckets[F.FlockID] = std::vector<size_t>();

    // Look through our neighbourhood
    for (size_t i = 0; i < Neighbourhood.size(); i++)
    {
        Boid &B = Neighbourhood[i];
        for (const Boid &FB : F.Neighbourhood)
        {
            /// NOTE: can do cool stuff like if the dist to their flock's COM is less
            // than the distance to this own flock's COM
            if (B.DistanceLT(FB, B.Params.CollisionRadius))
            {
                /// NOTE: this is a very simple rule... only checking if
                // their flock is larger/eq, then I send them over there
                bool FlockRule = (Size() <= F.Size());
                if (FlockRule)
                {
                    Buckets[F.FlockID].push_back(i); // copy the index over to indicate
                    // that this boid (B=Neighbourhood[i]) is moving over
                }
                break; // don't need to check the rest bc they are all in the same flock
                // ie. as soon as one member of their flock satisfies our condition, we just say ok
                // and delegate this boid (B) to join them
            }
        }
    }
}

void Flock::AssignToFlock(std::vector<Flock> &AllFlocks)
{
    // for all flocks that we are communicating with
    for (std::pair<const size_t, std::vector<size_t>> &V : Buckets)
    {
        const size_t OtherFlockID = V.first;    // key
        std::vector<size_t> &Bucket = V.second; // value

        // may be easier to atomically swap pointers
        // rather than a big ol' critical section
        for (const size_t EmigrantIdx : Bucket)
        {
#pragma omp critical
            {
                /// TODO: fine grained locking (per flock) rather than big critical over all
                // thread-safe insert all bucket values to this vector
                std::vector<Boid> &N = AllFlocks[OtherFlockID].Neighbourhood;

                /// NOTE: using std::move to non-destructively move the ownership
                Boid &Emigrant = Neighbourhood[EmigrantIdx]; // reference to emigrant boid
                Emigrant.FlockID = OtherFlockID;
                N.push_back(std::move(Emigrant));

                // Now Emigrant is in an unusable state & must be removed from this flock
                // O(1) swap with end of the other flock's neighbourhood for fast pop
                std::swap(Emigrant, Neighbourhood.back());
                // destroy the last element (Emigrant) in Neighbourhood
                Neighbourhood.pop_back(); // destructive
            }
        }
    }
    Valid = (Size() > 0); // need to have at least one boid to be a valid flock
}

void Flock::Recruit(Boid &B, Flock &BsFlock)
{
    if (!Valid) // make sure this flock is valid
        return;
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

size_t Flock::NearestFlockId(const std::vector<Flock> &AllFlocks) const
{
    // finds the flock physically nearest to this one
    size_t Idx = -1;            // should be a good number as long as >1 flocks exist
    double NearestDist = 1e300; // big num
    for (const Flock &F : AllFlocks)
    {
        if (!F.Valid)
            continue; // ignore invalid flocks
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
    if (!Valid) // make sure this flock is valid
        return;

    /// TODO: check if can-parallelize?
    for (const Boid &B : Neighbourhood)
    {
        B.Draw(I);
    }
}
