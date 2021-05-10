#include "Flock.hpp"
#include <algorithm>
#include <cassert>
#include <omp.h> // OpenMP

// declaring static variables
FlockParamsStruct Flock::Params;

int Flock::Size() const
{
    int S = Neighbourhood.size();
    assert(S >= 0);
    return S;
}

void Flock::SenseAndPlan(const size_t ThreadID, const std::vector<Flock> &AllFlocks)
{
    assert(Valid); // make sure this flock is valid
    for (Boid &B : Neighbourhood)
    {
        B.SenseAndPlan(AllFlocks, ThreadID);
    }
}

void Flock::Act(const double DeltaTime)
{
    // all boids advance one timestep, can be done asynrhconously bc indep
    assert(Valid); // make sure this flock is valid
    COM = Vec2D(0, 0);
    for (size_t i = 0; i < Neighbourhood.size(); i++)
    {
        Boid &B = Neighbourhood[i];
        B.Act(DeltaTime);
        COM += B.Position; // updates COM based off the most up-to-date boid positions
    }
    COM /= Neighbourhood.size(); // we know Neighbourhood.size() > 0 bc Valid
}

void Flock::Delegate(const std::vector<Flock> &AllFlocks)
{

    assert(Valid); // make sure this flock is valid
    // update flock decisions for local neighbourhood based off nearby flocks
    // can also maybe get "top 5 closest"
    const std::vector<const Flock *> NearbyFlocks = NearestFlocks(AllFlocks);
    if (NearbyFlocks.size() == 0)
    {
        std::cout << "No more flocks!" << std::endl;
        return;
    }

    // clear buckets from last Delegation
    Emigrants.clear(); // if not done first, may get double counting later

    // Look through our neighbourhood
    for (size_t i = 0; i < Neighbourhood.size(); i++)
    {
        Boid &B = Neighbourhood[i];
        bool Emigrated = false; // whether or not this boid is leaving the flock
        for (const Flock *F : NearbyFlocks)
        {
            for (const Boid &Peer : F->Neighbourhood)
            {
                /// NOTE: can do cool stuff like if the dist to their flock's COM is less
                // than the distance to this own flock's COM
                if (B.DistanceLT(Peer, B.Params.CollisionRadius))
                {
                    /// NOTE: this is a very simple rule... only checking if
                    // their flock is larger/eq, then I send them over there
                    bool FlockRule = (Size() <= F->Size());
                    if (FlockRule)
                    {
                        Emigrants[F->FlockID].push_back(B);
                        Emigrants[F->FlockID].back().FlockID = F->FlockID; // update latest bucket's FiD
                        Emigrated = true;                                  // indicate that this boid is part of the emigration bucket
                    }
                    break; // don't need to check the rest bc they are all in the same flock
                    // ie. as soon as one member of their flock satisfies our condition, we just say ok
                    // and delegate this boid (B) to join them
                }
            }
            if (Emigrated)
            {
                // Don't need to check any other flocks, already going to the nearest one
                break;
            }
        }
        if (!Emigrated)
        {
            // stores all the boids (unchanged) into what will be this flock's new neighbourhood
            Emigrants[FlockID].push_back(B);
        }
    }
#ifndef NDEBUG
    // No boid left behind
    size_t NumLeaving = 0;
    for (const Flock *F : NearbyFlocks)
    {
        NumLeaving += Emigrants[F->FlockID].size();
    }
    size_t NumStaying = Emigrants[FlockID].size();
    assert(NumLeaving + NumStaying == Neighbourhood.size());
#endif
}

void Flock::AssignToFlock(const std::vector<Flock> &AllFlocks)
{
    assert(Valid);
    if (AllFlocks.size() > 1) // if this is the last flock, do nothing
    {
        Neighbourhood.clear(); // clear my local neighbourhood

        for (const Flock &Other : AllFlocks)
        {
            // O(1) dictionary accesses
            if (Other.Emigrants.find(FlockID) != Other.Emigrants.end())
            {
                const std::vector<Boid> &Immigrants = Other.Emigrants.at(FlockID);
                // don't need a critical section bc writing to local, reading from remote
                Neighbourhood.insert(Neighbourhood.end(), Immigrants.begin(), Immigrants.end());
            }
        }
    }
    Valid = (Size() > 0); // need to have at least one boid to be a valid flock
}

void Flock::Recruit(Boid &B, Flock &BsFlock)
{
    assert(Valid);
    const size_t TheirFlockID = B.GetFlockID();
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

std::vector<const Flock *> Flock::NearestFlocks(const std::vector<Flock> &AllFlocks) const
{
    // finds the flocks physically nearest to this one
    std::vector<const Flock *> NearbyFlocks;
    /// TODO: figure out a better approach than this naive way
    std::vector<size_t> NearbyIdxs;
    for (size_t i = 0; i < Params.MaxNumComm; i++)
    {
        double NearestDist = 1e300; // big num
        size_t NearestIdx = 0;
        for (size_t j = 0; j < AllFlocks.size(); j++)
        {
            const Flock &F = AllFlocks[j];
            if (!F.Valid)
                continue; // ignore invalid flocks
            double FDist = (COM - F.COM).Size();
            if (FDist < NearestDist && F.FlockID != FlockID)
            {
                // make sure this boid hasn't been selected before
                bool AlreadyAdded = false;
                for (const size_t ExistingNearbyIdx : NearbyIdxs)
                {
                    if (j == ExistingNearbyIdx)
                    {
                        AlreadyAdded = true;
                        break;
                    }
                }
                if (!AlreadyAdded)
                {
                    NearestDist = FDist;
                    NearestIdx = j;
                }
            }
        }
        NearbyFlocks.push_back(&AllFlocks[NearestIdx]);
        NearbyIdxs.push_back(NearestIdx);
    }
    return NearbyFlocks;
}

void Flock::Draw(Image &I) const
{
    assert(Valid);

    /// TODO: check if can-parallelize?
    for (const Boid &B : Neighbourhood)
    {
        B.Draw(I);
    }
}

void Flock::CleanUp(std::vector<Flock> &AllFlocks)
{
    /// NOTE: this can probably be parallelized as well...
    // remove all empty (invalid) flocks
    auto IsInvalid = [](const Flock &F) {
        assert(F.Valid == (F.Size() > 0));
        return !F.Valid;
    };
    AllFlocks.erase(std::remove_if(AllFlocks.begin(), AllFlocks.end(), IsInvalid), AllFlocks.end());
#ifndef NDEBUG
    for (const Flock &A : AllFlocks)
    {
        assert(A.Valid);
    }
#endif
}