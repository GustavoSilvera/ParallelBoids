#include "Flock.hpp"
#include "Tracer.hpp"
#include <algorithm>
#include <cassert>

// declaring static variables
FlockParamsStruct Flock::Params;

bool Flock::IsValidFlock() const
{
    if (!Valid)
        return false;
    if (Size() > Params.MaxSize)
        return false;
    return true;
}

size_t Flock::Size() const
{
    return Neighbourhood.Size(FlockID);
}

void Flock::SenseAndPlan(const int TID, const std::vector<Flock> &AllFlocks)
{
    assert(IsValidFlock());                     // make sure this flock is valid
    assert(NLayout::GetType() == NLayout::AoS); // only on AoS type
    TIDs.SenseAndPlan = TID;
    for (size_t i = 0; i < Size(); i++)
    {
        Neighbourhood[i]->SenseAndPlan(this, AllFlocks);
    }
}

void Flock::Act(const double DeltaTime)
{
    // all boids advance one timestep, can be done asynrhconously bc indep
    assert(IsValidFlock());                     // make sure this flock is valid
    assert(NLayout::GetType() == NLayout::AoS); // only on AoS type
    COM = Vec2D(0, 0);
    for (size_t i = 0; i < Size(); i++)
    {
        Boid *B = Neighbourhood[i];
        B->Act(DeltaTime);
        COM += B->Position; // updates COM based off the most up-to-date boid positions
    }
    COM /= Size(); // we know Neighbourhood.size() > 0 bc Valid
}

void Flock::Delegate(const int TID, const std::vector<Flock> &AllFlocks)
{
    assert(IsValidFlock()); // make sure this flock is valid
    TIDs.Delegate = TID;
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
    for (size_t i = 0; i < Size(); i++)
    {
        Boid *B = Neighbourhood[i];
        bool Emigrated = false; // whether or not this boid is leaving the flock
        for (const Flock *F : NearbyFlocks)
        {
            Tracer::AddRead(FlockID, F->FlockID, Flock::DelegateOp);
            for (size_t j = 0; j < F->Size(); j++)
            {
                const Boid *Peer = F->Neighbourhood[j];
                Tracer::AddRead(B->GetFlockID(), Peer->GetFlockID(), Flock::SenseAndPlanOp);
                /// TODO: should I keep track of the boid->boid communication in traces too?
                // Tracer::AddRead(B.FlockID, Peer.FlockID, Flock::Delegate);
                /// NOTE: can do cool stuff like if the dist to their flock's COM is less
                // than the distance to this own flock's COM
                if (B->DistanceLT((*Peer), B->Params.CollisionRadius) && F->Size() < F->Params.MaxSize)
                {
                    /// NOTE: this is a very simple rule... only checking if
                    // their flock is larger/eq, then I send them over there
                    bool FlockRule = (Size() <= F->Size());
                    if (FlockRule)
                    {
                        Emigrants[F->FlockID].push_back((*B));              // dereference from Boid*
                        Emigrants[F->FlockID].back()->FlockID = F->FlockID; // update latest bucket's FiD
                        Emigrated = true; // indicate that this boid is part of the emigration bucket
                    }
                    break; // don't need to check the rest bc they are all in the same flock
                    // ie. as soon as one member of their flock satisfies our condition, we just say ok
                    // and delegate this boid (B) to join them
                }
            }

            /// TODO: Actually do need to check other flocks. Track best flock so far, and emigrate
            //        to best flock.
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

void Flock::AssignToFlock(const int TID, const std::vector<Flock> &AllFlocks)
{
    assert(IsValidFlock());
    TIDs.AssignToFlock = TID;
    if (AllFlocks.size() > 1) // if this is the last flock, do nothing
    {
        Neighbourhood.ClearLocal(); // clear my local neighbourhood
        for (const Flock &Other : AllFlocks)
        {
            // Tracer::AddRead(FlockID, Other.FlockID, Flock::AssignToFlockOp);
            // O(1) dictionary accesses
            if (Other.Emigrants.find(FlockID) != Other.Emigrants.end())
            {
                const std::vector<Boid> &Immigrants = Other.Emigrants.at(FlockID);
                // may require critical section if using Global boids vector
                Neighbourhood.Append(Immigrants);
            }
        }
    }
    Valid = (Size() > 0); // need to have at least one boid to be a valid flock
}

// void Flock::Recruit(Boid &B, Flock &BsFlock)
// {
//     /// NOTE: this is depracated
//     assert(IsValidFlock());
//     const size_t TheirFlockID = B.GetFlockID();
//     if (TheirFlockID == FlockID || Size() > int(Params.MaxSize))
//     {
//         // do nothing
//         return;
//     }
//     // find position of boid in other neighbourhood
//     std::vector<Boid> &OtherNeighbourhood = BsFlock.Neighbourhood;
//     // update the newcomer's flock id
//     B.FlockID = FlockID;
//     // move B over to our flock
//     Neighbourhood.push_back(std::move(B));
//     // O(1) swap B with end of the other flock's neighbourhood for fast pop
//     std::swap(B, OtherNeighbourhood.back());
//     // destroy the last element (unusable after std::move) in OtherNeighbourhood
//     OtherNeighbourhood.pop_back(); // destructive
// }

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
            if (!F.IsValidFlock())
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
    assert(IsValidFlock());

    /// TODO: check if can-parallelize?
    for (size_t i = 0; i < Size(); i++)
    {
        const Boid *B = Neighbourhood[i];
        B->Draw(I);
    }
}

void Flock::CleanUp(std::vector<Flock> &AllFlocks)
{
    /// NOTE: this can probably be parallelized as well...
    // remove all empty (invalid) flocks
    /// TODO: Implement this with 210-style filter for O(logn) span
    auto IsInvalid = [](const Flock &F) { return !F.Valid; };
    AllFlocks.erase(std::remove_if(AllFlocks.begin(), AllFlocks.end(), IsInvalid), AllFlocks.end());
#ifndef NDEBUG
    for (const Flock &A : AllFlocks)
    {
        assert(A.IsValidFlock());
    }
#endif
}