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
    return Neighbourhood.Size();
}

void Flock::SenseAndPlan(const int TID, const std::unordered_map<size_t, Flock> &AllFlocks)
{
    assert(IsValidFlock()); // make sure this flock is valid
    // assert(NLayout::GetType() == NLayout::Local); // only on Local type
    TIDs.SenseAndPlan = TID;
    std::vector<Boid *> Boids = Neighbourhood.GetBoids();
    for (Boid *B : Boids)
    {
        B->SenseAndPlan(TID, this, AllFlocks);
    }
}

void Flock::Act(const double DeltaTime)
{
    // all boids advance one timestep, can be done asynrhconously bc indep
    assert(IsValidFlock()); // make sure this flock is valid
    std::vector<Boid *> Boids = Neighbourhood.GetBoids();
    for (Boid *B : Boids)
    {
        B->Act(DeltaTime);
    }
}

void Flock::Delegate(const int TID, const std::unordered_map<size_t, Flock> &AllFlocks)
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
    std::vector<Boid *> Boids = Neighbourhood.GetBoids();
    for (const Boid *B : Boids)
    {
        bool Emigrated = false; // whether or not this boid is leaving the flock
        for (const Flock *F : NearbyFlocks)
        {
            Tracer::AddRead(FlockID, F->FlockID, Flock::DelegateOp);
            std::vector<Boid *> FBoids = F->Neighbourhood.GetBoids();
            for (const Boid *Peer : FBoids)
            {
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
                        Emigrants[F->FlockID].push_back((*B));             // dereference from Boid*
                        Emigrants[F->FlockID].back().FlockID = F->FlockID; // update latest bucket's FiD
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
            Emigrants[FlockID].push_back((*B));
        }
    }
#ifndef NDEBUG
    // No boid left behind
    size_t NumLeaving = 0;
    for (const Flock *F : NearbyFlocks)
    {
        NumLeaving += Emigrants[F->FlockID].size();
        for (const Boid &B : Emigrants[F->FlockID])
        {
            assert(B.IsValid());
        }
    }
    size_t NumStaying = Emigrants[FlockID].size();
    assert(NumLeaving + NumStaying == Neighbourhood.Size());
#endif
}

void Flock::AssignToFlock(const int TID, const std::unordered_map<size_t, Flock> &AllFlocks)
{
    assert(IsValidFlock());
    TIDs.AssignToFlock = TID;
    if (AllFlocks.size() > 1) // if this is the last flock, do nothing
    {
        Neighbourhood.ClearLocal(); // clear my local neighbourhood
        for (auto It = AllFlocks.begin(); It != AllFlocks.end(); It++)
        {
            const Flock &Other = It->second;
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

void Flock::ComputeBB()
{
    if (Size() == 0)
        return;
    assert(IsValidFlock());
    std::vector<Boid *> Boids = Neighbourhood.GetBoids();
    BoundingBox NewBB(Boids[0]->Position); // initialize to Boids[0]'s position
    for (const Boid *B : Boids)
    {
        if (B->Position[0] < NewBB.TopLeftX)
            NewBB.TopLeftX = B->Position[0];
        if (B->Position[0] > NewBB.BottomRightX)
            NewBB.BottomRightX = B->Position[0];
        if (B->Position[1] < NewBB.TopLeftY)
            NewBB.TopLeftY = B->Position[1];
        if (B->Position[1] > NewBB.BottomRightY)
            NewBB.BottomRightY = B->Position[1];
    }
    // assign new bounding box with most extreme boid positions
    BB = NewBB;
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

std::vector<const Flock *> Flock::NearestFlocks(const std::unordered_map<size_t, Flock> &AllFlocks) const
{
    // finds the flocks physically nearest to this one
    std::vector<const Flock *> NearbyFlocks;
    /// TODO: figure out a better approach than this naive way
    std::vector<size_t> NearbyFIDs;
    const Vec2D Centroid = BB.Centroid();
    for (size_t i = 0; i < Params.MaxNumComm; i++)
    {
        double NearestDist = 1e300;                       // big num
        size_t NearestFlockID = AllFlocks.begin()->first; // first key
        size_t j = 0;
        for (auto It = AllFlocks.begin(); It != AllFlocks.end(); It++)
        {
            const Flock &F = It->second;
            if (!F.IsValidFlock())
                continue; // ignore invalid flocks
            // distance to center of bounding boxes (centroids)
            double FDist = (Centroid - F.BB.Centroid()).Size();
            if (FDist < NearestDist && F.FlockID != FlockID)
            {
                // make sure this boid hasn't been selected before
                bool AlreadyAdded = false;
                for (const size_t ExistingNearbyFID : NearbyFIDs)
                {
                    if (F.FlockID == ExistingNearbyFID)
                    {
                        AlreadyAdded = true;
                        break;
                    }
                }
                if (!AlreadyAdded)
                {
                    NearestDist = FDist;
                    NearestFlockID = F.FlockID;
                }
            }
            j++;
        }
        NearbyFlocks.push_back(&(AllFlocks.find(NearestFlockID)->second)); // ptr to flock
        NearbyFIDs.push_back(NearestFlockID);
    }
    return NearbyFlocks;
}

void Flock::Draw(Image &I) const
{
    assert(IsValidFlock());

    /// TODO: check if can-parallelize?
    std::vector<Boid *> Boids = Neighbourhood.GetBoids();
    I.DrawStrokedRect(BB.TopLeftX, BB.TopLeftY, BB.BottomRightX, BB.BottomRightY);
    for (Boid *B : Boids)
    {
        B->Draw(I);
    }
}

void Flock::CleanUp(std::unordered_map<size_t, Flock> &AllFlocks)
{
    /// NOTE: this can probably be parallelized as well...
    // remove all empty (invalid) flocks
    /// TODO: Implement this with 210-style filter for O(logn) span
    // for vector
    // auto IsInvalid = [](const Flock &F) { return !F.Valid; };
    // AllFlocks.erase(std::remove_if(AllFlocks.begin(), AllFlocks.end(), IsInvalid), AllFlocks.end());

    // for unordered_map
    // graciously retrieved from https://en.cppreference.com/w/cpp/container/unordered_map/erase_if
    for (auto i = AllFlocks.begin(), last = AllFlocks.end(); i != last;)
    {
        if (!(i->second.Valid))
        {
            i = AllFlocks.erase(i);
        }
        else
        {
            ++i;
        }
    }
#ifndef NDEBUG
    for (auto It = AllFlocks.begin(), Last = AllFlocks.end(); It != Last; It++)
    {
        assert(It != AllFlocks.end());
        const Flock &F = It->second;
        assert(F.IsValidFlock());
    }
#endif
}