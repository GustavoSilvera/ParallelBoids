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
        B->SenseAndPlan(TID, AllFlocks);
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

    // clear buckets from last Delegation
    Emigrants.clear(); // if not done first, may get double counting later

    // Look through our neighbourhood
    const std::vector<Boid *> Boids = Neighbourhood.GetBoids();
    std::vector<std::pair<double, size_t>> BestBoidFlocks(Boids.size(),                    // corresponding to Boids
                                                          std::make_pair(1e300, FlockID)); // this flock
    for (auto It = AllFlocks.begin(); It != AllFlocks.end(); It++)
    {
        assert(It != AllFlocks.end());
        const Flock &F = It->second;
        Tracer::AddRead(FlockID, F.FlockID, Flock::DelegateOp);

        if (F.BB.IntersectsBB(BB, GlobalParams.BoidParams.NeighbourhoodRadius))
        {
            for (size_t b = 0; b < Boids.size(); b++)
            {
                const Boid *B = Boids[b];
                std::vector<Boid *> FBoids = F.Neighbourhood.GetBoids();
                for (const Boid *Peer : FBoids)
                {
                    if (Peer->BoidID == B->BoidID)
                        continue; // skip self
                    Tracer::AddRead(B->GetFlockID(), Peer->GetFlockID(), Flock::SenseAndPlanOp);
                    const double Dist = B->DistanceTo((*Peer));
                    /// NOTE: this is a very simple rule... only checking if
                    // their flock is larger/eq, then I send them over there
                    bool FlockRule = (Size() <= F.Size()) && (Dist < B->Params.CollisionRadius);

                    if (Dist < BestBoidFlocks[b].first && FlockRule)
                    {
                        // std::cout << Dist << std::endl;
                        BestBoidFlocks[b] = std::make_pair(Dist, F.FlockID);
                    }
                }
            }
        }
    }
    for (size_t b = 0; b < Boids.size(); b++)
    {
        size_t BestFlockID = BestBoidFlocks[b].second;
        Emigrants[BestFlockID].push_back((*Boids[b]));
        Emigrants[BestFlockID].back().FlockID = BestFlockID;
    }
#ifndef NDEBUG
    // No boid left behind
    size_t NumBuckets = 0;
    for (auto It = AllFlocks.begin(); It != AllFlocks.end(); It++)
    {
        const Flock F = It->second;
        NumBuckets += Emigrants[F.FlockID].size();
        for (const Boid &B : Emigrants[F.FlockID])
        {
            assert(B.IsValid());
        }
    }

    assert(NumBuckets == Neighbourhood.Size());
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