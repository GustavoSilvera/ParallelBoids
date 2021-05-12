#include "Boid.hpp"
#include "Flock.hpp"  // To see all other neighbourhoods
#include "Tracer.hpp" // to keep track of memory traces
#include <unordered_set>

// declaring static variables
size_t Boid::NumBoids;
BoidParamsStruct Boid::Params;

bool Boid::IsValid() const
{
    if (FlockPtr == nullptr)
        return false;
    assert(FlockPtr->FlockID != FlockID);
    return true;
}

size_t Boid::GetFlockID() const
{
    return FlockID;
}

void Boid::SenseAndPlan(const int TID, const std::vector<Flock> &AllFlocks)
{
    // reset current force factors
    assert(IsValid());
    a1 = Vec2D(0, 0);
    a2 = Vec2D(0, 0);
    a3 = Vec2D(0, 0);
    Vec2D RelCOM, RelCOV, Sep; // relative center-of-mass/velocity, & separation
    size_t NumCloseby = 0;
    // begin sensing all other boids in all other flocks
    ThreadID = TID;
    assert(FlockPtr->IsValidFlock());
    for (const Flock &F : AllFlocks)
    {
        assert(F.IsValidFlock());
        // if flock is close enough (correct bc bounding boxes)
        if (F.BB.IntersectsBB(FlockPtr->BB)) // if flock is close enough
        {
            std::vector<Boid *> Boids = F.Neighbourhood.GetBoids();
            for (const Boid *B : Boids)
            {
                std::cout << "planning" << std::endl;
                // begin planning for this boid for each boid that is sensed
                Plan((*B), RelCOM, RelCOV, Sep, NumCloseby);
            }
        }
    }

    if (NumCloseby > 0)
    {
        a1 = ((RelCOM / NumCloseby) - Position) * Params.Cohesion;
        a2 = Sep * Params.Separation; // dosent depent on NumCloseby but makes sense
        a3 = ((RelCOV / NumCloseby) - Velocity) * Params.Alignment;
    }
}

void Boid::SenseAndPlan(const int TID, const std::vector<Boid> &AllBoids)
{
    assert(IsValid());
    // reset current force factors
    a1 = Vec2D(0, 0);
    a2 = Vec2D(0, 0);
    a3 = Vec2D(0, 0);
    Vec2D RelCOM, RelCOV, Sep; // relative center-of-mass/velocity, & separation
    size_t NumCloseby = 0;
    /// NOTE: that thread ID's are handled by the Simulator openmp, not Flocks anymore
    ThreadID = TID;
    // begin sensing all other boids in all other flocks

    // use associative containers to do fast checks for boids
    std::unordered_set<size_t> FarAwayFlocks;
    std::unordered_set<size_t> NearbyFlocks;
    for (const Boid &B : AllBoids)
    {
        assert(B.IsValid());
        // we know this boid is close enough, so plan with it
        if (NearbyFlocks.find(B.FlockID) != NearbyFlocks.end())
        {
            // we know this boid is close enough, so plan with it
            Plan(B, RelCOM, RelCOV, Sep, NumCloseby);
        }
        else
        {
            // first check if the boid is in a far away flock
            if (FarAwayFlocks.find(B.FlockID) == FarAwayFlocks.end())
            {
                // if flock is too far away, add it to the "ignore" set
                if (B.FlockPtr->BB.IntersectsBB(FlockPtr->BB))
                {
                    // now we know this boid is in a "close enough" flock
                    // so we don't need to compute intersections anymore
                    NearbyFlocks.insert(B.FlockID);
                    Plan(B, RelCOM, RelCOV, Sep, NumCloseby);
                }
                else
                {
                    FarAwayFlocks.insert(B.FlockID);
                }
            }
            // else ignore, this case occurs if the boid is in a FarAwayFlock
        }
    }

    if (NumCloseby > 0)
    {
        a1 = ((RelCOM / NumCloseby) - Position) * Params.Cohesion;
        a2 = Sep * Params.Separation; // dosent depent on NumCloseby but makes sense
        a3 = ((RelCOV / NumCloseby) - Velocity) * Params.Alignment;
    }
}

void Boid::Plan(const Boid &B, Vec2D &RelativeCOM, Vec2D &AvgVel, Vec2D &SeparationDisp, size_t &NumCloseby) const
{
    assert(IsValid());
    // add to the tracer
    Tracer::AddRead(GetFlockID(), B.GetFlockID(), Flock::SenseAndPlanOp);

    if (B.BoidID == BoidID)
        return; // don't plan with self

    if (DistanceGT(B, Params.NeighbourhoodRadius))
        return; // too far away: ignore

    // Makes local decisions based off the current neighbours
    /// NOTE: (all logic is done on the current velocity/positions which are read-only)
    RelativeCOM += B.Position; // contribute to relative center-of-mass
    AvgVel += B.Velocity;      // contribute to average velocity
    if (DistanceLT(B, Params.CollisionRadius))
        SeparationDisp -= (B.Position - Position); // contribute to displacement
    // Finally increment the count for number of closeby boids
    NumCloseby++;
}

void Boid::Act(const double DeltaTime)
{
    assert(IsValid());
    /// NOTE: This function is meant to be independent from all other boids
    /// and thus can be run asynchronously, however it needs a barrier between itself
    /// and the Boid::Plan() function
    Acceleration = a1 + a2 + a3; // + a4
    Velocity = (Velocity + Acceleration).LimitMagnitude(Params.MaxVel);
    Position += Velocity * DeltaTime;
    // EdgeWrap(); // optional
}

void Boid::CollisionCheck(Boid &Neighbour)
{
    assert(IsValid());
    /// TODO: fix the tracking for high-tick timings
    if (Neighbour.BoidID != BoidID &&                                       // not self
        (Neighbour.Position - Position).SizeSqr() < sqr(2 * Params.Radius)) // only physical collision
    {
        const double OverlapAmnt = 1 - ((Neighbour.Position - Position).Size() / (2 * Params.Radius));
        Position -= (Neighbour.Position - Position) * OverlapAmnt;
    }
}

void Boid::Draw(Image &I) const
{
    assert(IsValid());
    Colour C(255, 255, 255);
    if (Params.ColourByThread)
    {
        C = IDColours[ThreadID % IDColours.size()];
    }
    else
    {
        C = IDColours[FlockID % IDColours.size()];
    }
    I.DrawSolidCircle(Position, Params.Radius, C);
    // also render line to indicate direction
    const size_t LineWidth = 2 * Params.Radius; // number pixels
    Vec2D Heading = Velocity.Norm();
    Vec2D End = Position + Heading * LineWidth;
    I.DrawLine(Position, End, C);
}

void Boid::EdgeWrap()
{
    // used to wrap the boids around to the other side of the window
    size_t MaxW = GlobalParams.ImageParams.WindowX - 1;
    size_t MaxH = GlobalParams.ImageParams.WindowY - 1;
    double ClampedX = Position[0];
    if (ClampedX < 0)
    {
        ClampedX += MaxW;
    }
    else if (ClampedX > MaxW)
    {
        ClampedX -= MaxW;
    }
    /// same for y's
    double ClampedY = Position[1];
    if (ClampedY < 0)
    {
        ClampedY += MaxH;
    }
    else if (ClampedY > MaxH)
    {
        ClampedY -= MaxH;
    }
    Position = Vec2D(ClampedX, ClampedY);
}

bool Boid::DistanceGT(const Boid &B, const double Rad) const
{
    return ((Position - B.Position).SizeSqr() > sqr(Rad));
}

bool Boid::DistanceLT(const Boid &B, const double Rad) const
{
    return ((Position - B.Position).SizeSqr() < sqr(Rad));
}

double Boid::DistanceTo(const Boid &B) const
{
    return (Position - B.Position).Size();
}

bool Boid::operator==(const Boid &B) const
{
    return BoidID == B.BoidID;
}

bool operator==(const Boid &B1, const Boid &B2)
{
    return B1.BoidID == B2.BoidID;
}