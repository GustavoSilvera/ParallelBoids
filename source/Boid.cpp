#include "Boid.hpp"
#include "Flock.hpp"  // To see all other neighbourhoods
#include "Tracer.hpp" // to keep track of memory traces
#include <unordered_set>

// declaring static variables
size_t Boid::NumBoids;
BoidParamsStruct Boid::Params;

bool Boid::IsValid() const
{
    if (BoidID > NumBoids)
        return false;
    if (FlockID > NumBoids)
        return false;
    return true;
}

size_t Boid::GetFlockID() const
{
    return FlockID;
}

void Boid::SenseAndPlan(const int TID, const std::unordered_map<size_t, Flock> &AllFlocks)
{
    // reset current force factors
    assert(IsValid());
    a1 = Vec2D(0, 0);
    a2 = Vec2D(0, 0);
    a3 = Vec2D(0, 0);
    Vec2D RelCOM, RelCOV, Sep; // relative center-of-mass/velocity, & separation
    size_t NumCloseby = 0;
    auto It = AllFlocks.find(FlockID);
    assert(It != AllFlocks.end());
    const Flock &ThisFlock = It->second;
    // begin sensing all other boids in all other flocks
    ThreadID = TID;
    for (auto It = AllFlocks.begin(); It != AllFlocks.end(); It++)
    {
        assert(It != AllFlocks.end());
        const Flock &F = It->second;

        assert(F.IsValidFlock());
        // if flock is close enough (correct bc bounding boxes)
        // after extending our BB
        if (F.BB.IntersectsBB(ThisFlock.BB, Params.NeighbourhoodRadius))
        {
            std::vector<Boid *> Boids = F.Neighbourhood.GetBoids();
            for (const Boid *B : Boids)
            {
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

void Boid::Act(const float DeltaTime)
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
        const float OverlapAmnt = 1 - ((Neighbour.Position - Position).Size() / (2 * Params.Radius));
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
    float ClampedX = Position[0];
    if (ClampedX < 0)
    {
        ClampedX += MaxW;
    }
    else if (ClampedX > MaxW)
    {
        ClampedX -= MaxW;
    }
    /// same for y's
    float ClampedY = Position[1];
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

bool Boid::DistanceGT(const Boid &B, const float Rad) const
{
    return ((Position - B.Position).SizeSqr() > sqr(Rad));
}

bool Boid::DistanceLT(const Boid &B, const float Rad) const
{
    return ((Position - B.Position).SizeSqr() < sqr(Rad));
}

float Boid::DistanceTo(const Boid &B) const
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

void Boid::Destroy()
{
    NumBoids = 0;
}