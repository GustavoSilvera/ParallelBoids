#include "Boid.hpp"
#include "Flock.hpp"
#include <omp.h>

// declaring NumBoids for all boids (static)
size_t Boid::NumBoids;

size_t Boid::GetFlockID() const
{
    return FlockID;
}

void Boid::SenseAndPlan(const std::vector<Flock> &Flocks, const int tID)
{
    ThreadID = tID;
    // reset current force factors
    a1 = Vec2D(0, 0);
    a2 = Vec2D(0, 0);
    a3 = Vec2D(0, 0);
    Vec2D RelCOM, RelCOV, Sep; // relative center-of-mass/velocity, & separation
    size_t NumCloseby = 0;
    // begin sensing all other boids in all other flocks

    /// This can be optimized heavily, for instance, what if we used an NxN array where
    // each point is a 'set' of boids, then we can quickly index to spacially local
    // boids
    for (const Flock &F : Flocks)
    {
        /// TODO: find the nearest boid and determine who else to look for
        for (const Boid &B : F.Neighbourhood)
        {
            // begin planning for this boid for each boid that is sensed
            Plan(B, RelCOM, RelCOV, Sep, NumCloseby);
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
    if (B.BoidID == BoidID)
        return; // don't plan with self
    if (DistanceGT(B, Params.NeighbourhoodRadius))
        return; // too far away: ignore

    // race condition
    // CollisionCheck(B); // resets position to avoid collisions

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
    /// NOTE: This function is meant to be independent from all other boids
    /// and thus can be run asynchronously, however it needs a barrier between itself
    /// and the Boid::Plan() function
    Acceleration = a1 + a2 + a3; // + a4
    Velocity = (Velocity + Acceleration).LimitMagnitude(Params.MaxVel);
    Position += Velocity * DeltaTime;
    EdgeWrap(); // optional
}

void Boid::CollisionCheck(Boid &Neighbour)
{
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