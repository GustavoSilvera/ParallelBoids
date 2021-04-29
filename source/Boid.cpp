#include "Boid.hpp"
#include "Flock.hpp"
#include <omp.h>

void Boid::Sense(std::vector<Flock> &AllFlocks, const int tID)
{
    // finds all the nearby neighbours
    /// TODO: figure out something smarter than clearing, maybe share across ticks
    SenseTid = tID;
    Neighbours.clear();
    /// NOTE: this can be optimized heavily
    for (Flock &F : AllFlocks)
    {
        /// find the nearest boid and determine who else to look for
        for (Boid &B : F.Neighbourhood)
        {
            if (B.BoidID != BoidID &&                                                // not self
                (B.Position - Position).SizeSqr() < sqr(Params.NeighbourhoodRadius)) // within this visibility
            {
                Neighbours.push_back(&B);
            }
        }
    }
}

void Boid::Plan(std::vector<Flock> &Flocks, const int tID)
{
    // reset current force factors
    a1 = Vec2D(0, 0);
    a2 = Vec2D(0, 0);
    a3 = Vec2D(0, 0);
    PlanTid = tID;
    // Makes local decisions based off the current neighbours
    if (Neighbours.size() > 0)
    {
        Vec2D RelativeCOM, AvgVel, SeparationDisp;
        for (Boid *Neighbour : Neighbours) // exactly the neighbours we care about
        {
            RelativeCOM += Neighbour->Position;
            AvgVel += Neighbour->Velocity;
            if ((Neighbour->Position - Position).SizeSqr() < sqr(Params.CollisionRadius)) // real close
            {
                SeparationDisp -= (Neighbour->Position - Position); // add to displacement
#pragma omp critical
                {
                    // NOTE: or find the max flock, instead of the first one
                    if (FlockID != Neighbour->FlockID && Flocks[FlockID].Size() >= Flocks[Neighbour->FlockID].Size())
                    {
                        // their flock is smaller, I recruit
                        Flocks[FlockID].Recruit(*Neighbour, Flocks);
                    }
                }
            }
        }
        // NumNeighbours > 0
        a1 = ((RelativeCOM / Neighbours.size()) - Position) * Params.Cohesion;
        a2 = SeparationDisp * Params.Separation;
        a3 = ((AvgVel / Neighbours.size()) - Velocity) * Params.Alignment;
    }
}

void Boid::Act(const double DeltaTime, const int tID)
{
    ActTid = tID;
    Acceleration = a1 + a2 + a3; // + a4
    Velocity = (Velocity + Acceleration).LimitMagnitude(Params.MaxVel);
    Position += Velocity * DeltaTime;
    CollisionCheck(); // resets position to avoid collisions
    EdgeWrap();
}

void Boid::CollisionCheck()
{
    /// TODO: fix the tracking for high-tick timings
    for (const Boid *Neighbour : Neighbours) // move away from any overlapping neighbours
    {
        if (Neighbour->BoidID != BoidID &&                                       // not self
            (Neighbour->Position - Position).SizeSqr() < sqr(2 * Params.Radius)) // only within neighbourhood
        {
            const double OverlapAmnt = 1 - ((Neighbour->Position - Position).Size() / (2 * Params.Radius));
            Position -= (Neighbour->Position - Position) * OverlapAmnt;
        }
    }
}

void Boid::Draw(Image &I) const
{
    Colour C(255, 255, 255);
    if (Params.ColourByThread)
    {
        if (SenseTid == PlanTid && PlanTid == ActTid)
            C = IDColours[ActTid % IDColours.size()];
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
