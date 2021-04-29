#ifndef BOID
#define BOID

#include "Image.hpp"
#include "Utils.hpp"
#include "Vec.hpp"
#include <omp.h>
#include <vector>

class Boid
{
  public:
    Boid(const double x0, const double y0, const double dx0, const double dy0, const size_t BID, const size_t FID)
    {
        Params = GlobalParams.BoidParams;
        Position = Vec2D(x0, y0);   // set posixtion
        Velocity = Vec2D(dx0, dy0); // set initial velocity
        FlockID = FID;              // initial flock assignment
        BoidID = BID;               // BoidID is unique per boid
    }
    Vec2D Position, Velocity, Acceleration;
    Vec2D a1, a2, a3;
    BoidParamsStruct Params;
    size_t FlockID, BoidID;
    const size_t MaxW = GlobalParams.ImageParams.WindowX - 1;
    const size_t MaxH = GlobalParams.ImageParams.WindowY - 1;
    // thread ID's for sense/plan/act loops
    int SenseTid = 0, PlanTid = 0, ActTid = 0;
    // list of current neighbours
    std::vector<Boid *> Neighbours;

    void Sense(std::vector<Boid> &AllBoids, const int tID)
    {
        // finds all the nearby neighbours
        /// TODO: figure out something smarter than clearing, maybe share across ticks
        SenseTid = tID;
        Neighbours.clear();
        for (Boid &B : AllBoids) // have to scan all boids (TODO: do we?)
        {
            if (B.BoidID != BoidID &&                                                // not self
                (B.Position - Position).SizeSqr() < sqr(Params.NeighbourhoodRadius)) // within this visibility
            {
                Neighbours.push_back(&B);
            }
        }
    }

    void Plan(std::vector<int> &FlockSizes, const int tID)
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
            for (const Boid *Neighbour : Neighbours) // exactly the neighbours we care about
            {
                RelativeCOM += Neighbour->Position;
                AvgVel += Neighbour->Velocity;
                if ((Neighbour->Position - Position).SizeSqr() < sqr(Params.CollisionRadius)) // real close
                {
                    SeparationDisp -= (Neighbour->Position - Position); // add to displacement
#pragma omp critical
                    {
                        // NOTE: or find the max flock, instead of the first one
                        if (FlockSizes[FlockID] <= FlockSizes[Neighbour->FlockID])
                        {
                            // their flock is larger, I join
                            FlockSizes[FlockID]--;
                            FlockID = Neighbour->FlockID;
                            FlockSizes[Neighbour->FlockID]++;
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

    void Act(const double DeltaTime, const int tID)
    {
        ActTid = tID;
        Acceleration = a1 + a2 + a3; // + a4
        Velocity = (Velocity + Acceleration).LimitMagnitude(Params.BoidMaxVelocity);
        Position += Velocity * DeltaTime;
        CollisionCheck(); // resets position to avoid collisions
        EdgeWrap();
    }

    void CollisionCheck()
    {
        /// TODO: fix the tracking for high-tick timings
        for (const Boid *Neighbour : Neighbours) // move away from any overlapping neighbours
        {
            if (Neighbour->BoidID != BoidID && // not self
                (Neighbour->Position - Position).SizeSqr() <
                    sqr(Params.BoidSize + Params.BoidSize)) // only within neighbourhood
            {
                const double OverlapAmnt =
                    1 - ((Neighbour->Position - Position).Size() / (Params.BoidSize + Params.BoidSize));
                Position -= (Neighbour->Position - Position) * OverlapAmnt;
            }
        }
    }

    void Draw(Image &I) const
    {
        // Colour C(255, 255, 255);
        // if (SenseTid == PlanTid && PlanTid == ActTid)
        // {
        //     C = IDColours[ActTid % IDColours.size()];
        // }
        const Colour C = IDColours[FlockID % IDColours.size()];
        I.DrawSolidCircle(Position, Params.BoidSize, C);
        // also render line to indicate direction
        const size_t LineWidth = 2 * Params.BoidSize; // number pixels
        Vec2D Heading = Velocity.Norm();
        Vec2D End = Position + Heading * LineWidth;
        I.DrawLine(Position, End, C);
    }

    void EdgeWrap()
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
};

#endif