#ifndef BOID
#define BOID

#include "Utils.hpp"
#include "Vec.hpp"
#include <omp.h>
#include <vector>

class Boid
{
  public:
    Boid(const double x0, const double y0, const double dx0, const double dy0, const size_t BID, const size_t FID,
         const Vec2D &WS)
    {
        Position = Vec2D(x0, y0);   // set posixtion
        Velocity = Vec2D(dx0, dy0); // set initial velocity
        FlockID = FID;              // initial flock assignment
        BoidID = BID;               // BoidID is unique per boid
        WindowSize = WS;
    }
    Vec2D Position, Velocity, Acceleration;
    Vec2D a1, a2, a3;
    size_t ProcID, FlockID, BoidID;
    const double Size = 4.0;
    Vec2D WindowSize;
    const double Cohesion = 0.5, Alignment = 1.5, Separation = 1.5;
    std::vector<Boid *> Neighbours;

    void Sense(std::vector<Boid> &AllBoids)
    {
        // finds all the nearby neighbours
        /// TODO: figure out something smarter than clearing, maybe share across ticks
        Neighbours.clear();
        const size_t Visibility = 100; // 100px radius to consider neighbours
        for (Boid &B : AllBoids)       // have to scan all boids (TODO: do we?)
        {
            if (B.BoidID != BoidID &&                                // not self
                (B.Position - Position).SizeSqr() < sqr(Visibility)) // within this visibility
            {
                Neighbours.push_back(&B);
            }
        }
    }

    void Plan(std::vector<int> &FlockSizes)
    {
        // reset current force factors
        a1 = Vec2D(0, 0);
        a2 = Vec2D(0, 0);
        a3 = Vec2D(0, 0);
        // Makes local decisions based off the current neighbours
        if (Neighbours.size() > 0)
        {
            Vec2D RelativeCOM, AvgVel, SeparationDisp;
            for (const Boid *Neighbour : Neighbours) // exactly the neighbours we care about
            {
                RelativeCOM += Neighbour->Position;
                AvgVel += Neighbour->Velocity;
                if ((Neighbour->Position - Position).SizeSqr() < sqr(4 * Size)) // real close
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
            a1 = ((RelativeCOM / Neighbours.size()) - Position) * Cohesion;
            a2 = SeparationDisp * Separation;
            a3 = ((AvgVel / Neighbours.size()) - Velocity) * Alignment;
        }
    }

    void Act(const double DeltaTime)
    {
        Acceleration = a1 + a2 + a3; // + a4
        Velocity = Boid::LimitVelocity(Velocity + Acceleration);
        Position += Velocity * DeltaTime;
        CollisionCheck(); // resets position to avoid collisions
        EdgeWrap();
    }

    void CollisionCheck()
    {
        /// TODO: fix the tracking for high-tick timings
        for (const Boid *Neighbour : Neighbours) // move away from any overlapping neighbours
        {
            if (Neighbour->BoidID != BoidID &&                                            // not self
                (Neighbour->Position - Position).SizeSqr() < sqr(Size + Neighbour->Size)) // only within neighbourhood
            {
                const double OverlapAmnt = 1 - ((Neighbour->Position - Position).Size() / (Size + Neighbour->Size));
                Position -= (Neighbour->Position - Position) * OverlapAmnt;
            }
        }
    }

    void Draw(Image &I) const
    {
        // const Colour C = IDColours[ProcID % IDColours.size()];
        const Colour C = IDColours[FlockID % IDColours.size()];
        I.DrawCircle(Position[0], Position[1], Size, C);
        /// TODO: make a "DrawLine" function
        // also render line to indicate direction
        const size_t LineWidth = 2 * Size; // number pixels
        Vec2D Heading = Velocity.Norm();
        for (size_t i = 0; i < LineWidth; i++)
        {
            Vec2D Pixel = Position + Heading * i;
            I.SetPixel(Pixel[0], Pixel[1], C);
        }
    }

    static Vec2D LimitVelocity(const Vec2D &Velocity)
    {
        const double MaxVel = 30;
        if (Velocity.SizeSqr() > sqr(MaxVel))
        {
            return (Velocity / Velocity.Size()) * MaxVel;
        }
        return Velocity;
    }

    void EdgeWrap()
    {
        const size_t MaxW = WindowSize[0] - 1;
        const size_t MaxH = WindowSize[1] - 1;
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