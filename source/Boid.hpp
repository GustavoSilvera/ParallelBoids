#ifndef BOID
#define BOID

#include "Utils.hpp"
#include "Vec.hpp"
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
    size_t ProcID, FlockID, BoidID;
    const double Size = 4.0;
    Vec2D WindowSize;
    double Cohesion = 0.5, Alignment = 1.5, Separation = 1.5;

    void CollisionCheck(std::vector<Boid> &AllBoids)
    {
        /// TODO: fix the tracking for high-tick timings
        for (const Boid &Neighbour : AllBoids) // move away from any overlapping neighbours
        {
            if (Neighbour.BoidID != BoidID &&                                           // not self
                (Neighbour.Position - Position).SizeSqr() < sqr(Size + Neighbour.Size)) // only within neighbourhood
            {
                const double OverlapAmnt = 1 - ((Neighbour.Position - Position).Size() / (Size + Neighbour.Size));
                Position -= (Neighbour.Position - Position) * OverlapAmnt;
            }
        }
    }

    void Update(std::vector<Boid> &AllBoids, std::vector<int> &FlockSizes, const double dt)
    {
        ProcID = 0;
        Vec2D RelativeCOM, Disp, AvgVel;
        size_t NumNeighbours = 0;
        const size_t Visibility = 100;   // 100px radius to consider neighbours
        for (Boid &Neighbour : AllBoids) // only one loop instead of three
        {
            if (Neighbour.BoidID != BoidID)
            {
                if ((Neighbour.Position - Position).SizeSqr() < sqr(Visibility)) // only within neighbourhood
                {
                    NumNeighbours++;                                               // increment neighbours
                    RelativeCOM += Neighbour.Position;                             // add to COM
                    AvgVel += Neighbour.Velocity;                                  // add to AvgVel
                    if ((Neighbour.Position - Position).SizeSqr() < sqr(4 * Size)) // real close
                    {
                        Disp -= (Neighbour.Position - Position); // add to displacement
#pragma omp critical
                        {
                            if (FlockSizes[FlockID] <= FlockSizes[Neighbour.FlockID])
                            {
                                // their flock is larger, I join them with probability based off distance

                                FlockSizes[FlockID]--;
                                FlockID = Neighbour.FlockID;
                                FlockSizes[Neighbour.FlockID]++;
                            }
                        }
                    }
                }
            }
        }
        if (NumNeighbours > 0)
        {
            RelativeCOM /= NumNeighbours;
            AvgVel /= NumNeighbours;
        }

        // compute acceleration factors
        Vec2D a1 = (RelativeCOM - Position) * Cohesion;
        Vec2D a2 = Disp * Separation;
        Vec2D a3 = (AvgVel - Velocity) * Alignment;
        // add more rules here
        Acceleration = a1 + a2 + a3; // + a4
        Velocity = Boid::LimitVelocity(Velocity + Acceleration);
        Position += Velocity * dt;
        CollisionCheck(AllBoids);
        EdgeWrap();
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
        const double MaxVel = 20;
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