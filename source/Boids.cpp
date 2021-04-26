#include "Utils.hpp"
#include "Vec.hpp"
#include <array>
#include <chrono>
#include <cstdlib>
#include <omp.h>
#include <string>
#include <vector>

/// TODO: don't use globals
const int NumBoids = 2000;
// colours for the threads
const int NumThreads = 2;
const std::vector<Colour> IDColours = {Colour(255, 0, 0),   Colour(0, 255, 0),   Colour(0, 0, 255),
                                       Colour(255, 255, 0), Colour(0, 255, 255), Colour(255, 0, 255),
                                       Colour(255, 128, 0), Colour(0, 128, 255), Colour(128, 0, 255),
                                       Colour(128, 255, 0), Colour(0, 255, 128), Colour(255, 0, 128)};
const Vec2D ScreenDim(1000, 1000);

double t = 0; // global time of the world

const double Cohesion = 1.0;
const double Alignment = 0.5;
const double Separation = 0;

class Boid_t
{
  public:
    Boid_t(const double x0, const double y0, const double dx0, const double dy0)
    {
        Position = Vec2D(x0, y0);   // set posixtion
        Velocity = Vec2D(dx0, dy0); // set initial velocity
        Acceleration = 0;
    }
    Vec2D Position; // 2d vector of doubles
    Vec2D Velocity;
    Vec2D Acceleration;
    const double Size = 4.0;
    size_t ProcID = 0;

    void CollisionCheck(std::vector<Boid_t> &AllBoids)
    {
        /// TODO: fix the tracking for high-tick timings
        for (const Boid_t &Neighbour : AllBoids) // move away from any overlapping neighbours
        {
            if ((Neighbour.Position - Position).SizeSqr() < sqr(Size + Neighbour.Size)) // only within neighbourhood
            {
                const double OverlapAmnt = 1 - ((Neighbour.Position - Position).Size() / (Size + Neighbour.Size));
                Position -= (Neighbour.Position - Position) * OverlapAmnt;
            }
        }
    }

    void Update(std::vector<Boid_t> &AllBoids, const double dt, const size_t ID)
    {
        ProcID = ID;
        Vec2D RelativeCOM, Disp, AvgVel;
        size_t NumNeighbours = 0;
        const size_t Visibility = 100;           // 100px radius to consider neighbours
        for (const Boid_t &Neighbour : AllBoids) // only one loop instead of three
        {
            if ((Neighbour.Position - Position).SizeSqr() < sqr(Visibility)) // only within neighbourhood
            {
                NumNeighbours++;                                               // increment neighbours
                RelativeCOM += Neighbour.Position;                             // add to COM
                AvgVel += Neighbour.Velocity;                                  // add to AvgVel
                if ((Neighbour.Position - Position).SizeSqr() < sqr(3 * Size)) // real close
                {
                    /// TODO: add some sort of collision
                    Disp -= (Neighbour.Position - Position); // add to displacement
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
        Velocity = Boid_t::LimitVelocity(Velocity + Acceleration);
        Position += Velocity * dt;
        CollisionCheck(AllBoids);
        EdgeWrap();
    }

    void Draw(Image &I) const
    {
        const Colour C = IDColours[ProcID % IDColours.size()];
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
        const size_t MaxW = ScreenDim[0] - 1;
        const size_t MaxH = ScreenDim[1] - 1;
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

void RenderFrame(Image &I, const std::vector<Boid_t> &AllBoids)
{
    // draw all the boids onto the frame
    for (const Boid_t &B : AllBoids)
    {
        B.Draw(I);
    }
    // draw the target onto the frame
    I.ExportPPMImage();
    I.Blank();
}

double ComputeFrame(std::vector<Boid_t> &AllBoids, Image &I, const double t, const double dt)
{
    auto StartTime = std::chrono::system_clock::now();
    // naive per-boid iteration
#pragma omp parallel for num_threads(NumThreads)
    for (size_t i = 0; i < AllBoids.size(); i++)
    {
        const size_t ProcID = omp_get_thread_num();
        AllBoids[i].Update(AllBoids, dt, ProcID);
    }
    auto EndTime = std::chrono::system_clock::now();
    std::chrono::duration<double> ElapsedTime = EndTime - StartTime;
    // Rendering is not part of our problem
    RenderFrame(I, AllBoids);
    return ElapsedTime.count();
}

std::vector<Boid_t> InitBoids()
{
    std::vector<Boid_t> AllBoids;
    for (size_t i = 0; i < NumBoids; i++)
    {
        const double x0 = std::rand() % int(ScreenDim[0]);
        const double y0 = std::rand() % int(ScreenDim[1]);
        const double dx0 = std::rand() % int(ScreenDim[0]);
        const double dy0 = std::rand() % int(ScreenDim[1]);
        Boid_t NewBoid(x0, y0, dx0, dy0);
        AllBoids.push_back(NewBoid);
    }
    return AllBoids;
}

int main()
{
    std::srand(0); // consistent seed
    double TimeBudget = 10.0;
    Image I(ScreenDim[0], ScreenDim[1]);
    std::vector<Boid_t> AllBoids = InitBoids();
    const double dt = 0.05;
    double ElapsedTime = 0;
    while (t <= TimeBudget)
    {
        ElapsedTime += ComputeFrame(AllBoids, I, t, dt);
        t += dt;
    }
    std::cout << std::endl << "Finished simulation! Took " << ElapsedTime << "s" << std::endl;
    return 0;
}

/// NOTE: resources:
/// PSEUDOCODE: http://www.vergenet.net/~conrad/boids/pseudocode.html
/// OVERVIEW: https://cs.stanford.edu/people/eroberts/courses/soco/projects/2008-09/modeling-natural-systems/boids.html
/// EXAMPLE: https://eater.net/boids