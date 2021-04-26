#include "Utils.hpp"
#include "Vec.hpp"
#include <array>
#include <chrono>
#include <cstdlib>
#include <omp.h>
#include <string>
#include <vector>

/// TODO: don't use globals
const int NumBoids = 1000;
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
const double Separation = 1.3;

class Boid_t
{
  public:
    Boid_t(int x0, int y0)
    {
        Position = Vec2D(x0, y0); // set posixtion
        Velocity = Vec2D(0, 0);   // set initial velocity
        Acceleration = 0;
    }
    Vec2D Position; // 2d vector of doubles
    Vec2D Velocity;
    Vec2D Acceleration;
    const double Size = 10.0;
    size_t ProcID = 0;
    Vec2D rule1(std::vector<Boid_t> &AllBoids) const
    {
        // "fly towards the centre of mass of neighbouring boids"
        Vec2D RelativeCOM;
        size_t NumNeighbours = 0;
        for (const Boid_t &Neighbour : AllBoids)
        {
            if ((Neighbour.Position - Position).NormSqr() < sqr(100))
            {
                // pushes away from nearby boids, displaces 0 if itself
                RelativeCOM += Neighbour.Position;
                NumNeighbours++;
            }
        }
        RelativeCOM /= NumNeighbours;
        return (RelativeCOM - Position);
    }

    Vec2D rule2(std::vector<Boid_t> &AllBoids) const
    {
        // slightly "steer away" from nearby boids to avoid collisions
        Vec2D Disp; // displacement away from neighbouring boids
        for (const Boid_t &Neighbour : AllBoids)
        {
            if ((Neighbour.Position - Position).NormSqr() < sqr(2 * Size))
            {
                // pushes away from nearby boids, displaces 0 if itself
                Disp -= (Neighbour.Position - Position);
            }
        }
        return Disp;
    }

    Vec2D rule3(std::vector<Boid_t> &AllBoids) const
    {
        // try to match velocity to the rest of the group
        Vec2D AvgVel;
        size_t NumNeighbours = 0;
        for (const Boid_t &Neighbour : AllBoids)
        {
            if ((Neighbour.Position - Position).NormSqr() < sqr(100))
            {
                // pushes away from nearby boids, displaces 0 if itself
                AvgVel += Neighbour.Velocity;
                NumNeighbours++;
            }
        }
        AvgVel /= NumNeighbours;
        return (AvgVel - Velocity);
    }

    void Update(std::vector<Boid_t> &AllBoids, const double dt, const size_t ID)
    {
        ProcID = ID;
        Vec2D v1 = rule1(AllBoids) * Cohesion;
        Vec2D v2 = rule2(AllBoids) * Separation;
        Vec2D v3 = rule3(AllBoids) * Alignment;
        // Vec2D v4 = rule4();
        // add more rules here
        Acceleration = v1 + v2 + v3; // + v4
        Velocity = Boid_t::LimitVelocity(Velocity + Acceleration);
        Position += Velocity * dt;
        EdgeWrap();
    }

    void Draw(Image &I) const
    {
        const Colour C = IDColours[ProcID % IDColours.size()];
        I.DrawCircle(Position[0], Position[1], Size, C);
        /// TODO: make a "DrawLine" function
        // also render line to indicate direction
        const size_t LineWidth = 2 * Size; // number pixels
        Vec2D HeadingVec = Velocity / Velocity.Norm();
        for (size_t i = 0; i < LineWidth; i++)
        {
            Vec2D Pixel = Position + HeadingVec * i;
            I.SetPixel(Pixel[0], Pixel[1], C);
        }
    }

    static Vec2D LimitVelocity(const Vec2D &Velocity)
    {
        const double MaxVel = 20;
        if (Velocity.NormSqr() > sqr(MaxVel))
        {
            return (Velocity / Velocity.Norm()) * MaxVel;
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
            ClampedX = MaxW;
        }
        else if (ClampedX > MaxW)
        {
            ClampedX = 0;
        }
        /// same for y's
        double ClampedY = Position[1];
        if (ClampedY < 0)
        {
            ClampedY = MaxH;
        }
        else if (ClampedY > MaxH)
        {
            ClampedY = 0;
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
    // I.DrawCircle(Target[0], Target[1], 5.0, Colour(255, 0, 0));
    I.ExportPPMImage();
    I.Blank();
}

double ComputeFrame(std::vector<Boid_t> &AllBoids, Image &I, const double t, const double dt)
{
    auto StartTime = std::chrono::system_clock::now();
    // naive per-boid iteration
#pragma omp parallel for num_threads(NumThreads)
    for (Boid_t &B : AllBoids)
    {
        const size_t ProcID = omp_get_thread_num();
        B.Update(AllBoids, dt, ProcID);
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
        int x0 = std::rand() % int(ScreenDim[0]);
        int y0 = std::rand() % int(ScreenDim[1]);
        Boid_t NewBoid(x0, y0);
        AllBoids.push_back(NewBoid);
    }
    return AllBoids;
}

int main()
{
    std::srand(0); // consistent seed
    double TimeBudget = 11.0;
    Image I(ScreenDim[0], ScreenDim[1]);
    std::vector<Boid_t> AllBoids = InitBoids();
    const double dt = 0.05;
    double ElapsedTime = 0;
    while (t < TimeBudget)
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