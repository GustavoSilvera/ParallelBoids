#include "Vec.hpp"
#include <array>
#include <cstdlib>
#include <omp.h>
#include <vector>

/// TODO: don't use globals
const int MaxWidth = 500;
const int MaxHeight = 500;
const int NumBoids = 100;

Vec2D COM;    // static centre of mass for all boids
Vec2D AvgVel; // static centre of mass for all boids
Vec2D Target; // flock's goal
double t = 0; // global time of the world
class Boid_t
{
  public:
    Boid_t(int x0, int y0)
    {
        Position = Vec2D(x0, y0); // set posixtion
        Velocity = Vec2D(0, 0);   // set initial velocity
    }
    Vec2D Position; // 2d vector of doubles
    Vec2D Velocity;
    const double Size = 5.0;
    Vec2D rule1() const
    {
        // "fly towards the centre of mass of neighbouring boids"
        float Ferocity = 0.05; // moves 10% of the way to the COM
        Vec2D RelativeCOM = ((COM * NumBoids) - Position) / (NumBoids - 1);
        return (RelativeCOM - Position) * Ferocity;
    }

    Vec2D rule2(std::vector<Boid_t> &AllBoids) const
    {
        // slightly "steer away" from nearby boids to avoid collisions
        Vec2D Disp; // displacement away from neighbouring boids
        const float AvoidFerocity = 1.55;
        for (const Boid_t &Neighbour : AllBoids)
        {
            if ((Neighbour.Position - Position).NormSqr() < sqr(2 * Size))
            {
                // pushes away from nearby boids, displaces 0 if itself
                Disp -= (Neighbour.Position - Position) * AvoidFerocity;
            }
        }
        return Disp;
    }

    Vec2D rule3() const
    {
        // try to match velocity to the rest of the group
        float Ferocity = 0.05; // moves 1/8th of the way to the AvgVel
        Vec2D RelativeAvgVel = ((AvgVel * NumBoids) - Velocity) / (NumBoids - 1);
        return (RelativeAvgVel - Velocity) * Ferocity;
    }

    Vec2D rule4() const
    {
        // move the flock's goal to some target position
        const double Ferocity = 0.15; // of which to go to this position
        return (Target - Position) * Ferocity;
    }

    void Update(std::vector<Boid_t> &AllBoids, const double dt)
    {
        Vec2D v1 = rule1();
        Vec2D v2 = rule2(AllBoids);
        Vec2D v3 = rule3();
        Vec2D v4 = rule4();
        // add more rules here
        Velocity = Boid_t::LimitVelocity(Velocity + v1 + v2 + v3 + v4);
        Position += Velocity * dt;
    }

    void Draw(std::array<std::array<Colour, MaxHeight>, MaxWidth> &Frame) const
    {
        DrawCircle(Frame, Position, Size, Colour(255, 255, 255));
        /// TODO: make a "DrawLine" functoin
        // also render line to indicate direction
        const size_t LineWidth = 2 * Size; // number pixels
        Vec2D HeadingVec = Velocity / Velocity.Norm();
        for (size_t i = 0; i < LineWidth; i++)
        {
            Vec2D Pixel = Position + HeadingVec * i;
            bool WithinWidth = (0 <= Pixel[0] && Pixel[0] < MaxWidth);
            bool WithinHeight = (0 <= Pixel[1] && Pixel[1] < MaxHeight);
            if (WithinWidth && WithinHeight) // draw boid within bound (triangle)
            {
                Frame[Pixel[0]][Pixel[1]] = Colour(255.0, 255.0, 255.0);
            }
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

    static void ComputeCOM(std::vector<Boid_t> &AllBoids)
    {
        // the "centre of mass" of all the boids
        COM = Vec2D(0, 0); // reset from last time
        for (const Boid_t &boid : AllBoids)
        {
            COM += boid.Position; // accumulate all boids
        }
        COM /= AllBoids.size(); // divide by count
    }

    static void ComputeAvgVel(std::vector<Boid_t> &AllBoids)
    {
        // the "centre of mass" of all the boids
        AvgVel = Vec2D(0, 0); // reset from last time
        for (const Boid_t &boid : AllBoids)
        {
            AvgVel += boid.Velocity; // accumulate all boids
        }
        AvgVel /= AllBoids.size(); // divide by count
    }

    static void ComputeTarget()
    {
        Target -= Vec2D(MaxWidth / 2.0, MaxHeight / 2.0);
        Target = Target.rotate(0.01); // how much rotation
        Target += Vec2D(MaxWidth / 2.0, MaxHeight / 2.0);
    }
};

void RenderFrame(const std::vector<Boid_t> &AllBoids)
{
    std::string FramePath = "Out/";
    std::string FrameTitle = "Boids" + std::to_string(t) + ".ppm";
    std::array<std::array<Colour, MaxHeight>, MaxWidth> Frame = BlankImage(MaxHeight, MaxWidth);
    // draw all the boids onto the frame
    for (const Boid_t &B : AllBoids)
    {
        B.Draw(Frame);
    }
    // draw the target onto the frame
    DrawCircle(Frame, Target, 5.0, Colour(255, 0, 0));
    WritePPMImage(Frame, MaxHeight, MaxWidth, FramePath + FrameTitle);
}

void ComputeFrame(std::vector<Boid_t> &AllBoids, const double t, const double dt)
{
    Boid_t::ComputeCOM(AllBoids);
    Boid_t::ComputeAvgVel(AllBoids);
    Boid_t::ComputeTarget();
    // naive per-boid iteration
    for (Boid_t &B : AllBoids)
    {
        B.Update(AllBoids, dt);
    }
    // Rendering is not part of our problem
    RenderFrame(AllBoids);
}

std::vector<Boid_t> InitBoids()
{
    std::vector<Boid_t> AllBoids;
    COM = Vec2D(0, 0);
    AvgVel = Vec2D(0, 0);
    Target = Vec2D(MaxWidth / 2.0, MaxHeight / 2.0) + Vec2D(200, 0); // middle of the screen + offset
    for (size_t i = 0; i < NumBoids; i++)
    {
        int x0 = std::rand() % MaxWidth;
        int y0 = std::rand() % MaxHeight;
        Boid_t NewBoid(x0, y0);
        AllBoids.push_back(NewBoid);
    }
    return AllBoids;
}

int main()
{
    std::srand(0); // consistent seed

    double TimeBudget = 15.0;
    std::vector<Boid_t> AllBoids = InitBoids();
    const double dt = 0.05;
    while (t < TimeBudget)
    {
        ComputeFrame(AllBoids, t, dt);
        t += dt;
    }
    std::cout << std::endl << "Finished simulation!" << std::endl;
    return 0;
}

/// NOTE: resources:
/// PSEUDOCODE: http://www.vergenet.net/~conrad/boids/pseudocode.html
/// OVERVIEW: https://cs.stanford.edu/people/eroberts/courses/soco/projects/2008-09/modeling-natural-systems/boids.html
/// EXAMPLE: https://eater.net/boids