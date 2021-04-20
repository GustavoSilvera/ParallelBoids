#include "Vector.hpp"
#include <cstdlib>
#include <vector>

/// TODO: don't use globals
const int MaxWidth = 500;
const int MaxHeight = 500;

Vec2D COM;    // static centre of mass for all boids
Vec2D AvgVel; // static centre of mass for all boids

class boid_t
{
  public:
    boid_t(int x0, int y0)
    {
        Position = Vec2D(x0, y0); // set posixtion
        Velocity = Vec2D(0, 0);   // set initial velocity
    }
    Vec2D Position; // 2d vector of doubles
    Vec2D Velocity;
    Vec2D rule1() const
    {
        // "fly towards the centre of mass of neighbouring boids"
        float Ferocity = 0.01; // moves 1% of the way to the COM
        return (COM - Position) * Ferocity;
    }

    Vec2D rule2(std::vector<boid_t> &AllBoids) const
    {
        // slightly "steer away" from nearby boids to avoid collisions
        Vec2D Disp; // displacement away from neighbouring boids
        for (const boid_t &Neighbour : AllBoids)
        {
            if ((Neighbour.Position - Position).NormSqr() < sqr(100.0))
            {
                // pushes away from nearby boids, displaces 0 if itself
                Disp -= (Neighbour.Position - Position);
            }
        }
        return Disp;
    }

    Vec2D rule3() const
    {
        // try to match velocity to the rest of the group
        float Ferocity = 0.125; // moves 1/8th of the way to the AvgVel
        return (AvgVel - Velocity) * Ferocity;
    }

    void Update(std::vector<boid_t> &AllBoids)
    {
        Vec2D v1 = rule1();
        Vec2D v2 = rule2(AllBoids);
        Vec2D v3 = rule3();
        // add more rules here
        Velocity = boid_t::LimitVelocity(Velocity + v1 + v2 + v3);
        Position += Velocity;
    }

    void Draw(std::vector<std::vector<Colour>> &Frame) const
    {
        // draws a singular (white) pixel for now
        const size_t X = Position[0];
        const size_t Y = Position[1];
        bool WithinWidth = (0 <= X && X < MaxWidth);
        bool WithinHeight = (0 <= Y && Y < MaxHeight);
        if (WithinWidth && WithinHeight)
        {
            Frame[Y][X] = Colour(255.0, 255.0, 255.0);
        }
    }

    static Vec2D LimitVelocity(const Vec2D Velocity)
    {
        const double MaxVel = 10;
        if (Velocity.NormSqr() > sqr(MaxVel))
        {
            return (Velocity / Velocity.Norm()) * MaxVel;
        }
        return Velocity;
    }

    static void ComputeCOM(std::vector<boid_t> &AllBoids)
    {
        // the "centre of mass" of all the boids
        COM = Vec2D(0, 0); // reset from last time
        for (const boid_t &boid : AllBoids)
        {
            COM += boid.Position; // accumulate all boids
        }
        COM /= AllBoids.size(); // divide by count
    }

    static void ComputeAvgVel(std::vector<boid_t> &AllBoids)
    {
        // the "centre of mass" of all the boids
        AvgVel = Vec2D(0, 0); // reset from last time
        for (const boid_t &boid : AllBoids)
        {
            AvgVel += boid.Velocity; // accumulate all boids
        }
        AvgVel /= AllBoids.size(); // divide by count
    }
};

void ComputeFrame(std::vector<boid_t> &AllBoids, const double t)
{
    std::string FramePath = "Out/";
    std::string FrameTitle = "Frame" + std::to_string(t) + ".ppm";
    std::vector<std::vector<Colour>> Frame = BlankImage(MaxWidth, MaxHeight);
    boid_t::ComputeCOM(AllBoids);    // technically incorrect
    boid_t::ComputeAvgVel(AllBoids); // technically incorrect
    for (boid_t &B : AllBoids)
    {
        B.Draw(Frame);
        B.Update(AllBoids);
    }
    WritePPMImage(Frame, MaxWidth, MaxHeight, FramePath + FrameTitle);
    return;
}

std::vector<boid_t> InitBoids()
{
    std::vector<boid_t> AllBoids;
    const int NumBoids = 100;
    COM = Vec2D(0, 0);
    AvgVel = Vec2D(0, 0);
    for (size_t i = 0; i < NumBoids; i++)
    {
        int x0 = std::rand() % MaxWidth;
        int y0 = std::rand() % MaxHeight;
        boid_t NewBoid(x0, y0);
        AllBoids.push_back(NewBoid);
    }
    return AllBoids;
}

int main()
{
    std::srand(0); // consistent seed

    double TimeBudget = 2.0;
    std::vector<boid_t> AllBoids = InitBoids();
    const double dt = 0.05;
    double t = 0;
    while (t < TimeBudget)
    {
        ComputeFrame(AllBoids, t);
        t += dt;
    }
    return 0;
}

/// NOTE: resources:
/// PSEUDOCODE: http://www.vergenet.net/~conrad/boids/pseudocode.html
/// OVERVIEW: https://cs.stanford.edu/people/eroberts/courses/soco/projects/2008-09/modeling-natural-systems/boids.html
/// BASELINE: https://eater.net/boids