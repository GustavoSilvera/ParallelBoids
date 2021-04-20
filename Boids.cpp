#include "Vector.hpp"
#include <vector>

/// TODO: don't use globals
const int MaxWidth = 500;
const int MaxHeight = 500;

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
    static Vec2D COM;    // static centre of mass for all boids
    static Vec2D AvgVel; // static centre of mass for all boids
    Vec2D rule1() const
    {
        // "fly towards the centre of mass of neighbouring boids"
        float Ferocity = 0.01; // moves 1% of the way to the COM
        return (boid_t::COM - Position) * Ferocity;
    }

    Vec2D rule2(std::vector<boid_t> &AllBoids) const
    {
        // slightly "steer away" from nearby boids to avoid collisions
        Vec2D Disp; // displacement away from neighbouring boids
        for (const boid_t &Neighbour : AllBoids)
        {
            if ((Neighbour.Position - Position).norm() < 100.0)
            {
                // pushes away from nearby boids, displaces 0 if itself
                Disp = Disp - (Neighbour.Position - Position);
            }
        }
        return Disp;
    }

    Vec2D rule3() const
    {
        // try to match velocity to the rest of the group
        float Ferocity = 0.125; // moves 1/8th of the way to the AvgVel
        return (boid_t::AvgVel - Velocity) * Ferocity;
    }

    void Update(std::vector<boid_t> &AllBoids)
    {
        Vec2D v1 = rule1();
        Vec2D v2 = rule2(AllBoids);
        Vec2D v3 = rule3();
        Velocity += v1 + v2 + v3;
        Position += Velocity;
    }

    void Draw(std::vector<std::vector<Colour>> &Frame) const
    {
        // draws a singular (white) pixel for now
        const size_t X = Position[0];
        const size_t Y = Position[1];
        Frame[Y][X] = Colour(1.0, 1.0, 1.0);
    }

    static void ComputeCOM(std::vector<boid_t> &AllBoids)
    {
        // the "centre of mass" of all the boids
        boid_t::COM = Vec2D(0, 0); // reset from last time
        for (const boid_t &boid : AllBoids)
        {
            boid_t::COM += boid.Position; // accumulate all boids
        }
        boid_t::COM /= AllBoids.size(); // divide by count
    }

    static void ComputeAvgVel(std::vector<boid_t> &AllBoids)
    {
        // the "centre of mass" of all the boids
        boid_t::AvgVel = Vec2D(0, 0); // reset from last time
        for (const boid_t &boid : AllBoids)
        {
            boid_t::AvgVel += boid.Velocity; // accumulate all boids
        }
        boid_t::AvgVel /= AllBoids.size(); // divide by count
    }
};

void ComputeFrame(std::vector<boid_t> &AllBoids, const double t)
{
    std::string FrameTitle = "Frame" + std::to_string(t) + ".png";
    std::vector<std::vector<Colour>> Frame = BlankImage(MaxWidth, MaxHeight);
    boid_t::ComputeCOM(AllBoids);    // technically incorrect
    boid_t::ComputeAvgVel(AllBoids); // technically incorrect
    for (boid_t &B : AllBoids)
    {
        B.Draw(Frame);
        B.Update(AllBoids);
    }
    WritePPMImage(Frame, MaxWidth, MaxHeight, FrameTitle);
    return;
}

std::vector<boid_t> InitBoids()
{
    std::vector<boid_t> AllBoids;
    const int NumBoids = 100;
    boid_t::COM = Vec2D(0, 0);
    boid_t::AvgVel = Vec2D(0, 0);
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