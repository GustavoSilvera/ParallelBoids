#include "Vec.hpp"
#include <cmath> // atan
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

    void Update(std::vector<boid_t> &AllBoids, const double dt)
    {
        Vec2D v1 = rule1();
        Vec2D v2 = rule2(AllBoids);
        Vec2D v3 = rule3();
        // add more rules here
        Velocity = boid_t::LimitVelocity(Velocity + v1 + v2 + v3);
        Position += Velocity * dt;
    }

    void Draw(std::vector<std::vector<Colour>> &Frame) const
    {
        // draws a singular (white) pixel for now
        const size_t X = Position[0];
        const size_t Y = Position[1];
        const size_t size = 5; // size of circle
        // render circle as body of boid
        for (size_t pX = X - size; pX < X + size; pX++)
        {
            for (size_t pY = Y - size; pY < Y + size; pY++)
            {
                bool WithinWidth = (0 <= pX && pX < MaxWidth);
                bool WithinHeight = (0 <= pY && pY < MaxHeight);
                Vec2D Pixel(pX, pY);
                if ((Pixel - Position).NormSqr() < sqr(size))
                {
                    if (WithinWidth && WithinHeight) // draw boid within bound (triangle)
                    {
                        Frame[pY][pX] = Colour(255.0, 255.0, 255.0);
                    }
                }
            }
        }
        // also render line to indicate direction
        const size_t LineWidth = 10; // number pixels
        Vec2D HeadingVec = Velocity / Velocity.Norm();
        for (size_t i = 0; i < LineWidth; i++)
        {
            Vec2D Pixel = Position + HeadingVec * i;
            bool WithinWidth = (0 <= Pixel[0] && Pixel[0] < MaxWidth);
            bool WithinHeight = (0 <= Pixel[1] && Pixel[1] < MaxHeight);
            if (WithinWidth && WithinHeight) // draw boid within bound (triangle)
            {
                Frame[Pixel[1]][Pixel[0]] = Colour(255.0, 255.0, 255.0);
            }
        }
    }

    static Vec2D LimitVelocity(const Vec2D &Velocity)
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

void ComputeFrame(std::vector<boid_t> &AllBoids, const double t, const double dt)
{
    /// TODO: figure out how to generate the Out/ directory
    std::string FramePath = "Out/";
    std::string FrameTitle = "Frame" + std::to_string(t) + ".ppm";
    std::vector<std::vector<Colour>> Frame = BlankImage(MaxWidth, MaxHeight);
    boid_t::ComputeCOM(AllBoids);    // technically incorrect
    boid_t::ComputeAvgVel(AllBoids); // technically incorrect
    for (boid_t &B : AllBoids)
    {
        B.Draw(Frame);
        B.Update(AllBoids, dt);
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

    double TimeBudget = 5.0;
    std::vector<boid_t> AllBoids = InitBoids();
    const double dt = 0.05;
    double t = 0;
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