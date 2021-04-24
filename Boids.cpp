#include "Vec.hpp"
#include <array>
#include <cmath> // atan
#include <cstdlib>
#include <vector>

/// TODO: don't use globals
const int MaxWidth = 500;
const int MaxHeight = 500;

Vec2D Target;        // flock's goal
double t = 0;        // global time of the world
size_t NumBoids = 0; // total number of boids in the flock
class Boid_t
{
  public:
    Boid_t(int x0, int y0)
    {
        Position = Vec2D(x0, y0); // set posixtion
        Velocity = Vec2D(0, 0);   // set initial velocity
        Index = NumBoids;
        NumBoids++;
    }
    Vec2D Position; // 2d vector of doubles
    Vec2D Velocity;
    size_t Index;
    const double Size = 5.0;
    Vec2D rule1(const std::vector<Boid_t> &AllBoids) const
    {
        // "fly towards the centre of mass of neighbouring boids"
        float Ferocity = 0.05;              // moves this % of the way to the COM
        const float NeighbourhoodSize = 50; // within 50 pixels
        Vec2D COM;
        size_t NumNeighbours = 0;
        for (const Boid_t &Neighbour : AllBoids)
        {
            if (Neighbour.Index != Index && (Neighbour.Position - Position).NormSqr() < NeighbourhoodSize)
            {
                COM += Neighbour.Position;
                NumNeighbours++;
            }
        }
        if (NumNeighbours > 0)
        {
            COM /= NumNeighbours;
        }
        return (COM - Position) * Ferocity;
    }

    Vec2D rule2(const std::vector<Boid_t> &AllBoids) const
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

    Vec2D rule3(const std::vector<Boid_t> &AllBoids) const
    {
        // try to match velocity to the rest of the group
        float Ferocity = 0.05;              // moves this % of the way to the AvgVel
        const float NeighbourhoodSize = 50; // within 50 pixels
        Vec2D AvgVel;
        size_t NumNeighbours = 0;
        for (const Boid_t &Neighbour : AllBoids)
        {
            if (Neighbour.Index != Index && (Neighbour.Position - Position).NormSqr() < NeighbourhoodSize)
            {
                AvgVel += Neighbour.Velocity;
                NumNeighbours++;
            }
        }
        if (NumNeighbours > 0)
        {
            AvgVel /= NumNeighbours;
        }
        return (AvgVel - Velocity) * Ferocity;
    }

    Vec2D rule4() const
    {
        // move the flock's goal to some target position
        const double Ferocity = 0.15; // of which to go to this position
        return (Target - Position) * Ferocity;
    }

    void Update(std::vector<Boid_t> &AllBoids, const double dt)
    {
        Vec2D v1 = rule1(AllBoids);
        Vec2D v2 = rule2(AllBoids);
        Vec2D v3 = rule3(AllBoids);
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

    static void ComputeTarget()
    {
        Target -= Vec2D(MaxWidth / 2.0, MaxHeight / 2.0);
        Target = Target.rotate(0.01); // how much rotation
        Target += Vec2D(MaxWidth / 2.0, MaxHeight / 2.0);
    }
};

void ComputeFrame(std::vector<Boid_t> &AllBoids, const double t, const double dt)
{
    /// TODO: figure out how to generate the Out/ directory
    std::string FramePath = "Out/";
    std::string FrameTitle = "Frame" + std::to_string(t) + ".ppm";
    std::array<std::array<Colour, MaxHeight>, MaxWidth> Frame = BlankImage(MaxHeight, MaxWidth);
    Boid_t::ComputeTarget();
    for (Boid_t &B : AllBoids)
    {
        B.Draw(Frame);
        B.Update(AllBoids, dt);
    }
    DrawCircle(Frame, Target, 10.0, Colour(255, 0, 0));
    WritePPMImage(Frame, MaxHeight, MaxWidth, FramePath + FrameTitle);
    return;
}

std::vector<Boid_t> InitBoids()
{
    std::vector<Boid_t> AllBoids;
    const int NumBoids = 100;
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

    double TimeBudget = 10.0;
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