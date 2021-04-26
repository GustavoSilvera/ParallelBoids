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
const int NumThreads = 16;
const std::vector<Colour> IDColours = {Colour(255, 0, 0),   Colour(0, 255, 0),   Colour(0, 0, 255),
                                       Colour(255, 255, 0), Colour(0, 255, 255), Colour(255, 0, 255),
                                       Colour(255, 128, 0), Colour(0, 128, 255), Colour(128, 0, 255),
                                       Colour(128, 255, 0), Colour(0, 255, 128), Colour(255, 0, 128)};
const Vec2D ScreenDim(500, 1000);

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
    size_t ProcID = 0;
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

    void Update(std::vector<Boid_t> &AllBoids, const double dt, const size_t ID)
    {
        ProcID = ID;
        Vec2D v1 = rule1();
        Vec2D v2 = rule2(AllBoids);
        Vec2D v3 = rule3();
        // Vec2D v4 = rule4();
        // add more rules here
        Velocity = Boid_t::LimitVelocity(Velocity + v1 + v2 + v3); // + v4
        Position += Velocity * dt;
    }

    void Draw(Image &I) const
    {
        const Colour C = IDColours[ProcID % IDColours.size()];
        I.DrawCircle(Position[0], Position[1], Size, C);
        /// TODO: make a "DrawLine" functoin
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

    static void ComputeTarget(const double DeltaTime)
    {
        Target -= Vec2D(ScreenDim[0] / 2.0, ScreenDim[1] / 2.0);
        Target = Target.rotate(DeltaTime); // how much rotation
        Target += Vec2D(ScreenDim[0] / 2.0, ScreenDim[1] / 2.0);
    }
};

void RenderFrame(Image &I, const std::vector<Boid_t> &AllBoids)
{
    std::string FramePath = "Out/";
    std::string FrameTitle = "Boids" + std::to_string(t) + ".ppm";
// draw all the boids onto the frame
#pragma omp parallel for
    for (const Boid_t &B : AllBoids)
    {
        B.Draw(I);
    }
    // draw the target onto the frame
    I.DrawCircle(Target[0], Target[1], 5.0, Colour(255, 0, 0));
    I.WritePPMImage(FramePath + FrameTitle);
    I.Blank();
}

double ComputeFrame(std::vector<Boid_t> &AllBoids, Image &I, const double t, const double dt)
{
    auto StartTime = std::chrono::system_clock::now();
    Boid_t::ComputeCOM(AllBoids);
    Boid_t::ComputeAvgVel(AllBoids);
    Boid_t::ComputeTarget(dt / 10);
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
    COM = Vec2D(0, 0);
    AvgVel = Vec2D(0, 0);
    Target = Vec2D(ScreenDim[0] / 2.0, ScreenDim[1] / 2.0) + Vec2D(200, 0); // middle of the screen + offset
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
    double TimeBudget = 5.0;
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