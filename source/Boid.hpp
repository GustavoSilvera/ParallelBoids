#ifndef BOID
#define BOID

#include "Image.hpp"
#include "Utils.hpp"
#include "Vec.hpp"
#include <vector>

// fwd declaration of flocks
class Flock;

class Boid
{
  public:
    Boid()
    {
        Params = GlobalParams.BoidParams;
    }

    Boid(const size_t FID) : Boid()
    {
        const double x0 = RandD(0, GlobalParams.ImageParams.WindowX, 3);
        const double y0 = RandD(0, GlobalParams.ImageParams.WindowY, 3);
        const double dx0 = RandD(-1 * GlobalParams.BoidParams.MaxVel, GlobalParams.BoidParams.MaxVel, 3);
        const double dy0 = RandD(-1 * GlobalParams.BoidParams.MaxVel, GlobalParams.BoidParams.MaxVel, 3);
        Position = Vec2D(x0, y0);   // set posixtion
        Velocity = Vec2D(dx0, dy0); // set initial velocity
        FlockID = FID;              // initial flock assignment
        BoidID = NumBoids;          // BoidID is unique per boid
        NumBoids++;                 // increment total number of boids
    }

    static size_t NumBoids; // one (shared) for ALL boids
    Vec2D Position, Velocity, Acceleration;
    Vec2D a1, a2, a3;
    BoidParamsStruct Params;
    size_t FlockID, BoidID, ThreadID;
    size_t MaxW = GlobalParams.ImageParams.WindowX - 1;
    size_t MaxH = GlobalParams.ImageParams.WindowY - 1;

    void SenseAndPlan(const std::vector<Flock> &AllFlocks, const int TiD);

    void Plan(const Boid &B, Vec2D &RCOM, Vec2D &RCOV, Vec2D &Sep, size_t &NC) const;

    void Act(const double DeltaTime);

    void CollisionCheck(Boid &B);

    void Draw(Image &I) const;

    void EdgeWrap();

    bool DistanceGT(const Boid &B, const double Rad) const;

    bool DistanceLT(const Boid &B, const double Rad) const;

    bool operator==(const Boid &B) const;
};

#endif