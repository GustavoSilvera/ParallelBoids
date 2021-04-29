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

    Boid(const size_t BID, const size_t FID) : Boid()
    {
        const double x0 = RandD(0, GlobalParams.ImageParams.WindowX, 3);
        const double y0 = RandD(0, GlobalParams.ImageParams.WindowY, 3);
        const double dx0 = RandD(-1 * GlobalParams.BoidParams.MaxVel, GlobalParams.BoidParams.MaxVel, 3);
        const double dy0 = RandD(-1 * GlobalParams.BoidParams.MaxVel, GlobalParams.BoidParams.MaxVel, 3);
        Position = Vec2D(x0, y0);   // set posixtion
        Velocity = Vec2D(dx0, dy0); // set initial velocity
        FlockID = FID;              // initial flock assignment
        BoidID = BID;               // BoidID is unique per boid
    }

    Vec2D Position, Velocity, Acceleration;
    Vec2D a1, a2, a3;
    BoidParamsStruct Params;
    size_t FlockID, BoidID;
    size_t MaxW = GlobalParams.ImageParams.WindowX - 1;
    size_t MaxH = GlobalParams.ImageParams.WindowY - 1;
    // thread ID's for sense/plan/act loops
    int SenseTid = 0, PlanTid = 0, ActTid = 0;
    // list of current neighbours
    std::vector<Boid *> Neighbours;

    void Sense(std::vector<Flock> &AllFlocks, const int tID);

    void Plan(std::vector<Flock> &Flocks, const int tID);

    void Act(const double DeltaTime, const int tID);

    void CollisionCheck();

    void Draw(Image &I) const;

    void EdgeWrap();
};

#endif