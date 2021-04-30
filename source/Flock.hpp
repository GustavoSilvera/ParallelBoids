#ifndef FLOCK
#define FLOCK

#include "Boid.hpp" // Boids
#include "Image.hpp"
#include "Vec.hpp"
#include <vector> // std::vector

class Flock
{
  public:
    Flock()
    {
        Params = GlobalParams.FlockParams;
    }
    Flock(const size_t FiD, const size_t Size) : Flock()
    {
        FlockID = FiD;
        for (size_t i = 0; i < Size; i++)
        {
            Boid NewBoid(FlockID);
            Neighbourhood.push_back(NewBoid);
        }
        NeighbourhoodBackup = Neighbourhood; // copy over
    }
    size_t FlockID;
    Vec2D COM; // center of mass of this flock
    FlockParamsStruct Params;
    std::vector<Boid> Neighbourhood;
    std::vector<Boid> NeighbourhoodBackup; // separate copy

    int Size() const;

    void SenseAndPlan(const size_t ThreadID, const std::vector<Flock> &AllFlocks);

    void Act(const double DeltaTime);

    void Delegate(std::vector<Flock> &Flocks);

    void Recruit(Boid &B, Flock &BsFlock);

    size_t NearestFlockId(std::vector<Flock> &AllFlocks);

    void Draw(Image &I) const;
};

#endif