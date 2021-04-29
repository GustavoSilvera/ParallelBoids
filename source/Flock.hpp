#ifndef FLOCK
#define FLOCK

#include "Boid.hpp" // Boids
#include "Image.hpp"
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
    }
    size_t FlockID;
    FlockParamsStruct Params;
    std::vector<Boid> Neighbourhood;

    int Size() const;

    void SenseAndPlan(const size_t ThreadID, const std::vector<Flock> &AllFlocks);

    void Act(const double DeltaTime, std::vector<Flock> &Flocks);

    void Recruit(Boid &B, std::vector<Flock> &AllFlocks);

    void Draw(Image &I) const;
};

#endif