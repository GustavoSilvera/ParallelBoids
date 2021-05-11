#ifndef FLOCK
#define FLOCK

#include "Boid.hpp"      // Boids
#include "Image.hpp"     // Image (for rendering)
#include "Vec.hpp"       // Vec2D (for COM)
#include <unordered_map> // std::unordered_map
#include <vector>        // std::vector

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
        Valid = (Size > 0);
    }
    size_t FlockID;

    enum FlockOp // enumerate different flock operations
    {
        SenseAndPlanOp,
        DelegateOp,
        AssignToFlockOp
    };

    struct TIDStruct // which threads took care of which flock operations
    {
        int SenseAndPlan, Delegate, AssignToFlock;
    };
    TIDStruct TIDs;

    bool Valid;
    Vec2D COM; // center of mass of this flock
    static FlockParamsStruct Params;
    std::vector<Boid> Neighbourhood;
    std::unordered_map<size_t, std::vector<Boid>> Emigrants; // buckets where the delegates go

    bool IsValidFlock() const;

    int Size() const;

    void SenseAndPlan(const int TID, const std::vector<Flock> &AllFlocks);

    void Act(const double DeltaTime);

    void Delegate(const int TID, const std::vector<Flock> &Flocks);

    void AssignToFlock(const int TID, const std::vector<Flock> &AllFlocks);

    void Recruit(Boid &B, Flock &BsFlock);

    std::vector<const Flock *> NearestFlocks(const std::vector<Flock> &AllFlocks) const;

    void Draw(Image &I) const;

    static void CleanUp(std::vector<Flock> &AllFlocks);
};

#endif