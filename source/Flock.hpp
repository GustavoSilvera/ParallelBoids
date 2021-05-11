#ifndef FLOCK
#define FLOCK

#include "Image.hpp"         // Image (for rendering)
#include "Neighbourhood.hpp" // Low level neighbourhood (SoA vs AoS)
#include "Vec.hpp"           // Vec2D (for COM)
#include <unordered_map>     // std::unordered_map
#include <vector>            // std::vector

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
            Neighbourhood.NewBoid(FlockID);
        }
        Valid = (Size > 0);
    }

    static void InitNeighbourhoodLayout()
    {
        // Initialize neighbourhood layout type
        if (NLayout::GetType() == NLayout::Invalid)
        {
            // only done once, static vars
            if (GlobalParams.FlockParams.UseSoA)
                NLayout::SetType(NLayout::SoA);
            else
                NLayout::SetType(NLayout::AoS);
        }
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
    NLayout Neighbourhood;
    std::unordered_map<size_t, NLayout> Emigrants; // buckets where the delegates go

    bool IsValidFlock() const;

    size_t Size() const;

    void SenseAndPlan(const int TID, const std::vector<Flock> &AllFlocks);

    void Act(const double DeltaTime);

    void Delegate(const int TID, const std::vector<Flock> &Flocks);

    void AssignToFlock(const int TID, const std::vector<Flock> &AllFlocks);

    std::vector<const Flock *> NearestFlocks(const std::vector<Flock> &AllFlocks) const;

    void Draw(Image &I) const;

    static void CleanUp(std::vector<Flock> &AllFlocks);
};

#endif