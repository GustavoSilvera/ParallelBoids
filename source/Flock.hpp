#ifndef FLOCK
#define FLOCK

#include "Boid.hpp"          // Boid
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
        BB = BoundingBox(Neighbourhood.GetBoidF(0)->Position); // always has 1 boid
        Valid = (Size > 0);
    }

    static void InitNeighbourhoodLayout()
    {
        // Initialize neighbourhood layout type
        if (NLayout::GetType() == NLayout::Invalid)
        {
            // only done once, static vars
            if (GlobalParams.FlockParams.UseLocalNeighbourhoods)
                NLayout::SetType(NLayout::Local);
            else
                NLayout::SetType(NLayout::Global);
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

    struct BoundingBox
    {
        double TopLeftX, TopLeftY;
        double BottomRightX, BottomRightY;
        BoundingBox() = default;
        bool IsValidBB() const
        {
            if (TopLeftX > BottomRightX)
                return false;
            if (TopLeftY > BottomRightY)
                return false;
            return true;
        }
        BoundingBox(const Vec2D &V0)
        {
            // initialize BB to a single point
            TopLeftX = V0[0] - 2;
            TopLeftY = V0[1] - 2;
            BottomRightX = V0[0] + 2; // single point
            BottomRightY = V0[1] + 2;
            assert(IsValidBB());
        }
        bool IntersectsBB(const BoundingBox &B) const
        {
            assert(IsValidBB());
            assert(B.IsValidBB());
            // outside the X bound
            // (they right of me || they left of me)
            if (B.TopLeftX >= BottomRightX || TopLeftX >= B.BottomRightX)
                return false;
            // outside the Y bound
            // (they below me || me below them)
            if (B.TopLeftY >= BottomRightY || TopLeftY >= B.BottomRightY)
                return false;
            // if within both, then they overlap
            return true;
        }
    };
    BoundingBox BB;

    bool Valid;
    Vec2D COM; // center of mass of this flock
    static FlockParamsStruct Params;
    NLayout Neighbourhood;
    std::unordered_map<size_t, std::vector<Boid>> Emigrants; // buckets where the delegates go

    bool IsValidFlock() const;

    size_t Size() const;

    void SenseAndPlan(const int TID, const std::vector<Flock> &AllFlocks);

    void Act(const double DeltaTime);

    void UpdateCOM();

    void Delegate(const int TID, const std::vector<Flock> &Flocks);

    void AssignToFlock(const int TID, const std::vector<Flock> &AllFlocks);

    void ComputeBB();

    std::vector<const Flock *> NearestFlocks(const std::vector<Flock> &AllFlocks) const;

    void Draw(Image &I) const;

    static void CleanUp(std::vector<Flock> &AllFlocks);
};

#endif