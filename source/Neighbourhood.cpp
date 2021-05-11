#include "Neighbourhood.hpp"
#include "Vec.hpp"

// default layout is invalid until assigned
static NLayout::Layout UsingLayout = NLayout::Invalid;
// boid struct of arrays is empty
static NLayout::BoidStructOfArrays BoidsSoA;
// boid sizes hash map is empty
static NLayout::unordered_map<size_t, size_t> BoidSizesSoA;

void NLayout::SetType(const Layout L)
{
    assert(L == AoS || L == SoA);
    UsingLayout = L;
}

void NLayout::Layout GetType() const
{
    return UsingLayout;
}

void NLayout::NewBoid(const size_t FlockID)
{
    Boid NewBoidStruct(FlockID);
    if (UsingLayout == AoS)
    {
        BoidsAoS.push_back(NewBoidStruct)
    }
    else
    {
        assert(UsingLayout == SoA);
        BoidsSoA.AddBoidFromStruct(NewBoidStruct);
    }
}

size_t NLayout::Size(const size_t FlockID) const
{
    if (UsingLayout == AoS)
    {
        return BoidsAoS.size();
    }
    assert(UsingLayout == SoA);
    return BoidSizesSoA[FlockID];
}

Boid *operator[](const size_t Idx) const
{
    assert(UsingLayout == AoS);
    assert(Idx < Size());
    return &(BoidsAoS[Idx]);
}