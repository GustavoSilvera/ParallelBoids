#ifndef NEIGHBOURHOOD
#define NEIGHBOURHOOD

#include "Boid.hpp"
#include "Vec.hpp"

class NLayout // options bw SoA or AoS
{
  public:
    NLayout() = default;
    void NewBoid(const size_t FlockID);
    void Size(const size_t FlockID) const;
    // for both neighbourhoods
    Boid *operator[](const size_t Idx) const;
    // types for the layout
    static void SetType(const Layout L);
    static Layout GetType() const;
    enum Layout
    {
        SoA,    // struct of arrays
        AoS,    // array of structs
        Invalid // initial (neither)
    };

  private:
    static Layout UsingLayout;
    // for "array of structs" neighbourhood
    std::vector<Boid> BoidsAoS;
    // for "struct of arrays" neighbourhood
    struct BoidsStructOfArrays
    {
        std::vector<Vec2D> Positions;
        std::vector<Vec2D> Velocities;
        std::vector<Vec2D> Accelerations;
        std::vector<Vec2D> a1;
        std::vector<Vec2D> a2;
        std::vector<Vec2D> a3;
        std::vector<size_t> FlockIDs;
        std::vector<size_t> BoidIDs;
        void AddBoidFromStruct(const Boid &B)
        {
            Positions.push_back(B.Position);
            Velocities.push_back(B.Velocity);
            Accelerations.push_back(B.Acceleration);
            a1.push_back(B.a1);
            a2.push_back(B.a2);
            a3.push_back(B.a3);
            FlockIDs.push_back(B.FlockID);
            BoidIDs.push_back(B.BoidID);
            // increment sizes
            BoidSizesSoA[B.FlockID]++;
        }
    };
    static unordered_map<size_t, size_t> BoidSizesSoA;
    static BoidStructOfArrays BoidsSoA;
};

#endif