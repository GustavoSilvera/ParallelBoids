#ifndef NEIGHBOURHOOD
#define NEIGHBOURHOOD

#include "Boid.hpp"
#include "Vec.hpp"
#include <unordered_map>

class NLayout // options bw SoA or AoS
{
  public:
    NLayout() = default;
    void NewBoid(const size_t FlockID);
    size_t Size(const size_t FlockID) const;
    void ClearLocal();
    void Append(const std::vector<Boid> &Immigrants);
    // for both layout types
    Boid *operator[](size_t Idx) const;
    // Boid *GetBoid(const size_t Idx);
    // types for the layout
    enum Layout
    {
        Local,  // local vector handled by a Flock
        Global, // global vector handled by Neighbourhood
        Invalid // initial (neither)
    };
    static NLayout::Layout GetType();
    static void SetType(const NLayout::Layout L);

  private:
    static NLayout::Layout UsingLayout;
    // NLayout::UsingLayout = Invalid;
    // for local (flock-based) neighbourhoods
    std::vector<Boid> BoidsLocal;
    // for a global (boid-based) neighbourhood
    static std::unordered_map<size_t, size_t> BoidsGlobalSizes;
    /// NOTE: one important thing about the boids in BoidsGlobal is that
    // their position in the vector remains constant throughout the sim
    // (unlike the BoidsLocal which move around to impact locality)
    static std::vector<Boid> BoidsGlobal;
};

#endif