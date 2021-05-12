#ifndef NEIGHBOURHOOD
#define NEIGHBOURHOOD

#include "Boid.hpp"
#include "Vec.hpp"
#include <iterator>      // std::advance
#include <unordered_map> // std::unordered_map
#include <unordered_set> // std::unordered_set

class NLayout // options bs local and global boid layout
{
  public:
    NLayout() = default;
    void NewBoid(const size_t FlockID);
    size_t Size() const;
    void ClearLocal();
    bool IsValid() const;
    std::vector<Boid *> GetBoids() const;
    std::vector<Boid> *GetAllBoidsPtr() const;
    void Append(const std::vector<Boid> &Immigrants);
    // for both layout types
    Boid *operator[](const size_t Idx) const;
    Boid *GetBoidF(const size_t Idx) const;
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
    // each flock still has an ID
    size_t FlockID;
    // for local (flock-based) neighbourhoods
    std::vector<Boid> BoidsLocal;
    // for a global (boid-based) neighbourhood
    struct FlockData
    {
        // need to manually keep track of where in BoidsGlobal each boid in a flock is
        std::unordered_set<size_t> BoidIDs;
        size_t GetBoidIdx(const size_t Idx) const
        {
            auto It = BoidIDs.begin();
            for (size_t i = 0; i < Idx; i++)
            {
                // unfortunately linear in Idx, since the unordered_set iterator is
                // a legacyForwardIterator (supports ++ in O(1))
                // https://en.cppreference.com/w/cpp/named_req/ForwardIteratorz
                It++;
            }
            return *It;
        }
        void Add(const Boid &B)
        {
            // add new Boid to the list of IDs
            BoidIDs.insert(B.BoidID);
        }
        void Remove(const Boid &B)
        {
            // O(1) find BoidID in set
            auto It = BoidIDs.find(B.BoidID);
            assert(It != BoidIDs.end());
            // O(1) erase from set
            BoidIDs.erase(It);
        }
        size_t Size() const
        {
            return BoidIDs.size();
        }
    };
    // per-flock boid data
    static std::unordered_map<size_t, FlockData> BoidsGlobalData;

    /// NOTE: one important thing about the boids in BoidsGlobal is that
    // their position in the vector remains constant throughout the sim
    // (unlike the BoidsLocal which move around to impact locality)
    static std::vector<Boid> BoidsGlobal;
};

#endif