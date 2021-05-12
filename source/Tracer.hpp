#ifndef TRACER
#define TRACER

#include "Flock.hpp"
#include "Utils.hpp"
#include <omp.h>
#include <vector>

class Tracer
{
  public:
    static void InitFlockMatrix(const size_t NumFlocks);
    static void SaveFlockMatrix(const std::unordered_map<size_t, Flock> &AllFlocks);
    // incrementors for reads/writes
    // static void AddWrite(const size_t F_Requestor, const size_t F_Holder, const Flock::FlockOp F);
    static void AddRead(const size_t F_Requestor, const size_t F_Holder, const Flock::FlockOp F);
    // incrementors for per-frame tick time
    static void AddTickT(const double ElapsedTime);
    // print everything to stdout
    static void Dump();

  private:
    static void AddReads(const size_t T_Requestor, const size_t T_Holder, const size_t Amnt);
    // static void AddWrites(const size_t T_Requestor, const size_t T_Holder, const size_t Amnt);
    Tracer(const size_t NumThreads) : MemoryOpMatrix(NumThreads, std::vector<MemoryOps>(NumThreads))
    {
    }
    static Tracer *Instance()
    {
        /// singleton class
        Params = GlobalParams.TracerParams;
        // only initializes static T the FIRST time
        static Tracer *T = new Tracer(Params.NumThreads);
        return T;
    }
    static TracerParamsStruct Params;
    struct MemoryOps
    {
        size_t Reads = 0;
        // size_t Writes = 0;
    };
    std::vector<std::vector<MemoryOps>> MemoryOpMatrix;

    struct FlockOps
    {
        MemoryOps SenseAndPlan;
        MemoryOps Delegate;
        MemoryOps AssignToFlock;
        Flock::TIDStruct RequestorTIDs, HolderTIDs;
    };
    static void AddFlockOps(const Tracer::FlockOps &FO);
    std::vector<std::vector<FlockOps>> CommunicationMatrix;
    std::vector<double> TickTimes;
};

#endif