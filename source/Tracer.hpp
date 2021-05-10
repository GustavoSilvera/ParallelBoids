#ifndef TRACER
#define TRACER

#include <vector>
#include "Utils.hpp"
#include <omp.h>

class Tracer
{
public:
    static void AddRead(const size_t P_Requestor, const size_t P_Holder);
    static void AddWrite(const size_t P_Requestor, const size_t P_Holder);
    static void Dump();

private:
    Tracer(const size_t NumThreads) : CommunicationMatrix(NumThreads, std::vector<TraceData>(NumThreads))
    {
        Params = GlobalParams.TracerParams;
    }
    static Tracer *Instance()
    {
        /// singleton class
        // only initializes static T the FIRST time
        static Tracer *T = new Tracer(Params.NumThreads);
        return T;
    }
    static TracerParamsStruct Params;
    struct TraceData
    {
        size_t Reads, Writes;
    };
    std::vector<std::vector<TraceData>> CommunicationMatrix;
};

#endif