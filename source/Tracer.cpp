#include "Tracer.hpp"
#include <iostream>

void Tracer::AddRead(const size_t P_Requestor, const size_t P_Holder)
{
    Tracer *T = Instance();
#pragma omp atomic
    T->CommunicationMatrix[P_Requestor][P_Holder].Reads++;
}

void Tracer::AddWrite(const size_t P_Requestor, const size_t P_Holder)
{
    Tracer *T = Instance();
#pragma omp atomic
    T->CommunicationMatrix[P_Requestor][P_Holder].Writes++;
}

void Tracer::Dump()
{
    const Tracer *T = Instance();
    for (const std::vector<TraceData> &CommunicationRow : T->CommunicationMatrix)
    {
        std::cout << "{ ";
        for (const TraceData &TD : CommunicationRow)
        {
            std::cout << " R:" << TD.Reads << " W:" << TD.Writes << " |";
        }
        std::cout << "}" << std::endl;
    }
}