#include "Tracer.hpp"

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