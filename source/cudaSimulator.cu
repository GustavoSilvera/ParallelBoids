#include "Flock.hpp"  // Flocks
#include "Tracer.hpp" // Tracer
#include "Utils.hpp"  // Params
#include "Vec.hpp"    // Vec3D
#include <chrono>     // timing threads
#include <omp.h>      // OpenMP
#include <string>     // cout
#include <vector>     // std::vector

#include <cuda.h>
#include <math.h>
#include <cuda_runtime.h>
#include <driver_functions.h>

struct GlobalConstants {
    size_t numBoids;

    float* position;
    float* velocity;
    int* flockID; 
    int* flockSize;
};

__constant__ GlobalConstants cuConstBoidData;
__constant__ ParamsStruct cuConstGlobalParams;

/** @return the result of adding the inputs together */
__device__ float2 operator+(const float2 A, const float2 B)
{ 
    return make_float2(A.x + B.x, A.y + B.y);
}

/** @return the result of subtracting B from A */
__device__ float2 operator-(const float2 A, const float2 B)
{
    return make_float2(A.x - B.x, A.y - B.y);
}

/** @return the result of multiplying each element of A by v */
__device__ float2 operator*(const float2 A, const int v) 
{
    return make_float2(A.x * v, A.y * v);
}

/** @return the result of dividing each element of A by v */
__device__ float2 operator/(const float2 A, const int v) 
{
    return make_float2(A.x / v, A.y / v);
}

/** @return the square of the l2 norm of the specified vector */
__device__ int sizeSqrd (float2 A) 
{
    return A.x*A.x + A.y*A.y;
}

/** @return whether or not A lies beyond a distance of radius R from B */
__device__ bool distGT (const float2 A, const float2 B, const int R)
{
    return sizeSqrd(A-B) > R*R;
}

/** @return whether or not A lies within a distance of radius R from B */
__device__ bool distLT (const float2 A, const float2 B, const int R)
{
    return sizeSqrd(A-B) < R*R;
}

__device__ float2 normalize(const float2 A) {
    return A / (sqrt(double(sizeSqrd(A))));
}

__device__ float2 limitMagnitudeKernel(const float2 A, const double maxMag)
{
    if (sizeSqrd(A) > maxMag * maxMag)
    {
        return normalize(A) * maxMag;
    }
    return A;
}
/** 
 * Computes the acceleration for the boid indexed by index in the current
 * tick.
 */ 
__device__ void 
senseAndPlanKernel(float2 &a1, float2 &a2, float2 &a3, int index) 
{
    int N = cuConstBoidData.numBoids;
    float2* position = (float2*)cuConstBoidData.position;
    float2* velocity = (float2*)cuConstBoidData.velocity;

    BoidParamsStruct boidParams = cuConstGlobalParams.BoidParams;

    float2 posUs = position[index];
    float2 velUs = velocity[index];

    float2 relCOM = make_float2(0.0,0.0);  
    float2 relCOV = make_float2(0.0,0.0);
    float2 sep = make_float2(0.0,0.0);

    int numCloseBy = 0;
    for(size_t i = 0; i < N; i++) {
        float2 posThem = position[i];
        float2 velThem = velocity[i];

        if (i != index 
            && !distGT(posUs,posThem, boidParams.NeighbourhoodRadius)) 
        {
            relCOM = relCOM + posThem;  
            relCOV = relCOV + velThem;
            if (distLT(posUs,posThem,boidParams.CollisionRadius))
            {
                sep = sep - (posThem - posUs);
            }
        }
        numCloseBy++;
    }

    if (numCloseBy > 0)
    {
        a1 = ((relCOM / numCloseBy) - posUs) * (boidParams.Cohesion);
        a2 = sep * (boidParams.Separation); // dosent depent on NumCloseby but makes sense
        a3 = ((relCOV / numCloseBy) - velUs) * (boidParams.Alignment);
    }
}

__device__ void 
actKernel(float2 a1, float2 a2, float2 a3, double deltaTime, int i)
{

    /// NOTE: This function is meant to be independent from all other boids
    /// and thus can be run asynchronously, however it needs a barrier between itself
    /// and the senseAndPlan() device function
    float2* posPtr = &(((float2*)(cuConstBoidData.position))[i]);
    float2* velPtr = &(((float2*)(cuConstBoidData.velocity))[i]);
    BoidParamsStruct params = cuConstGlobalParams.BoidParams;

    float2 acceleration = a1 + a2 + a3; // + a4
    *velPtr = limitMagnitudeKernel((*velPtr + acceleration), params.MaxVel);
    *posPtr = (*posPtr) + ((*velPtr) * deltaTime);
}

__global__ void sensePlanActKernel()
{
    const int index = blockIdx.x + threadIdx.x;
    float2 a1,a2,a3;
    senseAndPlanKernel(a1,a2,a3,index);
    __syncthreads();
    actKernel(a1,a2,a3,cuConstGlobalParams.SimulatorParams.DeltaTime,index);
}


class Simulator
{
  public:
    float* cudaDevicePosition;
    float* cudaDeviceVelocity;
    int* cudaDeviceFlockID;
    int* cudaDeviceFlockSize;

    int numBoids;
    float* position;
    float* velocity;
    int* flockID;
    int* flockSize;
    Simulator()
    {
        Params = GlobalParams.SimulatorParams;
        // Print out status
        std::cout << "Running on " << Params.NumBoids << " boids for " << Params.NumIterations << " iterations in a ("
                  << GlobalParams.ImageParams.WindowX << ", " << GlobalParams.ImageParams.WindowY << ") world with "
                  << Params.NumThreads << " threads" << std::endl;

        // Initialize neighbourhood layout for flocks before use
        Flock::InitNeighbourhoodLayout();
        // Spawn flocks
        for (size_t i = 0; i < Params.NumBoids; i++)
        {
            AllFlocks.push_back(Flock(i, 1));
        }

        // begin tracking which flocks communicate with which
        Tracer::InitFlockMatrix(AllFlocks.size());

        // Allocate and initialize device memory data
        Setup();

        // initialize image frame
        if (Params.RenderingMovie)
        {
            // only allocate memory if we're gonna use it
            I.Init();
        }
    }
    static SimulatorParamsStruct Params;
    std::vector<Flock> AllFlocks;
    std::vector<int> FlockSizes;
    Image I;

    void Simulate()
    {
        double ElapsedTime = 0;
        for (size_t i = 0; i < Params.NumIterations; i++)
        {
            ElapsedTime += Tick();
            std::cout << "Tick: " << i << "\r" << std::flush; // carriage return, no newline
        }
        std::cout << "Finished simulation! Took " << ElapsedTime << "s" << std::endl;
    }

    /**
     * Inserts the value v in the float2 array represented by arr
     * @pre i < |arr|/2
     */
    void insertFloat2(float* arr, Vec2D v, size_t i) {
        arr[2*i] = v[0];
        arr[2*i+1] = v[1];
    }

    /** @return a list of all the boids in the simulation */
    std::vector<Boid> GetAllBoids ()
    {
        return *(AllFlocks.begin()->Neighbourhood.GetAllBoidsPtr());
    }

    void InitBoidDataArrays() {
        std::vector<Boid> AllBoids = GetAllBoids();
        size_t vecSize = sizeof(float) * 2 * numBoids;
        size_t intSize = sizeof(int) * numBoids;

        position = (float*)malloc(vecSize);
        velocity = (float*)malloc(vecSize);
        flockID = (int*)malloc(intSize);
        flockSize = (int*)malloc(intSize);

        for (size_t i=0; i < AllBoids.size(); i++)
        {
            Boid boid = AllBoids[i];
            Vec2D pos = boid.Position;
            Vec2D vel = boid.Velocity;
            int fID = boid.FlockID;
            int fSize = 1;

            insertFloat2(position, pos, i);
            insertFloat2(velocity, vel, i);
            flockID[i] = fID;
            flockSize[i] = fSize;
        }
    }

    /**
     * @pre All Boid arrays have been initialized and are up to date.
     */
    void UpdateBoidPosAndVel() {
        std::vector<Boid> AllBoids = GetAllBoids();
        for (size_t i = 0; i < AllBoids.size(); i++)
        {
            Boid B = AllBoids[i];
            B.Position = position[i];
            B.Velocity = velocity[i];
        }
    }

    /** 
     * Sets up the memory required for the cuda device code
     *
     * @return The time it took to setup. 
     */
    double Setup() {
        auto StartTime = std::chrono::system_clock::now();

        // allocate boid and flock memory
        std::vector<Boid> AllBoids = GetAllBoids();
        numBoids = AllBoids.size();
        size_t vecSize = sizeof(float) * 2 * numBoids;
        size_t intSize = sizeof(int) * numBoids;
        cudaMalloc(&cudaDevicePosition, vecSize);
        cudaMalloc(&cudaDeviceVelocity, vecSize);
        cudaMalloc(&cudaDeviceFlockID, intSize);
        cudaMalloc(&flockSize, intSize);

        InitBoidDataArrays();
        
        cudaMemcpy(cudaDevicePosition,position,vecSize,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceVelocity,velocity,vecSize,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceFlockID,flockID,intSize,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceFlockSize,flockSize,intSize,cudaMemcpyHostToDevice);

        // set up constants struct for copying into device memory struct
        GlobalConstants constParams;
        constParams.numBoids = numBoids;
        constParams.position = cudaDevicePosition; 
        constParams.velocity = cudaDeviceVelocity;
        constParams.flockID = cudaDeviceFlockID;
        constParams.flockSize = flockSize;

        cudaMemcpyToSymbol(cuConstBoidData, &constParams, sizeof(GlobalConstants));
        cudaMemcpyToSymbol(cuConstGlobalParams, &Params, sizeof(GlobalParams));

        auto EndTime = std::chrono::system_clock::now();
        std::chrono::duration<double> ElapsedTime = EndTime - StartTime;

        return ElapsedTime.count(); // return wall clock time diff
    }

    double Tick()
    {
        // Run our actual problem (boid computation)
        auto StartTime = std::chrono::system_clock::now();

        const int threadsPerBlock = 512;
        const int numBlocks = (numBoids + threadsPerBlock - 1) / threadsPerBlock;

        sensePlanActKernel<<<numBlocks,threadsPerBlock>>>();
        cudaDeviceSynchronize();

        cudaMemcpy(position, 
                   cuConstBoidData.position,
                   numBoids * sizeof(float) * 2,
                   cudaMemcpyDeviceToHost);
        cudaMemcpy(velocity, 
                   cuConstBoidData.velocity,
                   numBoids * sizeof(float) * 2,
                   cudaMemcpyDeviceToHost);

        UpdateBoidPosAndVel();          

        for (size_t i = 0; i < AllFlocks.size(); i++)
        {
            AllFlocks[i].Delegate(omp_get_thread_num(), AllFlocks);
        }

        for (size_t i = 0; i < AllFlocks.size(); i++)
        {
            AllFlocks[i].AssignToFlock(omp_get_thread_num(), AllFlocks);
        }

        auto EndTime = std::chrono::system_clock::now();
        std::chrono::duration<double> ElapsedTime = EndTime - StartTime;
        // save tracer data
        Tracer::AddTickT(ElapsedTime.count());

        if (Params.RenderingMovie)
        {
            // Rendering is not part of our problem
            Render();
        }

        return ElapsedTime.count(); // return wall clock time diff
    }
    
    void Render()
    {
        // draw all the boids onto the frame
#pragma omp parallel for num_threads(Params.NumThreads) schedule(static)
        for (size_t i = 0; i < AllFlocks.size(); i++)
        {
            AllFlocks[i].Draw(I);
        }
        // draw the target onto the frame
        I.ExportPPMImage();
        I.Blank();
    }
};

// declaring static variables
SimulatorParamsStruct Simulator::Params;
ImageParamsStruct Image::Params;
TracerParamsStruct Tracer::Params;


// global params struct
ParamsStruct GlobalParams;

int main()
{
    std::srand(0); // consistent seed
    ParseParams("params/params.ini");
    Simulator Sim;
    Sim.Simulate();
    // Dump all tracer data
    Tracer::Dump();
    return 0;
}