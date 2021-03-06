#include "Flock.hpp"  // Flocks
#include "Tracer.hpp" // Tracer
#include "Utils.hpp"  // Params
#include "Vec.hpp"    // Vec3D
#include <chrono>     // timing threads
#include <omp.h>      // OpenMP
#include <string>     // cout
#include <unordered_map>
#include <vector> // std::vector

#include <cuda.h>
#include <cuda_runtime.h>
#include <driver_functions.h>
#include <math.h>

struct GlobalBoidData
{
    size_t numBoids;

    float *position;
    float *velocity;
    float *acceleration;
    int *flockID;
    int *flockSize;
};

__constant__ GlobalBoidData cuConstBoidData;
__constant__ ParamsStruct cuConstGlobalParams;

/** @return the result of adding the inputs together */
__device__ float2 operator+(const float2 &A, const float2 &B)
{
    return make_float2(A.x + B.x, A.y + B.y);
}

/** @return the result of subtracting B from A */
__device__ float2 operator-(const float2 &A, const float2 &B)
{
    return make_float2(A.x - B.x, A.y - B.y);
}

/** @return the result of multiplying each element of A by v */
__device__ float2 operator*(const float2 &A, const float v)
{
    return make_float2(A.x * v, A.y * v);
}

/** @return the result of dividing each element of A by v */
__device__ float2 operator/(const float2 &A, const float v)
{
    return make_float2(A.x / v, A.y / v);
}

/** @return the square of the l2 norm of the specified vector */
__device__ float sizeSqrd(const float2 &A)
{
    return A.x * A.x + A.y * A.y;
}

/** @return whether or not A lies beyond a distance of radius R from B */
__device__ bool distGT(const float2 &A, const float2 &B, const float R)
{
    return sizeSqrd(A - B) > (R * R);
}

/** @return whether or not A lies within a distance of radius R from B */
__device__ bool distLT(const float2 &A, const float2 &B, const float R)
{
    return sizeSqrd(A - B) < R * R;
}

__device__ float2 normalize(const float2 &A)
{
    return A / (sqrt(float(sizeSqrd(A))));
}

__device__ float2 limitMagnitudeKernel(const float2 &A, const float maxMag)
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
__global__ void senseAndPlanKernel()
{
    const int index = (blockIdx.x * blockDim.x) + threadIdx.x;

    if (index >= cuConstBoidData.numBoids)
        return;

    cuConstBoidData.flockID[index] = 1;
    int N = cuConstBoidData.numBoids;
    float2 a1 = make_float2(0.0, 0.0);
    float2 a2 = make_float2(0.0, 0.0);
    float2 a3 = make_float2(0.0, 0.0);
    float2 *position = (float2 *)cuConstBoidData.position;
    float2 *velocity = (float2 *)cuConstBoidData.velocity;
    float2 *acceleration = (float2 *)cuConstBoidData.acceleration;

    BoidParamsStruct boidParams = cuConstGlobalParams.BoidParams;

    float2 posUs = position[index];
    float2 velUs = velocity[index];

    float2 relCOM = make_float2(0.0, 0.0);
    float2 relCOV = make_float2(0.0, 0.0);
    float2 sep = make_float2(0.0, 0.0);

    int numCloseBy = 0;
    for (int i = 0; i < N; i++)
    {
        float2 posThem = position[i];
        float2 velThem = velocity[i];
        if (i != index && distLT(posUs, posThem, boidParams.NeighbourhoodRadius))
        {
            relCOM = relCOM + posThem;
            relCOV = relCOV + velThem;
            if (distLT(posUs, posThem, boidParams.CollisionRadius))
            {
                sep = sep - (posThem - posUs);
            }
            numCloseBy++;
        }
    }

    if (numCloseBy > 0)
    {
        a1 = ((relCOM / numCloseBy) - posUs) * (boidParams.Cohesion);
        a2 = sep * (boidParams.Separation); // dosent depent on NumCloseby but makes sense
        a3 = ((relCOV / numCloseBy) - velUs) * (boidParams.Alignment);
    }

    acceleration[index] = a1 + a2 + a3;
    cuConstBoidData.flockID[index] = 1;
}

__global__ void actKernel(float deltaTime)
{
    const int index = (blockIdx.x * blockDim.x) + threadIdx.x;

    if (index >= cuConstBoidData.numBoids)
        return;

    /// NOTE: This function is meant to be independent from all other boids
    /// and thus can be run asynchronously, however it needs a barrier between itself
    /// and the senseAndPlan() device function
    float2 *position = (float2 *)cuConstBoidData.position;
    float2 *velocity = (float2 *)cuConstBoidData.velocity;
    const float2 *acceleration = (float2 *)cuConstBoidData.acceleration;

    BoidParamsStruct params = cuConstGlobalParams.BoidParams;

    velocity[index] = limitMagnitudeKernel((velocity[index] + acceleration[index]), params.MaxVel);
    position[index] = position[index] + (velocity[index] * deltaTime);
}

class Simulator
{
  public:
    GlobalBoidData cuDevicePtrs;
    float *cudaDevicePosition;
    float *cudaDeviceVelocity;
    float *cudaDeviceAcceleration;
    int *cudaDeviceFlockID;
    int *cudaDeviceFlockSize;

    int numBoids;
    float *position;
    float *velocity;
    float *acceleration;
    int *flockID;
    int *flockSize;
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
            AllFlocks[i] = Flock(i, 1);
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
    std::unordered_map<size_t, Flock> AllFlocks;
    Image I;

    void Simulate()
    {
        float ElapsedTime = 0;
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
    void insertFloat2(float *arr, const Vec2D &v, const size_t i)
    {
        arr[2 * i] = v[0];
        arr[2 * i + 1] = v[1];
    }

    void InitBoidDataArrays()
    {
        std::vector<Boid> &AllBoids = *(AllFlocks.begin()->second.Neighbourhood.GetAllBoidsPtr());
        size_t vecSize = sizeof(float) * 2 * numBoids;
        size_t intSize = sizeof(int) * numBoids;

        position = (float *)malloc(vecSize);
        velocity = (float *)malloc(vecSize);
        acceleration = (float *)malloc(vecSize);
        flockID = (int *)malloc(intSize);
        flockSize = (int *)malloc(intSize);

        for (size_t i = 0; i < AllBoids.size(); i++)
        {
            const Boid &boid = AllBoids[i];

            insertFloat2(position, boid.Position, i);
            insertFloat2(velocity, boid.Velocity, i);
            insertFloat2(acceleration, boid.Acceleration, i);
            flockID[i] = boid.FlockID;
            flockSize[i] = 0;
        }
    }

    /**
     * @pre All Boid arrays have been initialized and are up to date.
     */
    void UpdateBoidPosAndVel()
    {
        std::vector<Boid> &AllBoids = *(AllFlocks.begin()->second.Neighbourhood.GetAllBoidsPtr());
        for (size_t i = 0; i < AllBoids.size(); i++)
        {
            Boid &B = AllBoids[i];
            B.Position = Vec2D(position[2 * i], position[2 * i + 1]);
            B.Velocity = Vec2D(velocity[2 * i], velocity[2 * i + 1]);
        }
    }

    /**
     * Sets up the memory required for the cuda device code
     *
     * @return The time it took to setup.
     */
    float Setup()
    {
        auto StartTime = std::chrono::system_clock::now();

        // allocate boid and flock memory
        std::vector<Boid> &AllBoids = *(AllFlocks.begin()->second.Neighbourhood.GetAllBoidsPtr());
        numBoids = AllBoids.size();
        size_t vecSize = sizeof(float) * 2 * numBoids;
        size_t intSize = sizeof(int) * numBoids;
        cudaMalloc(&cudaDevicePosition, vecSize);
        cudaMalloc(&cudaDeviceVelocity, vecSize);
        cudaMalloc(&cudaDeviceAcceleration, vecSize);
        cudaMalloc(&cudaDeviceFlockID, intSize);
        cudaMalloc(&flockSize, intSize);

        InitBoidDataArrays();

        cudaMemcpy(cudaDevicePosition, position, vecSize, cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceVelocity, velocity, vecSize, cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceAcceleration, acceleration, vecSize, cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceFlockID, flockID, intSize, cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceFlockSize, flockSize, intSize, cudaMemcpyHostToDevice);

        // set up constants struct for copying into device memory struct
        cuDevicePtrs.numBoids = numBoids;
        cuDevicePtrs.position = cudaDevicePosition;
        cuDevicePtrs.velocity = cudaDeviceVelocity;
        cuDevicePtrs.acceleration = cudaDeviceAcceleration;
        cuDevicePtrs.flockID = cudaDeviceFlockID;
        cuDevicePtrs.flockSize = flockSize;

        cudaMemcpyToSymbol(cuConstBoidData, &cuDevicePtrs, sizeof(GlobalBoidData));
        cudaMemcpyToSymbol(cuConstGlobalParams, &GlobalParams, sizeof(ParamsStruct));

        auto EndTime = std::chrono::system_clock::now();
        std::chrono::duration<float> ElapsedTime = EndTime - StartTime;

        return ElapsedTime.count(); // return wall clock time diff
    }

    float Tick()
    {
        // Run our actual problem (boid computation)
        auto StartTime = std::chrono::system_clock::now();

        const int threadsPerBlock = 64;
        const int numBlocks = (numBoids + threadsPerBlock - 1) / threadsPerBlock;

        senseAndPlanKernel<<<numBlocks, threadsPerBlock>>>();
        cudaDeviceSynchronize();
        actKernel<<<numBlocks, threadsPerBlock>>>(Params.DeltaTime);
        cudaDeviceSynchronize();

        cudaMemcpy(position, cuDevicePtrs.position, numBoids * sizeof(float) * 2, cudaMemcpyDeviceToHost);
        cudaMemcpy(velocity, cuDevicePtrs.velocity, numBoids * sizeof(float) * 2, cudaMemcpyDeviceToHost);

        UpdateBoidPosAndVel();
        std::vector<Flock *> AllFlockPtrs = GetAllFlocksVector();

        UpdateFlocks(AllFlockPtrs);

        auto EndTime = std::chrono::system_clock::now();

        std::chrono::duration<float> ElapsedTime = EndTime - StartTime;
        // save tracer data
        Tracer::AddTickT(ElapsedTime.count());

        if (Params.RenderingMovie)
        {
            // Rendering is not part of our problem
            Render();
        }

        return ElapsedTime.count(); // return wall clock time diff
    }

    std::vector<Flock *> GetAllFlocksVector() const
    {
        std::vector<Flock *> AllFlocksVec;
        for (auto It = AllFlocks.begin(); It != AllFlocks.end(); It++)
        {
            assert(It != AllFlocks.end());
            const Flock &F = It->second;
            AllFlocksVec.push_back(const_cast<Flock *>(&F));
        }
        assert(AllFlocksVec.size() == AllFlocks.size());
        return AllFlocksVec;
    }

    void UpdateFlocks(std::vector<Flock *> AllFlocksVec)
    {
#pragma omp parallel num_threads(Params.NumThreads) // spawns threads
        {
            if (GlobalParams.FlockParams.UseFlocks)
            {
                /// NOTE: the following parallel operations are per-flocks, not per-boids
#pragma omp for schedule(static)
                for (size_t i = 0; i < AllFlocksVec.size(); i++)
                {
                    AllFlocksVec[i]->Delegate(omp_get_thread_num(), AllFlocksVec);
                }
#pragma omp barrier
#pragma omp for schedule(static)
                for (size_t i = 0; i < AllFlocksVec.size(); i++)
                {
                    AllFlocksVec[i]->AssignToFlock(omp_get_thread_num());
                }
            }
#pragma omp barrier
#pragma omp for schedule(static)
            for (size_t i = 0; i < AllFlocksVec.size(); i++)
            {
                AllFlocksVec[i]->ComputeBB();
            }
        }
        // convert flock data to processor communications
        Tracer::SaveFlockMatrix(AllFlocks);
        // remove empty (invalid) flocks
        Flock::CleanUp(AllFlocks);
    }

    void Render()
    {
        // draw all the boids onto the frame
        std::vector<Flock *> AllFlocksVec = GetAllFlocksVector();
        for (size_t i = 0; i < AllFlocksVec.size(); i++)
        {
            AllFlocksVec[i]->Draw(I);
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
    ParseParams("params/cuda_params.ini");
    Simulator Sim;
    Sim.Simulate();
    // Dump all tracer data
    Tracer::Dump();
    return 0;
}