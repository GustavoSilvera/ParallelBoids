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

struct GlobalBoidData {
    size_t numBoids;

    float* position;
    float* velocity;
    float* acceleration;
    int* flockID; 
    int* flockSize;
};

__constant__ GlobalBoidData cuConstBoidData;
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
__device__ float2 operator*(const float2 A, const float v) 
{
    return make_float2(A.x * v, A.y * v);
}

/** @return the result of dividing each element of A by v */
__device__ float2 operator/(const float2 A, const float v) 
{
    return make_float2(A.x / v, A.y / v);
}

/** @return the square of the l2 norm of the specified vector */
__device__ float sizeSqrd (float2 A) 
{
    return A.x*A.x + A.y*A.y;
}

/** @return whether or not A lies beyond a distance of radius R from B */
__device__ bool distGT (const float2 A, const float2 B, const float R)
{
    // printf("A: (%.4f,%.4f)  || B: (%.4f,%.4f) || sizeSqrd(A-B): %.4f || R*R: %.4f\n", A.x,A.y,B.x,B.y,sizeSqrd(A-B),R*R);
    return sizeSqrd(A-B) > (R*R);
}

/** @return whether or not A lies within a distance of radius R from B */
__device__ bool distLT (const float2 A, const float2 B, const float R)
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
__global__ void 
senseAndPlanKernel() 
{
    const int index = blockIdx.x + threadIdx.x;
    int N = cuConstBoidData.numBoids;
    float2 a1,a2,a3;
    float2* position = (float2*)cuConstBoidData.position;
    float2* velocity = (float2*)cuConstBoidData.velocity;
    float2* acceleration = (float2*)cuConstBoidData.acceleration;

    //printf("%f\n",position[0].x);
    BoidParamsStruct boidParams = cuConstGlobalParams.BoidParams;

    float2 posUs = position[index];
    float2 velUs = velocity[index];

    float2 relCOM = make_float2(0.0,0.0);  
    float2 relCOV = make_float2(0.0,0.0);
    float2 sep = make_float2(0.0,0.0);

    // printf("nborradius: %f\n", boidParams.NeighbourhoodRadius);
    int numCloseBy = 0;
    for(int i = 0; i < N; i++) {
        float2 posThem = position[i];
        float2 velThem = velocity[i];
        if (i != index 
            && !distGT(posUs,posThem, boidParams.NeighbourhoodRadius)) 
        {
            // printf("N: %d || i: %d || index: %d || posUs: %8.4f || posThem: %8.4f || relCOM: %8.4f || sep: %8.4f || relCOV: %8.4f\n",
            //        N, i, index, posUs.x, posThem.x, relCOM.x,sep.x,relCOV.x);
            relCOM = relCOM + posThem;  
            relCOV = relCOV + velThem;
            if (distLT(posUs,posThem,boidParams.CollisionRadius))
            {
                sep = sep - (posThem - posUs);
            }
            numCloseBy++;
        }
        // else printf("%d Us: %d || Them: %d\n",N, index, i);
        // else printf("Us: %d || Them: %d || GT?: %d\n",index,i,(int)(distGT(posUs,posThem,boidParams.NeighbourhoodRadius)));//printf("posUS: (%f,%f) || posThem: (%f,%f)\n", posUs.x,posUs.y, posThem.x,posUs.y);
    }

    if (numCloseBy > 0)
    {
        // printf("relCOM: %f || sep: %f || relCOV: %f\n", relCOM.x,sep.x,relCOV.x);
        a1 = ((relCOM / numCloseBy) - posUs) * (boidParams.Cohesion);
        // printf("%f\n", (((relCOM / numCloseBy) - posUs)* (boidParams.Cohesion)).x);
        a2 = sep * (boidParams.Separation); // dosent depent on NumCloseby but makes sense
        a3 = ((relCOV / numCloseBy) - velUs) * (boidParams.Alignment);
    }
    
    // printf("Vel ptr: %p\n Acc val : ",((float*)cuConstBoidData.velocity));
    // // printf("Vel val: %f\n",*((float*)cuConstBoidData.velocity));
    // printf("Acc ptr: %p\n Acc val : ",((float*)cuConstBoidData.acceleration));
    // printf("Acc val: %f\n",((float*)acceleration)[0]);
    // update the acceleration of the boid
    // float ab4 = acceleration[index].x;
    // printf("a1: %f || a2: %f || a3: %f\n", a1.x,a2.x,a3.x);
    acceleration[index] = a1 + a2 + a3;
    // printf("Before: %f ||| After: %f\n",ab4, ((float*)acceleration)[0]);
}

__global__ void 
actKernel(double deltaTime)
{
    const int index = blockIdx.x + threadIdx.x;
    /// NOTE: This function is meant to be independent from all other boids
    /// and thus can be run asynchronously, however it needs a barrier between itself
    /// and the senseAndPlan() device function
    float2* position = (float2*)cuConstBoidData.position;
    float2* velocity = (float2*)cuConstBoidData.velocity;
    float2* acceleration = (float2*)cuConstBoidData.acceleration;

    BoidParamsStruct params = cuConstGlobalParams.BoidParams;

    velocity[index] = limitMagnitudeKernel((velocity[index] + acceleration[index]), params.MaxVel);
    // float2 before = position[index];
    position[index] = position[index] + (velocity[index] * deltaTime);
    // printf("Before: (%.4f,%.4f) || After: (%.4f,%.4f)\n", before.x,before.y,position[index].x,position[index].y);
}

class Simulator
{
  public:
    GlobalBoidData cuDevicePtrs;
    float* cudaDevicePosition;
    float* cudaDeviceVelocity;
    float* cudaDeviceAcceleration;
    int* cudaDeviceFlockID;
    int* cudaDeviceFlockSize;

    int numBoids;
    float* position;
    float* velocity;
    float* acceleration;
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

    void InitBoidDataArrays() {
        std::vector<Boid> &AllBoids = *(AllFlocks.begin()->Neighbourhood.GetAllBoidsPtr());
        size_t vecSize = sizeof(float) * 2 * numBoids;
        size_t intSize = sizeof(int) * numBoids;

        position = (float*)malloc(vecSize);
        velocity = (float*)malloc(vecSize);
        acceleration = (float*)malloc(vecSize);
        flockID = (int*)malloc(intSize);
        flockSize = (int*)malloc(intSize);

        for (size_t i=0; i < AllBoids.size(); i++)
        {
            Boid &boid = AllBoids[i];
            Vec2D pos = boid.Position;
            Vec2D vel = boid.Velocity;
            Vec2D acc = boid.Acceleration;
            int fID = boid.FlockID;
            int fSize = 1;

            insertFloat2(position, pos, i);
            insertFloat2(velocity, vel, i);
            insertFloat2(acceleration, acc,i);
            flockID[i] = fID;
            flockSize[i] = fSize;
        }
    }

    /**
     * @pre All Boid arrays have been initialized and are up to date.
     */
    void UpdateBoidPosAndVel() {
        std::vector<Boid> &AllBoids = *(AllFlocks.begin()->Neighbourhood.GetAllBoidsPtr());
        for (size_t i = 0; i < AllBoids.size(); i++)
        {
            Boid &B = AllBoids[i];
            B.Position = Vec2D(position[2*i],position[2*i+1]);
            B.Velocity = Vec2D(velocity[2*i],velocity[2*i+1]);
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
        std::vector<Boid> &AllBoids = *(AllFlocks.begin()->Neighbourhood.GetAllBoidsPtr());
        numBoids = AllBoids.size();
        size_t vecSize = sizeof(float) * 2 * numBoids;
        size_t intSize = sizeof(int) * numBoids;
        cudaMalloc(&cudaDevicePosition, vecSize);
        cudaMalloc(&cudaDeviceVelocity, vecSize);
        cudaMalloc(&cudaDeviceAcceleration, vecSize);
        cudaMalloc(&cudaDeviceFlockID, intSize);
        cudaMalloc(&flockSize, intSize);

        InitBoidDataArrays();
        
        cudaMemcpy(cudaDevicePosition,position,vecSize,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceVelocity,velocity,vecSize,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceAcceleration,acceleration,vecSize,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceFlockID,flockID,intSize,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDeviceFlockSize,flockSize,intSize,cudaMemcpyHostToDevice);

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
        std::chrono::duration<double> ElapsedTime = EndTime - StartTime;

        return ElapsedTime.count(); // return wall clock time diff
    }

    double Tick()
    {
        // Run our actual problem (boid computation)
        auto StartTime = std::chrono::system_clock::now();

        const int threadsPerBlock = 512;
        const int numBlocks = (numBoids + threadsPerBlock - 1) / threadsPerBlock;

        senseAndPlanKernel<<<numBlocks,threadsPerBlock>>>();
        cudaDeviceSynchronize();
        actKernel<<<numBlocks,threadsPerBlock>>>(Params.DeltaTime);
        cudaDeviceSynchronize();

        // GlobalBoidData Results;
        // cudaMemcpyFromSymbol(&Results.position, 
        //            "cuConstBoidData.position",
        //            sizeof(float*),
        //            0, cudaMemcpyDeviceToHost);
        // printf("%p", Results.position);
        cudaMemcpy(position, 
                   cuDevicePtrs.position,
                   numBoids * sizeof(float) * 2,
                   cudaMemcpyDeviceToHost);
        cudaMemcpy(velocity, 
                   cuDevicePtrs.velocity,
                   numBoids * sizeof(float) * 2,
                   cudaMemcpyDeviceToHost);

        //printf("%f\n",position[0]);
        UpdateBoidPosAndVel();     
#pragma omp parallel num_threads(Params.NumThreads)
    {
#pragma for schedule(static)
        for (size_t i = 0; i < AllFlocks.size(); i++)
        {
            AllFlocks[i].Delegate(omp_get_thread_num(), AllFlocks);
        }
#pragma omp barrier
#pragma for schedule(static)
        for (size_t i = 0; i < AllFlocks.size(); i++)
        {
            AllFlocks[i].AssignToFlock(omp_get_thread_num(), AllFlocks);
        }
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