#ifndef UTILS
#define UTILS

#include <cassert>
#include <cmath> // pow
#include <cstdio>
#include <fstream>
#include <iostream>
#include <vector>

inline float sqr(const float a)
{
    return a * a;
}

inline float RandD(const float lo, const float hi, const size_t granularity)
{
    const float Range = hi - lo;
    const float Scale = float(std::rand()) / RAND_MAX;
    return lo + Scale * Range;
}

inline bool stob(const std::string &s)
{
    // assuming s is either "true" or "false"
    return (s.at(0) == 't');
}

//////////// :PARAMS: //////////////

struct BoidParamsStruct
{
    float Cohesion, Alignment, Separation;
    float MaxVel, Radius;
    float NeighbourhoodRadius, CollisionRadius;
    bool ColourByThread;
};

struct SimulatorParamsStruct
{
    int NumThreads;
    size_t NumBoids, NumIterations;
    float DeltaTime;
    bool ParallelizeAcrossFlocks, RenderingMovie;
};

struct FlockParamsStruct
{
    bool UseFlocks;
    int MaxSize;
    size_t MaxNumComm, UseLocalNeighbourhoods;
    float WeightFlockSize, WeightFlockDist;
};

struct ImageParamsStruct
{
    size_t WindowX, WindowY;
    bool RenderBB;
};

struct TracerParamsStruct
{
    bool TrackMem, TrackTickT, TrackFlockSizes;
};

struct ParamsStruct
{
    BoidParamsStruct BoidParams;
    SimulatorParamsStruct SimulatorParams;
    FlockParamsStruct FlockParams;
    ImageParamsStruct ImageParams;
    TracerParamsStruct TracerParams;
};

// global params (extern for multiple .o files)
extern ParamsStruct GlobalParams;

inline void ParseParams(const std::string &FilePath)
{
    // create input stream to get file data
    std::ifstream Input(FilePath);
    if (!Input.is_open())
    {
        std::cout << "ERROR: could not open" << FilePath << std::endl;
        exit(1);
    }
    std::cout << "Reading params from " << FilePath << std::endl;

    // read from file into params
    std::string Tmp;
    const std::string Delim = "=";
    while (!Input.eof())
    {
        Input >> Tmp;
        if (Input.bad() || Input.fail())
            break;
        if (Tmp.at(0) == '[' || Tmp.at(0) == '#') // ignoring labels & comments
            continue;
        std::string ParamName = Tmp.substr(0, Tmp.find(Delim));
        std::string ParamValue = Tmp.substr(Tmp.find(Delim) + 1, Tmp.size());
        if (!ParamName.compare("num_boids"))
            GlobalParams.SimulatorParams.NumBoids = std::stoi(ParamValue);
        else if (!ParamName.compare("num_iters"))
            GlobalParams.SimulatorParams.NumIterations = std::stoi(ParamValue);
        else if (!ParamName.compare("num_threads"))
            GlobalParams.SimulatorParams.NumThreads = std::stoi(ParamValue);
        else if (!ParamName.compare("timestep"))
            GlobalParams.SimulatorParams.DeltaTime = std::stod(ParamValue);
        else if (!ParamName.compare("boid_radius"))
            GlobalParams.BoidParams.Radius = std::stod(ParamValue);
        else if (!ParamName.compare("cohesion"))
            GlobalParams.BoidParams.Cohesion = std::stod(ParamValue);
        else if (!ParamName.compare("alignment"))
            GlobalParams.BoidParams.Alignment = std::stod(ParamValue);
        else if (!ParamName.compare("separation"))
            GlobalParams.BoidParams.Separation = std::stod(ParamValue);
        else if (!ParamName.compare("collision_radius"))
            GlobalParams.BoidParams.CollisionRadius = std::stod(ParamValue);
        else if (!ParamName.compare("neighbourhood_radius"))
            GlobalParams.BoidParams.NeighbourhoodRadius = std::stod(ParamValue);
        else if (!ParamName.compare("max_vel"))
            GlobalParams.BoidParams.MaxVel = std::stod(ParamValue);
        else if (!ParamName.compare("window_x"))
            GlobalParams.ImageParams.WindowX = std::stoi(ParamValue);
        else if (!ParamName.compare("window_y"))
            GlobalParams.ImageParams.WindowY = std::stoi(ParamValue);
        else if (!ParamName.compare("render"))
            GlobalParams.SimulatorParams.RenderingMovie = stob(ParamValue);
        else if (!ParamName.compare("par_flocks"))
            GlobalParams.SimulatorParams.ParallelizeAcrossFlocks = stob(ParamValue);
        else if (!ParamName.compare("colour_mode"))
            GlobalParams.BoidParams.ColourByThread = stob(ParamValue);
        else if (!ParamName.compare("max_size"))
            GlobalParams.FlockParams.MaxSize = std::stoi(ParamValue);
        else if (!ParamName.compare("max_flock_delegation"))
            GlobalParams.FlockParams.MaxNumComm = std::stoi(ParamValue);
        else if (!ParamName.compare("is_local_neighbourhood"))
            GlobalParams.FlockParams.UseLocalNeighbourhoods = stob(ParamValue);
        else if (!ParamName.compare("track_mem"))
            GlobalParams.TracerParams.TrackMem = stob(ParamValue);
        else if (!ParamName.compare("track_tick_t"))
            GlobalParams.TracerParams.TrackTickT = stob(ParamValue);
        else if (!ParamName.compare("weight_flock_size"))
            GlobalParams.FlockParams.WeightFlockSize = std::stod(ParamValue);
        else if (!ParamName.compare("weight_flock_dist"))
            GlobalParams.FlockParams.WeightFlockDist = std::stod(ParamValue);
        else if (!ParamName.compare("use_flocks"))
            GlobalParams.FlockParams.UseFlocks = stob(ParamValue);
        else if (!ParamName.compare("track_flock_sizes"))
            GlobalParams.TracerParams.TrackFlockSizes = stob(ParamValue);
        else if (!ParamName.compare("render_flock_bounding_box"))
            GlobalParams.ImageParams.RenderBB = stob(ParamValue);
        else
            continue;
    }
}

#endif