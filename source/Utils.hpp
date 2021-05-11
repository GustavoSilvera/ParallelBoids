#ifndef UTILS
#define UTILS

#include <cassert>
#include <cmath> // pow
#include <cstdio>
#include <fstream>
#include <iostream>
#include <vector>

inline double sqr(const double a)
{
    return a * a;
}

inline double RandD(const double lo, const double hi, const size_t granularity)
{
    const double Range = hi - lo;
    const double Scale = double(std::rand()) / RAND_MAX;
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
    double Cohesion, Alignment, Separation;
    double MaxVel, Radius;
    double NeighbourhoodRadius, CollisionRadius;
    bool ColourByThread;
};

struct SimulatorParamsStruct
{
    size_t NumBoids, NumThreads, NumIterations;
    double DeltaTime;
    bool ParallelizeAcrossFlocks, RenderingMovie;
};

struct TracerParamsStruct
{
    size_t NumThreads;
};

struct FlockParamsStruct
{
    size_t MaxSize, MaxNumComm;
};

struct ImageParamsStruct
{
    size_t WindowX, WindowY;
};

struct ParamsStruct
{
    BoidParamsStruct BoidParams;
    SimulatorParamsStruct SimulatorParams;
    TracerParamsStruct TracerParams;
    FlockParamsStruct FlockParams;
    ImageParamsStruct ImageParams;
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
        {
            GlobalParams.SimulatorParams.NumThreads = std::stoi(ParamValue);
            GlobalParams.TracerParams.NumThreads = std::stoi(ParamValue);
        }
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
        else
            continue;
    }
}

#endif