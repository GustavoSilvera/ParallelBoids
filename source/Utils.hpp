#ifndef UTILS
#define UTILS

#include <cassert>
#include <cmath> // pow
#include <cstdio>
#include <fstream>
#include <iostream>
#include <omp.h>
#include <vector>

inline double sqr(const double a)
{
    return a * a;
}

double RandD(const double lo, const double hi, const size_t granularity)
{
    const double Range = hi - lo;
    const double Scale = double(std::rand()) / RAND_MAX;
    return lo + Scale * Range;
}

bool stob(const std::string &s)
{
    // assuming s is either "true" or "false"
    return (s.at(0) == 't');
}

//////////// :PARAMS: //////////////

struct BoidParamsStruct
{
    double Cohesion, Alignment, Separation;
    double BoidMaxVelocity, BoidSize;
    double NeighbourhoodRadius, CollisionRadius;
};

struct SimulatorParamsStruct
{
    size_t NumBoids, NumThreads, NumIterations;
    double DeltaTime;
    bool RenderingMovie;
};

struct ImageParamsStruct
{
    size_t WindowX, WindowY;
};

struct ParamsStruct
{
    BoidParamsStruct BoidParams;
    SimulatorParamsStruct SimulatorParams;
    ImageParamsStruct ImageParams;
};

// global params
ParamsStruct GlobalParams;

void ParseParams(const std::string &FilePath)
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
        if (Tmp.at(0) == '[') // ignoring labels
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
            GlobalParams.BoidParams.BoidSize = std::stod(ParamValue);
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
            GlobalParams.BoidParams.BoidMaxVelocity = std::stod(ParamValue);
        else if (!ParamName.compare("window_x"))
            GlobalParams.ImageParams.WindowX = std::stoi(ParamValue);
        else if (!ParamName.compare("window_y"))
            GlobalParams.ImageParams.WindowY = std::stoi(ParamValue);
        else if (!ParamName.compare("render"))
            GlobalParams.SimulatorParams.RenderingMovie = stob(ParamValue);
        else
            continue;
    }
}

#endif