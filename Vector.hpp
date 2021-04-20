#ifndef VECTOR
#define VECTOR

#include "Utils.hpp"
#include <cmath>
#include <vector>

/// TODO: use template magic
class Vec2D
{
  public:
    //////////// :CONSTRUCTORS: //////////////
    Vec2D() : Vec2D(0.0)
    {
    } // delegate constructor, default value 0

    Vec2D(double x, double y)
    {
        Data.push_back(x);
        Data.push_back(y);
    }

    Vec2D(const double init)
    {
        for (size_t i = 0; i < 2; i++)
        {
            // allocate data
            Data.push_back(0);
        }
    }

    Vec2D(const Vec2D *Copy) // duplicate vector
    {
        for (size_t i = 0; i < 2; i++)
        {
            // Copy data over
            Data.push_back((*Copy)[i]);
        }
    }

    double NormSqr() const
    {
        double sum = 0;
        for (size_t i = 0; i < 2; i++)
        {
            sum += sqr(Data[i]);
        }
        return sum;
    }

    double Norm() const
    {
        return std::sqrt(NormSqr());
    }

    //////////// :CREATION: //////////////
    Vec2D operator+(const Vec2D &Other) const
    {
        Vec2D Ret(this);
        for (size_t i = 0; i < 2; i++)
        {
            Ret.Data[i] += Other.Data[i];
        }
        return Ret;
    }

    Vec2D operator-(const Vec2D &Other) const
    {
        Vec2D Ret(this);
        for (size_t i = 0; i < 2; i++)
        {
            Ret.Data[i] -= Other.Data[i];
        }
        return Ret;
    }

    Vec2D operator/(const double Denom) const
    {
        Vec2D Ret(this);
        for (size_t i = 0; i < 2; i++)
        {
            Ret.Data[i] /= Denom;
        }
        return Ret;
    }

    Vec2D operator*(const double Scale) const
    {
        Vec2D Ret(this);
        for (size_t i = 0; i < 2; i++)
        {
            Ret.Data[i] *= Scale;
        }
        return Ret;
    }

    //////////// :ASSIGNMENT: //////////////
    void operator+=(const Vec2D &Other)
    {
        for (size_t i = 0; i < 2; i++)
        {
            Data[i] += Other.Data[i];
        }
    }

    void operator-=(const Vec2D &Other)
    {
        for (size_t i = 0; i < 2; i++)
        {
            Data[i] -= Other.Data[i];
        }
    }

    void operator/=(const double Denom)
    {
        for (size_t i = 0; i < 2; i++)
        {
            Data[i] /= Denom;
        }
    }

    void operator*=(const double Scale)
    {
        for (size_t i = 0; i < 2; i++)
        {
            Data[i] *= Scale;
        }
    }

    void operator=(const Vec2D &Other)
    {
        Data.clear();
        for (size_t i = 0; i < 2; i++)
        {
            // ensures elements are present
            Data.push_back(Other[i]);
        }
    }

    //////////// :GETTERS: //////////////
    double operator[](const size_t i) const
    {
        return Data[i];
    }

    // Easiest if Data is public
    std::vector<double> Data;
};

#endif