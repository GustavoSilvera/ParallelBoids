#ifndef VECT
#define VECT

#include "Utils.hpp"
#include <array>
#include <cmath>
#include <iomanip>

/// TODO: use template magic
#define VN 2
class Vec2D
{
  public:
    //////////// :CONSTRUCTORS: //////////////
    Vec2D() : Vec2D(0.0)
    {
    } // delegate constructor, default value 0

    Vec2D(float x, float y)
    {
        Data[0] = x;
        Data[1] = y;
    }

    Vec2D(const float init)
    {
        for (size_t i = 0; i < VN; i++)
        {
            // allocate data
            Data[i] = init;
        }
    }

    Vec2D(const std::array<float, VN> &Copy) // duplicate vector
    {
        for (size_t i = 0; i < VN; i++)
        {
            // Copy data over
            Data[i] = Copy[i];
        }
    }

    float SizeSqr() const
    {
        float sum = 0;
        for (size_t i = 0; i < VN; i++)
        {
            sum += sqr(Data[i]);
        }
        return sum;
    }

    float Size() const
    {
        return std::sqrt(SizeSqr());
    }

    Vec2D Norm() const
    {
        // divide by magnitude
        return Vec2D(Data) / Size();
    }

    //////////// :CREATION: //////////////
    Vec2D operator+(const Vec2D &Other) const
    {
        Vec2D Ret(Data);
        for (size_t i = 0; i < VN; i++)
        {
            Ret.Data[i] += Other.Data[i];
        }
        return Ret;
    }

    Vec2D operator-(const Vec2D &Other) const
    {
        Vec2D Ret(Data);
        for (size_t i = 0; i < VN; i++)
        {
            Ret.Data[i] -= Other.Data[i];
        }
        return Ret;
    }

    Vec2D operator/(const float Denom) const
    {
        Vec2D Ret(Data);
        for (size_t i = 0; i < VN; i++)
        {
            Ret.Data[i] /= Denom;
        }
        return Ret;
    }

    Vec2D operator*(const float Scale) const
    {
        Vec2D Ret(Data);
        for (size_t i = 0; i < VN; i++)
        {
            Ret.Data[i] *= Scale;
        }
        return Ret;
    }

    Vec2D Rotate(const float Angle) const
    {
        const float rX = cos(Angle) * Data[0] - sin(Angle) * Data[1];
        const float rY = sin(Angle) * Data[0] + cos(Angle) * Data[1];
        return Vec2D(rX, rY);
    }
    //////////// :ASSIGNMENT: //////////////
    void operator+=(const Vec2D &Other)
    {
        for (size_t i = 0; i < VN; i++)
        {
            Data[i] += Other.Data[i];
        }
    }

    void operator-=(const Vec2D &Other)
    {
        for (size_t i = 0; i < VN; i++)
        {
            Data[i] -= Other.Data[i];
        }
    }

    void operator/=(const float Denom)
    {
        for (size_t i = 0; i < VN; i++)
        {
            Data[i] /= Denom;
        }
    }

    void operator*=(const float Scale)
    {
        for (size_t i = 0; i < VN; i++)
        {
            Data[i] *= Scale;
        }
    }

    void operator=(const Vec2D &Other)
    {
        for (size_t i = 0; i < VN; i++)
        {
            // ensures elements are present
            Data[i] = Other.Data[i];
        }
    }

    void RotateInPlace(const float Angle)
    {
        float Tmp1 = cos(Angle) * Data[0] - sin(Angle) * Data[1];
        float Tmp2 = sin(Angle) * Data[0] + cos(Angle) * Data[1];
        Data[0] = Tmp1;
        Data[1] = Tmp2;
    }

    //////////// :GETTERS: //////////////
    float operator[](const size_t i) const
    {
        return Data[i];
    }

    Vec2D LimitMagnitude(const float MaxVec)
    {
        if (SizeSqr() > sqr(MaxVec))
        {
            return Norm() * MaxVec;
        }
        return (*this);
    }
    // Easiest if Data is public
    std::array<float, VN> Data; /// TODO: template the size
};

inline std::ostream &operator<<(std::ostream &OutStream, const Vec2D &V)
{
    OutStream << "(";
    for (size_t i = 0; i < VN - 1; i++)
    {
        OutStream << std::to_string(int(V[i])) << ",";
    }
    OutStream << std::to_string(int(V[VN - 1])) + ")";
    return OutStream;
}

#endif