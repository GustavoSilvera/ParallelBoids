#ifndef VECT
#define VECT

#include "Utils.hpp"
#include <array>
#include <cmath>

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
        Data[0] = x;
        Data[1] = y;
    }

    Vec2D(const double init)
    {
        for (size_t i = 0; i < 2; i++)
        {
            // allocate data
            Data[i] = init;
        }
    }

    Vec2D(const std::array<double, 2> &Copy) // duplicate vector
    {
        for (size_t i = 0; i < 2; i++)
        {
            // Copy data over
            Data[i] = Copy[i];
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

    Vec2D rotate(const double angle) const
    {
        /// TODO: make one that operates on vector itself
        const double rX = cos(angle) * Data[0] - sin(angle) * Data[1];
        const double rY = sin(angle) * Data[0] + cos(angle) * Data[1];
        return Vec2D(rX, rY);
    }

    //////////// :CREATION: //////////////
    Vec2D operator+(const Vec2D &Other) const
    {
        Vec2D Ret(Data);
        for (size_t i = 0; i < 2; i++)
        {
            Ret.Data[i] += Other.Data[i];
        }
        return Ret;
    }

    Vec2D operator-(const Vec2D &Other) const
    {
        Vec2D Ret(Data);
        for (size_t i = 0; i < 2; i++)
        {
            Ret.Data[i] -= Other.Data[i];
        }
        return Ret;
    }

    Vec2D operator/(const double Denom) const
    {
        Vec2D Ret(Data);
        for (size_t i = 0; i < 2; i++)
        {
            Ret.Data[i] /= Denom;
        }
        return Ret;
    }

    /// TODO: add template <typename T>
    Vec2D operator*(const double Scale) const
    {
        Vec2D Ret(Data);
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
        for (size_t i = 0; i < 2; i++)
        {
            // ensures elements are present
            Data[i] = Other[i];
        }
    }

    //////////// :GETTERS: //////////////
    double operator[](const size_t i) const
    {
        return Data[i];
    }

    // Easiest if Data is public
    std::array<double, 2> Data; /// TODO: template the size
};

void DrawCircle(std::array<std::array<Colour, Ht>, Wt> &Frame, const Vec2D &Position, const size_t Radius,
                const Colour &C)
{
    const size_t X = Position[0];
    const size_t Y = Position[1];
    // render circle as body of boid
    for (size_t pX = X - Radius; pX < X + Radius; pX++)
    {
        for (size_t pY = Y - Radius; pY < Y + Radius; pY++)
        {
            bool WithinWidth = (0 <= pX && pX < Wt);
            bool WithinHeight = (0 <= pY && pY < Ht);
            Vec2D Pixel(pX, pY);
            if ((Pixel - Position).NormSqr() < sqr(Radius))
            {
                if (WithinWidth && WithinHeight) // draw boid within bound (triangle)
                {
                    Frame[pX][pY] = C;
                }
            }
        }
    }
}

#endif