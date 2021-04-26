#ifndef UTILS
#define UTILS

#include <cassert>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <omp.h>
#include <vector>

inline double sqr(const double a)
{
    return a * a;
}

class Colour
{
  public:
    Colour(int r, int g, int b)
    {
        R = r;
        G = g;
        B = b;
    }
    Colour()
    {
        Colour(0, 0, 0);
    }
    double R;
    double G;
    double B;

    Colour Norm0_1() const
    {
        return Colour(R / 255.0, G / 255.0, B / 255.0);
    }

    Colour Norm0_255() const
    {
        return Colour(R * 255.0, G * 255.0, B * 255.0);
    }
};

class Image
{
  public:
    Image(const size_t W, const size_t H)
    {
        MaxWidth = W;
        MaxHeight = H;
        // Initialize all the data
        for (size_t i = 0; i < W; i++)
        {
            std::vector<Colour> Row;
            for (size_t j = 0; j < H; j++)
            {
                Row.push_back(Colour(0, 0, 0));
            }
            Data.push_back(Row);
        }
        assert(Data.size() == W && Data[0].size == H);
        // clear the data, all black
        Blank();
    }

    size_t MaxWidth;
    size_t MaxHeight;
    std::vector<std::vector<Colour>> Data;

    void SetPixel(const size_t X, const size_t Y, const Colour C)
    {
        bool WithinWidth = (0 <= X && X < MaxWidth);
        bool WithinHeight = (0 <= Y && Y < MaxHeight);
        if (WithinWidth && WithinHeight) // draw boid within bound (triangle)
        {
            Data[X][Y] = C;
        }
    }

    void Blank()
    {
#pragma omp parallel for
        for (int i = 0; i < MaxWidth; i++)
        {
            for (int j = 0; j < MaxHeight; j++)
            {
                SetPixel(i, j, Colour(0, 0, 0));
            }
        }
    }

    void DrawCircle(const size_t X, const size_t Y, const size_t Radius, const Colour &C)
    {
        // render circle as body of boid
        for (size_t pX = X - Radius; pX < X + Radius; pX++)
        {
            for (size_t pY = Y - Radius; pY < Y + Radius; pY++)
            {
                bool WithinWidth = (0 <= pX && pX < MaxWidth);
                bool WithinHeight = (0 <= pY && pY < MaxHeight);
                if (sqr(pX - X) + sqr(pY - Y) < sqr(Radius))
                {
                    if (WithinWidth && WithinHeight) // draw boid within bound (triangle)
                    {
                        SetPixel(pX, pY, C);
                    }
                }
            }
        }
    }

    void WritePPMImage(const std::string &Filename)
    {
        /// NOTE: requires data[i][j] to be RGB within (0, 0, 0) and (255, 255, 255)

        std::ofstream Img(Filename, std::ios_base::out | std::ios_base::binary);
        Img << "P6" << std::endl << MaxWidth << " " << MaxHeight << std::endl << "255" << std::endl; // write ppm header
        for (size_t j = 0; j < MaxHeight; ++j)
        {
            for (size_t i = 0; i < MaxWidth; ++i)
            {
                const Colour RGB = Data[i][j];
                Img << char(RGB.R) << char(RGB.G) << char(RGB.B);
            }
        }
        Img.close();
        std::cout << "Wrote image file: " << Filename << "\r" << std::flush; // carriage return, no newline
    }
};

#endif