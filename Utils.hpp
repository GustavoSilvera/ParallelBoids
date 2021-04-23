#ifndef UTILS
#define UTILS

#include <array>
#include <cstdio>
#include <fstream>
#include <iostream>
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

constexpr size_t Wt = 500;
constexpr size_t Ht = 500;

std::array<std::array<Colour, Ht>, Wt> BlankImage(const size_t H, const size_t W)
{
    std::array<std::array<Colour, Ht>, Wt> Img;
    for (int i = 0; i < W; i++)
    {
        std::array<Colour, Ht> Row;
        for (int j = 0; j < H; j++)
        {
            Row[i] = Colour(0, 0, 0);
        }
        Img[i] = Row;
    }
    return Img;
}

void WritePPMImage(const std::array<std::array<Colour, Ht>, Wt> &Data, const size_t H, const size_t W,
                   const std::string &Filename)
{
    /// NOTE: requires data[i][j] to be RGB within (0, 0, 0) and (255, 255, 255)

    std::ofstream Img(Filename, std::ios_base::out | std::ios_base::binary);
    Img << "P6" << std::endl << W << " " << H << std::endl << "255" << std::endl; // write ppm header
    for (size_t i = 0; i < H; ++i)
    {
        for (size_t j = 0; j < W; ++j)
        {
            const Colour RGB = Data[i][j];
            Img << char(RGB.R) << char(RGB.G) << char(RGB.B);
        }
    }
    Img.close();
    std::cout << "Wrote image file: " << Filename << "\r" << std::flush; // carriage return, no newline
}

#endif