#ifndef UTILS
#define UTILS

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

std::vector<std::vector<Colour>> BlankImage(const int Width, const int Height)
{
    std::vector<std::vector<Colour>> Img;
    for (int i = 0; i < Width; i++)
    {
        std::vector<Colour> Row;
        for (int j = 0; j < Height; j++)
        {
            Row.push_back(Colour(0, 0, 0));
        }
        Img.push_back(Row);
    }
    return Img;
}

void WritePPMImage(const std::vector<std::vector<Colour>> &Data, const int Width, const int Height,
                   const std::string &Filename)
{
    /// NOTE: requires data[i][j] to be RGB within (0, 0, 0) and (255, 255, 255)

    std::ofstream Img(Filename, std::ios_base::out | std::ios_base::binary);
    Img << "P6" << std::endl << Width << " " << Height << std::endl << "255" << std::endl; // write ppm header
    for (size_t i = 0; i < Height; ++i)
    {
        for (size_t j = 0; j < Width; ++j)
        {
            const Colour RGB = Data[i][j];
            Img << char(RGB.R) << char(RGB.G) << char(RGB.B);
        }
    }
    Img.close();
    std::cout << "Wrote image file: " << Filename << std::endl;
}

#endif