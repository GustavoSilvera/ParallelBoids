#ifndef IMAGE_H
#define IMAGE_H

#include "Vec.hpp"
#include <cmath> // pow
#include <fstream>
#include <iostream>
#include <vector>

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
    double R, G, B;

    Colour Norm0_1() const
    {
        return Colour(R / 255.0, G / 255.0, B / 255.0);
    }

    Colour Norm0_255() const
    {
        return Colour(R * 255.0, G * 255.0, B * 255.0);
    }
};

// A list of common colours
const std::vector<Colour> IDColours = {
    Colour(255, 0, 0),    Colour(0, 255, 0),     Colour(0, 0, 255),    Colour(255, 255, 0),   Colour(0, 255, 255),
    Colour(255, 0, 255),  Colour(255, 128, 0),   Colour(0, 128, 255),  Colour(128, 0, 255),   Colour(128, 255, 0),
    Colour(0, 255, 128),  Colour(255, 0, 128),   Colour(200, 255, 0),  Colour(255, 128, 80),  Colour(204, 204, 255),
    Colour(64, 224, 208), Colour(159, 226, 191), Colour(222, 50, 100), Colour(100, 150, 237), Colour(100, 100, 100)};

class Image
{
  public:
    Image()
    {
        Params = GlobalParams.ImageParams;
    }

    void Init()
    {
        // Initialize all the data
        Data = std::vector<std::vector<Colour>>(Params.WindowX, std::vector<Colour>(Params.WindowY));
    }

    ImageParamsStruct Params;
    std::vector<std::vector<Colour>> Data;
    size_t NumExported = 0;
    size_t NumLeading0s = 4; // max 9999 frames
    size_t MaxFrames = std::pow(10, NumLeading0s);

    void SetPixel(const size_t X, const size_t Y, const Colour &C)
    {
        /// TODO: do we need to check bounds always? even with EdgeWrap?
        bool WithinWidth = (0 <= X && X < Params.WindowX);
        bool WithinHeight = (0 <= Y && Y < Params.WindowY);
        if (WithinWidth && WithinHeight) // draw boid within bound (triangle)
        {
            Data[X][Y] = C;
        }
    }

    void SetPixel(const Vec2D &Pos, const Colour &C)
    {
        SetPixel(Pos[0], Pos[1], C);
    }

    void SetPixelW(const double X, const double Y, const Colour &C) // sets pixel with wrapping
    {
        const double MaxW = Params.WindowX - 1;
        const double MaxH = Params.WindowY - 1;
        double ClampedX = X;
        if (ClampedX < 0)
        {
            ClampedX += MaxW;
        }
        else if (ClampedX > MaxW)
        {
            ClampedX -= MaxW;
        }
        /// same for y's
        double ClampedY = Y;
        if (ClampedY < 0)
        {
            ClampedY += MaxH;
        }
        else if (ClampedY > MaxH)
        {
            ClampedY -= MaxH;
        }
        Data[ClampedX][ClampedY] = C;
    }

    void Blank()
    {
        const size_t BorderSize = 0;
        for (int i = 0; i < Params.WindowX; i++)
        {
            for (int j = 0; j < Params.WindowY; j++)
            {
                if (i < BorderSize || i > Params.WindowX - BorderSize || j < BorderSize ||
                    j > Params.WindowY - BorderSize)
                {
                    SetPixel(i, j, Colour(255, 255, 255));
                }
                else
                {
                    SetPixel(i, j, Colour(0, 0, 0));
                }
            }
        }
    }

    void DrawSolidCircle(const Vec2D &Center, const size_t Radius, const Colour &C)
    {
        const double X = Center[0];
        const double Y = Center[1];
        for (double pX = X - Radius; pX < X + Radius; pX++)
        {
            for (double pY = Y - Radius; pY < Y + Radius; pY++)
            {
                /// TODO: provide functionality for custom-sized radii
                if (sqr(pX - X) + sqr(pY - Y) < sqr(Radius))
                {
                    SetPixel(pX, pY, C);
                }
            }
        }
    }

    void SetOctants(const double X, const double x, const double Y, const double y, const Colour &C)
    {
        SetPixel(X + x, Y + y, C);
        SetPixel(X - x, Y + y, C);
        SetPixel(X + x, Y - y, C);
        SetPixel(X - x, Y - y, C);
        if (x != y)
        {
            SetPixel(X + y, Y + x, C);
            SetPixel(X - y, Y + x, C);
            SetPixel(X + y, Y - x, C);
            SetPixel(X - y, Y - x, C);
        }
    }

    void DrawStrokedCircle(const Vec2D &Center, const size_t Radius, const Colour &C)
    {
        // Faster Stroked circle, uses Mid-point circle drawing algorithm
        /// NOTE: this is slow
        const double X = Center[0];
        const double Y = Center[1];

        // Initialising the value of P
        int P = 1 - Radius;
        int x = Radius;
        int y = 0;
        while (x > y)
        {
            y++;
            if (P <= 0) // within perimiter
            {
                P = P + 2 * y + 1;
            }
            else // outside perimiter
            {
                x--;
                P = P + 2 * y - 2 * x + 1;
            }
            if (x < y)
                break;
            SetOctants(X, x, Y, y, C);
        }
    }

    void DrawStrokedCircleNaive(const Vec2D &Center, const size_t Radius, const Colour &C)
    {
        const double X = Center[0];
        const double Y = Center[1];

        for (double pX = X - Radius; pX < X + Radius; pX++)
        {
            for (double pY = Y - Radius; pY < Y + Radius; pY++)
            {
                /// TODO: provide functionality for custom-sized radii
                if (sqr(pX - X) + sqr(pY - Y) < sqr(Radius) && sqr(pX - X) + sqr(pY - Y) > sqr(Radius - 1))
                {
                    SetPixel(pX, pY, C);
                }
            }
        }
    }

    void DrawLine(const Vec2D &A, const Vec2D &B, const Colour &C)
    {
        Vec2D Direction = B - A;
        const double Magnitude = Direction.Size();
        Direction /= Magnitude; // normalize it
        /// TODO: add more granularity for lines
        for (size_t i = 0; i < Magnitude; i++)
        {
            /// TODO: verify this works
            const Vec2D Pixel = A + Direction * i;
            SetPixel(Pixel, C);
        }
    }

    void ExportPPMImage()
    {
        if (NumExported > MaxFrames)
        {
            std::cout << "Cannot export more than " << MaxFrames << " frames! " << std::endl;
            return;
        }
        std::string Path = "Out/";
        std::string NumStr = std::to_string(NumExported); // which frame this is
        std::string Filename = Path + std::string(NumLeading0s - NumStr.length(), '0') + NumStr + ".ppm";

        // Begin writing output stream
        std::ofstream Img(Filename, std::ios_base::out | std::ios_base::binary);
        Img << "P6" << std::endl
            << Params.WindowX << " " << Params.WindowY << std::endl
            << "255" << std::endl; // write ppm header
        /// TODO: make sure the cache locality works
        for (size_t j = 0; j < Params.WindowY; ++j)
        {
            for (size_t i = 0; i < Params.WindowX; ++i)
            {
                const Colour RGB = Data[i][j];
                Img << char(RGB.R) << char(RGB.G) << char(RGB.B);
            }
        }
        Img.close();
        NumExported++;                                                       // exported a new file
        std::cout << "Wrote image file: " << Filename << "\r" << std::flush; // carriage return, no newline
    }
};

#endif