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
    Colour(uint8_t r, uint8_t g, uint8_t b)
    {
        R = r;
        G = g;
        B = b;
    }
    Colour()
    {
        Colour(0, 0, 0);
    }
    uint8_t R, G, B;

    Colour Norm0_1() const
    {
        return Colour(R / 255, G / 255, B / 255);
    }

    Colour Norm0_255() const
    {
        return Colour(R * 255, G * 255, B * 255);
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
        // Initialize all the data (1d vector)
        Data = std::vector<Colour>(Params.WindowX * Params.WindowY);
    }

    static ImageParamsStruct Params;
    std::vector<Colour> Data;
    size_t NumExported = 0;
    size_t NumLeading0s = 4; // max 9999 frames
    size_t MaxFrames = std::pow(10, NumLeading0s);

    void SetData(const size_t X, const size_t Y, const Colour &C)
    {
        Data[X + Y * Params.WindowX] = C;
    }

    void SetPixel(const size_t X, const size_t Y, const Colour &C)
    {
        /// TODO: do we need to check bounds always? even with EdgeWrap?
        bool WithinWidth = (X < Params.WindowX);
        bool WithinHeight = (Y < Params.WindowY);
        if (WithinWidth && WithinHeight) // draw boid within bound (triangle)
        {
            SetData(X, Y, C);
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
        SetData(ClampedX, ClampedY, C);
    }

    void Blank()
    {
        const size_t BorderSize = 0;
        for (size_t i = 0; i < Params.WindowX; i++)
        {
            for (size_t j = 0; j < Params.WindowY; j++)
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

        /// TODO: can we write rows all at once instead of each pixel at once
        for (size_t i = 0; i < Data.size(); ++i)
        {
            const Colour &RGB = Data[i];
            Img << char(RGB.R) << char(RGB.G) << char(RGB.B);
        }
        Img.close();
        NumExported++; // exported a new file
        bool PrintLog = false;
        if (PrintLog)
            std::cout << "Wrote image file: " << Filename << "\r" << std::flush; // carriage return, no newline
    }
};

#endif