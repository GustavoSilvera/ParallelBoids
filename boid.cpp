#include <opencv2/opencv.hpp>
#include <vector>

/// TODO: don't use globals
const int MaxWidth = 500;
const int MaxHeight = 500;

class boid_t
{
    boid_t(int x0, int y0)
    {
        Position = cv::Vec2d(x0, y0); // set posixtion
        Velocity = cv::Vec2d(0, 0);   // set initial velocity
    }
    cv::Vec2d Position; // 2d vector of doubles
    cv::Vec2d Velocity;
    static cv::Vec2d COM(0, 0);    // static centre of mass for all boids
    static cv::Vec2d AvgVel(0, 0); // static centre of mass for all boids
    cv::Vec2d rule1() const
    {
        // "fly towards the centre of mass of neighbouring boids"
        float Ferocity = 0.01; // moves 1% of the way to the COM
        return (boid_t::COM - Position) * Ferocity;
    }

    cv::Vec2d rule2(std::vector<boid_t> &AllBoids) const
    {
        // slightly "steer away" from nearby boids to avoid collisions
        cv::Vec2d Disp; // displacement away from neighbouring boids
        for (const boid_t &Neighbour : AllBoids)
        {
            if (std::abs(Neighbour.Position - Position) < 100.0)
            {
                // pushes away from nearby boids, displaces 0 if itself
                Disp = Disp - (Neighbour.Position - Position);
            }
        }
        return Disp;
    }

    cv::Vec2d rule3() const
    {
        // try to match velocity to the rest of the group
        float Ferocity = 0.125; // moves 1/8th of the way to the AvgVel
        return (boid_t::AvgVel - Velocity) * Ferocity;
    }

    void Update(std::vector<boid_t> &AllBoids)
    {
        cv::Vec2d v1 = rule1();
        cv::Vec2d v2 = rule2(AllBoids);
        cv::Vec2d v3 = rule3(AllBoids);
        Velocity = Velocity + v1 + v2 + v3;
        Position = Positoin + Velocity;
    }

    void Draw(cv::Mat &Frame) const
    {
        // draws a singular (white) pixel for now
        cv::Point Location = cv::Point(Position[0], Position[1]);
        Frame.at<cv::Vec3b>(Location) = cv::Vec3b(255, 255, 255);
    }

    static void ComputeCOM(std::vector<boid_t> &AllBoids)
    {
        // the "centre of mass" of all the boids
        boid_t::COM = cv::Vec2d(0, 0); // reset from last time
        for (const boid_t &boid : AllBoids)
        {
            boid_t::COM += boid.Position; // accumulate all boids
        }
        boid_t::COM /= AllBoids.size(); // divide by count
    }

    static void ComputeAvgVel(std::vector<boid_t> &AllBoids)
    {
        // the "centre of mass" of all the boids
        boid_t::AvgVel = cv::Vec2d(0, 0); // reset from last time
        for (const boid_t &boid : AllBoids)
        {
            boid_t::AvgVel += boid.Velocity; // accumulate all boids
        }
        boid_t::AvgVel /= AllBoids.size(); // divide by count
    }
};

void ComputeFrame(std::vector<boid_t> &AllBoids, const double t)
{
    std::string FrameTitle = "Frame" + std::to_string(t) + ".png";
    cv::Mat Frame = cv::Mat::zeros(Size(MaxWidth, MaxHeight), cd::CV_8UC1);
    boid_t::ComputeCOM(AllBoids);    // technically incorrect
    boid_t::ComputeAvgVel(AllBoids); // technically incorrect
    for (boid_t &B : AllBoids)
    {
        B.Draw(Frame);
        B.Update(AllBoids);
    }
    cv::imwrite(FrameTitle, Frame); // should return true
    return
}

std::vector<boid_t> InitBoids()
{
    std::vector<boid_t> AllBoids;
    const int NumBoids = 100;
    for (size_t i = 0; i < NumBoids; i++)
    {
        int x0 = std::rand() % MaxWidth;
        int y0 = std::rand() % MaxHeight;
        boid_t NewBoid(x0, y0);
        AllBoids.push_back(NewBoid);
    }
    return AllBoids;
}

int main()
{
    double TimeBudget = 2.0;
    std::vector<boid_t> AllBoids = InitBoids();
    const double dt = 0.05;
    double t = 0;
    while (t < TimeBudget)
    {
        ComputeFrame(AllBoids, t);
    }
    return 0;
}

/// NOTE: resources:
/// PSEUDOCODE: http://www.vergenet.net/~conrad/boids/pseudocode.html
/// OVERVIEW: https://cs.stanford.edu/people/eroberts/courses/soco/projects/2008-09/modeling-natural-systems/boids.html
/// BASELINE: https://eater.net/boids