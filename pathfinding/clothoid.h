#ifndef __CLOTHOID_H__
#define __CLOTHOID_H__

#include <vector>

#include "vec2.h"

struct connector_t
{
    vec2d T, //tangent vector
          p; //position

    connector_t(vec2d _p, vec2d _T)
    {
        p = _p; 
        T = _T;
    }

    connector_t()
    {
    }
};

struct clothoid_pair_t
{
    double length, 
           a0, a1, //Scaling factors
           alpha0, alpha1, //Angle of starting tangent
           t0, t1, //Curve limits
           g_diff; //Difference between the actual g, and the limited g (g_diff = length of beginning straight line segment)

    connector_t p0, p1; // Point at the end of "g" and "h" respectively, with normal
    bool flip0, dir;
};

class ClothoidSpline
{
private:
    std::vector<connector_t> connectors;
    std::vector<vec2d> controlPoints;

    void construct(std::vector<vec2d> controlPoints, double tau);

public:
    std::vector<clothoid_pair_t> clothoidPairs;
    ClothoidSpline(std::vector<vec2d> controlPoints);
};

#endif
