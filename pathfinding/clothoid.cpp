#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "clothoid.h"
#include "vec2.h"
#include "fresnel.h"

using namespace fresnel;
using namespace std;

#ifndef PI
#ifdef M_PI
#define PI M_PI
#else
#define PI 3.14159265358979323846
#endif
#endif

inline double fx(double x, double k, double alpha)
{
    return sqrt(x)*(C2(x)*sin(alpha) - S2(x)*(k + cos(alpha))) + sqrt(alpha-x) * (S2(alpha-x)*(1.+k*cos(alpha)) - k*C2(alpha-x)*sin(alpha));
}

inline double dfx(double x, double k, double alpha)
{
    return (C2(x)*sin(alpha) - S2(x)*(k+cos(alpha)))/(2.*sqrt(x)) + (k*C2(alpha-x)*sin(alpha) - S2(alpha-x)*(1+k*cos(alpha)))/(2.*sqrt(alpha-x));
}

// Find a root of f(x) = 0
double newton(double init, double k, double alpha, double err = 1e-5)
{
    double xnext = init,
           xold = 0;

    while (abs(fx(xnext, k, alpha)) > err)
    {
        swap(xnext,xold);
        xnext = xold - fx(xold, k, alpha)/dfx(xold, k, alpha);
    }
    return xnext;
}

//Translate, then rotate
vec2d transrot(const vec2d& p, const vec2d& dp, double rot)
{
    return vec2d(cos(rot)*(p.x+dp.x) - sin(rot)*(p.y+dp.y), sin(rot)*(p.x+dp.x) + cos(rot)*(p.y+dp.y));
}

//Rotate, then translate
vec2d rottrans(const vec2d& p, const vec2d& dp, double rot)
{
    return vec2d(cos(rot)*(p.x) - sin(rot)*(p.y)+dp.x, sin(rot)*(p.x) + cos(rot)*(p.y)+dp.y);
}

//Class function definitions
void ClothoidSpline::construct(std::vector<vec2d> _controlPoints, double tau)
{
    if (_controlPoints.size() < 3)
        throw runtime_error("More than 3 control points are needed.");

    connectors.clear();

    // Figure out the connectors and their tangents(they're half way between each control vertex, plus the endpoints)
    connectors.push_back(connector_t(_controlPoints[0], (_controlPoints[0] - _controlPoints[1]).normalized()));

    for (size_t i = 1; i < _controlPoints.size()-2; i++)
    {
        vec2d p0 = _controlPoints[i],
              p1 = _controlPoints[i+1];
        vec2d cp = (p0+p1)/2.;
        connector_t con(cp, (p0-cp).normalized());
        connectors.push_back(con);
    }
    connectors.push_back(connector_t(_controlPoints.back(), (_controlPoints[_controlPoints.size()-2]-_controlPoints.back()).normalized()));

    //Copy the control points to the class local vector. Exclude the first and last points because they are connectors, not control points
    for (size_t i = 1; i < _controlPoints.size()-1; i++)
        controlPoints.push_back(_controlPoints[i]);

    for (size_t i = 0; i < controlPoints.size(); i++)
    {
        vec2d v = controlPoints[i];
        double normva = (v-connectors[i].p).length(),  // norm of one side
               normvb = (connectors[i+1].p-v).length(); // norm of the other side
        double cosalpha, alpha, k, g, h;    //angle between h and g, and the ratio of their lengths
        clothoid_pair_t clothoid; //the clothoid pair we are constructing

        vec2d vg,vh;
        connector_t p0,p1;

        if (normva > normvb)
        {
            p0 = connectors[i];
            p1 = connectors[i+1];
            p0.T = p0.T * -1;
        }
        else
        {
            p0 = connectors[i+1];
            p1 = connectors[i];
            p1.T = p1.T * -1;
        }

        clothoid.p0 = p0,
        clothoid.p1 = p1;

        if (abs(p0.p.x * (v.y - p1.p.y) + v.x * (p1.p.y - p0.p.y) + p1.p.x * (p0.p.y - v.y)) < 1e-6)
        {
            clothoid.straight_line = true;
            clothoid.length = (p1.p-p0.p).length();
            clothoidPairs.push_back(clothoid);
            continue;
        }
        else
            clothoid.straight_line = false;

        vg = v-p0.p;
        vh = p1.p-v;
        g = vg.length(); h = vh.length();

        cosalpha = min(max(dot(vg.normalized(), vh.normalized()), -1.), 1.);
        alpha = acos(cosalpha);

        {
            double glim = h * (C2(alpha)/S2(alpha)*sin(alpha) - cos(alpha));
            if (g > tau*glim + (1-tau)*h)
            {
                clothoid.g_diff = g - (tau*glim + (1-tau)*h);
                //g = tau*glim + (1-tau)*h;
                g = g - clothoid.g_diff; 
            }
            else
                clothoid.g_diff = 0;
        }

        k = g / h;
        clothoid.flip0 = (vg.x*vh.y - vg.y*vh.x) < 0;
        clothoid.alpha0 = atan2(p0.T.y, p0.T.x);
        clothoid.alpha1 = atan2(p1.T.y, p1.T.x);

        double t0 = newton(0.5*alpha, k, alpha, 1e-10),
               t1 = alpha - t0;
        
        clothoid.a0 = (g+h*cos(alpha))/(C2(t0)+sqrt((alpha-t0)/t0)*(C2(alpha-t0)*cos(alpha)+S2(alpha-t0)*sin(alpha)));
        clothoid.a1 = clothoid.a0*sqrt((alpha-t0)/t0);
        t0 = sqrt(t0*2./PI);
        t1 = sqrt(t1*2./PI);
        clothoid.t0 = t0;
        clothoid.t1 = t1;
        clothoid.length = clothoid.g_diff + t0 + t1;

        clothoidPairs.push_back(clothoid);
    }
}

ClothoidSpline::ClothoidSpline(std::vector<vec2d> controlPoints)
{
    construct(controlPoints, 0.75);
}
