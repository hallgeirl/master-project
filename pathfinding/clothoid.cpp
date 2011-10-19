/*
    Author: Hallgeir Lien <hallgeir.lien@gmail.com>
    Feel free to distribute or modify this file as you please. 
    All I ask is that you leave the author's name in the file.
*/
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <ctime>

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

namespace clothoid {
    /*
       Auxilliary functions 
     */

    inline double fx(double x, double k, double cosalpha, double sinalpha, double Cx, double Sx, double Cx2, double Sx2, double alpha)
    {
        return sqrt(x)*(Cx*sinalpha - Sx*(k + cosalpha)) + sqrt(alpha-x) * (Sx2*(1.+k*cosalpha) - k*Cx2*sinalpha);
    }

    inline double dfx(double x, double k, double cosalpha, double sinalpha, double Cx, double Sx, double Cx2, double Sx2, double alpha)
    {
        return (Cx*sinalpha - Sx*(k+cosalpha))/(2.*sqrt(x)) + (k*Cx2*sinalpha - Sx2*(1+k*cosalpha))/(2.*sqrt(alpha-x));
    }

    // Find a root of f(x) = 0
    double newton(double init, double k, double alpha, double err = 1e-5)
    {
        double xnext = init,
               xold = 0;

        double Cx = C2(xnext), Sx = S2(xnext), 
               Cx2 = C2(alpha-xnext), Sx2 = S2(alpha-xnext),
               cosalpha = cos(alpha), sinalpha = sin(alpha);

        double fx_ = fx(xnext, k, cosalpha, sinalpha, Cx, Sx, Cx2, Sx2, alpha);

        while (abs(fx_) > err)
        {
            swap(xnext,xold);
            xnext = xold - fx_/dfx(xold, k, cosalpha, sinalpha, Cx, Sx, Cx2, Sx2, alpha);
            Cx = C2(xnext); Sx = S2(xnext);
            Cx2 = C2(alpha - xnext); Sx2 = S2(alpha - xnext);
            fx_ = fx(xnext, k, cosalpha, sinalpha, Cx, Sx, Cx2, Sx2, alpha);
        }
        return xnext;
    }

    //Translate, then rotate
    inline vec2d transrot(const vec2d& p, const vec2d& dp, double rot)
    {
        return vec2d(cos(rot)*(p.x+dp.x) - sin(rot)*(p.y+dp.y), sin(rot)*(p.x+dp.x) + cos(rot)*(p.y+dp.y));
    }

    //Rotate, then translate
    inline vec2d rottrans(const vec2d& p, const vec2d& dp, double rot)
    {
        return vec2d(cos(rot)*(p.x) - sin(rot)*(p.y)+dp.x, sin(rot)*(p.x) + cos(rot)*(p.y)+dp.y);
    }

    /*
       ClothoidPair member functions
     */

    ClothoidPair::ClothoidPair(const vec2d& pa, const vec2d& pb, const vec2d& Ta, const vec2d& Tb, const vec2d& ctrl, double tau)
    {
        construct(connector_t(pa, Ta), connector_t(pb, Tb), ctrl, tau);
    }

    ClothoidPair::ClothoidPair(const connector_t& pa, const connector_t& pb, const vec2d& ctrl, double tau)
    {
        construct(pa, pb, ctrl, tau);
    }

    double ClothoidPair::length()
    {
        return length_total;
    }

    clothoid_point_t ClothoidPair::lookup(double t)
    {
        //Get lengths of each clothoid or segment
        double l[4];
        clothoid_point_t result;
        if (reverse)
        {
            //Reversed. First, we have the second clothoid, then we have the first clothoid and then the line.
            l[0] = 0;
            l[1] = t1*a1;
            l[2] = l[1] + t0*a0;
            l[3] = l[2] + g_diff;
        }
        else
        {
            //No reverse. First, we have a line. Then, we have the first clothoid, and finally the second.
            l[0] = 0;
            l[1] = g_diff;
            l[2] = l[1] + t0*a0;
            l[3] = l[2] + t1*a1;
        }

        if (t < 0 || t > l[3])
        {
            printf("Out of range: %lf (%lf) gdiff: %lf A0: %lf A1: %lf Reverse: %d\n", t, l[3], g_diff, t0*a0, t1*a1, reverse);
        }
        
        //Parameter is from the straight line
        if ((t < l[1] && !reverse) || (t >= l[2] && reverse))
        {
            double tt = t;
            if (reverse) 
            {
                tt -= l[2];
                tt = g_diff - tt;
            }
            result.pos = p0.p + p0.T*tt;
            result.curvature = 0;
        }

        //From first segment (g)
        else if ((t < l[2] && !reverse) || (t >= l[1] && reverse))
        {
            double tt = (t - l[1])/a0;
            if (reverse) tt = t0-tt;

            //from first segment
            vec2d p(a0*C1(tt), a0*S1(tt));

            // Inverse transform
            if (flip0)
                p.y *= -1;

            result.pos = rottrans(p, p0.p, alpha0) + p0.T * g_diff;
            result.curvature = PI*tt/a0;
        }
        //From second segment (h)
        else
        {
            double tt = t;
            if (!reverse) tt -= l[2];
            tt /= a1;
            if (reverse) tt = t1-tt;
            tt = t1-tt;
            vec2d p = vec2d(a1*C1(tt), a1*S1(tt));

            // Inverse transform
            if (!flip0)
                p.y *= -1;

            result.pos = rottrans(p, p1.p, alpha1);
            result.curvature = PI*tt/a1;
        }


//        result.integrated_curvature = 0.5*PI*(t0*t0 / a0 + t1*t1 / a1) / (t0*a0+t1*a0+g_diff);
        result.integrated_curvature = 0.5*PI*(t0*t0 / a0) + 0.5*PI*(t1*t1 / a1);

        return result;
    }

    double ClothoidPair::integratedCurvature()
    {
        return 0.5*PI*(t0*t0 / a0) + 0.5*PI*(t1*t1 / a1);
    }

    void ClothoidPair::construct(const connector_t& pa, const connector_t& pb, const vec2d& v, double tau)
    {
        double normva = (v-pa.p).length(),  // norm of one side
               normvb = (pb.p-v).length(); // norm of the other side
        double cosalpha, alpha, k, g, h;    //angle between h and g, and the ratio of their lengths

        vec2d vg,vh;

        if (normva > normvb)
        {
            p0 = pa;
            p1 = pb;
            p1.T = p1.T * -1;
            reverse = false;
        }
        else
        {
            p0 = pb;
            p1 = pa;
            p0.T = p0.T * -1;
            reverse = true;
        }

        //Check if the points are colinear. If so, we only need a straight line.
        if (abs(p0.p.x * (v.y - p1.p.y) + v.x * (p1.p.y - p0.p.y) + p1.p.x * (p0.p.y - v.y)) < 1e-6)
        {
            length_total = (p1.p-p0.p).length();
            g_diff = length_total;
            t0 = t1 = 0;
            a0 = a1 = 1;
            return;
        }

        vg = v-p0.p;
        vh = p1.p-v;
        g = vg.length(); h = vh.length();

        cosalpha = min(max(dot(vg.normalized(), vh.normalized()), -1.), 1.);
        alpha = acos(cosalpha);

        {
            double glim = h * (C2(alpha)/S2(alpha)*sin(alpha) - cosalpha);
            if (g > tau*glim + (1-tau)*h)
            {
                g_diff = g - (tau*glim + (1-tau)*h);
                g = g - g_diff; 
            }
            else
                g_diff = 0;
        }

        k = g / h;
        flip0 = (vg.x*vh.y - vg.y*vh.x) < 0;
        alpha0 = atan2(p0.T.y, p0.T.x);
        alpha1 = atan2(p1.T.y, p1.T.x);

        t0 = newton(0.5*alpha, k, alpha, 1e-5);
//        t0 = alpha/2.;
        t1 = alpha - t0;

        //Compute scaling factors
        a0 = (g+h*cosalpha)/(C2(t0)+sqrt((alpha-t0)/t0)*(C2(alpha-t0)*cosalpha+S2(alpha-t0)*sin(alpha)));
        a1 = a0*sqrt((alpha-t0)/t0);

        //Convert parameter from angle to length
        t0 = sqrt(t0*2./PI);
        t1 = sqrt(t1*2./PI);

        length_total = g_diff + t0*a0 + t1*a1;
    }



    /*
       ClothoidSpline member functions
     */
    void ClothoidSpline::construct(const std::vector<vec2d>& _controlPoints, double tau)
    {
        if (_controlPoints.size() < 3)
            throw runtime_error("More than 3 control points are needed.");

        //Clear previous spline (if any) 
        connectors.clear();
        controlPoints.clear();
        clothoidPairs.clear();
        lengths.clear();

        // For first pair, the distance is 0
        lengths.push_back(0);

        // Figure out the connectors and their tangents(they're half way between each control vertex, plus the endpoints)
        connectors.push_back(connector_t(_controlPoints[0], (_controlPoints[1] - _controlPoints[0]).normalized()));

        for (size_t i = 1; i < _controlPoints.size()-2; i++)
        {
            vec2d p0 = _controlPoints[i],
                  p1 = _controlPoints[i+1];
            vec2d cp = (p0+p1)/2.;
            connector_t con(cp, (cp-p0).normalized());
            connectors.push_back(con);
        }
        connectors.push_back(connector_t(_controlPoints.back(), (_controlPoints.back() - _controlPoints[_controlPoints.size()-2]).normalized()));
//        connectors.push_back(connector_t(_controlPoints.back(), (_controlPoints[_controlPoints.size()-2]-_controlPoints.back()).normalized()));

        //Copy the control points to the class local vector. Exclude the first and last points because they are connectors, not control points
        for (size_t i = 1; i < _controlPoints.size()-1; i++)
            controlPoints.push_back(_controlPoints[i]);

        for (size_t i = 0; i < controlPoints.size(); i++)
        {
            vec2d v = controlPoints[i];
            ClothoidPair clothoid(connectors[i], connectors[i+1], v, tau); //the clothoid pair we are constructing
            //        clothoid.construct(connectors[i], connectors[i+1], v, tau);
            clothoidPairs.push_back(clothoid);
            lengths.push_back(clothoid.length()+lengths[i]);
        }
    }

    ClothoidSpline::ClothoidSpline(const std::vector<vec2d>& controlPoints, double tau)
    {
        construct(controlPoints, tau);
    }

    clothoid_point_t ClothoidSpline::lookup(double t)
    {
        int result = lookupClothoidPairIndex(t);

        //    printf("clothoid no. %d t: %lf tmod: %lf length (at beg/end): %lf/%lf\n", result, t, t-lengths[result], lengths[result], lengths[result+1]);
        return clothoidPairs[result].lookup(t-lengths[result]);
    }

    size_t ClothoidSpline::lookupClothoidPairIndex(double t)
    {
        //Check that t is within the range of the spline
        if (t < 0 || t > lengths.back())
            throw runtime_error("Parameter t outside of curve range");

        //Determine which pair of clothoids we need by a binary search by finding the biggest length < t
        int lower = 0, upper = lengths.size()-1;
        int result = -1;
        while (lower <= upper && result < 0)
        {
            int i = (upper+lower)/2;
            if (lengths[i] <= t)
            {
                if (lengths[i+1] < t)
                    lower = i+1;
                else
                    result = i;
            }
            else
                upper = i;
        }

        return result;
    }

    double ClothoidSpline::length()
    {
        return lengths.back();
    }

}
