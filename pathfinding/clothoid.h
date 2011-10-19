#ifndef __CLOTHOID_H__
#define __CLOTHOID_H__

#include <vector>

#include "vec2.h"

namespace clothoid
{
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

    struct clothoid_point_t
    {
        double t, curvature;
        vec2d pos;
        double integrated_curvature;
    };

    class ClothoidPair
    {
        private:
            double length_total, 
                   a0, a1, //Scaling factors
                   alpha0, alpha1, //Angle of starting tangent
                   t0, t1, //Curve limits
                   g_diff; //Difference between the actual g, and the limited g (g_diff = length of beginning straight line segment)

            connector_t p0, p1;  // Point at the end of "g" and "h" respectively, with normal
            bool flip0, reverse; //reverse indicates that the clothoids in the pair is switched

        private:
            void construct(const connector_t& pa, const connector_t& pb, const vec2d& ctrl, double tau);

        public:
            ClothoidPair(const vec2d& pa, const vec2d& pb, const vec2d& Ta, const vec2d& Tb, const vec2d& ctrl, double tau = 0.75);
            ClothoidPair(const connector_t& pa, const connector_t& pb, const vec2d& ctrl, double tau = 0.75);
            clothoid_point_t lookup(double t); //Lookup a coordinate in the clothoid pair
            double length();
            double integratedCurvature();
    };

    class ClothoidSpline
    {
        private:
            std::vector<connector_t> connectors;
            std::vector<vec2d> controlPoints;
            std::vector<double> lengths;          // Accumulated curve length at each clothoid pair's starting point
            std::vector<ClothoidPair> clothoidPairs;

        private:
            void construct(const std::vector<vec2d>& controlPoints, double tau);

        public:
            ClothoidSpline(const std::vector<vec2d>& controlPoints, double tau = 0.75);

            clothoid_point_t lookup(double t);
            size_t lookupClothoidPairIndex(double t);
            double length();
    };
}
#endif
