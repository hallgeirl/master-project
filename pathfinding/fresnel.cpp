#include <cmath>
#include "fresnel.h"

using namespace std;

#ifndef PI
#ifdef M_PI
#define PI M_PI
#else
#define PI 3.14159265358979323846
#endif
#endif

// Some constants used to compute f(x) and g(x)
const double fn[] = {0.318309844, 9.34626e-8, -0.09676631, 0.000606222, 0.325539361, 0.325206461, -7.450551455, 32.20380908, -78.8035274, 118.5343352, -102.4339798, 39.06207702};
const double gn[] = {0, 0.101321519, -4.07292e-05, -0.152068115, -0.046292605, 1.622793598, -5.199186089, 7.477942354, -0.695291507, -15.10996796, 22.28401942, -10.8996849};

const int taylor_terms = 11;
const int cs_terms = 12;

double cn[taylor_terms];
double sn[taylor_terms];

bool has_computed_taylor = false;

namespace fresnel
{

    void compute_taylor()
    {
        cn[0] = 1;
        sn[0] = PI/6.;
        for (int n = 0; n < taylor_terms-1; n++) 
        {
            cn[n+1] = -(PI*PI) * (4*n+1)*cn[n] / (4*(2*n+1)*(2*n+2)*(4*n+5));
            sn[n+1] = -(PI*PI) * (4*n+3)*sn[n] / (4*(2*n+2)*(2*n+3)*(4*n+7));
        }
    }

    double fresnelG(double t)
    {
        double ans = 0;
        for (int i = 0; i < cs_terms; i++) 
            ans += gn[i] * pow(t,-2*i-1);

        return ans;
    }

    double fresnelF(double t)
    {
        double ans = 0;
        for (int i = 0; i < cs_terms; i++)
            ans += fn[i] * pow(t,-2*i-1);
        return ans;
    }

    double S1(double t)
    {
        if (!has_computed_taylor)
            compute_taylor();

        double s = 0;
        if (t < 1.6)
        {
            for (int i = 0; i < taylor_terms; i++) 
                s += sn[i]*pow(t,4*i+3);
            return s;
        }
        else
            return 0.5-fresnelF(t)*cos(PI*t*t/2.)-fresnelG(t)*sin(PI*t*t/2.);
    }   

    double C1(double t)
    {
        if (!has_computed_taylor)
            compute_taylor();

        double c = 0;
        if (t < 1.6)
        {
            for (int i = 0; i < taylor_terms; i++)
                c += cn[i]*pow(t,4*i+1);
            return c;
        }
        else
            return 0.5-fresnelG(t)*cos(PI*t*t/2.)+fresnelF(t)*sin(PI*t*t/2.);
    }

    double S2(double theta)
    {
        return S1(sqrt(2.*theta/PI));
    }

    double C2(double theta)
    {
        return C1(sqrt(2.*theta/PI));
    }
}
