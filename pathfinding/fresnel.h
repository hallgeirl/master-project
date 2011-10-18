#ifndef __FRESNEL_H__
#define __FRESNEL_H__

namespace fresnel {
    // Fresnel integral with parameter t, where t is the length of the curve
    double S1(double t);
    double C1(double t);

    // Fresnel integral with parameter theta, where theta is the tangent angle deviation from the beginning of the curve
    double S2(double theta);
    double C2(double theta);
}

#endif
