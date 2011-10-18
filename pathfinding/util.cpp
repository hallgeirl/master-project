#include <algorithm>

#include "util.h"

using namespace std;

int gcd(int a, int b)
{
    while (a > 0 && b > 0)
    {
        if (a > b)
            a -= b;
        else
            b -= a;
    }

    return max(a,b);
}
