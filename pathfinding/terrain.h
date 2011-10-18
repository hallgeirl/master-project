#ifndef __TERRAIN_H__
#define __TERRAIN_H__

#include <cmath>
#include <algorithm>

struct terrain_t
{
    float* data;         //Terrain data
    int height, width;   //Height and width of terrain (in # points)
    float point_spacing; //Distance between points

    terrain_t(int _width, int _height, float _point_spacing)
    {
        height = _height;
        width = _width;
        point_spacing = _point_spacing;
        data = new float[width*height];
    }

    ~terrain_t()
    {
        delete[] data;
    }

    float getPointBilinear(float x, float y) const
    {
        float y_1 = floor(y/point_spacing)*point_spacing;
        float y_2 = ceil(y/point_spacing)*point_spacing;
        if (y_2 == y_1) y_2 += point_spacing;

        float x_1 = floor(x/point_spacing)*point_spacing;
        float x_2 = ceil(x/point_spacing)*point_spacing;
        if (x_1 == x_2) x_2 += point_spacing;

        float fx_1 = getPointNearest(x_1, y_1)*(x_2-x)/(x_2-x_1) + getPointNearest(x_2, y_1)*(x-x_1)/(x_2-x_1);
        float fx_2 = getPointNearest(x_1, y_2)*(x_2-x)/(x_2-x_1) + getPointNearest(x_2, y_2)*(x-x_1)/(x_2-x_1);
        float fp = fx_1*(y_2-y)/(y_2-y_1) + fx_2*(y-y_1)/(y_2-y_1);

        return fp;
    }

    float getPointNearest(float x, float y) const
    {
        int i = round(y/point_spacing),
            j = round(x/point_spacing);

            i = std::max(std::min(i, height-1), 0);
            j = std::max(std::min(j, width-1), 0);

        return data[i*width+j];
    }

    vec2d gridToPoint(const vec2i& a) const
    {
        return vec2d(((float)a.x)*point_spacing, ((float)a.y)*point_spacing);
    }
};
#endif
