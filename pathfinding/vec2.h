#ifndef __VEC2__H_
#define __VEC2__H_

template<class T>
struct vec2_t
{
    T x, y;
    vec2_t()
    {
        x = y = -1;
    }
    vec2_t(T _x, T _y)
    {
        x = _x; y = _y;
    }

    float length()
    {
        return sqrt(x*x+y*y);
    }

    vec2_t normalized()
    {
        float len = length();
        return vec2_t(x/len, y/len);
    }

    T dot(vec2_t<T> v)
    {
        return x*v.x+y*v.y;
    }

    vec2_t operator-(vec2_t b) const
    {
        return vec2_t(x-b.x, y-b.y);
    }

    vec2_t operator+(vec2_t b) const
    {
        return vec2_t(x+b.x, y+b.y);
    }

    vec2_t operator*(float b) const
    {
        return vec2_t(x*b, y*b);
    }

    vec2_t operator/(float b) const
    {
        return vec2_t(x/b, y/b);
    }
    
    bool operator< (const vec2_t& n2) const 
    { 
        if (x != n2.x) return x < n2.x; 
        else return y < n2.y; 
    }

    bool operator== (const vec2_t& n2) const
    {
        return x == n2.x && y == n2.y;
    }
};

typedef vec2_t<int> vec2i;
typedef vec2_t<float> vec2f;
typedef vec2_t<float> vec2d;

namespace std 
{
    template<>
    struct hash<vec2i> : public unary_function<vec2i, size_t>
    {
        size_t operator()(const vec2i& v) const
        {
            return v.x + (v.y << 16);
        }
    };
}

template<class T>
double dot(vec2_t<T> a, vec2_t<T> b)
{
    return a.x*b.x+a.y*b.y;
}

#endif
