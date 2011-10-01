#include <FreeImage.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <map>
#include <algorithm>
#include <vector>
#include <assert.h>
#include <limits>
#include <unordered_map>

using namespace std;

float weight_slope = 200.f,
      weight_curvature = 200.f,
      weight_road = 1.f;

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

    T dot(vec2_t& v)
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

            i = max(min(i, height-1), 0);
            j = max(min(j, width-1), 0);

        return data[i*width+j];
    }

    vec2f gridToPoint(const vec2i& a) const
    {
        return vec2f(((float)a.x)*point_spacing, ((float)a.y)*point_spacing);
    }
};

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

struct node_t
{
    vec2i p;
    float cost_g, cost_h;

    node_t(vec2i _p, float _cost_g, float _cost_h) : p(_p), cost_g(_cost_g), cost_h(_cost_h)
    {
    }

    float cost_f() const
    {
        return cost_g + cost_h;
    }

    bool operator< (const node_t& n2) const
    {
        return cost_f() > n2.cost_f();
    }

    bool operator> (const node_t& n2) const
    {
        return cost_f() < n2.cost_f();
    }

    bool operator== (const node_t& n2) const
    {
        return p == n2.p;
    }

    bool operator== (const vec2i n2) const
    {
        return p == n2;
    }
};

//inline interpolate(const  terrain_t& terrain, const vec2_t& a, const vec2_t& b)


//Heuristic for computing the cost from a to b.
inline float h(const terrain_t& terrain, const vec2f& a, const vec2f& b)
{
    float dx = a.x-b.x, dy = a.y-b.y, 
          dz = 0;
        //dz = terrain.getPointBilinear(b.x, b.y) - terrain.getPointBilinear(a.x, a.y);
    return sqrt(dx*dx+dy*dy+dz*dz);
}

inline float transfer_slope(const terrain_t& terrain, const vec2f& a, const vec2f& b)
{
    float dx = a.x-b.x, dy = a.y-b.y;
    float dz = terrain.getPointBilinear(b.x, b.y) - terrain.getPointBilinear(a.x, a.y);
    float slope = fabs(dz/sqrt(dx*dx+dy*dy));
//    float k0 = 0.5;
    
//    if (slope > k0)
//    {
//        printf("slope is infinity\n");
//        fflush(stdout);
//        return numeric_limits<float>::infinity();
//    }

    return weight_slope*(slope+slope*slope);
}

inline float transfer_curvature(const terrain_t& terrain, const vec2f& a, const vec2f& b, const vec2f& prev)
{
    vec2f vb = b-a;
    vec2f va = a-prev;
    float lenA = va.length();
    float lenB = vb.length();

    if (lenA == 0 || lenB == 0) return 0;

    va = va * (1./lenA);
    vb = vb * (1./lenB);

    float cos_theta = va.dot(vb);
    cos_theta = fmin(cos_theta, 1);
    cos_theta = fmax(cos_theta, -1);

    float theta = acos(cos_theta);
    if (theta != theta)
    {
        printf("theta %f %f %f %f %f\n", va.dot(vb), va.x, va.y, vb.x, vb.y);
        fflush(stdout);
    }
    
//    printf("curvature %f\n", 2.f*sin(theta/2.f)/sqrt(lenA*lenB));

    return weight_curvature*2.f*sin(theta/2.f)/sqrt(lenA*lenB);
}

//Transfer function for cost of making the road itself. This is dependent on the length of the road (how much material is used, basically)
inline float transfer_road(const terrain_t& terrain, const vec2f& a, const vec2f& b)
{
    float dx = a.x-b.x, dy = a.y-b.y;
    float dz = 0;
//    float dz = terrain.getPointBilinear(b.x, b.y) - terrain.getPointBilinear(a.x, a.y);

    return weight_road*sqrt(dx*dx+dy*dy+dz*dz);
}

//Integrate the cost over the path segment 
inline float cost(const terrain_t& terrain, const vec2f& a, const vec2f& b, const vec2f& prev)
{
    float i = 0;
    float cost_slope = 0;
    vec2f dir = b-a;
    const float step = 1./dir.length();

    for (float t = 0; t < 1.f; t = i*step, i++)
    {
        float t1 = t, t2 = t+step;
        if (t2 > 1.f)
            t2 = 1.f;

        vec2f _a = dir*t1 + a,
              _b = dir*t2 + a;
        
        cost_slope += transfer_slope(terrain, _a, _b) * (t2-t1);
    }

//    return transfer_road(terrain, a, b);
    float cost_road = transfer_road(terrain, a, b),
//          cost_slope = transfer_slope(terrain, a, b),
          cost_curvature = transfer_curvature(terrain, a, b, prev);
        
    if (cost_road+cost_slope+cost_curvature == numeric_limits<float>::infinity())
        return numeric_limits<float>::infinity();

    return cost_road + cost_slope + cost_curvature;
}

vector<vec2i> pathFind(const terrain_t& terrain, vec2i start, vec2i end, int grid_density)
{
    map<vec2i, vec2i> predecessor;
    unordered_map<vec2i, pair<bool,float> > in_open;
    unordered_map<vec2i, bool> closed;
    vector<node_t> open;

    start.x = ((int)(start.x/grid_density))*grid_density;
    start.y = ((int)(start.y/grid_density))*grid_density;
    end.x = ((int)(end.x/grid_density))*grid_density;
    end.y = ((int)(end.y/grid_density))*grid_density;

    // k = neighborhood radius
    int k = 5;

    //Make a list of neighbors with gcd(i,j) == 1
    vector<vec2i> neighborhood;
    for (int i = 0; i <= k; i++)
    {
        for (int j = 0; j <= k; j++)
        {
            if (i == 0 && j == 0) continue;

            if (gcd(i,j) == 1)
            {
                neighborhood.push_back(vec2i(i,j));
                if (i != 0)
                    neighborhood.push_back(vec2i(-i,j));
                if (j != 0)
                    neighborhood.push_back(vec2i(i,-j));
                if (j != 0 && i != 0)
                    neighborhood.push_back(vec2i(-i,-j));
            }
        }
    }

    printf("Neighborhood:\n");
    for (size_t i = 0; i < neighborhood.size(); i++)
        printf("%d,%d\n", neighborhood[i].x, neighborhood[i].y);

    fflush(stdout);

    node_t current(start, 0, 0);
    predecessor[start] = start;
    open.push_back(node_t(start, 0, h(terrain, terrain.gridToPoint(start), terrain.gridToPoint(end))));
    make_heap(open.begin(), open.end());

    while (open.size() > 0)
    {
//        sleep(1);
//        printf("Open:\n");
//        for (size_t i = 0; i < open.size(); i++)
//        {
//            printf("%d %d: g: %.3f h: %.3f f: %.3f\n", open[i].p.x, open[i].p.y, open[i].cost_g, open[i].cost_h, open[i].cost_f());
//        }

        //Get the next best node and pop it from the heap
        current = open.front();
        pop_heap(open.begin(), open.end());
        open.pop_back();

        //Mark this node as not being in the open list, and move it to the closed list.
        in_open[current.p] = pair<bool, float>(false, 0);
        closed[current.p] = true;
        
        static int ii = 0;
        ii++;
        if (ii % 1000 == 0)
        {
            printf("current %d,%d\tnopen %ld\tnclosed %ld\tf %f\th %f\n", current.p.x, current.p.y, open.size(), closed.size(), current.cost_f(), current.cost_h);
            fflush(stdout);
        }

//        printf("Closed:\n");
//        for (map<vec2i, bool>::iterator it = closed.begin(); it != closed.end(); it++)
//        {
//            printf("%d %d\n", it->first.x, it->first.y);
//        }
//        fflush(stdout);

        //Reached the end?
        if (current == end)
        {
            break;
        }

        //Push the neighbors into the heap
        for (size_t i = 0; i < neighborhood.size(); i++)
        {
            //Position of next node
            vec2i pos = current.p + neighborhood[i]*grid_density;
            if (pos.x >= 0 && pos.y >= 0 && pos.x < terrain.width && pos.y < terrain.height)
            {
                if (closed.find(pos) != closed.end())
                {
                    continue;
                }

                node_t n(pos, 
                        current.cost_g + cost(terrain, terrain.gridToPoint(current.p), 
                            terrain.gridToPoint(pos), 
                            terrain.gridToPoint(predecessor[current.p])),
                        h(terrain, terrain.gridToPoint(pos), terrain.gridToPoint(end)));

//                if (n.cost_g == numeric_limits<float>::infinity())
//                    continue;

                unordered_map<vec2i, pair<bool, float> >::iterator in_open_it = in_open.find(n.p);
                if (in_open_it == in_open.end() || !(in_open_it->second.first))
                {
                    open.push_back(n); push_heap(open.begin(), open.end());
                    in_open[n.p] = pair<bool, float>(true, n.cost_g);
                    predecessor[n.p] = current.p;
                }

                //We found a better path to the node n
                else if (in_open_it != in_open.end() && in_open_it->second.first && in_open_it->second.second > n.cost_g)
                {
                    //Update the cost in the cache
                    in_open_it->second.second = n.cost_g;

                    //Find the element in the open list
//                    vector<node_t>::iterator open_it = find(open.begin(), open.end(), n);
//                    size_t j = distance(open.begin(), open_it);
                    
                    for (size_t j = 0; i < open.size(); i++)
                    {
                        if (open[j].p == n.p)
                        {
                            open[j].cost_g = n.cost_g;
                            int heapnode = j;
                            //While the current node has a higher cost than the parent
                            while (open[heapnode].cost_f() > open[(heapnode+1)/2 - 1].cost_f() && heapnode > 0)
                            {
                                swap(open[heapnode], open[(heapnode+1)/2 - 1]);
                                heapnode = (heapnode+1)/2 - 1;
                            }
                            break;
                        }
                    }
                    predecessor[n.p] = current.p;
//                    make_heap(open.begin(), open.end());
                }
            }
        }
    }

//    printf("Predecessors (successor -> predecessor):\n");
//    for (map<vec2i, vec2i>::iterator it = predecessor.begin(); it != predecessor.end(); it++)
//    {
//        printf("%d,%d <- %d,%d\n", it->first.x, it->first.y, it->second.x, it->second.y);
//    }

    cout << "Backtracing" << endl;

    //Backtrace
    vector<vec2i> result;

    vec2i current_pos = predecessor[end];
    result.push_back(end);

    while (!(current_pos == start))
    {
//        printf("current %d %d\n", current_pos.x, current_pos.y);
        result.push_back(current_pos);
        current_pos = predecessor[current_pos];
        //Get next one in line
    }
    result.push_back(start);
    for (size_t i = 0; i < result.size(); i++)
    {
        printf("%d, %d\n", result[i].x, result[i].y);
    }
    fflush(stdout);

    return result;
}


int main(int argc, char** argv)
{
    //Terrain dimensions
    int h = 1024, w = 1024;
//    int h = 300, w = 300;
    float spacing = 1.;
    int grid_density = 8;

    //Terrain storage
    unsigned short* terrain_raw = new unsigned short[h*w];
    terrain_t       terrain(h, w, spacing);

    //Terrain heights
    float h_min = 0;
    float h_max = 2700.3;

    //Starting and ending points
//    vec2i start(700, 570), end(900, 24);
    vec2i end(70, 570), start(900, 24);
//    vec2i start(700, 300), end(600, 500);
//    vec2i start(10, 250), end(280, 20);
//    vec2i start(2, 3), end(50, 70);

    //Input and output filenames
    const char* in = argv[1], * out = 0;
    if (argc > 2) out = argv[2];

    //Read terrain
    ifstream input(in, ios::binary);
    input.read((char*)terrain_raw, w*h*2);
    input.close();

    //Map the integers from the terrain to an actual height
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            int base = i*w+j;
            terrain.data[base] = ((double)terrain_raw[base]/65536.) * (h_max-h_min) + h_min;
            if (terrain.data[base] < h_min || terrain.data[base] > h_max) cout << terrain.data[base] << endl;
        }
    }

    vector<vec2i> path = pathFind(terrain, start, end, grid_density);

    FreeImage_Initialise();

    FIBITMAP* bm = FreeImage_Allocate(w, h, 24);

    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            int base = i*w+j;
            unsigned char color = (unsigned char)((double)terrain_raw[base] / 65536. * 256.);
            RGBQUAD rgb;
            rgb.rgbRed = rgb.rgbBlue = rgb.rgbGreen = color;
            FreeImage_SetPixelColor(bm, j, i, &rgb);
        }
    }

    float step = (1./(float)grid_density)/5.;
    for (float t = 2; t < path.size(); t+= step)
    {
        RGBQUAD rgb;
        rgb.rgbGreen = rgb.rgbBlue = 0;
        rgb.rgbRed = 255;

        float x = 0, y = 0;

//        for (size_t i = t-2; i < t+1; i++)
//        if (t >= 2 && t < path.size() - 2)
        {
            for (size_t i = 0; i < path.size(); i++)
            {
                float b = 0;
                float t_j = i-2;
                float t_1 = t_j+1,
                      t_2 = t_j+2,
                      t_3 = t_j+3;

//                if (t_j < 1) t_j = 1;
//                if (t_1 < 1) t_1 = 1;
//                if (t_2 < 1) t_2 = 1;
//                if (t_3 < 1) t_3 = 1;

                if (t <= t_1 && t >= t_j)
                    b = 0.5*pow(t-t_j, 2.);
                else if (t <= t_2 && t >= t_1)
                    b = 0.5-pow(t-t_1, 2.) + t-t_1;
                else if (t >= t_2 && t <= t_3)
                    b = 0.5*pow(1-(t-t_2), 2.);

                x += ((float)path[i].x)*b;
                y += ((float)path[i].y)*b;
            }
        }
        printf("Setting %f,%f\n", x,y);

        FreeImage_SetPixelColor(bm, (int)round(x), (int)round(y), &rgb);
    }

    for (size_t i = 0; i < path.size(); i++)
    {
        RGBQUAD rgb;
        rgb.rgbRed = rgb.rgbBlue = 0;
        rgb.rgbGreen = 255;
        FreeImage_SetPixelColor(bm, path[i].x, path[i].y, &rgb);
    }

    //Save image
    if (out != 0)
    {
        printf("Saving image %s...\n", out);

        FreeImage_Save(FIF_PNG, bm, out);

    }

    FreeImage_Unload(bm);

    FreeImage_DeInitialise();
    delete[] terrain_raw;

    return 0;
}
