#include <FreeImage.h>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <map>
#include <algorithm>
#include <vector>
#include <assert.h>
#include <limits>
#include <unordered_map>
#include <sstream>

#include "util.h"
#include "vec2.h"
#include "terrain.h"
#include "clothoid.h"
#include "fresnel.h"

using namespace std;
using namespace fresnel;
using namespace clothoid;

#ifndef PI
#ifdef M_PI
#define PI M_PI
#else
#define PI 3.14159265358979323846
#endif
#endif

#define PRINT_ALL(cont, counter, fmt, ...) for (size_t counter = 0; counter < cont.size(); counter++) printf(fmt, __VA_ARGS__);

float weight_slope = 200.f,
      weight_curvature = 200.f,
      weight_road = 1.f;

vec2d transrot(const vec2d& p, const vec2d& dp, double rot);

//Rotate, then translate
vec2d rottrans(const vec2d& p, const vec2d& dp, double rot);

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
inline float h(const terrain_t& terrain, const vec2d& a, const vec2d& b)
{
    float dx = a.x-b.x, dy = a.y-b.y, 
          dz = 0;
        //dz = terrain.getPointBilinear(b.x, b.y) - terrain.getPointBilinear(a.x, a.y);
    return sqrt(dx*dx+dy*dy+dz*dz)*weight_road;
}

inline float get_slope(const terrain_t& terrain, const vec2d& a, const vec2d& b)
{
    float dx = a.x-b.x, dy = a.y-b.y;
    float dz = terrain.getPointBilinear(b.x, b.y) - terrain.getPointBilinear(a.x, a.y);
    float slope = fabs(dz/sqrt(dx*dx+dy*dy));

    return slope;
}

inline float transfer_slope(const terrain_t& terrain, const vec2d& a, const vec2d& b)
{
    float k0 = 2;
    
    float slope = get_slope(terrain, a, b);
    if (slope > k0)
    {
//        printf("slope is infinity\n");
//        fflush(stdout);
//        return numeric_limits<float>::infinity();
    }
    
    return weight_slope*(slope+slope*slope);
}

inline float transfer_curvature(const terrain_t& terrain, const vec2d& a, const vec2d& b, const vec2d& prev)
{
    vec2d vb = b-a;
    vec2d va = a-prev;
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
        printf("theta is NAN! %f %f %f %f %f\n", va.dot(vb), va.x, va.y, vb.x, vb.y);
        fflush(stdout);
    }
    
    return weight_curvature*2.f*sin(theta/2.f)/sqrt(lenA*lenB);
}

//Transfer function for cost of making the road itself. This is dependent on the length of the road (how much material is used, basically)
inline float transfer_road(const terrain_t& terrain, const vec2d& a, const vec2d& b)
{
    float dx = a.x-b.x, dy = a.y-b.y;
    float dz = 0;
//    float dz = terrain.getPointBilinear(b.x, b.y) - terrain.getPointBilinear(a.x, a.y);

    return weight_road*sqrt(dx*dx+dy*dy+dz*dz);
}

//Integrate the cost over the path segment 
inline float cost(const terrain_t& terrain, const vec2d& a, const vec2d& b, const vec2d& prev)
{
    float i = 0;
    float cost_slope = 0;
    vec2d dir = b-a;

    const float step = 1.*terrain.point_spacing/dir.length();

    for (float t = 0; t < 1.f; t = i*step, i++)
    {
        float t1 = t, t2 = t+step;
        if (t2 > 1.f)
            t2 = 1.f;

        vec2d _a = dir*t1 + a,
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

vector<vec2d> pathFind(const terrain_t& terrain, vec2i start, vec2i end, int grid_density)
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

//    printf("Neighborhood:\n");
//    for (size_t i = 0; i < neighborhood.size(); i++)
//        printf("%d,%d\n", neighborhood[i].x, neighborhood[i].y);
//
//    fflush(stdout);

    node_t current(start, 0, 0);
    predecessor[start] = start;
    open.push_back(node_t(start, 0, h(terrain, terrain.gridToPoint(start), terrain.gridToPoint(end))));
    make_heap(open.begin(), open.end());

    while (open.size() > 0)
    {
//        sleep(1);
//        printf("Open:\n");
//        PRINT_ALL(open, i, "%d %d: g: %.3f h: %.3f f: %.3f\n", open[i].p.x, open[i].p.y, open[i].cost_g, open[i].cost_h, open[i].cost_f());

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
    vector<vec2d> result;

    vec2i current_pos = predecessor[end];
    result.push_back(terrain.gridToPoint(end));

    while (!(current_pos == start))
    {
        result.push_back(terrain.gridToPoint(current_pos));
        current_pos = predecessor[current_pos];
    }
    result.push_back(terrain.gridToPoint(start));
//    PRINT_ALL(result, i, "%lf, %lf\n", result[i].x, result[i].y);

    return result;
}

long filesize(ifstream& f)
{
    long current = f.tellg();
    f.seekg(0, ios::beg);
    long beg = f.tellg();
    f.seekg(0, ios::end);
    long end = f.tellg();
    f.seekg(current, ios::beg);

    return end-beg;
}

void writeRoadXML(string filename, const vector<vec2d>& controlPoints, const terrain_t& terrain)
{
    vec2d startDirV = controlPoints[1] - controlPoints[0];
    startDirV = startDirV * (1./startDirV.length());
    double startDir = acos(fmin(fmax(startDirV.dot(vec2d(1.,0.)),-1), 1));

    // Determine which quadrant the direction is in
    if (startDirV.y < 0)
        startDir = 2.*PI - startDir;

    ifstream head_f("roadxml_template_head.rnd");
    ifstream tail_f("roadxml_template_tail.rnd");

    int head_size = filesize(head_f);
    char* head_buf = new char[head_size];
    head_f.read(head_buf, head_size);

    int tail_size = filesize(tail_f);
    char* tail_buf = new char[tail_size];
    tail_f.read(tail_buf, tail_size);

    ofstream output(filename.c_str());
    output.write(head_buf, head_size);
    output << "        <XYCurve direction=\"0\" x=\"" << (float)controlPoints[0].x << "\" y=\"" << (float)controlPoints[0].y << "\">\n";
    output << "          <ClothoidSpline type=\"spline\">\n";
    for (size_t i = 1; i < controlPoints.size(); i++)
    {
        output << "            <Vectord2 x=\"" << controlPoints[i].x-controlPoints[0].x << "\" y=\"" << controlPoints[i].y-controlPoints[0].y << "\" />\n";
    }
    output << "          </ClothoidSpline>\n";
    output << "        </XYCurve>\n";

    // Form the polynomials needed for the SZCurve
    ClothoidSpline spline(controlPoints);


    double step = 50;
    vec2d pos_prev = spline.lookup(0),
          pos_next = spline.lookup(step);
    output << "        <SZCurve>\n";
    output << "          <Polynomial>\n";
    output << "            <begin direction=\"" << get_slope(terrain, pos_prev, pos_next) << "\" x=\"0\" y=\"" << terrain.getPointBilinear(pos_prev.x, pos_prev.y) << "\" />\n";

    for (int t = step; t < spline.length()-step-1e-6; t += step)
    {
        pos_prev = pos_next;
        pos_next = spline.lookup(t+step);

        stringstream ss_point;
        ss_point <<  "direction=\"" << get_slope(terrain, pos_prev, pos_next) << "\" x=\"" << t << "\" y=\"" << terrain.getPointBilinear(pos_prev.x, pos_prev.y) << "\"";
        output << "            <end " << ss_point.str() << " />\n";
        output << "          </Polynomial>\n";
        output << "          <Polynomial>\n";
        output << "            <begin " << ss_point.str() << " />\n";
    }
//    vec2d last = controlPoints.back(), secondLast = controlPoints[controlPoints.size()-2];
//    length += (last-secondLast).length();
    output <<  "            <end direction=\"0\" x=\"" << spline.length() << "\" y=\"" << terrain.getPointBilinear(pos_next.x, pos_next.y) << "\" />\n";
    output << "          </Polynomial>\n";
    output << "        </SZCurve>\n";
    output << "        <BankingCurve>\n";
    output << "          <Polynomial>\n";
    output << "            <begin direction=\"0\" x=\"0\" y=\"0\" />\n";
    output << "            <end direction=\"0\" x=\"" << spline.length() << "\" y=\"0\" />\n";
    output << "          </Polynomial>\n";
    output << "        </BankingCurve>\n";
    output << "        <Portion endDistance=\"" << spline.length() << "\" endProfile=\"defaultNoMotorvei\" name=\"\" startProfile=\"defaultNoMotorvei\"/>\n";
    output.write(tail_buf, tail_size);

    output.close();

    delete[] head_buf;
    delete[] tail_buf;
}

void setPixelColor(FIBITMAP* bm, int x, int y, int r, int g, int b)
{
    RGBQUAD rgb;
    rgb.rgbRed = r;
    rgb.rgbGreen = g; 
    rgb.rgbBlue = b;
    FreeImage_SetPixelColor(bm, x, y, &rgb);
}

int main(int argc, char** argv)
{
    //Terrain dimensions
    int h = 1024, w = 1024;
//    int h = 300, w = 300;
    float spacing = 10.;
    int grid_density = 4;

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

    vector<vec2d> path = pathFind(terrain, start, end, grid_density);


//    vector<vec2d> path;
//    path.push_back(vec2d(1750,2750));
//    path.push_back(vec2d(1750,4000));
//    path.push_back(vec2d(3000,5000));
//    path.push_back(vec2d(5000,5000));
//    path.push_back(vec2d(5750,3500));
//    path.push_back(vec2d(6500,4750));
    ClothoidSpline clothoidSpline(path);

    writeRoadXML("someroadxml.rnd", path, terrain);

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

//    for (double t = 0; t < clothoidSpline.length(); t += 10)
    
//    double tmin = 1500;
//    double tmax = tmin+1500;
    double tmin = 0, tmax = clothoidSpline.length();
//    tmax = 2300;

    for (double t = tmin; t < tmax; t += 5)
    {
        vec2d p = clothoidSpline.lookup(t);
//        int color = int((t-tmin)) % 256;
        int color = 255;
        int red = 1;
//        int green = clothoidSpline.lookupClothoidPairIndex(t) % 2;
        int green = 0;
//        int blue = clothoidSpline.clothoidPairs[clothoidSpline.lookupClothoidPairIndex(t)].reverse;
        int blue = 0;
//        printf("%d is flipped: %d\n", clothoidSpline.lookupClothoidPairIndex(t), blue);
        setPixelColor(bm, int(p.x/terrain.point_spacing), int(p.y/terrain.point_spacing), color*red, color*green,color*blue);
    }

    printf("Length of curve: %lf\n", clothoidSpline.length());

    
    for (size_t i = 0; i < path.size(); i++)
    {
        RGBQUAD rgb;
        rgb.rgbRed = rgb.rgbBlue = 0;
        rgb.rgbGreen = 255;
//        FreeImage_SetPixelColor(bm, path[i].x/terrain.point_spacing, path[i].y/terrain.point_spacing, &rgb);
        FreeImage_SetPixelColor(bm, path[i].x/terrain.point_spacing, path[i].y/terrain.point_spacing, &rgb);
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
