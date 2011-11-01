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
#include <getopt.h>

#include <sys/time.h>

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

double getTime()
{
    timeval t;
    gettimeofday(&t, 0);
    return (double)t.tv_sec + (double)t.tv_usec / 1000000.;
}

double weight_slope = 100.f*1.,
      weight_curvature = 100.f*1,
      weight_road = 1.f;

//vec2d transrot(const vec2d& p, const vec2d& dp, double rot);
//
////Rotate, then translate
//vec2d rottrans(const vec2d& p, const vec2d& dp, double rot);

struct node_t
{
    vec2i p;
    double cost_g, cost_h;

    node_t(vec2i _p, double _cost_g, double _cost_h) : p(_p), cost_g(_cost_g), cost_h(_cost_h)
    {
    }

    double cost_f() const
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


//Heuristic for computing the cost from a to b.
inline double h(const terrain_t& terrain, const vec2d& a, const vec2d& b)
{
    double dx = a.x-b.x, dy = a.y-b.y;
//          dz = 0;
//        dz = terrain.getPointBilinear(b.x, b.y) - terrain.getPointBilinear(a.x, a.y);
    double len = sqrt(dx*dx+dy*dy);
//    return len*weight_road + dz*weight_slope/len; // *weight_slope;
    return len*weight_road; // *weight_slope;
}

inline double get_slope(const terrain_t& terrain, const vec2d& a, const vec2d& b)
{
    double dx = a.x-b.x, dy = a.y-b.y;
    double dz = terrain.getPointBilinear(b.x, b.y) - terrain.getPointBilinear(a.x, a.y);
//    double length = sqrt(dx*dx+dy*dy);
    double slope = dz/sqrt(dx*dx+dy*dy);

    return slope;
}

inline double get_slope_no_norm(const terrain_t& terrain, const vec2d& a, const vec2d& b)
{
    double dz = terrain.getPointBilinear(b.x, b.y) - terrain.getPointBilinear(a.x, a.y);

    return dz;
}

inline double transfer_slope(const terrain_t& terrain, const vec2d& a, const vec2d& b)
{
    if (weight_slope == 0) return 0;
//    double k0 = 0.5;
    double k0 = 1;

//    double slope = fabs(get_slope(terrain, a, b));
    double slope = fabs(get_slope_no_norm(terrain, a, b));
    if (slope > k0)
    {
//        printf("slope is infinity\n");
//        fflush(stdout);
//        return numeric_limits<double>::infinity();
    }
    

    return weight_slope*slope;
//    return slope;
//    return weight_slope*(slope+slope*slope);
}

inline double transfer_curvature(const terrain_t& terrain, const vec2d& a, const vec2d& b, const vec2d& prev)
{
    if (weight_curvature == 0) return 0;
    vec2d vb = b-a;
    vec2d va = a-prev;
    double lenA = va.length();
    double lenB = vb.length();

    if (lenA == 0 || lenB == 0) return 0;

    va = va * (1./lenA);
    vb = vb * (1./lenB);

    double cos_theta = va.dot(vb);
    cos_theta = fmax(fmin(cos_theta, 1),-1);

    double theta = acos(cos_theta);
    if (theta != theta)
    {
        printf("theta is NAN! %f %f %f %f %f\n", va.dot(vb), va.x, va.y, vb.x, vb.y);
        fflush(stdout);
    }

    double k0 = 0.03;
    double curvature = 2.f*sin(theta/2.f)/sqrt(lenA*lenB);
    if (curvature > k0) 
        return numeric_limits<double>::infinity();

    return weight_curvature*curvature;
}

//Transfer function for cost of making the road itself. This is dependent on the length of the road (how much material is used, basically)
inline double transfer_road(const terrain_t& terrain, const vec2d& a, const vec2d& b)
{
    double dx = a.x-b.x, dy = a.y-b.y;
    double dz = 0;
//    double dz = terrain.getPointBilinear(b.x, b.y) - terrain.getPointBilinear(a.x, a.y);
    return weight_road*sqrt(dx*dx+dy*dy+dz*dz);
}

//Integrate the cost over the path segment 
double cost(const terrain_t& terrain, const vec2d& a, const vec2d& v, const vec2d& b, bool first, bool last)
{
    //do we have enough points to make a curve? If not, return a cost of 0.
    
    double i = 0;
    double cost_slope = 0;
    vec2d dir = b-v;

    const double tn = dir.length();
    dir = dir / tn;
    const double step = 0.5*terrain.point_spacing;
    for (double t = 0; t < tn; t = i*step, i++)
    {
        double t1 = t, t2 = t+step;
        if (t2 > tn)
            t2 = tn;

        vec2d _a = dir*t1 + v,
              _b = dir*t2 + v;
        
        cost_slope += transfer_slope(terrain, _a, _b);
//        cost_slope += transfer_slope(terrain, _a, _b) * (t2-t1);
//        if (transfer_slope(terrain, _a, _b) > 4)
//            printf("slope %lf, (%lf,%lf)-(%lf,%lf): %lf-%lf\n", transfer_slope(terrain, _a, _b), _a.x, _a.y, _b.x, _b.y, terrain.getPointBilinear(_a.x, _a.y), terrain.getPointBilinear(_b.x, _b.y));
    }

    double cost_road = transfer_road(terrain, v, b),
          cost_curvature = transfer_curvature(terrain, v, b, a);
        
    if (cost_road+cost_slope+cost_curvature == numeric_limits<double>::infinity())
        return numeric_limits<double>::infinity();

    return cost_road + cost_slope + cost_curvature;
}

vector<vec2d> pathFind(const terrain_t& terrain, vec2i start, vec2i end, int grid_density, int k)
{
    map<vec2i, vec2i> predecessor;
    unordered_map<vec2i, double> in_open;
    unordered_map<vec2i, bool> closed;
    vector<node_t> open;

    start.x = ((int)(start.x/grid_density))*grid_density;
    start.y = ((int)(start.y/grid_density))*grid_density;
    end.x = ((int)(end.x/grid_density))*grid_density;
    end.y = ((int)(end.y/grid_density))*grid_density;


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
//    PRINT_ALL(neighborhood, i, "%d,%d\n", neighborhood[i].x, neighborhood[i].y);

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


        //If the node is already in the closed list, we have already processed it (this may happen if the node is added more than once due to a better route)
        unordered_map<vec2i, bool>::iterator closed_it = closed.find(current.p);
        if (closed_it != closed.end()) 
        {
            continue;
        }

        //Mark this node as not being in the open list, and move it to the closed list.
        in_open.erase(current.p);
        closed[current.p] = true;
        
//        static int ii = 0;
//        ii++;
//        if (ii % 1000 == 0)
//        {
//            printf("current %d,%d\tnopen %zd\tnclosed %zd\tf %f\th %f\n", current.p.x, current.p.y, open.size(), closed.size(), current.cost_f(), current.cost_h);
//            fflush(stdout);
//        }

//        printf("Closed:\n");
//        for (map<vec2i, bool>::iterator it = closed.begin(); it != closed.end(); it++)
//        {
//            printf("%d %d\n", it->first.x, it->first.y);
//        }
//        fflush(stdout);

        //Reached the end?
        if (current == end)
        {
            //Get the final cost
//            printf("Final cost %lf, %lf\n", current.cost_g, current.cost_h);
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

                //First curve segment?
                const vec2i& v = current.p;
                const vec2i& pa = predecessor[current.p];
                const vec2i& pb = pos;
                
                node_t n(pos, 
                        current.cost_g + cost(terrain, 
                                              terrain.gridToPoint(pa), 
                                              terrain.gridToPoint(v), 
                                              terrain.gridToPoint(pb),
                                              predecessor[current.p] == start,
                                              pos == end),
                        h(terrain, terrain.gridToPoint(pos), terrain.gridToPoint(end)));

                if (n.cost_g == numeric_limits<double>::infinity())
                    continue;

                unordered_map<vec2i, double>::iterator in_open_it = in_open.find(n.p);
                if (in_open_it == in_open.end() || (in_open_it != in_open.end() && in_open_it->second > n.cost_g))
                {
                    open.push_back(n); push_heap(open.begin(), open.end());
                    in_open[n.p] = n.cost_g;
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

//    cout << "Backtracking" << endl;

    //Backtrace
    vector<vec2d> result;

    vec2i current_pos = predecessor[end];
    result.push_back(terrain.gridToPoint(end));

    while (!(current_pos == start))
    {
//        printf("current pos: %d %d\n", current_pos.x, current_pos.y);
        result.push_back(terrain.gridToPoint(current_pos));
        current_pos = predecessor[current_pos];
    }
    result.push_back(terrain.gridToPoint(start));

    reverse(result.begin(), result.end());

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
    output << "        <XYCurve direction=\"0\" x=\"" << (double)controlPoints[0].x << "\" y=\"" << (double)controlPoints[0].y << "\">\n";
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
    clothoid_point_t pos_prev = spline.lookup(0),
          pos_next = spline.lookup(step);
    output << "        <SZCurve>\n";
    output << "          <Polynomial>\n";
    output << "            <begin direction=\"" << get_slope(terrain, pos_prev.pos, pos_next.pos) << "\" x=\"0\" y=\"" << terrain.getPointBilinear(pos_prev.pos.x, pos_prev.pos.y) << "\" />\n";

    for (int t = step; t < spline.length()-step-1e-6; t += step)
    {
        pos_prev = pos_next;
        pos_next = spline.lookup(t+step);

        stringstream ss_point;
        ss_point <<  "direction=\"" << get_slope(terrain, pos_prev.pos, pos_next.pos) << "\" x=\"" << t << "\" y=\"" << terrain.getPointBilinear(pos_prev.pos.x, pos_prev.pos.y) << "\"";
        output << "            <end " << ss_point.str() << " />\n";
        output << "          </Polynomial>\n";
        output << "          <Polynomial>\n";
        output << "            <begin " << ss_point.str() << " />\n";
    }
//    vec2d last = controlPoints.back(), secondLast = controlPoints[controlPoints.size()-2];
//    length += (last-secondLast).length();
    output <<  "            <end direction=\"0\" x=\"" << spline.length() << "\" y=\"" << terrain.getPointBilinear(pos_next.pos.x, pos_next.pos.y) << "\" />\n";
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

void usage()
{
    printf("Usage:\n./pathfind [OPTIONS] <input raw file> <output filename w/o extension>\n");
    printf("Options may be:\n");
    printf("\t-a: Minimum elevation (default: 0)\n");
    printf("\t-b: Maximum elevation (default: 2700)\n");
    printf("\t-s: Resolution of heightmap (default: 10)\n");
    printf("\t-d: Density of grid points (default: 4)\n");
}

int main(int argc, char** argv)
{
    //Parse command line arguments
    string output_name = "default";
    string input_name = "default";
    string output_roadxml, output_ctrlpoints;

    //Terrain heights
    double h_min = 900;
    double h_max = 2550;

    //Terrain dimensions
//    int h = 768, w = h;
    int h = 1024, w = h;
    double spacing = 10.;
    int grid_density = 4;
    // k = neighborhood radius
    int k = 5;
    
    //timers
    double t_pathfind;

    int c;
    while ((c = getopt(argc, argv, "a:b:s:d:x:y:")) != EOF)
    {
        switch (c)
        {
            case 'a':
                h_min = atof(optarg);
            break;
            case 'b':
                h_max = atof(optarg);
            break;
            case 'x':
                w = atoi(optarg);
            break;
            case 'y':
                h = atoi(optarg);
            break;
            case 's':
                spacing = atof(optarg);
                break;
            case 'd':
                grid_density = atoi(optarg);
                break;
            default:
                usage();
                exit(1);
                break;
       }
    }
    srand(time(0));

    //Terrain storage
    unsigned short* terrain_raw = new unsigned short[h*w];
    terrain_t       terrain(h, w, spacing);


    vec2i start,end;
    //Starting and ending points
//    vec2i start(10, 750), end(400, 170);
//    vec2i start(10, 360), end(750, 10);
//    vec2i start(0, 0), end(4000, 4000);
//    vec2i end(70, 570), start(900, 24);

    start = vec2i(0,0);
    end = vec2i(w-1,h-1);

    //Input and output filenames
    input_name = argv[optind], output_name = argv[optind+1];

    //Read terrain
    ifstream input(input_name.c_str(), ios::binary);
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

    t_pathfind = -getTime();
    vector<vec2d> path = pathFind(terrain, start, end, grid_density, k);
    t_pathfind += getTime();

//    vector<vec2d> path;
//    for (int i = 0; i < 5; i++)
//        path.push_back(vec2d(rand() % 10240, rand() % 10240));
//    path.push_back(vec2d(1750,2750));
//    path.push_back(vec2d(1750,4000));
//    path.push_back(vec2d(3000,5000));
//    path.push_back(vec2d(5000,5000));
//    path.push_back(vec2d(5750,3500));
//    path.push_back(vec2d(6500,4750));
    ClothoidSpline clothoidSpline(path, 0.75);

    writeRoadXML(output_name + ".rnd", path, terrain);

    //Write the road file
    {
        ofstream f(output_name + ".rd");
        f << output_name + ".obj\n";
        for (size_t i = 0; i < path.size(); i++)
        {
            f << path[i].x << " " << path[i].y << "\n";
        }
        f.close();
    }

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
    
    double tmin = 0, tmax = clothoidSpline.length();

    for (double t = tmin; t < tmax; t += 5)
    {
        clothoid_point_t p = clothoidSpline.lookup(t);
//        printf("p %lf %lf %lf\n", p.pos.x, p.pos.y, clothoidSpline.length());
        double color = 255;
        double red = 1;
        double green = 1;
//        double green = 1.-min(p.curvature*100.,1.);
//        double green = 1.-min(p.integrated_curvature*500,1.);
        double blue = 1;
        blue = green;
        setPixelColor(bm, int(p.pos.x/terrain.point_spacing), int(p.pos.y/terrain.point_spacing), color*red, color*green,color*blue);
    }

    //Find cost of curve
    double step = 1;
    double cost_road = clothoidSpline.length()*weight_road,
           cost_slope = 0,
           cost_curvature = 0;
    for (double t = tmin; t < tmax; t += step)
    {
        double t0 = max(t-step, tmin), t1 = t, t2 = min(t+step, tmax-1e-5);
        clothoid_point_t p0, p1, p2;
        p0 = clothoidSpline.lookup(t0);
        p1 = clothoidSpline.lookup(t1);
        p2 = clothoidSpline.lookup(t2);
        cost_curvature += weight_curvature*p1.curvature*(t2-t1);

        cost_slope += transfer_slope(terrain, p1.pos, p2.pos);

//        printf("%lf %lf %lf, %lf %lf\n", cost_slope, p1.pos.x, p1.pos.y, p2.pos.x, p2.pos.y);
//        printf("cost %lf\n", cost_total);
//double cost(const terrain_t& terrain, const vec2d& a, const vec2d& v, const vec2d& b, const vec2d T0, bool first, bool last)
    }
    double cost_total = cost_road + cost_slope + cost_curvature;
//    printf("Total cost: %lf, Length of curve: %lf\n", cost_total, clothoidSpline.length());
//    printf("Breakup of costs:\n\tLength:\t%lf\n\tSlope:\t%lf\n\tCurvature:\t%lf\n", cost_road, cost_slope, cost_curvature);
    
    for (size_t i = 0; i < path.size(); i++)
    {
        RGBQUAD rgb;
        rgb.rgbRed = rgb.rgbBlue = 0;
        rgb.rgbGreen = 255;
        FreeImage_SetPixelColor(bm, path[i].x/terrain.point_spacing, path[i].y/terrain.point_spacing, &rgb);
    }

    //Save image
    {
//        printf("Saving image %s...\n", (output_name + ".png").c_str());

        FreeImage_Save(FIF_PNG, bm, (output_name + ".png").c_str());

    }

    printf("%lf %lf\n", t_pathfind, cost_total);

    FreeImage_Unload(bm);

    FreeImage_DeInitialise();
    delete[] terrain_raw;

    return 0;
}
