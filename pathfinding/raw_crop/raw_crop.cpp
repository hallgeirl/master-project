#include <fstream>

using namespace std;

int main(int argc, char** argv)
{
    int h = 1024, w = 1024;
    int h_out = 300, w_out = 300;
    const char* in = argv[1], * out = argv[2];

    unsigned short* terrain_in = new unsigned short[h*w];
    unsigned short* terrain_out = new unsigned short[h_out*w_out];

    printf("Cropping %s (%dx%d) to %s (%dx%d)...\n", in, w, h, out, w_out, h_out);
    fflush(stdout);

    //Read terrain
    ifstream input(in, ios::binary);
    input.read((char*)terrain_in, w*h*2);
    input.close();

    int i_out = 0, j_out = 0;

    for (int i = h-h_out; i < h; i++)
    {
        for (int j = w-w_out; j < w; j++)
        {
            int base = i*w+j;
            int base_out = i_out*w_out+j_out;
            terrain_out[base_out] = terrain_in[base];
            j_out++;
        }
        i_out++;
        j_out = 0;
    }

    ofstream output(out, ios::binary);
    output.write((char*)terrain_out, w_out*h_out*2);
    output.close();

    return 0;
}
