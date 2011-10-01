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

int main(int argc, char** argv)
{
    //Input and output filenames
    const char* in = argv[1], * out = 0;
    if (argc > 2) out = argv[2];

    FreeImage_Initialise();

    //Read image
    FIBITMAP* bm = FreeImage_Load(FIF_PNG, in);
    
    //Save to RGB raw
    ofstream output(out, ios::binary);
    for (size_t i = 0; i < FreeImage_GetHeight(bm); i++)
    {
        for (size_t j = 0; j < FreeImage_GetWidth(bm); j++)
        {
            RGBQUAD rgb;
            FreeImage_GetPixelColor(bm, j, i, &rgb);
            output.write((const char*)&rgb.rgbRed, 1);
            output.write((const char*)&rgb.rgbGreen, 1);
            output.write((const char*)&rgb.rgbBlue, 1);
        }
    }
    output.close();

    FreeImage_Unload(bm);

    FreeImage_DeInitialise();

    return 0;
}
