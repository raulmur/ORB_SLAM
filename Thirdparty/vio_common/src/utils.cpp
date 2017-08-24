#include "vio/utils.h"
#include <cmath>

#include <algorithm> //transform
#include <sstream>


namespace vio{
bool to_bool(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    std::istringstream is(str);
    bool b;
    is >> std::boolalpha >> b;
    return b;
}

/// Round up to next higher power of 2 (return x if it's already a power
/// of 2).
int pow2roundup (int x)
{
    if (x < 0)
        return 0;
    --x;
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;
    return x+1;
}
// maxEdge maximum edge length in pixel units
int GetDownScale(int w, int h, int maxEdge)
{
    float ds= float(std::max(w,h))/ maxEdge;
    return pow2roundup((int)std::ceil(ds));
}
}
