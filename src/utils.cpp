#include "utils.h"
#include <cmath>
//remove matches that have u1c falls outside of [xl, xr)
std::vector<p_match> cropMatches(const std::vector<p_match> &pMatches, float xl, float xr)
{
    std::vector<p_match> resMatches;
    resMatches.reserve(pMatches.size());
    for(auto it= pMatches.begin(), ite= pMatches.end(); it!=ite; ++it)
    {
        if(it->u1c<xl || it->u1c>=xr)
            continue;
        else
            resMatches.push_back(*it);
    }
    return resMatches;
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
