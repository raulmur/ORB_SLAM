#include "utils.h"
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
