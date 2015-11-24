#ifndef UTILS_H
#define UTILS_H
#include "viso2/p_match.h"
#include <vector>
std::vector<p_match> cropMatches(const std::vector<p_match> &pMatches, float xl, float xr);
/// Round up to next higher power of 2 (return x if it's already a power
/// of 2).
int pow2roundup (int x);
int GetDownScale(int w, int h, int maxEdge= 2000);
#endif
