#ifndef UTILS_H
#define UTILS_H
#include "viso2/p_match.h"
#include <vector>
std::vector<p_match> cropMatches(const std::vector<p_match> &pMatches, float xl, float xr);
#endif
