#ifndef RAND_SAMPLER_H_
#define RAND_SAMPLER_H_

namespace ScaViSLAM{
// method to generate samples copied from g2o example ba_demo.cpp
class Sample {
public:
    static int uniform(int from, int to);
    static double uniform();
    static double gaussian(double sigma);
};

double uniform_rand(double lowerBndr, double upperBndr);

double gauss_rand(double mean, double sigma);
}
#endif
