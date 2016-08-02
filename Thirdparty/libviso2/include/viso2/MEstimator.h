// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

// MEstimator.h
//
// Defines various MEstimators which can be used by the Tracker and
// the Bundle adjuster. Not that some of the inputs are square
// quantities!

#ifndef __MESTIMATOR_H
#define __MESTIMATOR_H
#include <TooN/TooN.h>
#include <vector>
#include <algorithm>
#include <cassert>

using namespace TooN;

struct Tukey
{
  inline static double FindSigmaSquared(std::vector<double> &vdErrorSquared);
  inline static double SquareRootWeight(double dErrorSquared, double dSigmaSquared);
  inline static double Weight(double dErrorSquared, double dSigmaSquared);
  inline static double ObjectiveScore(double dErrorSquared, double dSigmaSquared);
};

struct Cauchy
{
  inline static double FindSigmaSquared(std::vector<double> &vdErrorSquared);
  inline static double SquareRootWeight(double dErrorSquared, double dSigmaSquared);
  inline static double Weight(double dErrorSquared, double dSigmaSquared);
  inline static double ObjectiveScore(double dErrorSquared, double dSigmaSquared);
};

struct Huber
{
  inline static double FindSigmaSquared(std::vector<double> &vdErrorSquared);
  inline static double SquareRootWeight(double dErrorSquared, double dSigmaSquared);
  inline static double Weight(double dErrorSquared, double dSigmaSquared);
  inline static double ObjectiveScore(double dErrorSquared, double dSigmaSquared);
};

struct LeastSquares
{
  inline static double FindSigmaSquared(std::vector<double> &vdErrorSquared);
  inline static double SquareRootWeight(double dErrorSquared, double dSigmaSquared);
  inline static double Weight(double dErrorSquared, double dSigmaSquared);
  inline static double ObjectiveScore(double dErrorSquared, double dSigmaSquared);
};


inline double Tukey::Weight(double dErrorSquared, double dSigmaSquared)
{
  double dSqrt = SquareRootWeight(dErrorSquared, dSigmaSquared);
  return dSqrt * dSqrt;
}

inline double Tukey::SquareRootWeight(double dErrorSquared, double dSigmaSquared)
{
  if(dSigmaSquared<1e-8)
  {
      if(dErrorSquared<1e-8)
          return 1.0;
      else
          return 0;
  }
  if(dErrorSquared > dSigmaSquared)
    return 0.0;
  else
    return 1.0 - (dErrorSquared / dSigmaSquared);
}

inline double Tukey::ObjectiveScore(double dErrorSquared, const double dSigmaSquared)
{
  // NB All returned are scaled because
  // I'm not multiplying by sigmasquared/6.0
  if(dErrorSquared > dSigmaSquared)
    return 1.0;
  double d = 1.0 - dErrorSquared / dSigmaSquared;
  return (1.0 - d*d*d);
}


inline double Tukey::FindSigmaSquared(std::vector<double> &vdErrorSquared)
{ 
  double dSigmaSquared; 
  assert(vdErrorSquared.size() > 0);
  std::sort(vdErrorSquared.begin(), vdErrorSquared.end());
  double dMedianSquared = vdErrorSquared[vdErrorSquared.size() / 2];
  double dSigma = 1.4826 * (1 + 5.0 / (vdErrorSquared.size() * 2 - 6)) * sqrt(dMedianSquared);
  dSigma =  4.6851 * dSigma;
  dSigmaSquared = dSigma * dSigma;
  return dSigmaSquared;
}


///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////

inline double Cauchy::Weight(double dErrorSquared, double dSigmaSquared)
{
  return 1.0 / (1.0 + dErrorSquared / dSigmaSquared);
}

inline double Cauchy::SquareRootWeight(double dErrorSquared, double dSigmaSquared)
{
  return sqrt(Weight(dErrorSquared, dSigmaSquared));
}

inline double Cauchy::ObjectiveScore(double dErrorSquared, const double dSigmaSquared)
{
  return log(1.0 + dErrorSquared / dSigmaSquared);
}


inline double Cauchy::FindSigmaSquared(std::vector<double> &vdErrorSquared)
{ 
  double dSigmaSquared; 
  assert(vdErrorSquared.size() > 0);
  std::sort(vdErrorSquared.begin(), vdErrorSquared.end());
  double dMedianSquared = vdErrorSquared[vdErrorSquared.size() / 2];
  double dSigma = 1.4826 * (1 + 5.0 / (vdErrorSquared.size() * 2 - 6)) * sqrt(dMedianSquared);
  dSigma =  4.6851 * dSigma;
  dSigmaSquared = dSigma * dSigma;
  return dSigmaSquared;
}


///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////

inline double Huber::Weight(double dErrorSquared, double dSigmaSquared)
{
  if(dErrorSquared < dSigmaSquared)
    return 1;
  else
    return sqrt(dSigmaSquared / dErrorSquared);
}

inline double Huber::SquareRootWeight(double dErrorSquared, double dSigmaSquared)
{
  return sqrt(Weight(dErrorSquared, dSigmaSquared));
}

inline double Huber::ObjectiveScore(double dErrorSquared, const double dSigmaSquared)
{
  if(dErrorSquared< dSigmaSquared)
    return 0.5 * dErrorSquared;
  else
    {
      double dSigma = sqrt(dSigmaSquared);
      double dError = sqrt(dErrorSquared);
      return dSigma * ( dError - 0.5 * dSigma);
    }
}


inline double Huber::FindSigmaSquared(std::vector<double> &vdErrorSquared)
{ 
  double dSigmaSquared; 
  assert(vdErrorSquared.size() > 0);
  std::sort(vdErrorSquared.begin(), vdErrorSquared.end());
  double dMedianSquared = vdErrorSquared[vdErrorSquared.size() / 2];
  double dSigma = 1.4826 * (1 + 5.0 / (vdErrorSquared.size() * 2 - 6)) * sqrt(dMedianSquared);
  dSigma =  1.345 * dSigma;
  dSigmaSquared = dSigma * dSigma;
  return dSigmaSquared;
}

///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////

inline double LeastSquares::Weight(double dErrorSquared, double dSigmaSquared)
{
  return 1.0;
}

inline double LeastSquares::SquareRootWeight(double dErrorSquared, double dSigmaSquared)
{
  return 1.0;
}

inline double LeastSquares::ObjectiveScore(double dErrorSquared, const double dSigmaSquared)
{
  return dErrorSquared;
}


inline double LeastSquares::FindSigmaSquared(std::vector<double> &vdErrorSquared)
{ 
  if(vdErrorSquared.size() == 0)
    return 0.0;
  double dSum = 0.0;
  for(unsigned int i=0; i<vdErrorSquared.size(); i++)
    dSum+=vdErrorSquared[i];
  return dSum / vdErrorSquared.size();
}

#endif


