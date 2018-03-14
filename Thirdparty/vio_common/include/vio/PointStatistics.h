
#ifndef POINT_STATISTICS_H
#define POINT_STATISTICS_H
#include <Eigen/Dense>
#include <vector>
struct PointStatistics
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PointStatistics(int USE_N_LEVELS_FOR_MATCHING=0)
    : num_matched_points(USE_N_LEVELS_FOR_MATCHING)
  {
    num_points_grid2x2.setZero();
    num_points_grid3x3.setZero();

    for (int l=0; l<USE_N_LEVELS_FOR_MATCHING; ++l)
    {
      num_matched_points[l]=0;
    }
  }
  PointStatistics(const PointStatistics & other)
  {
      num_matched_points=other.num_matched_points;
      num_points_grid2x2=other.num_points_grid2x2;
      num_points_grid3x3=other.num_points_grid3x3;
  }
  PointStatistics & operator=(const PointStatistics & other)
  {
      if(this==&other)
          return *this;
      num_matched_points=other.num_matched_points;
      num_points_grid2x2=other.num_points_grid2x2;
      num_points_grid3x3=other.num_points_grid3x3;
      return *this;
  }
  ~PointStatistics()
  {
     num_matched_points.clear();
  }
  bool isWellDistributed()
  {
      double deviation=0;
      for(int i=0; i<2; ++i)
          for(int j=0; j<2; ++j)
              deviation+=abs(num_points_grid3x3(i,j)/num_matched_points[0]-1/9.0);
      return deviation<3/9.0;
  }
 
  int numFeatureLessCorners3x3(int n_featureless_cell_thr=10){
      int num_featuerless_corners=0;
      for (int i=0; i<3; ++i)
          for (int j=0; j<3; ++j)
              if (num_points_grid3x3(i,j)<n_featureless_cell_thr)
                  ++num_featuerless_corners;
      return num_featuerless_corners;
  }
  int numFeatureLessCorners2x2(int n_featureless_cell_thr=15){
      int num_featuerless_corners=0;
      for (int i=0; i<2; ++i)
          for (int j=0; j<2; ++j)
              if (num_points_grid2x2(i,j)<n_featureless_cell_thr)
                  ++num_featuerless_corners;
      return num_featuerless_corners;
  }
  std::vector<int> num_matched_points; //number of matched points at each pyramid level
  Eigen::Matrix2i num_points_grid2x2;
  Eigen::Matrix3i num_points_grid3x3;//number of points in each cell of the 3x3 grid at level 0
};
#endif
