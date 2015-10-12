
#ifndef POSTPROC_H
#define POSTPROC_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <map>
#include <utility> //pair
#include "sophus/se3.hpp"


#ifndef _MSC_VER
  #include <stdint.h>
#else
  typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#endif

#define endll endl << endl // double end line definition

class PostProcessor {

public:
    std::string sInputPath;
    std::string sOutputPath;
    std::vector<std::pair <double, Sophus::SE3d> > mRawPoses; //time and pose
    std::vector<std::pair<double, Sophus::SE3d> > mKeyPoses;// time and pose
    std::vector<std::pair <double, Sophus::SE3d> > mReprojedPoses; //time and pose
    std::vector< Eigen::Matrix<double, 9,1> > mSpeedBias;
    //input dir must not be trailed with slash
PostProcessor(std::string input_dir, std::string output_dir);
~PostProcessor();
void Reset();
void Run(const std::vector<int>seqNum, bool bApplyReloc=true);
void Run(std::string sInputFilename, std::string sOutputFilename, bool bApplyReloc=true);
void LoadData(const std::string sInputFile);
void LoadData2(const std::string sInputFile);
void LoadData3(const std::string sInputFile);
void ProcessPoses();
void SaveData(const std::string sOutputFile);
};
void InspectTsukubaEulerAngles(const std::string dir);
void ConvertTsukubaGT2KITTIFormat(const std::string poseFile, const std::string sOutputFile);

#endif // Postproc_H
