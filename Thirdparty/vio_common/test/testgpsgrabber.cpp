#include "vio/GPSGrabber.h"
#include <iomanip>

int main(){
  const std::string gps_file = "../test/rtkliboutput.pos";
  std::vector<Eigen::Matrix<double, 7, 1> > gpsdata;
  double startTime = 0, finishTime = 1e10;

  vio::loadGPSData(gps_file, gpsdata, startTime, finishTime);
  std::cout << "The first two GPS entry should be "<< std::endl<<
  "1870 315421.600    595013.4873  -4856399.4368   4078395.2865   5   4   3.5249   6.6332   5.0773   4.3969  -5.6267  -3.7690 -1117.20    0.0"<< std::endl<<
  "1870 315421.800    595013.4817  -4856399.2399   4078395.1854   5   4   3.5253   6.6343   5.0784   4.3977  -5.6278  -3.7697 -1117.00    0.0"<< std::endl;
  std::cout << "The GPSGrabber first entry is "<< std::endl<<std::setprecision(11)<< gpsdata[0].transpose() << std::endl;

  std::cout << "Last GPS entry should be "<< std::endl<< 
  "1870 318062.600    592575.4207  -4856607.9652   4078416.8873   1   8   0.0033   0.0053   0.0061  -0.0013  -0.0039  -0.0028   0.00  999.9"
  << std::endl;
  std::cout << "The GPSGrabber last entry is "<< std::endl<<std::setprecision(11) << gpsdata.back().transpose() << std::endl;
  
  vio::testReadTOWECEF();
  vio::gpstk_example1();


}

