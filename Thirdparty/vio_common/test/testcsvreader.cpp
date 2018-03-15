#include "vio/CsvReader.h"

int main(){
  const std::string okvisFile = "../test/OkvisOutput.csv";
  
  std::vector<Eigen::Matrix<double, 19, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 19, 1> > > csvData;
  double startTime = 0, finishTime = 1e10;
  vio::loadCsvData(okvisFile, csvData,  startTime, finishTime);
  std::cout << "first entry: "<< csvData[0].transpose()<< std::endl;
  std::cout << "Last entry: "<< csvData.back().transpose()<< std::endl;

}

