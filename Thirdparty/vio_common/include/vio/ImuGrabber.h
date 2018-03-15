
#ifndef IMU_GRABBER_H
#define IMU_GRABBER_H

#include "vio/DataGrabber.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace vio{

class CSFMDataPattern{
        virtual std::ostream& print(std::ostream &) const;
        virtual std::istream& read(std::istream &);

    public:
        friend std::ostream& operator << (std::ostream &os, const CSFMDataPattern & rhs)
        {
            return rhs.print(os);
        }
        friend std::istream & operator>>(std::istream &is, CSFMDataPattern &rhs)
        {
            return rhs.read(is);
        }
        virtual ~CSFMDataPattern(){}
    public:        
        double timestamp;
        double txyz[3];
        double qxyzw[4];
    };
void loadCSFMOutput(std::string csfmFile, std::vector<std::pair<double, Eigen::Vector3d> > & vTimeAndPose);
void saveCSFMOutput(std::string csfmFile, const std::vector<std::pair<double, Eigen::Vector3d> > & vTimeAndPose);

// double timestamp, accel xyz m/s^2, gyro xyz rad/sec
typedef std::vector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1> > > RawImuMeasurementVector;

enum IMUFileType {MicroStrainCSV=0, PlainText, SensorStreamCSV, IndexedPlainText};
class IMUGrabber:DataGrabber{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    struct MicroStrainCSVPattern{
        int GPS_TFlags;
        int GPS_Week;
        double GPS_TOW;
        double IMU_Timestamp;
        int IMU_Sync_Flags;
        int IMU_Sync_Seconds;
        double	IMU_Sync_Nanoseconds;
        double awxyz[6];
        MicroStrainCSVPattern():GPS_TFlags(-1),GPS_Week(-1), GPS_TOW(-1), IMU_Timestamp(-1),IMU_Sync_Flags(-1),
        IMU_Sync_Seconds(-1),IMU_Sync_Nanoseconds(-1){
            awxyz[0]=0;
            awxyz[1]=0;
            awxyz[2]=0;
        }
    };
    struct PlainTextPattern{
        double GPS_TOW;
        double awxyz[6];
    };

    struct IndexedPlainTextPattern{
        int index;
        double GPS_TOW; // timestamp not necessarily of GPS
        double awxyz[6];
        const double linearAccelerationUnit;
        const double angularRateUnit;
        IndexedPlainTextPattern():linearAccelerationUnit(2*9.80665/32768),
            angularRateUnit(2000*M_PI/ (180*32768)){}
    };

    // sensorstream app in android phones
    struct SensorStreamCSVPattern{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double timestamp;
        static const int accelerometerIndex =3;
        static const int gyroIndex =4;
        static const int magnetIndex =5;
        std::vector< std::pair<int, Eigen::Vector3d> > measurements; // ordered in accelerometer, gyro, and magnetometer order, but some can be missing
    };

    IMUGrabber(const std::string file, IMUFileType ft, double sample_interval=0.01);
    //note the last entry of the last imu measurement segment is kept in the new segment
    bool getObservation(double tk);
    void print(const std::string message="");
    Eigen::Matrix<double,7,1> transMat;     //temporary IMU reading container which also holds p(k)-1 IMU data
    RawImuMeasurementVector measurement;                            //IMU readings from t(p(k)-1) to t(p(k+1)-1)
    IMUFileType file_type;
};
std::ostream& operator << (std::ostream &os, const IMUGrabber::MicroStrainCSVPattern & rhs);
std::istream & operator>>(std::istream &is, IMUGrabber::MicroStrainCSVPattern &rhs);
std::ostream& operator << (std::ostream &os, const IMUGrabber::PlainTextPattern & rhs);
std::istream & operator>>(std::istream &is, IMUGrabber::PlainTextPattern &rhs);
std::ostream& operator << (std::ostream &os, const IMUGrabber::IndexedPlainTextPattern & rhs);
std::istream & operator>>(std::istream &is, IMUGrabber::IndexedPlainTextPattern &rhs);
std::ostream& operator << (std::ostream &os, const IMUGrabber::SensorStreamCSVPattern & rhs);
std::istream & operator>>(std::istream &is, IMUGrabber::SensorStreamCSVPattern &rhs);

class StatesGrabber:DataGrabber{
public:
    StatesGrabber(const std::string file, int numFields, int removeHeaderLines=1,
                  double sample_interval=0.01, const char _delimiter=' '):
        DataGrabber(file, sample_interval),
        mnNumFields(numFields),
        measurement(numFields, -1),
        delimiter(_delimiter)
    {
        std::string tempStr;
        for(int jack=0; jack< removeHeaderLines; ++jack)
            getline(stream, tempStr);//remove explanatary lines
    }
  
 
    //get Observation very close to tk
    bool getObservation(double tk=0.0);
    //get next observation entry in the file
    bool getNextObservation();


    const size_t mnNumFields; 
    std::vector<double> measurement;
    // E.g., states at t(k): t(k), position of sensor in world frame,
    // quaternion from sensor to world frame[xyzw], velocity of sensor in world frame, accelerometer biases, and gyro biases
    const char delimiter;
};


void loadKITTIPoses(std::string file, std::vector<std::pair<double, Eigen::Matrix<double,3,4> > > & vTimeAndPose);
void computeKITTIRf2s();

void testIMUGrabber();
}
#endif // IMU_GRABBER_H
