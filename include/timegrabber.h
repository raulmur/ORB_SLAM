
#ifndef TIMEGRABBER_H
#define TIMEGRABBER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>

class TimeGrabber
{
public:
    TimeGrabber();
    TimeGrabber(const std::string time_file_name);
    TimeGrabber & operator=(const TimeGrabber &)=delete;
    TimeGrabber(const TimeGrabber&)=delete;
    ~TimeGrabber();
    bool init(const std::string time_file_name);

    // this reading function only works for KITTI timestamp files
    double readTimestamp(int line_number);

    double extractTimestamp(int frame_number);

    std::string time_file;
    std::ifstream time_stream;
    int last_line_index; //must be initialized as -1
    double last_line_time;
    std::string last_left_image_name;
};
class DataGrabber{
public:
    DataGrabber(const std::string file, double interval);
    DataGrabber(const DataGrabber &)=delete;
    DataGrabber & operator=(const DataGrabber &)=delete;
    virtual ~DataGrabber(){
        if(stream.is_open())
        {
            stream.close();
        }
    }
    //get next bundle of observations from t(p(k-1)-1) to t(p(k)-1) given t(k), where t(p(k)-1)<=t(k), t(p(k))>t(k)
    virtual bool getObservation(double tk){
        tkm1=tk;
        return false;
    }
protected:
    bool reinit_data;              //do we need to grab sensor data until the current frame timestamp t(k) from an unknown timestamp
    bool is_measurement_good;         //does the measurement fit our requirements?
    std::string file;     //sensor data file name
    std::ifstream stream; //sensor data stream
    double tkm1; // t(k-1), timestamp of last frame
    const double interval; //maximum allowed difference between retrieved and expected timestamp
};
class GPSGrabber:DataGrabber{
public:
    struct RtklibPosPattern{
        int GPS_Week;
        double GPS_TOW;
        double xyz_ecef[3];
        int Q;
        int ns;
        double sdxyz[3];
        double sdxy_yz_zx[3];//square root of the absolute values of xy, yz, zx components of the covariance matrix,
        // their signs represents the sign of these components
        double age;
        double ratio;
    };

    GPSGrabber(const std::string file, double interval=1e-5):DataGrabber(file, interval){
        transMat[0]=-1;
        measurement[0]=-1;
        std::string tempStr;
        getline(stream,tempStr); //remove header part
        //N.B. we also delete the first line of observation
        while(tempStr.find('%')!=std::string::npos)
        {
            getline(stream,tempStr);
        }
    }
    bool getObservation(double tk);

    Eigen::Matrix<double, 7,1> transMat; //temporary sensor reading which also holds data at t(p(k)-1);
    Eigen::Matrix<double, 7,1> measurement; //GPS TOW, XYZ ECEF, Q, number of satellites, std XYZ in ECEF
};
enum IMUFileType {MicroStrainCSV=0, PlainText};
class IMUGrabber:DataGrabber{
public:

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

    IMUGrabber(const std::string file, IMUFileType ft, double sample_interval=0.01):DataGrabber(file, sample_interval), file_type(ft)
    {
        int header_lines=16;
        if(file_type==PlainText)
            header_lines=1;
        std::string tempStr;
        for(int i=0; i<header_lines;++i)
            getline(stream, tempStr);           //remove explanatory header
    }

    bool getObservation(double tk);

    Eigen::Matrix<double,7,1> transMat;     //temporary IMU reading container which also holds p(k)-1 IMU data
    std::vector<Eigen::Matrix<double, 7,1> > measurement;                            //IMU readings from t(p(k)-1) to t(p(k+1)-1)
    IMUFileType file_type;
};
std::ostream& operator << (std::ostream &os, const IMUGrabber::MicroStrainCSVPattern & rhs);
std::istream & operator>>(std::istream &is, IMUGrabber::MicroStrainCSVPattern &rhs);
std::ostream& operator << (std::ostream &os, const IMUGrabber::PlainTextPattern & rhs);
std::istream & operator>>(std::istream &is, IMUGrabber::PlainTextPattern &rhs);
std::ostream& operator << (std::ostream &os, const GPSGrabber::RtklibPosPattern & rhs);
std::istream & operator>>(std::istream &is, GPSGrabber::RtklibPosPattern &rhs);

class StatesGrabber:DataGrabber{
public:
    StatesGrabber(const std::string file, double sample_interval=0.01):DataGrabber(file, sample_interval)
    {
        std::string tempStr;
        getline(stream, tempStr);//remove first explanatary line
    }
    bool getObservation(double tk);
    Eigen::Matrix<double, 17,1> measurement;  //states at t(k): t(k), position of sensor in world frame,
    // quaternion from sensor to world frame, velocity of sensor in world frame, accelerometer biases, and gyro biases
};

#endif // TIMEGRABBER_H
