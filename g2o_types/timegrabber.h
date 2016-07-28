
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
        ~CSFMDataPattern(){}
    public:        
        double timestamp;
        double txyz[3];
        double qxyzw[4];
    };
void loadCSFMOutput(std::string csfmFile, std::vector<std::pair<double, Eigen::Vector3d> > & vTimeAndPose);
void saveCSFMOutput(std::string csfmFile, const std::vector<std::pair<double, Eigen::Vector3d> > & vTimeAndPose);

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

    IMUGrabber(const std::string file, IMUFileType ft, double sample_interval=0.01);

    bool getObservation(double tk);

    Eigen::Matrix<double,7,1> transMat;     //temporary IMU reading container which also holds p(k)-1 IMU data
    std::vector<Eigen::Matrix<double, 7,1> > measurement;                            //IMU readings from t(p(k)-1) to t(p(k+1)-1)
    IMUFileType file_type;
};
std::ostream& operator << (std::ostream &os, const IMUGrabber::MicroStrainCSVPattern & rhs);
std::istream & operator>>(std::istream &is, IMUGrabber::MicroStrainCSVPattern &rhs);
std::ostream& operator << (std::ostream &os, const IMUGrabber::PlainTextPattern & rhs);
std::istream & operator>>(std::istream &is, IMUGrabber::PlainTextPattern &rhs);


class StatesGrabber:DataGrabber{
public:
    StatesGrabber(const std::string file, double sample_interval=0.01):DataGrabber(file, sample_interval)
    {
        std::string tempStr;
        getline(stream, tempStr);//remove first explanatary line
    }
    bool getObservation(double tk);
    Eigen::Matrix<double, 17+6,1> measurement;  //states at t(k): t(k), position of sensor in world frame,
    // quaternion from sensor to world frame, velocity of sensor in world frame, accelerometer biases, and gyro biases
};

#endif // TIMEGRABBER_H
