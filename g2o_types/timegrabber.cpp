
#include "timegrabber.h"
#include "eigen_utils.h"

#include <gpstk-2.5.linux.x86_64/SystemTime.hpp>
#include <gpstk-2.5.linux.x86_64/CommonTime.hpp>
#include <gpstk-2.5.linux.x86_64/CivilTime.hpp>
#include <gpstk-2.5.linux.x86_64/YDSTime.hpp>
#include <gpstk-2.5.linux.x86_64/GPSWeekSecond.hpp>
#include <gpstk-2.5.linux.x86_64/Position.hpp>

using namespace gpstk;
using namespace std;
TimeGrabber::TimeGrabber():
    last_line_index(-1), last_line_time(-1){
}
TimeGrabber::TimeGrabber(const string time_file_name):time_file(time_file_name),
    time_stream(time_file_name.c_str()),
    last_line_index(-1), last_line_time(-1){
    if(!time_stream.is_open())
    {
        std::cout << "Error opening timestamp file!"<<endl;
    }
}

TimeGrabber::~TimeGrabber(){
    time_stream.close();
}
bool TimeGrabber::init(const string time_file_name){
    time_file=time_file_name;
    time_stream.open(time_file_name.c_str());
    last_line_index=-1;
    last_line_time=-1;
    if(!time_stream.is_open())
    {
        std::cout << "Error opening timestamp file!"<<endl;
        return false;
    }
    return true;
}
// this reading function only works for KITTI timestamp files
double TimeGrabber::readTimestamp(int line_number)
{
    string tempStr;
    double precursor(-1);
    if(last_line_index>line_number){
        cerr<<"Retracing to read timestamps is unsupported!"<<endl;
        return -1;
    }
    if(last_line_index==line_number)
        return last_line_time;
    while(last_line_index<line_number){
        time_stream>>precursor;
        if(time_stream.fail())
            break;
        getline(time_stream, tempStr);       //remove the remaining part, this works even when it is empty
        ++last_line_index;
    }
    if(last_line_index<line_number)
    {
        cerr<<"Failed to find this line in time file!"<<endl;
        return -1;
    }
    last_line_time=precursor;
    return last_line_time;
}
//extract time and return left image filename, tailored for Malaga urban dataset
// in this function frame_number is 0 for the first two lines in the /*_IMAGE.txt file
double TimeGrabber::extractTimestamp(int frame_number)
{
    string tempStr;
    double precursor(-1);
    if(last_line_index>frame_number){
        cerr<<"Retracing to read timestamps is unsupported!"<<endl;
        return -1;
    }
    if(last_line_index==frame_number)
    {
        return last_line_time;
    }
    while(last_line_index<frame_number){
        getline(time_stream, tempStr);
        if(time_stream.fail())
            break;
        last_left_image_name=tempStr;
        getline(time_stream, tempStr);       //also read in the right image name
        precursor=atof(tempStr.substr(12, 17).c_str());
        ++last_line_index;
    }
    if(last_line_index<frame_number)
    {
        cerr<<"Failed to find this line in time file!"<<endl;
        return -1;
    }
    last_line_time=precursor;
    return last_line_time;
}
DataGrabber::DataGrabber(const std::string file, double interval):reinit_data(true),is_measurement_good(false),
    file(file), stream(file.c_str()), tkm1(-1), interval(interval){
    if(!stream.is_open())
        std::cerr<<"Cannot open "<<file<<std::endl;
}

std::istream& CSFMDataPattern::read(std::istream & is)
{
    is>>timestamp>>txyz[0]>>
            txyz[1]>>txyz[2]>>
            qxyzw[0]>>qxyzw[1]>>qxyzw[2]>>qxyzw[3];
    return is;
}
std::ostream& CSFMDataPattern::print(std::ostream & os) const
{
    char delim=' ';
    os<<std::setprecision(12)<<timestamp<< delim<<txyz[0]<< delim<< txyz[1]<<delim<<
        txyz[2]<<delim<<std::setprecision(6)<<qxyzw[0]<<delim<<qxyzw[1]<<delim<<qxyzw[2]<<delim<<qxyzw[3]<<endl;
    return os;
}

void loadCSFMOutput(string csfmFile, vector<pair<double, Eigen::Vector3d> > & vTimeAndPose)
{
    ifstream stream(csfmFile);
    string tempStr;
    CSFMDataPattern pat;
    vTimeAndPose.clear();
    while(!stream.eof()){
        getline(stream, tempStr);
        if(stream.fail()){
            break;
        }
        stringstream line_str(tempStr);
        line_str>>pat;
        Eigen::Vector3d txyz; txyz<< pat.txyz[0], pat.txyz[1], pat.txyz[2];
        std::pair<double, Eigen::Vector3d> meas= make_pair(pat.timestamp, txyz);
        vTimeAndPose.push_back(meas);
    }
    stream.close();
}

void saveCSFMOutput(string csfmFile, const vector<pair<double, Eigen::Vector3d> > & vTimeAndPose)
{
    ofstream stream(csfmFile);
    CSFMDataPattern pat;

    for(auto it= vTimeAndPose.begin(), ite= vTimeAndPose.end(); it!=ite; ++it){
        pat.timestamp= it->first;
        pat.txyz[0] = it->second[0];
        pat.txyz[1] = it->second[1];
        pat.txyz[2] = it->second[2];
        pat.qxyzw[0] = 0;
        pat.qxyzw[1] = 0;
        pat.qxyzw[2] = 0;
        pat.qxyzw[3] = 0;
        stream<< pat;
    }
    stream.close();
}


std::ostream& operator << (std::ostream &os, const IMUGrabber::MicroStrainCSVPattern & rhs)
{
    os<< rhs.GPS_TFlags<<','<<rhs.GPS_Week<<','<<rhs.GPS_TOW<<','<<
         rhs.awxyz[0]<<','<<rhs.awxyz[1]<<','<<rhs.awxyz[2]<<','<<
         rhs.awxyz[3]<<','<<rhs.awxyz[4]<<','<<rhs.awxyz[5];
    return os;
}
std::istream & operator>>(std::istream &is, IMUGrabber::MicroStrainCSVPattern &rhs)
{
    char delim;
    is>>rhs.GPS_TFlags>>delim>>rhs.GPS_Week>>delim>>rhs.GPS_TOW>>delim>>
            rhs.IMU_Timestamp>>delim>>rhs.IMU_Sync_Flags>>delim>>rhs.IMU_Sync_Seconds>>delim>>
            rhs.IMU_Sync_Nanoseconds>>delim>>
            rhs.awxyz[0]>>delim>>rhs.awxyz[1]>>delim>>rhs.awxyz[2]>>delim>>
            rhs.awxyz[3]>>delim>>rhs.awxyz[4]>>delim>>rhs.awxyz[5];
    rhs.awxyz[0]*=9.80665;
    rhs.awxyz[1]*=9.80665;
    rhs.awxyz[2]*=9.80665;
    return is;
}
std::ostream& operator << (std::ostream &os, const IMUGrabber::PlainTextPattern & rhs)
{
    os<< rhs.GPS_TOW<<' '<<
         rhs.awxyz[0]<<' '<<rhs.awxyz[1]<<' '<<rhs.awxyz[2]<<' '<<
         rhs.awxyz[3]<<' '<<rhs.awxyz[4]<<' '<<rhs.awxyz[5];
    return os;
}
std::istream & operator>>(std::istream &is, IMUGrabber::PlainTextPattern &rhs)
{
    is>>rhs.GPS_TOW>>rhs.awxyz[0]>>rhs.awxyz[1]>>rhs.awxyz[2]>>
            rhs.awxyz[3]>>rhs.awxyz[4]>>rhs.awxyz[5];
    return is;
}


IMUGrabber::IMUGrabber(const std::string file, IMUFileType ft, double sample_interval):
    DataGrabber(file, sample_interval), file_type(ft)
{
    int header_lines=1;
    if(file_type==PlainText)
        header_lines=1;
    std::string tempStr;
    for(int i=0; i<header_lines;++i)
        getline(stream, tempStr);           //remove explanatory header
}
//get next bundle of imu observations given t(k), for the first frame startIndex, no measurement should be returned
// assume the starting time is larger than the first epoch of imu data
bool IMUGrabber::getObservation(double tk)
{
    int lineNum=0;      //how many lines have been read in
    string tempStr;

    if(reinit_data){//grab IMU data until its timestamp is greater than t(startIndex)
        while(!stream.eof()){
            getline(stream, tempStr);
            if(stream.fail())
                break;
            stringstream line_str(tempStr);
            if(file_type==MicroStrainCSV){
                IMUGrabber::MicroStrainCSVPattern pat;
                line_str>>pat;
                if(pat.IMU_Sync_Nanoseconds==-1){//caution: lines are not of IMU readings
                    assert(pat.awxyz[2]==0);
                    continue;
                }
                transMat[0]=pat.GPS_TOW;
                for (int j=1; j<transMat.rows(); ++j)
                    transMat[j]=pat.awxyz[j-1];
            }
            else if(file_type==PlainText)
            {
                IMUGrabber::PlainTextPattern pat;
                line_str>>pat;
                transMat[0]=pat.GPS_TOW;
                for (int j=1; j<transMat.rows(); ++j)
                    transMat[j]=pat.awxyz[j-1];
            }

            ++lineNum;
            if(transMat[0]>tk)
                break;
            measurement.push_back(transMat);
        }
        if(transMat[0]>tk&&measurement.size()){
            reinit_data=false;
        }
    }
    else{
        measurement.front()=measurement.back(); //t(p(k-1)-1)
        measurement.resize(1);
        measurement.push_back(transMat);        //last stranded measurement, t(p(k-1))
        assert(measurement[0][0]<=tkm1);

        while(!stream.eof()){
            getline(stream, tempStr);
            if(stream.fail())
                break;
            stringstream line_str(tempStr);
            if(file_type==MicroStrainCSV){
                IMUGrabber::MicroStrainCSVPattern pat;
                line_str>>pat;
                if(pat.IMU_Sync_Nanoseconds==-1){//caution: lines are not of IMU readings
                    assert(pat.awxyz[2]==0);
                    continue;
                }
                transMat[0]=pat.GPS_TOW;
                for (int j=1; j<transMat.rows(); ++j)
                    transMat[j]=pat.awxyz[j-1];
            }
            else if(file_type==PlainText)
            {
                IMUGrabber::PlainTextPattern pat;
                line_str>>pat;
                transMat[0]=pat.GPS_TOW;
                for (int j=1; j<transMat.rows(); ++j)
                    transMat[j]=pat.awxyz[j-1];
            }
            ++lineNum;
            if(transMat[0]>tk)
                break;
            measurement.push_back(transMat);
        }
        if(transMat[0]>tk||(tk-measurement.back()[0]<interval))
            //the second condition applies at the end of imu file.
            // Otherwise, something wrong with reading IMU data
        {
            is_measurement_good=true;
            //    Eigen::Vector3d tempVs0inw;
            //    ScaViSLAM::predictStates(T_s1_to_w, speed_bias_1, time_pair,
            //                             measurement, imu.gwomegaw, imu.q_n_aw_babw,
            //                           &pred_T_s2_to_w, &tempVs0inw, NULL);
        }
        else{
            is_measurement_good=false;
            measurement.clear();
            reinit_data=true;
            assert(false);//this should never happen with continuous IMU readings
        }
    }
    tkm1=tk;
    return is_measurement_good;
}
bool StatesGrabber::getObservation(double tk)
{
    //Assume each line in States file: GPS TOW, i.e., t(k), position of sensor in world frame,
    // quaternion wxyz from sensor to world frame, velocity of sensor in world frame, accelerometer biases, and gyro biases
    // diagnoal elements of the shape matrices S_a, and S_g, where S_a + I= T_a, a_m = T_a*a +b_a +n_a, w_m = T_g*w +b_g +n_g
    double precursor=0; //used to read in something tentatively
    string tempStr;
    if(!stream.eof()){
        stream>>precursor;
        if(stream.fail())
            return false;
        measurement[0]=precursor;
        for (int j=1; j<measurement.rows(); ++j)
            stream>>measurement[j];
        getline(stream, tempStr);       //remove the remaining part, this works even when it is empty
        is_measurement_good=true;
    }
    tkm1=tk;
    return is_measurement_good;
}
