
#include "timegrabber.h"
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
        time_stream.open(time_file_name.c_str()),
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

bool GPSGrabber::getObservation(double tk)
{
    //Assume each line in gps_file: GPSWeek, TOW, ecef X,Y,Z, Q and no of satels, and std ecef X,Y,Z

    int lineNum=0;      //how many lines have been read in
    string tempStr;
    RtklibPosPattern pat;
    if(reinit_data){//grab data until its timestamp is greater than t(startIndex)
        while(!stream.eof()){
            getline(stream, tempStr);
            if(stream.fail())
                break;
            stringstream line_str(tempStr);
            line_str>>pat;
            transMat[0]=pat.GPS_TOW;
            for (int j=1; j<4; ++j)
                transMat[j]=pat.xyz_ecef[j-1];
            for (int j=4; j<7; ++j)
                transMat[j]=pat.sdxyz[j-4];
            ++lineNum;
            if(abs(transMat[0]-tk)<interval)
                break;
            measurement=transMat;
        }
        if(abs(transMat[0]-tk)<interval&&measurement[0]!=-1){
            reinit_data=false;
            is_measurement_good=true;
            measurement=transMat;
        }
    }
    else{
        assert(abs(measurement[0]-tkm1)<interval);
        measurement=transMat;        //last stranded measurement, t(p(k-1))

        while(!stream.eof()){
            getline(stream, tempStr);
            if(stream.fail())
                break;
            stringstream line_str(tempStr);
            line_str>>pat;
            transMat[0]=pat.GPS_TOW;
            for (int j=1; j<4; ++j)
                transMat[j]=pat.xyz_ecef[j-1];
            for (int j=4; j<7; ++j)
                transMat[j]=pat.sdxyz[j-4];
            ++lineNum;
            if(abs(transMat[0]-tk)<interval)
                break;
            measurement=transMat;
        }
        if(abs(transMat[0]-tk)<interval)
            //the second condition applies at the end of data file.
            // Otherwise, something wrong with reading data
        {
            is_measurement_good=true;
            measurement=transMat;
        }
        else{
            is_measurement_good=false;
            reinit_data=true;
            assert(false);//this should never happen with continuous sensor readings
        }
    }
    tkm1=tk;
    return is_measurement_good;
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

std::ostream& operator << (std::ostream &os, const GPSGrabber::RtklibPosPattern& rhs)
{
    char delim=' ';
    os<< rhs.GPS_Week<<delim<<rhs.GPS_TOW<<delim<<rhs.xyz_ecef[0]<<delim<<
         rhs.xyz_ecef[1]<<delim<<rhs.xyz_ecef[2]<<delim<<rhs.Q<<delim<<
         rhs.ns<<delim<<rhs.sdxyz[0]<<delim<<rhs.sdxyz[1]<<delim<<rhs.sdxyz[2]<<
         delim<<rhs.sdxy_yz_zx[0]<<delim<<rhs.sdxy_yz_zx[1]<<delim<<rhs.sdxy_yz_zx[2]<<
         delim<<rhs.age<<delim<<rhs.ratio;
    return os;
}
std::istream & operator>>(std::istream &is, GPSGrabber::RtklibPosPattern &rhs)
{
    is>> rhs.GPS_Week>>rhs.GPS_TOW>>rhs.xyz_ecef[0]>>
         rhs.xyz_ecef[1]>>rhs.xyz_ecef[2]>>rhs.Q>>
         rhs.ns>>rhs.sdxyz[0]>>rhs.sdxyz[1]>>rhs.sdxyz[2]>>
         rhs.sdxy_yz_zx[0]>>rhs.sdxy_yz_zx[1]>>rhs.sdxy_yz_zx[2]>>
         rhs.age>>rhs.ratio;
    return is;
}

//get next bundle of imu observations given t(k), for the first frame startIndex, no measurement should be returned
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
    // quaternion from sensor to world frame, velocity of sensor in world frame, accelerometer biases, and gyro biases
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
