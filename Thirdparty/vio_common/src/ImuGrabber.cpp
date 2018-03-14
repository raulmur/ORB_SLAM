
#include "vio/ImuGrabber.h"
#include <iomanip>

using namespace std;
namespace vio{

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

std::ostream& operator << (std::ostream &os, const IMUGrabber::IndexedPlainTextPattern & rhs)
{
    os<< rhs.index<< rhs.GPS_TOW<<' '<<
         rhs.awxyz[0]<<' '<<rhs.awxyz[1]<<' '<<rhs.awxyz[2]<<' '<<
         rhs.awxyz[3]<<' '<<rhs.awxyz[4]<<' '<<rhs.awxyz[5];
    return os;
}

std::istream & operator>>(std::istream &is, IMUGrabber::IndexedPlainTextPattern &rhs)
{
    is>>rhs.index>> rhs.GPS_TOW>>rhs.awxyz[0]>>rhs.awxyz[1]>>rhs.awxyz[2]>>
            rhs.awxyz[3]>>rhs.awxyz[4]>>rhs.awxyz[5];
    rhs.GPS_TOW*= 0.001;
    rhs.awxyz[0]*= rhs.linearAccelerationUnit;
    rhs.awxyz[1]*= rhs.linearAccelerationUnit;
    rhs.awxyz[2]*= rhs.linearAccelerationUnit;
    rhs.awxyz[3]*= rhs.angularRateUnit;
    rhs.awxyz[4]*= rhs.angularRateUnit;
    rhs.awxyz[5]*= rhs.angularRateUnit;
    return is;
}

std::ostream& operator << (std::ostream &os, const IMUGrabber::SensorStreamCSVPattern & rhs)
{
    os<< rhs.timestamp<<' ';
    for(const std::pair<int, Eigen::Vector3d>& obs: rhs.measurements){
         os<<obs.first<<' '<<(obs.second)[0]<<' '<<(obs.second)[1]<<' '<<(obs.second)[2]<<' ';
    }
    return os;
}

std::istream & operator>>(std::istream &is, IMUGrabber::SensorStreamCSVPattern &rhs)
{
    char delim;
    is>>rhs.timestamp;
    int obsIndex;    
    Eigen::Vector3d obs;
    while(is>>delim)
    {
        is>>obsIndex>>delim>>obs[0] >>delim>> obs[1]>>delim>> obs[2];
        rhs.measurements.push_back(std::make_pair(obsIndex, obs));
    }
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
    std::cout <<"Reading in imu file "<< file<< std::endl;
}
void IMUGrabber::print(const std::string message)
{
    if(!message.empty())
        std::cout << message <<std::endl;
    for(const Eigen::Matrix<double, 7, 1>& item: measurement)
    {
        std::cout<< std::setprecision(12) << item[0] << " "<< item.tail<6>().transpose() << std::endl;
    }
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
                if(pat.IMU_Sync_Nanoseconds==-1){
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
            else if(file_type==IndexedPlainText)
            {
                IMUGrabber::IndexedPlainTextPattern pat;
                line_str>>pat;
                transMat[0]=pat.GPS_TOW;
                for (int j=1; j<transMat.rows(); ++j)
                    transMat[j]=pat.awxyz[j-1];
            }
            else if(file_type == SensorStreamCSV)
            {
                IMUGrabber::SensorStreamCSVPattern pat;
                line_str>>pat;
                transMat[0]=pat.timestamp;
                if(pat.measurements.size()>=2 && pat.measurements[0].first== SensorStreamCSVPattern::accelerometerIndex &&
                        pat.measurements[1].first== SensorStreamCSVPattern::gyroIndex)
                {
                    for (int j=1; j<4; ++j)
                        transMat[j]=pat.measurements[0].second[j-1];
                    for (int j=4; j<7; ++j)
                        transMat[j]=pat.measurements[1].second[j-4];
                }
                else{
                    ++lineNum;
                    continue;
                }
            }

            ++lineNum;
            if(transMat[0]>tk)
                break;
            measurement.push_back(transMat);
        }
        if(transMat[0]>tk)
            reinit_data=false;

    }
    else{
        if(tk< transMat[0])
        {
            tkm1=tk;
            std::cerr<<"Try to obtain inertial measurements until "<<std::setprecision(12)<<tk<<" which is however before the latest retrieved entry of time "<< transMat[0]<<std::endl;
            std::cerr<<"from the file stream. A possible cause is a gap in the inertial data which can be filled by linear interpolation."<< std::endl;
            is_measurement_good = false;
            return is_measurement_good;
        }
        if(measurement.size()){
            // keep the last imu measurement from the last retrieval
            measurement.front()=measurement.back(); //t(p(k-1)-1)
            measurement.resize(1);
            assert(measurement[0][0]<=tkm1);
        }
        measurement.push_back(transMat);        //last stranded measurement, t(p(k-1))

        while(!stream.eof()){
            getline(stream, tempStr);
            if(stream.fail())
                break;
            stringstream line_str(tempStr);
            if(file_type==MicroStrainCSV){
                IMUGrabber::MicroStrainCSVPattern pat;
                line_str>>pat;
                if(pat.IMU_Sync_Nanoseconds==-1){
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
            else if(file_type==IndexedPlainText)
            {
                IMUGrabber::IndexedPlainTextPattern pat;
                line_str>>pat;
                transMat[0]=pat.GPS_TOW;
                for (int j=1; j<transMat.rows(); ++j)
                    transMat[j]=pat.awxyz[j-1];
            }
            else if(file_type == SensorStreamCSV)
            {
                IMUGrabber::SensorStreamCSVPattern pat;
                line_str>>pat;
                transMat[0]=pat.timestamp;
                if(pat.measurements.size()>=2 && pat.measurements[0].first== SensorStreamCSVPattern::accelerometerIndex &&
                        pat.measurements[1].first== SensorStreamCSVPattern::gyroIndex)
                {
                    for (int j=1; j<4; ++j)
                        transMat[j]=pat.measurements[0].second[j-1];
                    for (int j=4; j<7; ++j)
                        transMat[j]=pat.measurements[1].second[j-4];
                }
                else{
                    ++lineNum;
                    continue;
                }
            }

            ++lineNum;
            if(transMat[0]>tk)
                break;
            measurement.push_back(transMat);
        }
    }
    if(transMat[0]>tk)
        is_measurement_good=true;
    else
        is_measurement_good=false; //this can heppen when the timestamp goes beyond those in the imu file.

    tkm1=tk;
    return is_measurement_good;
}

bool StatesGrabber::getObservation(double tk)
{
    //Assume each line in States file: GPS TOW, i.e., t(k), position of sensor in world frame,
    // quaternion wxyz from sensor to world frame, velocity of sensor in world frame, accelerometer biases, and gyro biases
    double precursor=0; //used to read in something tentatively
    string tempStr;
    double epsilonTime = 1e-5;
    if(tk - tkm1> epsilonTime){
        while(!stream.eof()){
            stream>>precursor;
            if(stream.fail())
                return false;
            measurement[0]=precursor;
            char trashbin = '0';
            if (delimiter != ' ') {
                for (size_t j=1; j<measurement.size(); ++j)
                    stream>>trashbin>>measurement[j];
            } else {
                for (size_t j=1; j<measurement.size(); ++j)
                    stream>>measurement[j];
            }
            getline(stream, tempStr);       //remove the remaining part, this works even when it is empty
            tkm1=precursor;
            if(std::fabs(precursor - tk)< epsilonTime)
            {
                is_measurement_good=true;
                break;
            }
            else if(precursor - tk >=epsilonTime)
            {
                std::cerr <<"Warn: current time exceeds required time in state grabber "<<std::endl;
                is_measurement_good = false;
                break;
            }
        }
        if(stream.eof())
        {
            is_measurement_good = false;
        }
    }
    else if(std::fabs(tkm1 - tk)< epsilonTime)
    {
        is_measurement_good =true;
    }
    else
    {
        std::cerr <<"Warn: current time well above required time in state grabber "<<std::endl;
        is_measurement_good = false;
    }

    return is_measurement_good;
}

bool StatesGrabber::getNextObservation()
{
    //Assume each line in States file: GPS TOW, i.e., t(k), position of sensor in world frame,
    // quaternion xyzw from sensor to world frame, velocity of sensor in world frame, accelerometer biases, and gyro biases
    double precursor=0; //used to read in something tentatively
    string tempStr;

    if(!stream.eof()){
        stream>>precursor;
        if(stream.fail())
            return false;
        measurement[0]=precursor;
        if (delimiter == ' ') {
            for (size_t j=1; j<measurement.size(); ++j)
                stream>>measurement[j];
        } else {
            char trashbin = '0';
            for (size_t j=1; j<measurement.size(); ++j)
                stream >> trashbin >> measurement[j];
        }
        getline(stream, tempStr);       //remove the remaining part, this works even when it is empty

        if(precursor > tkm1)
        {
            is_measurement_good=true;
        }
        else
        {
            std::cerr <<"Warn: current entry time is less than previous one in state grabber "<<std::endl;
            is_measurement_good = false;
        }
        tkm1=precursor;
    }else
    {
        is_measurement_good = false;
    }
    return is_measurement_good;
}


void loadKITTIPoses(string file, vector<pair<double, Eigen::Matrix<double,3,4> > > & vTimeAndPose)
{
    StatesGrabber sg(file, 12, 0); // Note by default States

    vTimeAndPose.clear();
    vTimeAndPose.reserve(5000);
    int counter =0;
    while(sg.getNextObservation()){
        Eigen::Matrix<double,3,4> currentPose;
        currentPose<< sg.measurement[0], sg.measurement[1], sg.measurement[2], sg.measurement[3],
                sg.measurement[4], sg.measurement[5], sg.measurement[6], sg.measurement[7],
                sg.measurement[8], sg.measurement[9], sg.measurement[10], sg.measurement[11];

        std::pair<double, Eigen::Matrix<double,3,4> > meas= make_pair(counter*0.1f, currentPose);
        vTimeAndPose.push_back(meas);
        ++counter;
    }
    cout <<"loaded KITTI poses "<< vTimeAndPose.size()<< endl;
    cout <<"the last pose is "<< endl << vTimeAndPose.back().second<< endl;
}

void computeKITTIRf2s(){
    std::vector<std::pair<double, Eigen::Matrix<double,3,4> > > vTimeAndPose;
    string file = "/media/jhuai/Seagate/jhuai/kitti/data_odometry_poses/dataset/poses/00.txt";

    loadKITTIPoses(file, vTimeAndPose);
    ofstream output_euler("/home/jhuai/Desktop/temp/rot_truth.txt");
    for(size_t i=1; i< vTimeAndPose.size(); ++i){
        Eigen::Matrix3d Rf2s= vTimeAndPose[i].second.block<3,3>(0,0).transpose()*vTimeAndPose[i-1].second.block<3,3>(0,0);
        Eigen::Vector3d ea = Rf2s.eulerAngles(2,1,0);
        output_euler<< i<<" "<<ea.transpose()<< endl;
    }
}


}
