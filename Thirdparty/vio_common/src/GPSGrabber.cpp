
#include "vio/GPSGrabber.h"
#include "vio/eigen_utils.h"

#ifdef HAVE_GPSTK
#include <gpstk-2.5.linux.x86_64/SystemTime.hpp>
#include <gpstk-2.5.linux.x86_64/CommonTime.hpp>
#include <gpstk-2.5.linux.x86_64/CivilTime.hpp>
#include <gpstk-2.5.linux.x86_64/YDSTime.hpp>
#include <gpstk-2.5.linux.x86_64/GPSWeekSecond.hpp>
#include <gpstk-2.5.linux.x86_64/Position.hpp>
#endif

using namespace std;

namespace vio {

bool GPSGrabber::getNextObservation()
{
    string tempStr;
    if(!stream.eof()){
        getline(stream, tempStr);
        if(stream.fail()){
            is_measurement_good=false;
            return false;
        }
        stringstream line_str(tempStr);
        line_str>>(*pat_);
        transMat[0]=pat_->GPS_TOW;
        for (int j=1; j<4; ++j)
            transMat[j]=pat_->xyz_ecef[j-1];
        for (int j=4; j<7; ++j)
            transMat[j]=pat_->sdxyz[j-4];
        measurement=transMat;
//        if(pat_->GPS_TOW - tkm1>0.01 && pat_->GPS_TOW- tkm1< 5)
//        {
            tkm1= pat_->GPS_TOW;
            is_measurement_good=true;
//        }
//        else
//            is_measurement_good=false;
    }
    else is_measurement_good=false;
    return is_measurement_good;
}

GPSGrabber::GPSGrabber(const std::string file, double interval, GPSSolutionType patType):
    DataGrabber(file, interval){
    transMat[0]=-1;
    measurement[0]=-1;
    std::string tempStr;
    getline(stream,tempStr); //remove header part
    //N.B. we also delete the first line of observation
    while(tempStr.find('%')!=std::string::npos)
        getline(stream,tempStr);

    switch(patType){
    case TOW_ECEF:
        pat_= new RtklibPosPattern();
        break;
#ifdef HAVE_GPSTK
    case UTC_ECEF:
        pat_= new RtklibPosPatternUTCECEF();
        break;
    case TOW_NED:
        pat_= new RtklibPosPatternTOWNED();
        break;
    case UTC_NED:
#endif
    default:
        cerr<<"Unimplemented file format!"<<endl;
        break;
    }
}

GPSGrabber::~GPSGrabber()
{
    if(pat_)
        delete pat_;
    pat_=NULL;
}
bool GPSGrabber::getObservation(double tk)
{
    //Assume each line in gps_file: GPSWeek, TOW, ecef X,Y,Z, Q and no of satels, and std ecef X,Y,Z
    int lineNum=0;      //how many lines have been read in
    string tempStr;

    if(reinit_data){//grab data until its timestamp is greater than t(startIndex)
        while(!stream.eof()){
            getline(stream, tempStr);
            if(stream.fail())
                break;
            stringstream line_str(tempStr);
            line_str>>(*pat_);
            transMat[0]=pat_->GPS_TOW;
            for (int j=1; j<4; ++j)
                transMat[j]=pat_->xyz_ecef[j-1];
            for (int j=4; j<7; ++j)
                transMat[j]=pat_->sdxyz[j-4];
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
            line_str>>(*pat_);
            transMat[0]=pat_->GPS_TOW;
            for (int j=1; j<4; ++j)
                transMat[j]=pat_->xyz_ecef[j-1];
            for (int j=4; j<7; ++j)
                transMat[j]=pat_->sdxyz[j-4];
            ++lineNum;
            if(abs(transMat[0]-tk)<interval)
                break;
            measurement=transMat;
        }
        if(abs(transMat[0]-tk)<interval)
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


std::ostream& GPSGrabber::RtklibPosPattern::print(std::ostream & os) const
{
    char delim=' ';

    os<< GPS_Week<<delim<<GPS_TOW<<delim<<xyz_ecef[0]<<delim<<
         xyz_ecef[1]<<delim<<xyz_ecef[2]<<delim<<Q<<delim<<
         ns<<delim<<sdxyz[0]<<delim<<sdxyz[1]<<delim<<sdxyz[2]<<
         delim<<sdxy_yz_zx[0]<<delim<<sdxy_yz_zx[1]<<delim<<sdxy_yz_zx[2]<<
         delim<<age<<delim<<ratio;
    return os;
}
std::istream& GPSGrabber::RtklibPosPattern::read(std::istream & is)
{
    is>> GPS_Week>>GPS_TOW>>xyz_ecef[0]>>
            xyz_ecef[1]>>xyz_ecef[2]>>Q>>
            ns>>sdxyz[0]>>sdxyz[1]>>sdxyz[2]>>
            sdxy_yz_zx[0]>>sdxy_yz_zx[1]>>sdxy_yz_zx[2]>>
            age>>ratio;
    return is;
}

std::ostream& GPSGrabber::RtklibPosPatternUTCECEF::print(std::ostream & os) const
{
    char delim=' ';
    os<< ymd[0]<<'/'<<ymd[1]<<'/'<<ymd[2]<<' '<<hour<<':'<< minute<<':'
              <<second<<delim<< xyz_ecef[0]<<delim<<
                xyz_ecef[1]<<delim<<xyz_ecef[2]<<delim<<Q<<delim<<
                ns<<delim<<sdxyz[0]<<delim<<sdxyz[1]<<delim<<sdxyz[2]<<
                delim<<sdxy_yz_zx[0]<<delim<<sdxy_yz_zx[1]<<delim<<sdxy_yz_zx[2]<<
                delim<<age<<delim<<ratio;
    return os;
}

std::ostream& GPSGrabber::RtklibPosPatternTOWNED::print(std::ostream & os) const
{
    char delim=' ';
    os<< GPS_Week<<delim<<GPS_TOW<<delim<< llh_geo[0]<<delim<<
         llh_geo[1]<<delim<<llh_geo[2]<<delim<<Q<<delim<<
         ns<<delim<<sdneu[0]<<delim<<sdneu[1]<<delim<<sdneu[2]<<
         delim<<sdne_eu_un[0]<<delim<<sdne_eu_un[1]<<delim<<sdne_eu_un[2]<<
         delim<<age<<delim<<ratio;
    return os;
}

#ifdef HAVE_GPSTK
std::istream& GPSGrabber::RtklibPosPatternTOWNED::read(std::istream & is)
{
    is>> GPS_Week>>GPS_TOW>>llh_geo[0]>>
            llh_geo[1]>>llh_geo[2]>>Q>>
            ns>>sdneu[0]>>sdneu[1]>>sdneu[2]>>
            sdne_eu_un[0]>>sdne_eu_un[1]>>sdne_eu_un[2]>>
            age>>ratio;
    //convert NED to ECEF
    gpstk::Position d;
    d.setGeodetic(llh_geo[0], llh_geo[1], llh_geo[2]);
    d.transformTo(gpstk::Position::Cartesian);
    xyz_ecef[0] = d.X();
    xyz_ecef[1] = d.Y();
    xyz_ecef[2] = d.Z();
    Eigen::Matrix3d Ce2n=vio::llh2dcm(Eigen::Vector3d(llh_geo[0]*(M_PI/180), llh_geo[1]*(M_PI/180), llh_geo[2]));
    Eigen::Matrix3d Rneu;
    Rneu<< copysign(sdneu[0]*sdneu[0], sdneu[0]),  copysign(sdne_eu_un[0]*sdne_eu_un[0], sdne_eu_un[0]), copysign(sdne_eu_un[2]*sdne_eu_un[2], sdne_eu_un[2]),
            copysign(sdne_eu_un[0]*sdne_eu_un[0], sdne_eu_un[0]),  copysign(sdneu[1]*sdneu[1], sdneu[1]), copysign(sdne_eu_un[1]*sdne_eu_un[1], sdne_eu_un[1]),
            copysign(sdne_eu_un[2]*sdne_eu_un[2], sdne_eu_un[2]),  copysign(sdne_eu_un[1]*sdne_eu_un[1], sdne_eu_un[1]), copysign(sdneu[2]*sdneu[2], sdneu[2]);
    Eigen::Matrix3d Recef=Ce2n.transpose()*Rneu*Ce2n;
    sdxyz[0]= copysign(sqrt(abs(Recef(0,0))), Recef(0,0));
    sdxyz[1]= copysign(sqrt(abs(Recef(1,1))), Recef(1,1));
    sdxyz[2]= copysign(sqrt(abs(Recef(2,2))), Recef(2,2));
    sdxy_yz_zx[0]= copysign(sqrt(abs(Recef(0,1))), Recef(0,1));
    sdxy_yz_zx[1]= copysign(sqrt(abs(Recef(1,2))), Recef(1,2));
    sdxy_yz_zx[2]= copysign(sqrt(abs(Recef(2,0))), Recef(2,0));
    return is;
}


std::istream& GPSGrabber::RtklibPosPatternUTCECEF::read(std::istream & is)
{
    char delim=' ';
    is>> ymd[0]>>delim>>ymd[1]>>delim>>ymd[2]>>hour>>delim
            >>minute>>delim>>second>>xyz_ecef[0]>>
            xyz_ecef[1]>>xyz_ecef[2]>>Q>>
            ns>>sdxyz[0]>>sdxyz[1]>>sdxyz[2]>>
            sdxy_yz_zx[0]>>sdxy_yz_zx[1]>>sdxy_yz_zx[2]>>
            age>>ratio;
    //convert of time
    gpstk::CivilTime civtime(ymd[0], ymd[1], ymd[2], hour, minute, second, gpstk::TimeSystem::GPS);
    gpstk::GPSWeekSecond gpstime(civtime);
    GPS_Week = gpstime.week;
    GPS_TOW = gpstime.sow;
    return is;
}

void testReadTOWECEF(){
    string tempStr= "1752 414790.200   40.003227423  -83.042986648   212.6010   2   7   0.0086   0.0090   0.0241  -0.0068   0.0101  -0.0108   0.20    2.6";
    string target = "1752 414790.200 592619.2852 -4856617.0869 4078396.7487 2 7 0.0085 0.0163 0.0200 0.0051 -0.0167 -0.0082 0.20 2.6";
    stringstream line_str(tempStr);
    GPSGrabber::RtklibPosPattern * pat= new GPSGrabber::RtklibPosPatternTOWNED();
    line_str>>(*pat);
    cout<<"Expected result"<< std::endl<< target<<endl;
    cout<< ((GPSGrabber::RtklibPosPattern)(*pat))<<endl;
    cout<< "Actual result "<< std::endl<< std::setprecision(10)<< *pat<<endl;
}

// test converting different times
int gpstk_example1()
{
    //2015/11/11 15:38:38.6 GPST (week1870 315518.6s)
    //2015/11/11 22:04:25.8 GPST (week1870 338665.8s)
    gpstk::CivilTime civtime2(2015, 11, 11, 15, 38, 38.6, gpstk::TimeSystem::GPS);
    gpstk::GPSWeekSecond gpstime2(civtime2);
    std::cout<< "Expected gps week and second of week: 1870 315518.6"<< std::endl;
    cout<<"Calculted gpg week and second of week "<< gpstime2.week<< " "<<setprecision(10)<< gpstime2.sow<<endl;

    gpstk::GPSWeekSecond gpstime3(1870, 338665.8, gpstk::TimeSystem::GPS);
    gpstk::CivilTime civtime3(gpstime3);
    std::cout<< "Expected civil time 2015/11/11 22:04:25.8"<< std::endl;
    cout<<"Calculated civil time "<< setprecision(6)<< civtime3 <<" and civil time second " << civtime3.second<<endl;
    return 0;
}
#else
std::istream& GPSGrabber::RtklibPosPatternTOWNED::read(std::istream & is)
{
    assert(false);
    return is;
}

std::istream& GPSGrabber::RtklibPosPatternUTCECEF::read(std::istream & is)
{
    assert(false);
    return is;
}
void testReadTOWECEF(){
}

int gpstk_example1()
{
    return 0;
}
#endif


void loadGPSData(string gps_file, vector<Eigen::Matrix<double, 7, 1> > &gpsdata, double startGPSTime, double finishGPSTime)
{
    GPSGrabber gg(gps_file, 0.004);
    while(gg.getNextObservation()){
        Eigen::Matrix<double, 7,1> gps_obs=gg.measurement;
        if(gps_obs[0]< startGPSTime || gps_obs[0]> finishGPSTime)
            continue;
        gpsdata.push_back(gps_obs);
    }
}

} //namespace
