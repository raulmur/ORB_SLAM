
#ifndef GPS_GRABBER_H
#define GPS_GRABBER_H

#include "vio/DataGrabber.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>
namespace vio{

enum GPSSolutionType{TOW_ECEF=0, TOW_NED, UTC_ECEF, UTC_NED};
class GPSGrabber:vio::DataGrabber{
public:    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // GPS TOW
    class RtklibPosPattern{
        virtual std::ostream& print(std::ostream &) const;
        virtual std::istream& read(std::istream &);
    public:
        friend std::ostream& operator << (std::ostream &os, const RtklibPosPattern & rhs)
        {
            return rhs.print(os);
        }
        friend std::istream & operator>>(std::istream &is, RtklibPosPattern &rhs)
        {
            return rhs.read(is);
        }
        virtual ~RtklibPosPattern(){}
    public:
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
    //civil time, UTC(GMT)
    class RtklibPosPatternUTCECEF: public RtklibPosPattern{
    public:
        int ymd[3]; //year, month, date
        int hour;
        int minute;
        double second;

        std::ostream& print(std::ostream & os) const;
        std::istream& read(std::istream & is);
    };
    //GTOW, lat, lon, h, sdn,e,u
    class RtklibPosPatternTOWNED: public RtklibPosPattern{
    public:
        double llh_geo[3];//lat deg, lon deg, height
        double sdneu[3]; //meters
        double sdne_eu_un[3];

        std::ostream& print(std::ostream & os) const;
        std::istream& read(std::istream & is);
    };

    // TODO: besides removing the header part, the first line of GPS observation is also deleted in the constructor
    GPSGrabber(const std::string file, double interval=1e-5, GPSSolutionType patType= GPSSolutionType::TOW_ECEF);
    virtual ~GPSGrabber();

    bool getObservation(double tk);
    bool getNextObservation();
    Eigen::Matrix<double, 7,1> transMat; //temporary sensor reading which also holds data at t(p(k)-1);
    Eigen::Matrix<double, 7,1> measurement; //GPS TOW, XYZ ECEF, Q, number of satellites, std XYZ in ECEF
    RtklibPosPattern * pat_;
};
void loadGPSData(std::string gps_file, std::vector<Eigen::Matrix<double, 7, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 7, 1> > > &gpsdata,
                 double startGPSTime, double finishGPSTime);
void testReadTOWECEF();
int gpstk_example1();

}
#endif // GPSGRABBER_H
