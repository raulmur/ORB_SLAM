
#ifndef CSVREADER_H
#define CSVREADER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "vio/DataGrabber.h" // only for datagrabber base class
namespace vio{
enum OUTPUT_FILE_TYPE{OKVIS_OUTPUT_FILE=0};
class CsvReader:DataGrabber{
public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
    class LinePattern{
        virtual std::ostream& print(std::ostream &) const=0;
        virtual std::istream& read(std::istream &)=0;
    public:
        friend std::ostream& operator << (std::ostream &os, const LinePattern & rhs)
        {
            return rhs.print(os);
        }
        friend std::istream & operator>>(std::istream &is, LinePattern &rhs)
        {
            return rhs.read(is);
        }
        virtual ~LinePattern(){}
        virtual Eigen::VectorXd toEigenVector()=0;
        virtual double getTime()=0;
    };

    class OkvisOutputPattern: public LinePattern{

        virtual std::ostream& print(std::ostream &) const;
        virtual std::istream& read(std::istream &);
    public:
        OkvisOutputPattern();       
        virtual ~OkvisOutputPattern(){}
        virtual Eigen::VectorXd toEigenVector(); //sec, nsec, frameid, pxyz, qxyzw, speed and bg ba
	virtual double getTime();
    public:
        //each line of the csv file contains timestamp(unit nsec) frameIdInSource p_WS_W_x p_WS_W_y p_WS_W_z
        // q_WS_x q_WS_y q_WS_z q_WS_w v_WS_W_x v_WS_W_y v_WS_W_z b_g_x b_g_y b_g_z b_a_x b_a_y b_a_z
        // and optionally, p_SC_S_x p_SC_S_y p_SC_S_z q_SC_x q_SC_y q_SC_z q_SC_w

        std::string time_;
        uint32_t sec_;
        uint32_t nsec_;
        int frameIdInSource;
        double p_WS_[3];
        double q_WS_[4];
        double sb_WS_[9];
        double pq_SC_[7];
    };
   
    
    CsvReader(const std::string file, const OUTPUT_FILE_TYPE fileType=OKVIS_OUTPUT_FILE, double interval=1e-5);
    virtual ~CsvReader();

    bool getNextObservation();
    Eigen::VectorXd transMat; //temporary sensor reading which also holds data at t(p(k)-1);
    Eigen::VectorXd measurement; // the current measurement
    LinePattern * pat_;
};

// use CsvReader to load data from the okvis output csv file in which each line contains frameIdInSource
void loadCsvData(std::string csvFile, std::vector<Eigen::Matrix<double, 19, 1>,
                 Eigen::aligned_allocator<Eigen::Matrix<double, 19, 1> > > &csvData,
                 double startTime=0, double finishTime=0);


class ViclamOutputReader:vio::DataGrabber{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    class ViclamOutputPattern{

        virtual std::ostream& print(std::ostream &) const;
        virtual std::istream& read(std::istream &);
    public:
        ViclamOutputPattern();
        friend std::ostream& operator << (std::ostream &os, const ViclamOutputPattern & rhs)
        {
            return rhs.print(os);
        }
        friend std::istream & operator>>(std::istream &is, ViclamOutputPattern &rhs)
        {
            return rhs.read(is);
        }
        virtual ~ViclamOutputPattern(){}
    public:
        //each line of the viclam output file contains timestamp(unit nsec) p_WS_W_x p_WS_W_y p_WS_W_z
        // q_WS_x q_WS_y q_WS_z q_WS_w v_WS_W_x v_WS_W_y v_WS_W_z b_g_x b_g_y b_g_z b_a_x b_a_y b_a_z


        std::string time_;
        uint32_t sec_;
        uint32_t nsec_;
        double p_WS_[3];
        double q_WS_[4];
        double sb_WS_[9];
    };


    ViclamOutputReader(const std::string file, double interval=1e-5);
    virtual ~ViclamOutputReader();


    bool getNextObservation();
    Eigen::Matrix<double, 19, 1> transMat; //temporary sensor reading which hold the last line of data
    Eigen::Matrix<double, 19, 1> measurement; //sec, nsec, placeholder, pxyz, qxyzw, speed and bg ba
    ViclamOutputPattern * pat_;
};
//load all data of a viclam output file
void loadViclamData(std::string viclamFile, std::vector<Eigen::Matrix<double, 19, 1>,
                    Eigen::aligned_allocator<Eigen::Matrix<double, 19, 1> > > &viclamData,
                    double startTime=0, double finishTime=0);
}

#endif // CSVREADER_H
