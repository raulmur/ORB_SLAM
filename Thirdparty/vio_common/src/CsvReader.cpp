
#include "vio/CsvReader.h"
#include <iomanip>

using namespace std;
namespace vio{
Eigen::VectorXd CsvReader::OkvisOutputPattern::toEigenVector(){
        Eigen::Matrix<double, 19, 1> transMat;
        transMat[0]=sec_;
        transMat[1]=nsec_;
        transMat[2]=frameIdInSource;
        for (int j=3; j<6; ++j)
            transMat[j]=p_WS_[j-3];
        for (int j=6; j<10; ++j)
            transMat[j]=q_WS_[j-6];
        for (int j=10; j<19; ++j)
            transMat[j]=sb_WS_[j-10];
        return transMat;
}
double CsvReader::OkvisOutputPattern::getTime()
{
    return nsec_*1e-9 + sec_;
}

bool CsvReader::getNextObservation()
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
        transMat = pat_->toEigenVector();

 
        measurement=transMat;

        tkm1= pat_->getTime();
        is_measurement_good=true;

    }
    else is_measurement_good=false;
    return is_measurement_good;
}

CsvReader::CsvReader(const std::string file, const OUTPUT_FILE_TYPE fileType, double interval):
    DataGrabber(file, interval){
    
    std::string tempStr;
    getline(stream,tempStr); //remove header part

    switch(fileType){
    case OKVIS_OUTPUT_FILE:
        pat_= new OkvisOutputPattern();
        transMat = Eigen::Matrix<double, 19, 1>::Zero();
        measurement = Eigen::Matrix<double, 19, 1>::Zero();
        break;
    default:
        std::cerr<<"File type not supported by CsvReader "<<std::endl;
        break;
    }
    transMat[0]=-1;
    measurement[0]=-1;
    
}
CsvReader::~CsvReader()
{
    if(pat_)
        delete pat_;
    pat_=NULL;
}

CsvReader::OkvisOutputPattern::OkvisOutputPattern():pq_SC_{0,0,0,0,0,0,0}{

}

std::ostream& CsvReader::OkvisOutputPattern::print(std::ostream & os) const
{
    char delim=',';

    os<< sec_<<"."<< std::setw(9) << std::setfill('0') << nsec_ << delim<< frameIdInSource <<
         delim<< p_WS_[0]<<delim<<p_WS_[1]<<delim<<
         p_WS_[2]<<delim<<q_WS_[0]<<delim<<q_WS_[1]<<delim<<
         q_WS_[2]<<delim<<q_WS_[3]<<delim<<sb_WS_[0]<<delim<<
         sb_WS_[1]<<delim<<
         sb_WS_[2]<<delim<<
         sb_WS_[3]<<delim<<
         sb_WS_[4]<<delim<<
         sb_WS_[5]<<delim<<
         sb_WS_[6]<<delim<<
         sb_WS_[7]<<delim<<
         sb_WS_[8]<<delim<< pq_SC_[0]<< delim <<pq_SC_[1]<< delim <<pq_SC_[2]<< delim <<
                pq_SC_[3]<< delim <<pq_SC_[4]<< delim <<pq_SC_[5]<< delim << pq_SC_[6];

    return os;
}
std::istream& CsvReader::OkvisOutputPattern::read(std::istream & is)
{   
    getline(is, time_, ',');
    std::string trunk = time_.substr (0, time_.length()-9);
    istringstream ss1(trunk);
    ss1>> sec_;

    std::string residuals = time_.substr (time_.length()-9);
    istringstream ss2(residuals);
    ss2>> nsec_;

    char delim;
    is>> frameIdInSource >> delim>> p_WS_[0]>> delim >>p_WS_[1]>> delim >>p_WS_[2]>> delim >>
            q_WS_[0]>> delim >>q_WS_[1]>> delim >>q_WS_[2]>> delim >> q_WS_[3]>> delim >>
            sb_WS_[0]>> delim >>sb_WS_[1]>> delim >> sb_WS_[2]>> delim >>sb_WS_[3]>> delim >> sb_WS_[4]>> delim >>sb_WS_[5]>> delim >>
            sb_WS_[6]>> delim >>sb_WS_[7]>> delim >> sb_WS_[8];
    if(is>> delim)
    {
        is>> pq_SC_[0]>> delim >>pq_SC_[1]>> delim >>pq_SC_[2]>> delim >>
                pq_SC_[3]>> delim >>pq_SC_[4]>> delim >>pq_SC_[5]>> delim >> pq_SC_[6];
    }
    return is;
}

void loadCsvData(string csvFile, vector<Eigen::Matrix<double, 19, 1>,
                 Eigen::aligned_allocator<Eigen::Matrix<double, 19, 1> > > &csvData, double startTime, double finishTime)
{
    if(finishTime<=0)
        finishTime = 1E20;
    CsvReader cg(csvFile, OKVIS_OUTPUT_FILE, 0.004);
    while(cg.getNextObservation()){
        Eigen::Matrix<double, 19,1> obs=cg.measurement;
        double time = obs[0] +1e-9*obs[1];
        if(time< startTime)
            continue;
        if(time> finishTime)
            break;
        csvData.push_back(obs);
    }
}



bool ViclamOutputReader::getNextObservation()
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
        transMat[0]=pat_->sec_;
        transMat[1]=pat_->nsec_;
        transMat[2]= 0;
        for (int j=3; j<6; ++j)
            transMat[j]=pat_->p_WS_[j-3];
        for (int j=6; j<10; ++j)
            transMat[j]=pat_->q_WS_[j-6];
        for (int j=10; j<19; ++j)
            transMat[j]=pat_->sb_WS_[j-10];

        measurement=transMat;

        tkm1= pat_->nsec_;
        is_measurement_good=true;
    }
    else is_measurement_good=false;
    return is_measurement_good;
}

ViclamOutputReader::ViclamOutputReader(const std::string file, double interval):
    DataGrabber(file, interval){
    transMat[0]=-1;
    measurement[0]=-1;
//    std::string tempStr;
//    getline(stream,tempStr); //remove header part

    pat_= new ViclamOutputPattern();

}
ViclamOutputReader::~ViclamOutputReader()
{
    if(pat_)
        delete pat_;
    pat_=NULL;
}

ViclamOutputReader::ViclamOutputPattern::ViclamOutputPattern(){

}

std::ostream& ViclamOutputReader::ViclamOutputPattern::print(std::ostream & os) const
{
    char delim=' ';

    os<< sec_<<"."<< std::setw(9) << std::setfill('0') << nsec_ << delim<< "0" <<
         delim<< p_WS_[0]<<delim<<p_WS_[1]<<delim<<
         p_WS_[2]<<delim<<q_WS_[0]<<delim<<q_WS_[1]<<delim<<
         q_WS_[2]<<delim<<q_WS_[3]<<delim<<sb_WS_[0]<<delim<<
         sb_WS_[1]<<delim<<
         sb_WS_[2]<<delim<<
         sb_WS_[3]<<delim<<
         sb_WS_[4]<<delim<<
         sb_WS_[5]<<delim<<
         sb_WS_[6]<<delim<<
         sb_WS_[7]<<delim<<
         sb_WS_[8];

    return os;
}
std::istream& ViclamOutputReader::ViclamOutputPattern::read(std::istream & is)
{
    getline(is, time_, ' ');
    std::string trunk = time_.substr (0, time_.length()-9);
    istringstream ss1(trunk);
    ss1>> sec_;

    std::string residuals = time_.substr (time_.length()-9);
    istringstream ss2(residuals);
    ss2>> nsec_;

    is>> p_WS_[0]>> p_WS_[1] >>p_WS_[2] >>
            q_WS_[0] >>q_WS_[1] >>q_WS_[2] >> q_WS_[3] >>
            sb_WS_[0] >>sb_WS_[1] >> sb_WS_[2] >>sb_WS_[3] >> sb_WS_[4] >>sb_WS_[5] >>
            sb_WS_[6] >>sb_WS_[7] >> sb_WS_[8];
    return is;
}

void loadViclamData(string viclamFile, vector<Eigen::Matrix<double, 19, 1>,
                    Eigen::aligned_allocator<Eigen::Matrix<double, 19, 1> > > &viclamData, double startTime, double finishTime)
{
    if(finishTime<=0)
        finishTime = 1E20;
    ViclamOutputReader vg(viclamFile, 0.004);
    while(vg.getNextObservation()){
        Eigen::Matrix<double, 19,1> obs=vg.measurement;
        double time = obs[0] + 1e-9*obs[1];
        if(time< startTime)
            continue;
        if(time> finishTime)
            break;
        viclamData.push_back(obs);
    }
}
}
