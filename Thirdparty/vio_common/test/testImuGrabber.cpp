
#include "vio/ImuGrabber.h"

int main()
{
    using namespace vio;
    std::string mImuFile0("../test/IndexedImu.txt");
    IMUGrabber mIG0(mImuFile0, IndexedPlainText,0.014);

    mIG0.getObservation(0);
    mIG0.print("get observation 0");
    mIG0.getObservation(505.417);
    mIG0.print("get observation 505.418");


    std::string mImuFile("../test/testimugrabber.csv");
    IMUGrabber mIG(mImuFile, SensorStreamCSV,0.005);

    mIG.getObservation(0);
    mIG.print("get observation 0");
    mIG.getObservation(511951.63293);
    mIG.print("get observation 511951.63293");
    mIG.getObservation(511951.72555);
    mIG.print("get observation 511951.72555");

    mIG.getObservation(511951.75116);
    mIG.print("get observation 511951.75116");
    mIG.getObservation(511951.88972);
    mIG.print("get observation 511951.88972");
    return 0;
}
